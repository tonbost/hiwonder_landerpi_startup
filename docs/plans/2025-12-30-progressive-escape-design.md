# Progressive Escape Design for Exploration

**Date:** 2025-12-30
**Status:** Implemented
**Problem:** Robot gets stuck oscillating back-and-forth during exploration, not turning enough to find escape routes

## Architecture Update (2025-12-30)

**Original issue:** SSH-based sensor reads were too slow (~0.1Hz, 8-10 seconds per read) due to:
- Mac → SSH → Pi → Docker → ROS2 → response chain
- Network latency and command overhead

**Solution:** On-robot architecture where exploration runs directly on Raspberry Pi:
- `robot_explorer.py` - Self-contained exploration script running on Pi
- `deploy_explorer.py` - Deployment and control from Mac
- Uses local `docker exec` for fast sensor reads (~8Hz)

**Commands:**
```bash
# On-robot approach (recommended)
uv run python deploy_explorer.py deploy              # Upload to robot (one time)
uv run python deploy_explorer.py start --duration 10 # Start (streams output)
uv run python deploy_explorer.py stop                # Emergency stop

# Legacy SSH approach (slower)
uv run python validation/test_exploration.py start --yes
```

## Problem Analysis

Current exploration behavior issues:
1. No proactive scanning - robot only turns when a distant sector appears "best"
2. Minimal escape behavior - backs up 2 seconds and does one turn
3. Reactive decision-making - oscillates between same 2 sectors
4. No oscillation detection - doesn't recognize back-and-forth pattern
5. Stuck timeout too long (10 seconds)

## Solution: Progressive Escape

Detect oscillation early and escalate through increasingly aggressive escape maneuvers. Uses arm-mounted depth camera for fast directional scanning.

## Oscillation Detection

Track recent turn directions and detect patterns early.

**Turn History Buffer:**
```
Last 6 turns: [L, R, L, R, L, R] → Oscillation (alternating pattern)
Last 6 turns: [L, L, R, L, R, R] → No pattern, continue normally
```

**Detection Criteria:**
- Oscillation: 3+ alternating turns within 10 seconds
- Corner stuck: `obstacle_stop` triggers 3+ times while trying to go forward

**New State Variables:**
```python
turn_history: list[tuple[str, float]]  # (direction, timestamp)
forward_blocked_count: int              # times obstacle_stop triggered
last_escape_level: int                  # 0=none, 1=wide, 2=reverse, 3=scan
```

## Escalation Levels

### Level 1: Wide Turn
- **Trigger:** Oscillation detected OR forward blocked 3x
- **Action:** Turn 90° toward side with more open space + arm quick glance
- **Duration:** ~1.5 seconds
- **On failure:** Wait 3 seconds, escalate to Level 2

### Level 2: Reverse + Turn 180°
- **Trigger:** Level 1 didn't help
- **Action:**
  1. Back up 0.5m (2 seconds)
  2. Turn 180° (face opposite direction)
  3. Try moving forward
- **Duration:** ~4 seconds
- **On failure:** Escalate to Level 3

### Level 3: Full 360° Scan
- **Trigger:** Level 2 didn't help
- **Action:**
  1. Stop completely
  2. Arm sweeps 180° with depth camera (2-3 seconds)
  3. If opening found in front hemisphere → turn toward it
  4. If no opening → chassis 360° rotation with lidar (6 seconds)
  5. Turn to face widest opening
  6. Move forward
- **Duration:** ~8-10 seconds
- **On failure:** Announce "I'm trapped", wait for manual help

### Cooldown
- After successful escape: reset to Level 0
- Don't re-escalate for 15 seconds

## Arm-Mounted Depth Camera Scanning

The depth camera is mounted on the robotic arm (servo 1 controls pan). Advantages:
- Faster than rotating whole robot (~2-3 sec vs 6-8 sec)
- More stable readings (no motion blur)
- Can check different heights (arm has vertical range)

**Arm Scan Routine:**
```python
# Servo 1 position: 200=right, 500=center, 800=left
1. Move arm to scan-start (servo 1 = 200, looking right)
2. Sweep to scan-end (servo 1 = 800, looking left)
3. Sample depth at 5-7 positions during sweep
4. Record: angle → min_depth, avg_depth, clear_width
5. Return arm to neutral (servo 1 = 500)
```

**Integration with Escape Levels:**

| Level | Lidar | Depth Camera (Arm) |
|-------|-------|-------------------|
| 1 - Wide Turn | Check sides | Quick glance left/right |
| 2 - Reverse | Rear check | Not used (facing forward) |
| 3 - Full Scan | Backup: 360° rotation | Primary: Arm sweep 180° |

## Implementation Structure

### New Files

```
validation/exploration/
├── escape_handler.py    # NEW: Progressive escape logic
├── arm_scanner.py       # NEW: Arm-mounted depth camera scanning
├── explorer.py          # MODIFY: integrate escape handler
└── frontier_planner.py  # MODIFY: add oscillation detection
```

### escape_handler.py

```python
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional
import time

class EscapeLevel(Enum):
    NONE = 0
    WIDE_TURN = 1
    REVERSE_180 = 2
    FULL_SCAN = 3
    TRAPPED = 4

@dataclass
class EscapeResult:
    success: bool
    direction: Optional[float]  # angle to turn toward
    message: str

class EscapeHandler:
    def __init__(
        self,
        move_func: Callable[[float, float, float], bool],
        stop_func: Callable[[], None],
        arm_scanner: 'ArmScanner',
        config: Optional['EscapeConfig'] = None,
    ):
        self.move = move_func
        self.stop = stop_func
        self.arm_scanner = arm_scanner
        self.config = config or EscapeConfig()

        self.level = EscapeLevel.NONE
        self.last_escape_time: float = 0
        self.cooldown: float = 15.0

    def check_should_escape(self, oscillating: bool, blocked_count: int) -> bool:
        """Check if escape should be triggered."""
        if time.time() - self.last_escape_time < self.cooldown:
            return False
        return oscillating or blocked_count >= 3

    def execute_escape(self) -> EscapeResult:
        """Execute current escape level, escalating if needed."""
        self.level = EscapeLevel(self.level.value + 1)

        if self.level == EscapeLevel.WIDE_TURN:
            return self._execute_wide_turn()
        elif self.level == EscapeLevel.REVERSE_180:
            return self._execute_reverse_180()
        elif self.level == EscapeLevel.FULL_SCAN:
            return self._execute_full_scan()
        else:
            return EscapeResult(False, None, "I'm trapped")

    def reset(self) -> None:
        """Reset after successful escape."""
        self.level = EscapeLevel.NONE
        self.last_escape_time = time.time()

    def _execute_wide_turn(self) -> EscapeResult:
        """Level 1: Turn 90° toward more open side."""
        # Quick arm glance to determine better side
        left_depth = self.arm_scanner.quick_glance("left")
        right_depth = self.arm_scanner.quick_glance("right")

        # Turn toward more open side
        if left_depth > right_depth:
            direction = 90.0  # Turn left
        else:
            direction = -90.0  # Turn right

        return EscapeResult(True, direction, f"Wide turn {'left' if direction > 0 else 'right'}")

    def _execute_reverse_180(self) -> EscapeResult:
        """Level 2: Back up and turn 180°."""
        # Back up
        self.move(-0.15, 0, 0)
        time.sleep(2.0)
        self.stop()

        # Turn 180°
        return EscapeResult(True, 180.0, "Reverse and turn around")

    def _execute_full_scan(self) -> EscapeResult:
        """Level 3: Full scan with arm + optional chassis rotation."""
        # Arm sweep first
        readings = self.arm_scanner.full_sweep()
        best_angle = self.arm_scanner.find_best_opening(readings)

        if best_angle is not None:
            return EscapeResult(True, best_angle, f"Arm scan found opening at {best_angle}°")

        # No opening in front - need chassis rotation for rear check
        # This will be handled by caller using lidar during rotation
        return EscapeResult(False, None, "No opening found in front hemisphere")
```

### arm_scanner.py

```python
from dataclasses import dataclass
from typing import Callable, List, Optional
import time

@dataclass
class DepthReading:
    angle: float          # arm angle (degrees from center)
    min_depth: float      # closest point (meters)
    avg_depth: float      # average depth (meters)
    clear_width: float    # estimated clear width (meters)

class ArmScanner:
    # Servo 1 positions: 200=right, 500=center, 800=left
    SERVO_RIGHT = 200
    SERVO_CENTER = 500
    SERVO_LEFT = 800

    def __init__(
        self,
        arm_move_func: Callable[[int, int], bool],  # servo_id, position
        get_depth_func: Callable[[], Optional[dict]],  # returns depth data
        settle_time: float = 0.3,
    ):
        self.arm_move = arm_move_func
        self.get_depth = get_depth_func
        self.settle_time = settle_time

    def quick_glance(self, direction: str) -> float:
        """Quick look left or right, return average depth."""
        target = self.SERVO_LEFT if direction == "left" else self.SERVO_RIGHT

        self.arm_move(1, target)
        time.sleep(self.settle_time)

        depth_data = self.get_depth()
        self.arm_move(1, self.SERVO_CENTER)  # Return to center

        if depth_data and depth_data.get("avg_depth"):
            return depth_data["avg_depth"]
        return 0.0  # No data = blocked

    def full_sweep(self) -> List[DepthReading]:
        """Sweep arm across 180° field, collecting depth readings."""
        readings = []

        # Positions to sample: right to left
        positions = [200, 300, 400, 500, 600, 700, 800]
        angles = [-60, -40, -20, 0, 20, 40, 60]  # Approximate degrees from center

        for pos, angle in zip(positions, angles):
            self.arm_move(1, pos)
            time.sleep(self.settle_time)

            depth_data = self.get_depth()
            if depth_data:
                readings.append(DepthReading(
                    angle=angle,
                    min_depth=depth_data.get("min_depth", 0),
                    avg_depth=depth_data.get("avg_depth", 0),
                    clear_width=depth_data.get("clear_width", 0),
                ))

        # Return to center
        self.arm_move(1, self.SERVO_CENTER)
        return readings

    def find_best_opening(self, readings: List[DepthReading]) -> Optional[float]:
        """Find the angle with the most open space."""
        if not readings:
            return None

        # Find reading with greatest average depth (most open)
        best = max(readings, key=lambda r: r.avg_depth)

        # Only return if there's meaningful clearance (> 1m)
        if best.avg_depth > 1.0:
            return best.angle
        return None
```

### Changes to explorer.py

```python
# Add to ExplorationController.__init__:
self.escape_handler = EscapeHandler(move_func, stop_func, arm_scanner)
self.turn_history: list[tuple[str, float]] = []
self.forward_blocked_count: int = 0

# Add method:
def _record_turn(self, direction: str) -> None:
    """Record a turn direction for oscillation detection."""
    now = time.time()
    self.turn_history.append((direction, now))
    # Keep only last 10 seconds
    self.turn_history = [(d, t) for d, t in self.turn_history if now - t < 10.0]

def _is_oscillating(self) -> bool:
    """Detect oscillation pattern in recent turns."""
    if len(self.turn_history) < 4:
        return False

    # Check for alternating pattern in last 6 turns
    recent = [d for d, _ in self.turn_history[-6:]]
    alternations = sum(1 for i in range(1, len(recent)) if recent[i] != recent[i-1])

    return alternations >= 3

# Modify step() to check for escape:
def step(self) -> bool:
    # ... existing safety checks ...

    # Check for escape condition
    if self._is_oscillating() or self.forward_blocked_count >= 3:
        if self.escape_handler.check_should_escape(
            self._is_oscillating(),
            self.forward_blocked_count
        ):
            result = self.escape_handler.execute_escape()
            if result.success and result.direction:
                # Execute the escape turn
                self._execute_turn(result.direction)
                self.escape_handler.reset()
                self.forward_blocked_count = 0
                self.turn_history.clear()
            elif not result.success:
                self.speak(result.message)
            return True

    # ... rest of existing step logic ...
```

## Flow Diagram

```
Normal Exploration
       │
       ▼
┌─────────────────────┐
│ Oscillation detected │──No──► Continue exploring
│ OR blocked 3x?       │
└─────────────────────┘
       │ Yes
       ▼
┌─────────────────────┐
│ Level 1: Wide Turn   │──Success──► Resume (cooldown 15s)
│ (90° + arm glance)   │
└─────────────────────┘
       │ Fail (still stuck after 3s)
       ▼
┌─────────────────────┐
│ Level 2: Reverse 180°│──Success──► Resume (cooldown 15s)
│ (backup + turn)      │
└─────────────────────┘
       │ Fail (still stuck after 3s)
       ▼
┌─────────────────────┐
│ Level 3: Full Scan   │──Success──► Resume (cooldown 15s)
│ (arm sweep + lidar)  │
└─────────────────────┘
       │ Fail
       ▼
   "I'm trapped"
   (wait for help)
```

## Expected Behavior Improvements

| Metric | Before | After |
|--------|--------|-------|
| Stuck detection time | 10 seconds | 3-5 seconds |
| Simple corner escape | ~12 seconds | ~2 seconds (Level 1) |
| Complex trap escape | Often fails | ~10 seconds (Level 3) |
| Uses depth camera | Passive only | Active scanning |
| Oscillation handling | None | Detected + prevented |

## Testing Plan

1. **Unit tests:** Oscillation detection, escape level logic
2. **Simulation:** Test with mocked sensor data
3. **Real robot tests:**
   - Corner trap (Level 1 should suffice)
   - Dead-end corridor (Level 2 needed)
   - Surrounded by obstacles (Level 3 needed)
   - Confirm arm sweep works with depth camera

## Implementation Status

### Completed

1. **Progressive escape system** (`validation/exploration/escape_handler.py`)
   - 3-level escalation: Wide Turn → Reverse 180 → Full Scan → Trapped
   - Cooldown between escapes to prevent thrashing

2. **Oscillation detection** (in `explorer.py`)
   - Turn history tracking with 10-second window
   - Triggers on 3+ alternating turns OR 3+ blocked attempts

3. **Arm scanner integration** (`validation/exploration/arm_scanner.py`)
   - Servo 1 pan control for camera scanning
   - Quick glance and full sweep modes
   - Scan pose verified for horizontal camera orientation

4. **Parameter tuning**
   - Forward speed: 0.35 m/s (was 0.2 m/s)
   - Decision rate: 4 Hz (was 2 Hz)
   - Stop distance: 0.15 m (was 0.20 m)
   - Slow distance: 0.30 m (was 0.50 m)
   - Blocked distance: 0.40 m (was 0.80 m)

5. **On-robot architecture** (replaces SSH-based approach)
   - `robot_explorer.py` runs on Pi for 8Hz sensor reads
   - `deploy_explorer.py` for deployment and control
   - Local `docker exec` instead of SSH for sensor access

### Files Created/Modified

**New files:**
- `validation/exploration/escape_handler.py` - Progressive escape state machine
- `validation/exploration/arm_scanner.py` - Arm-mounted camera scanning
- `robot_explorer.py` - On-robot exploration script
- `deploy_explorer.py` - Deployment and control tool

**Modified files:**
- `validation/exploration/explorer.py` - Integrated escape handler, oscillation detection
- `validation/exploration/sensor_fusion.py` - Updated distance thresholds
- `validation/exploration/frontier_planner.py` - Updated blocked threshold
- `CLAUDE.md` - Updated exploration commands
- `.claude/skills/landerpi-lidar/SKILL.md` - Updated with new architecture
- `.claude/agents/landerpi-robot.md` - Updated skill loading matrix
