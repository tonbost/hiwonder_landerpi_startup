# LanderPi Troubleshooting Guide

## Connection Issues

### SSH Connection Refused

**Symptoms:**
- `Connection refused` error
- Port 22 not responding

**Checks:**
```bash
# From local machine
ping <robot_ip>
nc -zv <robot_ip> 22
```

**Solutions:**
1. Verify robot is powered on and booted
2. Check if SSH service is running: `systemctl status ssh`
3. Enable SSH if disabled: `sudo systemctl enable --now ssh`
4. Check firewall: `sudo ufw status`

### SSH Permission Denied

**Symptoms:**
- `Permission denied (publickey,password)`

**Solutions:**
1. Verify password is correct
2. Check if password auth is enabled in `/etc/ssh/sshd_config`:
   ```
   PasswordAuthentication yes
   ```
3. Restart SSH: `sudo systemctl restart ssh`

### Network Unreachable

**Symptoms:**
- `No route to host`
- Cannot ping robot

**Solutions:**
1. Verify robot and computer on same network
2. Check robot's IP: Connect HDMI, run `ip addr`
3. Check for IP conflict
4. Verify router/switch connectivity

## ROS2 Issues

### ros2 Command Not Found

**Cause:** ROS2 not installed or not sourced

**Solutions:**
1. Source ROS2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Add to ~/.bashrc for persistence
3. If ROS2 not installed, flash HiWonder image

### Package Not Found

**Symptoms:**
- `Package 'ros_robot_controller' not found`

**Solutions:**
1. Check workspace exists: `ls ~/ros2_ws`
2. Source workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
3. Rebuild if needed:
   ```bash
   cd ~/ros2_ws && colcon build
   ```

### Topic Not Publishing

**Symptoms:**
- `ros2 topic echo` shows no data

**Diagnosis:**
```bash
ros2 topic list
ros2 topic info <topic_name>
ros2 node list
```

**Solutions:**
1. Start required node/launch file
2. Check node is running: `ros2 node list`
3. Check for errors in node output

## Hardware Issues

### Serial Port Access Denied

**Symptoms:**
- `Permission denied: '/dev/ttyUSB0'`
- `Permission denied: '/dev/ttyACM0'`

**Solutions:**
```bash
# Add user to dialout group
sudo usermod -aG dialout $USER

# Apply immediately (or logout/login)
newgrp dialout

# Verify
groups | grep dialout
```

### I2C Device Not Found

**Symptoms:**
- `ls /dev/i2c*` shows no devices
- `i2cdetect` fails

**Solutions:**
1. Enable I2C in boot config:
   ```bash
   sudo nano /boot/firmware/config.txt
   # Add: dtparam=i2c_arm=on
   ```
2. Reboot
3. Load module: `sudo modprobe i2c-dev`

### vcgencmd Not Working

**Symptoms:**
- `VCHI initialization failed`
- Cannot read temperature

**Solutions:**
```bash
# Create device
sudo mknod /dev/vcio c 100 0
sudo chmod 666 /dev/vcio

# Make persistent
echo 'KERNEL=="vcio", MODE="0666"' | sudo tee /etc/udev/rules.d/99-vcio.rules
sudo udevadm control --reload-rules
```

### USB Camera Not Detected

**Symptoms:**
- Camera not in `lsusb` output
- `/dev/video*` not created

**Solutions:**
1. Check USB connection
2. Try different USB port
3. Check dmesg for errors: `dmesg | tail -20`
4. Install udev rules for camera

## Docker Issues

### Docker Permission Denied

**Symptoms:**
- `Got permission denied while trying to connect to Docker daemon`

**Solutions:**
```bash
sudo usermod -aG docker $USER
newgrp docker
```

### Docker Image Not Found

**Symptoms:**
- `Unable to find image 'landerpi-ros2:latest' locally`

**Solutions:**
```bash
# Build the image
cd ~/landerpi_ros
docker build -t landerpi-ros2 .

# Or pull base image
docker pull ros:humble-ros-base
```

### Container Exits Immediately

**Symptoms:**
- Container starts then stops
- `docker ps` shows nothing

**Diagnosis:**
```bash
docker logs <container_id>
docker ps -a  # Show stopped containers
```

## Motion Control Issues

### Robot Not Moving

**Checklist:**
1. [ ] ROS2 controller running?
2. [ ] Publishing to correct topic?
3. [ ] Velocity within limits?
4. [ ] STM32 controller connected?
5. [ ] Battery charged?

**Diagnosis:**
```bash
# Check topic is receiving commands
ros2 topic echo /ros_robot_controller/cmd_vel

# Check controller node
ros2 node list | grep robot

# Check serial connection
ls /dev/ttyACM*
```

### Robot Veers Off Course

**Cause:** Calibration needed

**Solution:**
1. Run linear velocity calibration
2. Run angular velocity calibration
3. Update `calibrate_params.yaml`

See calibration section in tutorials.

### Motors Making Noise But Not Moving

**Possible causes:**
1. Mechanical obstruction
2. Wheels not properly attached
3. Motor driver issue
4. Excessive load

**Solutions:**
1. Check for debris/obstructions
2. Verify wheel attachment
3. Check motor connections
4. Reduce payload

## System Issues

### Wrong Ubuntu Version

**Symptoms:**
- Ubuntu 25.10 instead of 22.04/24.04
- ROS2 packages incompatible

**Solution:**
Flash the official HiWonder system image which includes:
- Correct Ubuntu version
- Pre-installed ROS2
- Pre-configured ros2_ws
- All drivers and dependencies

### Out of Disk Space

**Diagnosis:**
```bash
df -h
du -sh ~/* | sort -h
```

**Solutions:**
```bash
# Clean apt cache
sudo apt clean

# Remove old Docker images
docker system prune -a

# Clean ROS2 build artifacts
rm -rf ~/ros2_ws/build ~/ros2_ws/log
```

### High CPU/Memory Usage

**Diagnosis:**
```bash
htop
# or
top -o %CPU
```

**Common culprits:**
- Multiple ROS2 nodes
- Camera processing
- SLAM/Navigation

**Solutions:**
1. Stop unnecessary nodes
2. Reduce camera resolution
3. Limit SLAM update rate
