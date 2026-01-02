# Hailo 8 Troubleshooting Guide

## Common Issues

### Device not found on PCIe

**Symptoms:**
```
lspci | grep -i hailo
(no output)
```

**Solutions:**
1. Check physical connection of Hailo module
2. Verify PCIe is enabled in `/boot/firmware/config.txt`:
   ```
   dtparam=pciex1
   ```
3. Reboot after config changes

### /dev/hailo0 not found

**Symptoms:**
```
ls /dev/hailo*
ls: cannot access '/dev/hailo*': No such file or directory
```

**Solutions:**
1. Load kernel module manually:
   ```bash
   sudo modprobe hailo_pci
   ```
2. Check DKMS status:
   ```bash
   dkms status
   ```
3. Rebuild DKMS module:
   ```bash
   sudo dkms install hailort/$(hailortcli --version | grep -oP '\d+\.\d+\.\d+')
   ```
4. Reboot

### Permission denied

**Symptoms:**
```
hailortcli scan
Error: Permission denied
```

**Solutions:**
1. Add user to hailo group:
   ```bash
   sudo usermod -aG hailo $USER
   ```
2. Logout and login
3. Or use sudo: `sudo hailortcli scan`

### Python import fails

**Symptoms:**
```python
>>> import hailo_platform
ModuleNotFoundError: No module named 'hailo_platform'
```

**Solutions:**
1. Install Python bindings:
   ```bash
   pip install hailort --break-system-packages
   ```
2. Check Python version (requires 3.8+)

### HEF model fails to load

**Symptoms:**
```
HEF validation failed
```

**Solutions:**
1. Verify HEF was compiled for Hailo-8 (not Hailo-8L or Hailo-15)
2. Check HEF file integrity:
   ```bash
   hailortcli parse-hef model.hef
   ```
3. Re-convert model with correct target

### Inference is slow

**Symptoms:**
- FPS lower than expected
- High latency

**Solutions:**
1. Check thermal throttling:
   ```bash
   hailortcli fw-control identify | grep -i temp
   ```
2. Ensure async pipeline is used (not sync)
3. Check input preprocessing isn't bottleneck

### Out of memory during inference

**Symptoms:**
```
HailoRTStatusException: HAILO_OUT_OF_HOST_MEMORY
```

**Solutions:**
1. Reduce batch size to 1
2. Use streaming inference
3. Free system memory

### VDevice creation fails

**Symptoms:**
```python
VDevice() raises exception
```

**Solutions:**
1. Check device permissions: `ls -la /dev/hailo0`
2. Only one VDevice per process - close previous instances
3. Verify no other process is using device: `lsof /dev/hailo0`

### Model mismatch

**Symptoms:**
```
Input shape mismatch
```

**Solutions:**
1. Verify image preprocessing matches model expectations:
   - Input size: 640x640 for YOLOv11n
   - Color format: RGB normalized to 0-1
2. Check HEF input specifications:
   ```bash
   hailortcli parse-hef model.hef | grep input
   ```

## Diagnostic Commands

```bash
# Full system check
uv run python deploy_hailo8.py check

# Device status
hailortcli scan
hailortcli fw-control identify

# Kernel module status
lsmod | grep hailo
dmesg | grep -i hailo

# PCIe link status
lspci -vv | grep -A20 Hailo

# Benchmark model
hailortcli benchmark model.hef
```

## Getting Help

1. Check Hailo documentation: https://hailo.ai/developer-zone/documentation/
2. Hailo community forum: https://community.hailo.ai/
3. GitHub issues on this project
