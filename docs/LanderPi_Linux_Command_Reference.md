# LanderPi Robot - Linux Command Reference Guide

Based on analysis of the LanderPi Linux tutorials, this guide provides essential commands and concepts for setting up and managing your LanderPi robot.

## Important Linux Directory Structure

### Essential System Directories

1. **Root Directory (/)**
   - The top-level directory of the entire Linux file system
   - All other directories branch from here

2. **/home**
   - User home directories
   - Your personal files and configurations
   - Example: `/home/pi` for the default Raspberry Pi user

3. **/etc**
   - System configuration files
   - Network settings, service configurations
   - Critical for robot setup

4. **/usr**
   - User programs and applications
   - `/usr/bin` - User executable programs
   - `/usr/local` - Locally installed software

5. **/var**
   - Variable data files
   - `/var/log` - System and application logs (important for debugging)
   - `/var/tmp` - Temporary files

6. **/tmp**
   - Temporary files (cleared on reboot)
   - Used for temporary storage during operations

7. **/opt**
   - Optional software packages
   - Often used for third-party applications

8. **/dev**
   - Device files
   - Hardware devices appear as files here
   - Important for accessing robot sensors and actuators

## Essential Linux Commands

### File and Directory Operations

```bash
# List files and directories
ls                  # Basic listing
ls -l               # Detailed listing with permissions
ls -la              # Include hidden files (starting with .)
ls -lh              # Human-readable file sizes

# Change directory
cd /path/to/dir     # Navigate to specific directory
cd ~                # Go to home directory
cd ..               # Go up one directory level
cd -                # Return to previous directory

# Print working directory
pwd                 # Show current directory path

# Create directory
mkdir directory_name              # Create single directory
mkdir -p path/to/nested/dir      # Create nested directories

# Remove files/directories
rm filename                       # Delete file
rm -r directory                   # Delete directory and contents
rm -rf directory                  # Force delete (use carefully!)

# Copy files/directories
cp source destination             # Copy file
cp -r source_dir dest_dir        # Copy directory recursively

# Move/rename files
mv source destination             # Move or rename file/directory

# View file contents
cat filename                      # Display entire file
less filename                     # View file page by page (q to quit)
head filename                     # Show first 10 lines
head -n 20 filename              # Show first 20 lines
tail filename                     # Show last 10 lines
tail -f filename                 # Follow file updates (useful for logs)

# Search for files
find /path -name "filename"      # Find file by name
find . -type f -name "*.py"      # Find Python files in current directory
```

### System Information Commands

```bash
# System information
uname -a            # Display all system information
hostname            # Show system hostname
uptime              # Show system uptime
df -h               # Disk space usage (human-readable)
free -h             # Memory usage
top                 # Real-time system monitor (q to quit)
htop                # Enhanced system monitor (if installed)

# Process management
ps aux              # List all running processes
ps aux | grep python   # Find Python processes
kill PID            # Terminate process by ID
killall process_name   # Kill all instances of a process

# Network information
ifconfig            # Network interface configuration
ip addr             # Modern alternative to ifconfig
ping hostname       # Test network connectivity
```

### Package Management (Raspberry Pi/Debian)

```bash
# Update package lists
sudo apt-get update

# Upgrade installed packages
sudo apt-get upgrade

# Install new package
sudo apt-get install package_name

# Remove package
sudo apt-get remove package_name

# Search for packages
apt-cache search keyword

# Clean up
sudo apt-get autoremove    # Remove unnecessary packages
sudo apt-get clean         # Clear package cache
```

### Text Editing

```bash
# Nano (beginner-friendly)
nano filename       # Edit file with nano

# Vim (advanced)
vim filename        # Edit file with vim
# Press 'i' for insert mode, 'Esc' then ':wq' to save and quit

# Create/edit with redirect
echo "content" > file.txt      # Overwrite file
echo "content" >> file.txt     # Append to file
```

## Linux Permissions Management

### Understanding Permissions

Linux uses a permission system with three levels:
- **User (u)**: File owner
- **Group (g)**: Group members
- **Others (o)**: Everyone else

Three types of permissions:
- **Read (r)**: Permission to read file (value: 4)
- **Write (w)**: Permission to modify file (value: 2)
- **Execute (x)**: Permission to run file (value: 1)

### Viewing Permissions

```bash
ls -l filename

# Example output:
# -rwxr-xr-- 1 pi pi 1234 Dec 22 10:00 script.py
# |  |  |
# |  |  └── Others permissions (r--)
# |  └───── Group permissions (r-x)
# └──────── User permissions (rwx)
```

### Changing Permissions

```bash
# Symbolic method
chmod u+x filename          # Add execute permission for user
chmod g+w filename          # Add write permission for group
chmod o-r filename          # Remove read permission for others
chmod a+x filename          # Add execute for all (user, group, others)

# Numeric method (more common)
chmod 755 filename          # rwxr-xr-x (owner: all, group/others: read+execute)
chmod 644 filename          # rw-r--r-- (owner: read+write, others: read only)
chmod 777 filename          # rwxrwxrwx (all permissions for everyone - use carefully!)
chmod 700 filename          # rwx------ (only owner has access)

# Common permission values:
# 755 - Standard for executable scripts
# 644 - Standard for regular files
# 700 - Private executable (only owner)
# 600 - Private file (only owner can read/write)
```

### Changing Ownership

```bash
# Change file owner
sudo chown username filename

# Change file owner and group
sudo chown username:groupname filename

# Change ownership recursively
sudo chown -R username:groupname directory/

# Change only group
sudo chgrp groupname filename
```

## Robot-Specific Commands

### Service Management (for robot control software)

```bash
# SystemD service control
sudo systemctl start service_name     # Start service
sudo systemctl stop service_name      # Stop service
sudo systemctl restart service_name   # Restart service
sudo systemctl status service_name    # Check service status
sudo systemctl enable service_name    # Enable on boot
sudo systemctl disable service_name   # Disable on boot

# View service logs
journalctl -u service_name           # View service logs
journalctl -u service_name -f        # Follow service logs
```

### GPIO and Hardware Access

```bash
# Common locations for device access
ls /dev/tty*        # List serial devices
ls /dev/video*      # List video devices (cameras)
ls /dev/i2c*        # List I2C devices (sensors)

# Check USB devices
lsusb               # List USB devices
dmesg | tail        # Check recent kernel messages (device connections)
```

### Python Environment

```bash
# Python version check
python --version
python3 --version

# Install Python packages
pip3 install package_name
sudo pip3 install package_name        # System-wide install

# List installed packages
pip3 list

# Run Python scripts
python3 script.py
```

### Network Configuration

```bash
# Check network interfaces
ifconfig
ip addr show

# Check network connectivity
ping 8.8.8.8                         # Ping Google DNS
ping -c 4 hostname                   # Send 4 pings only

# View network routes
route -n
ip route show

# Configure static IP (edit network configuration)
sudo nano /etc/dhcpcd.conf

# Restart networking
sudo systemctl restart networking
```

## Important Commands for Robot Setup

### SSH Access

```bash
# Connect to robot remotely
ssh pi@robot_ip_address

# Copy files to robot
scp file.py pi@robot_ip:/home/pi/

# Copy files from robot
scp pi@robot_ip:/home/pi/file.py ./
```

### System Monitoring

```bash
# Monitor CPU temperature (Raspberry Pi)
vcgencmd measure_temp

# Monitor system resources
watch -n 1 'vcgencmd measure_temp && free -h && df -h'

# Check system logs
sudo tail -f /var/log/syslog
```

### Backup and Safety

```bash
# Create backup of important files
tar -czf backup_$(date +%Y%m%d).tar.gz /path/to/important/files

# Extract backup
tar -xzf backup_file.tar.gz

# Make script executable
chmod +x script.sh

# Create symbolic link
ln -s /path/to/original /path/to/link
```

## Best Practices for LanderPi Setup

1. **Always use sudo for system-level changes**
   - Editing config files in /etc
   - Installing packages
   - Changing system services

2. **Set proper permissions for scripts**
   - Executable scripts: `chmod 755`
   - Configuration files: `chmod 644`
   - Private keys/credentials: `chmod 600`

3. **Keep system updated**
   ```bash
   sudo apt-get update && sudo apt-get upgrade -y
   ```

4. **Monitor logs for debugging**
   ```bash
   tail -f /var/log/syslog
   journalctl -f
   ```

5. **Use virtual environments for Python projects**
   ```bash
   python3 -m venv robot_env
   source robot_env/bin/activate
   ```

6. **Back up important configurations before changes**
   ```bash
   sudo cp /etc/config.file /etc/config.file.backup
   ```

## Quick Reference: Permission Numbers

| Number | Permission | Description |
|--------|-----------|-------------|
| 0 | --- | No permissions |
| 1 | --x | Execute only |
| 2 | -w- | Write only |
| 3 | -wx | Write and execute |
| 4 | r-- | Read only |
| 5 | r-x | Read and execute |
| 6 | rw- | Read and write |
| 7 | rwx | All permissions |

## Common File Locations for Robot Setup

- Configuration files: `/etc/` or `/home/pi/.config/`
- User scripts: `/home/pi/scripts/`
- Log files: `/var/log/` or `/home/pi/logs/`
- Service files: `/etc/systemd/system/`
- Device access: `/dev/`
- Temporary files: `/tmp/` or current directory

## Troubleshooting Commands

```bash
# Check if service is running
sudo systemctl status service_name

# View recent system messages
dmesg | tail -50

# Check disk space
df -h

# Check memory usage
free -h

# View active network connections
netstat -tuln
ss -tuln        # Modern alternative

# Find large files
du -h --max-depth=1 | sort -hr | head -10

# Check which process is using a port
sudo lsof -i :8080

# Test Python script syntax
python3 -m py_compile script.py
```

---

This reference guide is compiled from the LanderPi Linux tutorials (Lessons 5, 6, and 7) and provides the essential commands needed for setting up and managing your robot system.
