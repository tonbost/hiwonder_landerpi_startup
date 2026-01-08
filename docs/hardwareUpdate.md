To move your Raspberry Pi setup from an SD card to an NVMe drive using rpi-clone, you generally do not need to flash the NVMe drive beforehand. The cloning process creates the partitions and copies the filesystem for you.

Here is the step-by-step guide to getting this done.

1. Preparation & Installation
First, ensure your NVMe drive is connected via your PCIe hat and that rpi-clone is installed.

Bash
# Install rpi-clone if you haven't already
git clone https://github.com/billw2/rpi-clone.git
cd rpi-clone
sudo cp rpi-clone rpi-clone-setup /usr/local/sbin
2. The Cloning Command
You do not need to format the NVMe drive first. rpi-clone will initialize the destination disk.

Identify your NVMe drive: Usually, it is /dev/nvme0n1. You can confirm this by running lsblk.

Run the clone command:

Bash
sudo rpi-clone nvme0n1
What happens: It will ask for confirmation, partition the NVMe, and sync your data.

Note: If you are using a brand new NVMe, it might ask if you want to initialize it; type yes.

3. Enable PCIe Gen 3
By default, the Raspberry Pi 5 uses PCIe Gen 2. To get the full speed of your NVMe, you need to force Gen 3 in your boot configuration.

Open the config file:

Bash
sudo nano /boot/firmware/config.txt
Add the following lines to the bottom of the file:

Plaintext
# Enable PCIe Gen 3 speeds
dtparam=pciex1_gen=3
Save and exit (Ctrl+O, Enter, Ctrl+X).


To update the bootloader configuration directly via the command line (without using the interactive raspi-config menus), you use the rpi-eeprom-config tool.

1. Update the Bootloader Firmware
Before editing the configuration, ensure your bootloader firmware is the latest version.

Bash
# Check if updates are available
sudo rpi-eeprom-update

# Apply all available updates
sudo rpi-eeprom-update -a

# Reboot to apply firmware changes
sudo reboot
2. Edit the Bootloader Config (Manual)
The following command opens the current bootloader configuration in your default text editor (usually nano).

Bash
sudo rpi-eeprom-config --edit
Once the editor opens, find or add these lines to prioritize NVMe:

BOOT_ORDER=0xf416: This tells the Pi to try: NVMe (6), then SD (1), then USB (4).

PCIE_PROBE=1: Essential for Raspberry Pi 5 if you are using a non-official NVMe hat to ensure the Pi scans the PCIe bus at startup.


3. Non-Interactive Scripting (Automated)If you want to update the config via a script without opening an editor, you can pipe the configuration change directly.Bash# 1. Extract current config to a file
vcgencmd bootloader_config > bootconf.txt

# 2. Modify the file (e.g., using sed to change BOOT_ORDER)
sed -i 's/^BOOT_ORDER=.*/BOOT_ORDER=0xf416/' bootconf.txt

# 3. Add PCIE_PROBE if it doesn't exist
if ! grep -q "PCIE_PROBE" bootconf.txt; then
    echo "PCIE_PROBE=1" >> bootconf.txt
fi

# 4. Apply the new config and reboot
sudo rpi-eeprom-config --apply bootconf.txt
sudo reboot
Understanding the BOOT_ORDER CodeThe BOOT_ORDER is read right-to-left.2 Each digit represents a boot device:| Digit | Device || :--- | :--- || 1 | SD Card || 4 | USB Mass Storage |3| 6 | NVMe (Raspberry Pi 5) |4| f | Restart/Loop |5+3A value of 0xf416 means: 6 (NVMe) $\rightarrow$ 1 (SD) $\rightarrow$ 4 (USB) $\rightarrow$ f (Loop).