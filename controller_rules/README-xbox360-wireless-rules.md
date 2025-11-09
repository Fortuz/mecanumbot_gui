# Xbox 360 Wireless Adapter udev Rules

This repository contains udev rules for the Microsoft Xbox 360 Wireless Adapter (ID 045e:0719) to ensure proper permissions and access on Linux systems.

## Overview

The Xbox 360 Wireless Adapter allows you to connect Xbox 360 wireless controllers to your Linux system. Without proper udev rules, the device may not be accessible to user-space applications or may require root privileges to function correctly.

## Device Information

- **Device**: Microsoft Corp. Xbox 360 Wireless Adapter
- **Vendor ID**: 045e (Microsoft)
- **Product ID**: 0719
- **Bus Type**: USB

## Files Included

- `99-xbox360-wireless.rules` - The udev rules file

## Installation

### Prerequisites

- Linux system with udev support
- Root/sudo access for installation
- User account in the `plugdev` group

### Step-by-Step Installation

1. **Copy the rules file to the udev rules directory:**
   ```bash
   sudo cp 99-xbox360-wireless.rules /etc/udev/rules.d/
   ```

2. **Set appropriate permissions:**
   ```bash
   sudo chmod 644 /etc/udev/rules.d/99-xbox360-wireless.rules
   ```

3. **Reload udev rules:**
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

4. **Add your user to the plugdev group (if not already a member):**
   ```bash
   sudo usermod -a -G plugdev $USER
   ```

5. **Log out and log back in** for group changes to take effect.

## What the Rules Do

The udev rules file contains several rules that:

1. **Grant user access** to the Xbox 360 Wireless Adapter USB device
2. **Set proper permissions** (0666 - read/write for all users)
3. **Assign device ownership** to the `plugdev` group
4. **Enable systemd integration** with the `uaccess` tag
5. **Cover related device nodes** like `hidraw` and `input` devices

## Verification

To verify the rules are working correctly:

1. **Check if the device is detected:**
   ```bash
   lsusb | grep 045e:0719
   ```

2. **Verify udev rule application:**
   ```bash
   udevadm info -a -p $(udevadm info -q path -n /dev/bus/usb/001/003)
   ```
   *(Replace the device path with your actual device path)*

3. **Check device permissions:**
   ```bash
   ls -la /dev/bus/usb/001/003
   ```
   *(The device should show permissions like `crw-rw-rw-`)*

## Troubleshooting

### Common Issues

1. **Permission Denied Errors:**
   - Ensure you're in the `plugdev` group: `groups $USER`
   - Log out and back in after adding to the group
   - Verify the rules file is in the correct location

2. **Rules Not Applied:**
   - Check for syntax errors in the rules file
   - Reload udev rules manually
   - Restart the udev service: `sudo systemctl restart systemd-udevd`

3. **Device Not Detected:**
   - Verify the device is connected: `lsusb`
   - Check dmesg for USB-related messages: `dmesg | grep -i usb`
   - Try a different USB port

### Debug Commands

```bash
# Monitor udev events
sudo udevadm monitor --environment --udev

# Test rule matching
udevadm test $(udevadm info -q path -n /dev/bus/usb/001/003)

# Check systemd journal for udev messages
journalctl -u systemd-udevd -f
```

## Compatibility

These rules have been tested with:
- Ubuntu 18.04+
- Debian 10+
- Fedora 30+
- Arch Linux
- Other systemd-based distributions

## Xbox 360 Controller Support

After installing these rules, you may also want to install the Xbox 360 controller driver:

```bash
# Ubuntu/Debian
sudo apt install xboxdrv

# Fedora
sudo dnf install xboxdrv

# Arch Linux
sudo pacman -S xboxdrv
```

## Security Considerations

The rules grant read/write access (0666) to all users for the Xbox 360 Wireless Adapter. This is generally safe for gaming peripherals but consider your security requirements:

- For more restrictive access, change `MODE="0666"` to `MODE="0664"`
- Create a dedicated group instead of using `plugdev` if needed
- The `uaccess` tag ensures only locally logged-in users can access the device

## Contributing

If you encounter issues or have improvements:

1. Check the device ID matches your adapter: `lsusb`
2. Test the rules on your system
3. Submit issues with system information and error messages

## License

This project is released under the same license as the parent repository.

## References

- [Writing udev rules](http://www.reactivated.net/writing_udev_rules.html)
- [systemd udev documentation](https://www.freedesktop.org/software/systemd/man/udev.html)
- [Xbox 360 Controller on Linux](https://wiki.archlinux.org/title/Gamepad#Xbox_360_controller)