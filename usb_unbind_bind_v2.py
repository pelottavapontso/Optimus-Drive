import os
import time

def write_to_sysfs(path, value):
    """Write a value to a sysfs file."""
    try:
        with open(path, 'w') as f:
            f.write(value)
    except PermissionError:
        print(f"Permission denied: Unable to write to {path}. Try running as root.")
    except FileNotFoundError:
        print(f"File not found: {path}. Ensure the USB port identifier is correct.")

def unbind_usb(port):
    """Unbind (disconnect) the USB device on a specific port."""
    print(f"Unbinding USB device on port {port}...")
    write_to_sysfs('/sys/bus/usb/drivers/usb/unbind', port)

def bind_usb(port):
    """Bind (reconnect) the USB device on a specific port."""
    print(f"Binding USB device on port {port}...")
    write_to_sysfs('/sys/bus/usb/drivers/usb/bind', port)

def power_off_usb(bus, port):
    """Turn off USB power (if supported)."""
    power_path = f"/sys/bus/usb/devices/{bus}-{port}/power/control"
    print(f"Turning off power for {bus}-{port}...")
    write_to_sysfs(power_path, "auto")  # Some systems require "auto" to turn off power

def power_on_usb(bus, port):
    """Turn on USB power (if supported)."""
    power_path = f"/sys/bus/usb/devices/{bus}-{port}/power/control"
    print(f"Turning on power for {bus}-{port}...")
    write_to_sysfs(power_path, "on")

if __name__ == "__main__":
    # Define the USB ports to reset
    ports = ["1-2.1", "1-2.2", "1-2.3"]
    bus = "1"  # Bus number (adjust based on your system)

    # Step 1: Unbind all devices from these ports
    for port in ports:
        unbind_usb(port)

    # Step 2: Turn off power (if supported)
    for port in ports:
        power_off_usb(bus, port)

    time.sleep(2)  # Wait for power-down to take effect

    # Step 3: Turn power back on
    for port in ports:
        power_on_usb(bus, port)

    time.sleep(1)  # Short delay before rebinding

    # Step 4: Bind the devices back
    for port in ports:
        bind_usb(port)

    print("USB full reset (unbind, power cycle, rebind) complete.")
