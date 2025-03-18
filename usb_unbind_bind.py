# Code for reset USB-ports for USB-contoroller and ESP32's.

import time

def write_to_sysfs(path, value):
    try:
        with open(path, 'w') as f:
            f.write(value)
    except PermissionError:
        print(f"Permission denied: Unable to write to {path}. Try running as root.")
    except FileNotFoundError:
        print(f"File not found: {path}. Ensure the correct USB port identifier.")

def bind_usb(port):
    write_to_sysfs('/sys/bus/usb/drivers/usb/bind', port)

def unbind_usb(port):
    write_to_sysfs('/sys/bus/usb/drivers/usb/unbind', port)

if __name__ == "__main__":
    port_1 = "1-2.3"         # USB-controller port
    port_2 = "1-2.1"         # Pedal ESP32 port
    port_3 = "1-2.2"         # Steering ESP32 port
    
    unbind_usb(port_1)
    time.sleep(1)  # Uncomment to enable delay
    unbind_usb(port_2)
    time.sleep(1)
    unbind_usb(port_3)
    time.sleep(1)
    bind_usb(port_1)
    time.sleep(1)
    bind_usb(port_2)
    time.sleep(1)
    bind_usb(port_3)
    


