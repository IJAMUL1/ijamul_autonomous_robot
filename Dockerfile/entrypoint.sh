#!/bin/bash

set -e

# Change to the workspace directory
cd /ijamul_ws/

# Change permissions for the serial device (adjust the device path as needed)
if [ -e /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 ]; then
    sudo chmod 777 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
else
    echo "Serial device not found"
fi

# Run i2cdetect and change permissions for the I2C device
if command -v i2cdetect &> /dev/null; then
    sudo i2cdetect -r -y 1
    sudo chmod 666 /dev/i2c-1
else
    echo "i2cdetect command not found"
fi

echo "provided argument: $@"

# Execute the provided command
exec "$@"