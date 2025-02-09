#!/bin/bash

# Define the udev rule content
UDEV_RULE='SUBSYSTEM=="tty*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", SYMLINK+="hello-esp"'

# File path for the udev rule
RULE_FILE="/etc/udev/rules.d/99-stretch_custom.rules"

# Create the udev rule file and write the rule
echo "Creating udev rule file at $RULE_FILE..."
echo "$UDEV_RULE" > "$RULE_FILE"

# Reload udev rules and trigger changes
echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

# Check if the symlink exists
echo "Checking if /dev/hello-esp exists..."
if [ -e /dev/hello-esp ]; then
    echo "Success: /dev/hello-esp is available."
else
    echo "Warning: /dev/hello-esp is not available. Ensure your ESP32 is connected."
fi
