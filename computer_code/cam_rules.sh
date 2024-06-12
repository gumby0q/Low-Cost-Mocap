#!/bin/bash

# User to be added to the groups
USERNAME="anatolii" # Replace your_username with the actual username

# Add user to the necessary groups
echo "Adding user $USERNAME to dialout and plugdev groups..."
sudo usermod -a -G dialout $USERNAME
sudo usermod -a -G plugdev $USERNAME

# Create udev rule for the specific USB device
echo "Creating udev rule for the Sony Playstation Eye camera..."
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1415", ATTR{idProduct}=="2000", MODE="0666"' | sudo tee /etc/udev/rules.d/99-playstation-eye.rules

# Reload udev rules and trigger them
echo "Reloading udev rules..."
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Script execution completed. Please log out and log back in for group changes to take effect."
