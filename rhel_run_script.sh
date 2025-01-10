#!/bin/bash

# Check for root permissions
if [ $(id -u) -ne 0 ]; then
    echo "Please run with sudo permission!"
    exit 1
fi

USER=$(echo $SUDO_USER)

# Download gcc-arm-none-eabi tar package
echo -e "\nDownloading gcc-arm-none-eabi tar package...\n"
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2

# Check if the tar file exists
if [ -f "./gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2" ]; then
    echo -e "\ngcc-arm-none-eabi tar package found"
    if [ -d "../gcc-arm-none-eabi-8-2018-q4-major" ]; then
        echo "Error: ../gcc-arm-none-eabi-8-2018-q4-major directory already exists. Please remove it if empty."
        exit 1
    else
        # Extract tar file
        echo "Extracting gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2..."
        su $USER -c "tar xf ./gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2 -C ../"
    fi
else
    echo -e "\nError: gcc-arm-none-eabi tar package NOT found"
    exit 1
fi

# Install required packages
echo -e "\nInstalling required packages..."
dnf -q makecache
sleep 1
dnf install -y libusbx-devel libusb-devel libudev-devel make gcc-c++ git readline-devel ncurses-devel

# Current working directory
PWD=$(pwd)

# Clone AmbiqSuiteSDK repository
echo -e "\nCloning AmbiqSuiteSDK repository..."
if [ -d "../AmbiqSuiteSDK" ]; then
    echo "Error: AmbiqSuiteSDK directory already exists. Please remove it if empty."
    exit 1
else
    echo "Directory does not exist, creating directory and cloning now..."
    mkdir ../AmbiqSuiteSDK
    git clone --recursive https://github.com/sparkfun/AmbiqSuiteSDK.git ../AmbiqSuiteSDK
    chown $USER:$USER -R ../AmbiqSuiteSDK
fi

# Export GCC_ARM_EABI PATH to .bashrc
echo -e "\nExporting GCC path into .bashrc file..."
if grep -q "GCC_ARM_EABI_ROOT" /home/$USER/.bashrc; then
    echo "GCC path already present."
else
    echo "export PATH=\"$PWD/../gcc-arm-none-eabi-8-2018-q4-major/bin:\$PATH\"" >> /home/$USER/.bashrc
    echo "export GCC_ARM_EABI_ROOT=\"$PWD/../gcc-arm-none-eabi-8-2018-q4-major\"" >> /home/$USER/.bashrc
fi

# Export AmbiqSuiteSDK PATH to .bashrc
if grep -q "AmbiqSuiteSDK" /home/$USER/.bashrc; then
    echo "AmbiqSuiteSDK path already present."
else
    echo "export AmbiqSuiteSDK=\"$PWD/../AmbiqSuiteSDK\"" >> /home/$USER/.bashrc
fi

# Check and add user to the dialout group
echo "Checking if the user is in the dialout group..."
if groups $USER | grep -q dialout; then
    echo "User is already in the dialout group."
    LOGOUT_WARNING=0
else
    echo "Adding user to the dialout group..."
    usermod -a -G dialout $USER
    LOGOUT_WARNING=1
fi
sleep 1

echo "Completed"
sleep 1
echo -e "\nPlease run the command: \"source ~/.bashrc\"\n"

if [ $LOGOUT_WARNING -eq 1 ]; then
    echo "Please log out and log in again to activate the dialout group changes."
fi