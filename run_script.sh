## Author : Basharat, Basharat
## Email  : Basharat.martin@noaa.gov
## date	  : 02/07/2023

#!/bin/bash

## check root
if [ $(id -u) -ne 0 ]
then 
	echo "Please run with sudo permission !"
	exit
fi

USER=$(echo $SUDO_USER)

## download gcc-arm-non-eabi-8-2018 for linux64
echo "\nDownloading gcc-arm-none-eabi tar package ... \n\n"
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2

## check if msp430-gcc.tar.gz file exists
if [ -f "./gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2" ]
then 
	echo "\ngcc-arm-none-eabi tar package found"
	if [ -d "../gcc-arm-none-eabi-8-2018-q4-major" ] 
	then
    	echo "Error: ../gcc-arm-none-eabi-8-2018-q4-major Directory already exists. Please remove if it is empty \n\n"
		exit
	else 
		## extract gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2 tar file 
		echo "Extracting gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2 file ..."
		su $USER -c "tar xf ./gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2 -C ../"
	fi
else
	echo "\nError: gcc-arm-none-eabi tar package NOT found"
	exit
fi

## install required packages
echo "\nUpdating the repositories ..."
apt -qq update
sleep 1
apt install --no-install-recommends -y libusb-dev libusb-0.1-4 libudev-dev make build-essential g++ git libreadline-dev libncursesw5
sleep 1


PWD=$(pwd)

## copy AmbiqSuiteSDK from its git repository 
echo "\nGit cloning AmbiqSuiteSDK ... "
## check if AmbiqSuiteSDK directory exists already
if [ -d "../AmbiqSuiteSDK" ] 
then
    echo "Error: AmbiqSuiteSDK Directory already exists. Please remove if it is empty"
	exit
else
    echo "Directory does not exists, creating directory and cloning now ..."
	mkdir ../AmbiqSuiteSDK
fi
git clone --recursive https://github.com/sparkfun/AmbiqSuiteSDK.git ../AmbiqSuiteSDK
chown $USER:$USER -R ../AmbiqSuiteSDK

## export the GCC_ARM_EABI PATH
echo "\nExporting GCC PATH into .bashrc file ..."

if [ $(grep -r "GCC_ARM_EABI_ROOT" /home/$USER/.bashrc > /dev/null; echo $?) -eq 0 ]
then 
	echo "GCC PATH already present"

else
	echo "export PATH=\"$PWD/../gcc-arm-none-eabi-8-2018-q4-major/bin:\$PATH\"" >> /home/$USER/.bashrc
	echo "export GCC_ARM_EABI_ROOT=\"$PWD/../gcc-arm-none-eabi-8-2018-q4-major\"" >> /home/$USER/.bashrc
fi

## export the AmbiqSuiteSDK PATH
if [ $(grep -r "AmbiqSuiteSDK" /home/$USER/.bashrc > /dev/null; echo $?) -eq 0 ]
then 
	echo "AmbiqSuiteSDK PATH already present"

else
	echo "export AmbiqSuiteSDK=\"$PWD/../AmbiqSuiteSDK\"" >> /home/$USER/.bashrc
fi

echo "Checking if a user in the dialout group ..."
if [ $(groups $USER | grep dialout >/dev/null; echo $?) -eq 0 ]
then
	echo "User in already in the dialout group ..."
	LOGOUT_WARNING=0
else
	echo "User is not in the dialout group, Adding into the dialout group ..."
	usermod -a -G dialout $USER
	LOGOUT_WARNING=1
fi
sleep 1

echo "completed"
sleep 1
echo "\n"
echo "Please run the command: \"source ~/.bashrc\""
echo "\n"

if [ $LOGOUT_WARNING -eq 1 ]
then
	echo "Please log out and log in again to activate user to dialout group"
fi

