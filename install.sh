#!/bin/bash

echo "-------------------------------"
echo "IronCar install by XBrain team"
echo "-------------------------------"

sudo apt-get update -y

sudo apt-get install libblas-dev liblapack-dev libatlas-base-dev gfortran -y
sudo apt-get install libhdf5-dev -y

sudo apt-get install npm -y
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install nodejs -y


# Python
sudo pip3 install -r requirements_raspi.txt

var=`python3 -c 'import sys; print(sys.version_info[:])'`  # Get the Python version
set -- $var
v=$2  # version number
v=${v:0:1}

if [ $v -eq 4 ];
then
    echo "Python 3.4";
    wget https://github.com/samjabrahams/tensorflow-on-raspberry-pi/releases/download/v1.1.0/tensorflow-1.1.0-cp34-cp34m-linux_armv7l.whl
    sudo pip3 install tensorflow-1.1.0-cp34-cp34m-linux_armv7l.whl
elif [ $v -eq 5 ]
then
    echo "Python 3.5";
    wget https://www.dropbox.com/s/gy4kockdbdyx85j/tensorflow-1.0.1-cp35-cp35m-linux_armv7l.whl?dl=1
    mv tensorflow-1.0.1-cp35-cp35m-linux_armv7l.whl?dl=1 tensorflow-1.0.1-cp35-cp35m-linux_armv7l.whl
    sudo pip3 install tensorflow-1.0.1-cp35-cp35m-linux_armv7l.whl
else
    echo "There is no good version of Python";
    python3 --version
fi

# Npm
npm install


sudo apt-get install -y python-smbus
sudo apt-get install -y i2c-tools


if grep -Fxq 'i2c-bcm2708' /etc/modules
then
    echo $'modules already configured'
else
    echo 'i2c-bcm2708' | sudo tee -a /etc/modules
fi


if grep -Fxq 'i2c-dev' /etc/modules
then
    echo $'modules already configured'
else
    echo 'i2c-dev' | sudo tee -a /etc/modules
fi

file="/etc/modprobe.d/raspi-blacklist.conf"

if [ -f "$file" ]
then
    if grep -Fxq 'blacklist spi-bcm2708' /etc/modprobe.d/raspi-blacklist.conf
    then
        sudo sed -i 's/blacklist spi-bcm2708/#blacklist spi-bcm2708/' /etc/modprobe.d/raspi-blacklist.conf
	sudo sed -i 's/blacklist i2c-bcm2708/#blacklist i2c-bcm2708/' /etc/modprobe.d/raspi-blacklist.conf
    fi
else
    echo "no blacklist.conf file to modify, everything alright!"
fi

if grep -Fxq 'dtparam=i2c1=on' /boot/config.txt
then
    echo $'config file already configured'
else
    echo 'dtparam=i2c1=on' | sudo tee -a /boot/config.txt
fi

if grep -Fxq 'dtparam=i2c_arm=on' /boot/config.txt
then
    echo $'config file already configured'
else
    echo 'dtparam=i2c_arm=on' | sudo tee -a /boot/config.txt
fi

if grep -Fxq 'dtparam=i2c1=off' /boot/config.txt
then
    sudo sed -i 's/dtparam=i2c1=off/#dtparam=i2c1=off/' /boot/config.txt
fi
if grep -Fxq 'dtparam=i2c_arm=off' /boot/config.txt
then
    sudo sed -i 's/dtparam=i2c_arm=off/#dtparam=i2c_arm=off/' /boot/config.txt
fi

read -p "Would you like to enable the picamera (y/n)? " CAMCONT
if [ "$CAMCONT" = "y" ]; then
    if ! grep -Fxq 'start_x=1' /boot/config.txt
    then
        sed -i "s/start_x=0/#start_x=0/g" /boot/config.txt
        echo 'start_x=1' | sudo tee -a /boot/config.txt
        echo 'camera enabled'
    else
        echo 'camera was already enabled'
    fi
fi

read -p "Would you like to augment the swap size allocated to 1000MB (y/n)? 100 MB by default" CONT
if [ "$CONT" = "y" ]; then
    if grep -Fxq 'CONF_SWAPSIZE=100' /boot/config.txt
    then
        sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=1000/' /etc/dphys-swapfile
	sudo /etc/init.d/dphys-swapfile stop
	sudo /etc/init.d/dphys-swapfile start
	echo "swap size set to 1000MB"
    else
        echo "SWAPSIZE alsready modified, please modify it by hand if you want to change it again"
    fi
else
    echo "SWAP size was not changed"
fi


read -p "We need a reboot, do you want to reboot now (y/n)?" CONT
if [ "$CONT" = "y" ]; then
  sudo reboot
else
  echo "This install needs a reboot to finish"
fi
