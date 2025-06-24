**Note**: all versions of ROS discussed here are ROS2. 

# Installing ROS on the raspberry pi

Ideal is to use Ubuntu rather than the RaspberryPi OS.
Alternative is to install ROS in a docker container.

On a RPi running Ubuntu


We use debian binary packages.

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

    sudo apt install software-properties-common
    sudo add-apt-repository universe

    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update && sudo apt install ros-dev-tools
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-jazzy-desktop

Test the installation:

from ssh from a remote host (the sys76) we can:

    $ source /opt/ros/jazzy/setup.bash
    vio@vio-2404:~$ ros2 run demo_nodes_cpp talker
    [INFO] [1731474715.420151136] [talker]: Publishing: 'Hello World: 1'
    [INFO] [1731474716.420117582] [talker]: Publishing: 'Hello World: 2'
    [INFO] [1731474717.420115804] [talker]: Publishing: 'Hello World: 3'
    [INFO] [1731474718.420124041] [talker]: Publishing: 'Hello World: 4'
    [INFO] [1731474719.420099071] [talker]: Publishing: 'Hello World: 5'
    [INFO] [1731474720.420121914] [talker]: Publishing: 'Hello World: 6'
    [INFO] [1731474721.420127513] [talker]: Publishing: 'Hello World: 7'


And on the rpi, we do:

    source /opt/ros/jazzy/setup.bash
    ros2 run demo_nodes_py listener

And we will see the listener messages.


https://robofoundry.medium.com/oak-d-lite-camera-ros2-setup-1e74ed03350d

Ubuntu for Raspberry Pi is available here.

Make sure to confirm that you have selected the correct version as described in REP-2000.

You can now install ROS 2 using the normal binary installation instructions for Ubuntu Linux.

And here:
https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html#ubuntu-linux-on-raspberry-pi-with-binary-ros-2-install

## On a RPi running RaspberryPi OS


### custom .deb package

https://github.com/Ar-Ray-code/rpi-bullseye-ros2

wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-jazzy-desktop-0.3.2_20240525_arm64.deb
sudo apt install ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb
pip install --break-system-packages empy==3.3.4
sudo pip install --break-system-packages vcstool colcon-common-extensions

The first command will end with 

N: Download is performed unsandboxed as root as file '/home/pi/Downloads/ros-jazzy-desktop-0.3.2_20240525_arm64.deb' couldn't be accessed by user '_apt'. - pkgAcquire::Run (13: Permission denied)


### using docker.

    

# Packages installs

(note: for usage, see README)

## cv-bridge

sudo apt install ros-jazzy-cv-bridge
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
E: Unable to locate package ros-jazzy-cv-bridge

So we have to install from source.

    cd ROS2_WS/
    mkcd cv_bridge
    mkcd src
    git clone https://github.com/ros-perception/vision_opencv.git -b humble
    cd ..
    colcon build --symlink-install
    source install/local_setup.bash 
    colcon test

The test build finished successfully.

