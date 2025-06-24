# explored packages

## picamera_ros2

https://github.com/Ar-Ray-code/picamera_ros2


Tested on a pi5 with arducam UC-788 (with OV9821, mono, global shutter), and picamera 2.1.
For the arducam it works with ti installed the arducam libcamera fork, not sure if the stock libcamera would work out of the box.

Install

sudo apt install build-essential cmake git libcamera-dev libopencv-dev libdrm-dev libboost-dev libboost-program-options-dev python3-typeguard

    git clone https://github.com/Ar-Ray-code/picamera_ros2.git ~/ros2_ws/picamera_ros2/src/picamera_ros2
    cd ~/ros2_ws/picamera_ros2 # or whatever name
    vcs import ./src < ./src/picamera_ros2/generate_parameter_library.repos
    colcon build

Change the camera parameters in picamera_ros2/picamera_param/src/picamera_parameters.yaml

It's video_height and video_width, not camera_height / width.

Then rebuild (`colcon build`)

There should be a way to change parameters without having to rebuild, TODO check how

Change the publisher depth to 1 in picamera_ros2/picamera_ros2/src/picamera_pub.cpp:

    this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 1);

The 10 or 1 above is the queue size or history depth for the publisher, that is the max number of messages that will be stored in the publisher's outgoing message queue if a subscriber is not keeping up with the messages being published. 
Essentially, it determines how many messages the publisher will buffer before older messages are dropped when the subscriber is slow to receive them. 
A larger queue size can help handle temporary spikes in message rate or slow subscribers, while a smaller queue size might be preferred in situations where memory is constrained or older messages are not valuable. 

On the rpikimera, go to ~/ros2_ws/picamera_ros2

    source install/setup.bash

Then run:

    ros2 run picamera_ros2 picamera_pub_exec

Check the bandwith with:

    ros2 topic bw /image_raw

# create a package

## python

The most basic command is:

    ros2 pkg create mypackagename --build-type ament_python 


To also add nodes, license information:

    ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name img_cvbridge cvbridge_helloworld_py

## copying data from an existing package

- create the package:

    ros2 pkg create mypackagename --build-type ament_python 

- copy the relevant source file, most likely an exisiting node, rename the node if needed.
- edit the copied file(s) to  edit the node name

- edit the setup.py file to add the node name: 

    ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name img_cvbridge cvbridge_helloworld_py

example: 
in the node file (here it's 'tof_ptcl_line.py'): 

    class TOFPublisher(Node):

        def __init__(self, tof: ArducamCamera, camera_info: ArducamInfo):
            super().__init__('pseudo_slam') # pseudo_slam is the name of the node

update the setup.py:

    entry_points={
        'console_scripts': [
            'pseudo_slam= tof_pseudo_slam.tof_ptcl_line:main'
        ],

where:
    - pseudo_slam is the node name
    - tof_pseudo_slam is the name of the package and path to the executable
    - tof_ptcl_line is the name of the python file (tof_ptcl_line.py)


