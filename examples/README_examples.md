# examples

## how to use

### pyhon

Create a ros workspace directory anywhere.

    mkcd ~/myros_ws

Create a package of the desired name:

    ros2 pkg create myrosexamplepkgname --build-type ament_python

copy the .py file from the example to myrosexamplepkgname/myrosexamplepkgname, for exampple:

    cp examples/python/example.py ~/myros_ws/myrosexamplepkgname/myrosexamplepkgname/.

Edit the setup.py to include the node name and path to executable (or replace the created setup.py with the one in the examples folder, if a setup.py is included).

iexample, in setup.py for the rviz_marker_tuto:

        entry_points={
        'console_scripts': [
                    'rvizmarker = rviz_marker.rviz_marker_tuto:main'
        ],




## visualization topics

See:
https://github.com/ros2/common_interfaces/blob/rolling/visualization_msgs/msg/Marker.msg

and 
https://wiki.ros.org/rviz/DisplayTypes/Marker (the doc is for ros1 but most seem to apply to ros2)

### Capabilities

it's possible to show stl files:
https://robotics.stackexchange.com/questions/111255/ros2-humble-rviz-not-loading-stl-file

stl from openscad (dig more into this)
https://github.com/ros2/rviz/issues/712

### examples

Allows to show spheres, etc...

In rviz, make sure to set the name of the "Fixed frame" to that set in the code, e.g. base_link, or whatever name is chosen.

This is defined by frame_id in the code:

self.marker.header.frame_id = '/base_link'

Code inspired by: https://docs.hello-robot.com/0.3/ros2/example_4/
and:
and also:
https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html

See also this package:
https://index.ros.org/p/rviz_visual_tools/

Thgis video (ros1 but may be interesting it uses sphere_list):
https://www.youtube.com/watch?v=Cy_oU54tPzE
and code:
https://drive.google.com/file/d/1yGD5ESu07pDXoXKPt-2t-oFtxU3c7COk/view

About visualization, see also:
https://docs.m2stud.io/cs/ros_additional/06-L3-rviz/


## using cv_bridge

### python

install cv_bridge

Create a package in a workspace directory of your choice, example `~/ROS2_WS/`

    ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name img_cvbridge cvbridge_helloworld_py

Edit img_cvbridge.py to include the content of the same file in the examples/python directory.

Build:

    colcon build

Source:

source install/setup.bash
source path_to_cv_bridgeinstall/setup.bash

Run:

    ros2 run cvbridge_helloworld_py img_cvbridge 

in another terminal or computer, checck the topics

    $ ros2 topic list

    /parameter_events
    /random_img
    /rosout

See the result in rviz2, add the topic 


### cpp

create the package in your ws of choice.

    ros2 pkg create cv_bridge_helloworld_cpp --dependencies rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV

    cd bridge_helloworld_cpp/cv_bridge_helloworld_cpp/src
    vi img_cvbridge_node.cpp

Copy the file of the same name in the examples/cpp/cv_bridge_helloworld there or its content above.

Replace the CMakeLists.txt with the one present in the directory mentionned above, or edit the CMakeLists.txt created by the ros2 pkg create command above and add the following before ament_package():


    add_executable(img_cvbridge_node src/img_cvbridge_node.cpp)
    ament_target_dependencies(img_cvbridge_node std_msgs sensor_msgs cv_bridge image_transport OpenCV)
    install(TARGETS
       img_cvbridge_node
       DESTINATION lib/${PROJECT_NAME}
     )

Build:

    colcon build --packages-select cv_bridge_helloworld_cpp
    
    (colcon build works equally well as we only have one package here)

    source install/setup.bash

run it:

    ros2 run cv_bridge_helloworld_cpp img_cvbridge_node

Observe the live image being published by using rviz2 or the following command:

    ros2 run image_view image_view --ros-args --remap image:=/random_image

This requires image_view to be installed.

    sudo apt install ros-humble-image-view


followed this:

https://www.theconstruct.ai/how-to-integrate-opencv-with-a-ros2-c-node/

### other example to consider

https://stackoverflow.com/questions/72690021/how-to-process-a-image-message-with-opencv-from-ros2

 cv_bridge demo in ROS2 
https://gist.github.com/nightduck/a07c185faad82aeaacbfa87298d035c0

https://github.com/Daviesss/Integrating-Open-cv-with-ROS-2/blob/master/ros2_cv/ros2_cv/open_cv_integration.py

https://ibrahimmansur4.medium.com/integrating-opencv-with-ros2-a-comprehensive-guide-to-computer-vision-in-robotics-66b97fa2de92

-> https://github.com/ibrahimmansur4/ROS2-Projects/tree/main/line_follower
-> https://github.com/ibrahimmansur4/Vision_tracker_drone

https://industrial-training-master.readthedocs.io/en/foxy/_source/session5/OpenCV-in-Python.html

## Errors, glitches

For all examples, if using different platform (computers) for listener and publisher, make sure the publisher and listener are both on the same `ROS_DOMAIN_ID`

If not use:

    export ROS_DOMAIN_ID=<common_id>
example:     

    export ROS_DOMAIN_ID=7

