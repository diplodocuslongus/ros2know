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


