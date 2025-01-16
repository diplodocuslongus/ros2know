# PX4 

## swarm 


from: https://github.com/diplodocuslongus/PX4_Swarm_Controller



cd ../../forked_repos/
 1999  l
 2000  mkcd UAV
 2001  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
 bash ./PX4-Autopilot/Tools/setup/ubuntu.sh # will install a lot of packages!
 2002  cd PX4-Autopilot/
 2003  make px4_sitl
 2004  pip install --user -U empy==3.3.4 pyros-genmsg setuptools
 2005  make px4_sitl
 2006  pip3 install kconfiglib
 2007  make px4_sitl
 2008  cd ..
 2009  l
 2010  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
 2011  cd Micro-XRCE-DDS-Agent/
 2012  mkcd build
 2013  cmake ..
 2014  make
 2015  sudo make install
 2016  sudo ldconfig /usr/local/lib/
 2017  cd ~
 2018  l
 2019  cd ROS2_workspaces/
 2020  l
 2021  mkcd PX4_swarm
 2022  mkcd src
 2023  git clone https://github.com/artastier/PX4_Swarm_Controller.git
 2024  mv PX4_Swarm_Controller px4_swarm_controller
 2025  mv -i px4_swarm_controller/sitl_multiple_run.sh ~/Programs/forked_repos/UAV/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh
 2026  mv px4_swarm_controller/custom_msgs/ ~/ROS2_workspaces/PX4_swarm/src/
 2027  colcon build --packages-select custom_msgs

 Need to set the proper path for cmake to find the forked px4...

 Error:

 Make Error at CMakeLists.txt:13 (find_package):
  By not providing "Findpx4_msgs.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "px4_msgs",
  but CMake did not find one.

  Could not find a package configuration file provided by "px4_msgs" with any
  of the following names:

    px4_msgsConfig.cmake
    px4_msgs-config.cmake

  Add the installation prefix of "px4_msgs" to CMAKE_PREFIX_PATH or set
  "px4_msgs_DIR" to a directory containing one of the above files.  If
  "px4_msgs" provides a separate development package or SDK, be sure it has
  been installed.

