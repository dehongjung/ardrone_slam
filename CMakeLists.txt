cmake_minimum_required(VERSION 2.8)
project(ardrone_slam)

find_package(catkin REQUIRED)
catkin_package()

find_package(OpenCV 2)
include_directories(${OpenCV_INCLUDE_DIRS})


#catkin_python_setup()


install(PROGRAMS src/drone_controller.py  
		 src/drone_status.py  
		 src/drone_video_display.py  
		 src/joystick_controller.py  
		 src/keyboard_controller.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)