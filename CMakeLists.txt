cmake_minimum_required(VERSION 2.8.3)

project(kd45_test)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp

)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  resources/
)

install(PROGRAMS
  scripts/kd45_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
