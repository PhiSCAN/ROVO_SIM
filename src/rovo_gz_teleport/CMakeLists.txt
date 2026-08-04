cmake_minimum_required(VERSION 3.16)
project(rovo_gz_teleport)

# AUTOMOC skips .hh headers unless this is NEW, and a SHARED library links
# happily with the resulting undefined Qt metaobject symbols -- the failure
# only shows up as gz refusing to load the plugin at runtime.
cmake_policy(SET CMP0100 NEW)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui9 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin3 REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport14 REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim9 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2 Widgets)

# gz-sim9 exports only its core target; the GUI half (GuiSystem, which gives a
# panel access to the ECM) ships as a bare .so alongside it.
find_library(GZ_SIM_GUI_LIB
  NAMES gz-sim9-gui
  HINTS ${gz-sim9_LIBRARY_DIRS} ${gz_sim_vendor_DIR}/../../../lib
  REQUIRED)
get_target_property(GZ_SIM_INCLUDE_DIRS gz-sim9::gz-sim9
  INTERFACE_INCLUDE_DIRECTORIES)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(RovoTeleport SHARED
  src/RovoTeleport.cc
  src/RovoTeleport.qrc
)
target_include_directories(RovoTeleport PRIVATE src ${GZ_SIM_INCLUDE_DIRS})
target_link_libraries(RovoTeleport
  gz-gui9::gz-gui9
  gz-msgs11::gz-msgs11
  gz-plugin3::register
  gz-transport14::gz-transport14
  gz-sim9::gz-sim9
  ${GZ_SIM_GUI_LIB}
  Qt5::Core Qt5::Quick Qt5::QuickControls2 Qt5::Widgets
)

# gz discovers GUI plugins on GZ_GUI_PLUGIN_PATH; the launch file points it here.
install(TARGETS RovoTeleport DESTINATION lib/${PROJECT_NAME})

ament_package()
