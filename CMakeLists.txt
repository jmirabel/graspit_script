cmake_minimum_required(VERSION 3.0.0)
project(graspit_script)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(graspit)

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations

ADD_DEFINITIONS(${QT_DEFINITIONS})

set(MOCS
    include/graspit_interface.h)

qt5_wrap_cpp(GENERATED_SOURCES ${MOCS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(graspit_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_library(graspit_interface SHARED
    src/graspit_interface.cpp
    src/main.cpp
    ${GENERATED_SOURCES}
    ${MOCS})


target_include_directories(graspit_interface
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GRASPIT_INCLUDE_DIRS}
  # ${QT_INCLUDES}
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(graspit_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${QT_DEFINITIONS})
# add_dependencies(graspit_interface ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(graspit_interface
  # graspit
  ${QT_LIBRARIES}
  Qt5::Core Qt5::Widgets
)

install(TARGETS graspit_interface
  LIBRARY DESTINATION lib/plugins
)