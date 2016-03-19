cmake_minimum_required(VERSION 2.8.3)
project(nekonote_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

## Mark description directories for installation
foreach(dir meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
