# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(__PROJECT_GROUP_LINK_COMPONENTS ON)

set(PLANE_COMPONENT_DIRS ${PLANE_COMPONENT_DIRS}
                        "./my_components/core"
                        "./my_components/drivers"
                        "./my_components/drivers/general"
                        "./my_components/drivers/i2c_devices"
                        "./my_components/drivers/spi_devices"
                        "./my_components/drivers/boshc"
                         "./my_components/lib" )

set(EXTRA_COMPONENT_DIRS "./my_components" "${EXTRA_COMPONENT_DIRS} ${PLANE_COMPONENT_DIRS}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESPDrone)
