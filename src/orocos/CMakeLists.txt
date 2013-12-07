cmake_minimum_required(VERSION 2.8)

# NOT STRICTLY NECESSARY, BUT USEFUL
find_package(Orocos-RTT REQUIRED)
if(Orocos-RTT_FOUND)
    ADD_DEFINITIONS(-DOROCOS_TARGET=gnulinux)
    # Defines the orocos_* cmake macros. See that file for additional documentation.
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
else()
    message("OROCOS not found: the OrocosState classes will not be compiled")
endif()

message(" -- REQUIRED OROCOS DATA --")
message("${OROCOS-RTT_INCLUDE_DIRS}")
message("${OROCOS-RTT_LIBRARIES}")
message("${OROCOS-RTT_DEFINITIONS}")


