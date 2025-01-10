cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME "yeastcpptestharness")
project(${PROJECT_NAME})

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SOURCES
    "src/*.cpp"
    "src/*.c"
)

# Add the library source files
add_executable(${PROJECT_NAME}
    ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
${CMAKE_CURRENT_SOURCE_DIR}/../yeastcppwpilibdrivecontroller/build/libyeastcppwpilibdrivecontroller.a
${CMAKE_CURRENT_SOURCE_DIR}/../yeastcppwpilibodometryprovider/build/libyeastcppwpilibodometryprovider.a
${CMAKE_CURRENT_SOURCE_DIR}/../yeastcpppathplannertrajectoryfollower/build/libyeastcpppathplannertrajectoryfollower.a
${CMAKE_CURRENT_SOURCE_DIR}/../yeastcpp/build/libyeastcpp.a

wpiHal
wpilibc
wpimath
wpinet
wpiutil
ntcore
wpilibNewCommands
ntcore
wpiutil
wpinet
wpimath
wpilibc
wpiHal
)

include_directories(SYSTEM
../yeastcppwpilibdrivecontroller/include
../yeastcppwpilibodometryprovider/include
../yeastcpppathplannertrajectoryfollower/include
../yeastcpp/include
/usr/local/include/eigen3
/usr/local/include/wpimath
/usr/local/include/wpiutil
)

# Specify include directories for the library (if needed)
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${ADDITIONAL_INCLUDE_PATHS}
)
