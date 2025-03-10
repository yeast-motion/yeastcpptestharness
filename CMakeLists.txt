cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(PROJECT_NAME "yeastcpptestharness")
project(${PROJECT_NAME})

add_compile_options(-rdynamic -Wl,-whole-archive)
add_link_options()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(wpilib REQUIRED)
find_package(wpilibNewCommands REQUIRED)

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

    ntcore
    wpimath
    wpiutil
    wpilibc
    wpilibNewCommands
    hal
)

include_directories(SYSTEM
../yeastcppwpilibdrivecontroller/include
../yeastcppwpilibodometryprovider/include
../yeastcpppathplannertrajectoryfollower/include
../yeastcpp/include
)

# Specify include directories for the library (if needed)
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${ADDITIONAL_INCLUDE_PATHS}
)
