cmake_minimum_required(VERSION 3.2)

# Add depthai-core dependency
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)

set(TARGET_NAME test-app)
project(${TARGET_NAME})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -O1")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES "include" "libusb" "libusb-1.0")
find_library(LIBUSB_LIBRARY NAMES usb-1.0 PATH_SUFFIXES "lib")


# Add source files
add_executable("${TARGET_NAME}"
        src/main.cpp)

# Link with libraries
target_link_libraries(${TARGET_NAME}
        PUBLIC
        depthai-core
        ${CMAKE_THREAD_LIBS_INIT}
        PUBLIC ${LIBUSB_LIBRARY}
        )

target_include_directories(${TARGET_NAME}
        PUBLIC "${LIBUSB_INCLUDE_DIR}"
        )