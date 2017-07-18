cmake_minimum_required(VERSION 2.8.9)
project(numob_sdk)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

# Pure C SDK, for serial protocol
add_library(numob_serial SHARED src/serial.c)
set_target_properties(numob_serial PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Serial command tool example
add_executable(numob_serial_cli examples/numob_serial_cli.c src/serial.c)

# C++ SDK, for mobile base
add_library(numob_sdk SHARED src/sdk.cpp src/serial.c)
set_target_properties(numob_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

#C++ SDK example
add_executable(numob_sdk_example examples/numob_sdk_example.cpp)
target_link_libraries(numob_sdk_example numob_sdk)


install(TARGETS numob_serial numob_sdk numob_serial_cli numob_sdk_example
        ARCHIVE DESTINATION /usr/local/lib/
        LIBRARY DESTINATION /usr/local/lib/
        RUNTIME DESTINATION /usr/local/bin/
        )

install(FILES include/numob/serial.h include/numob/serial_type.h include/numob/sdk.h
        DESTINATION /usr/local/include/numob/
        )
