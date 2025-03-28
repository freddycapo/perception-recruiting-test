cmake_minimum_required(VERSION 3.10)

project(Eagle-recruit)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)
file(GLOB SOURCES "src/*.cpp")

find_package(OpenCV REQUIRED)

add_executable(Eagle-recruit ${SOURCES} )

target_include_directories(Eagle-recruit PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(Eagle-recruit PRIVATE ${OpenCV_LIBS})
