set(TARGET_NAME 00)
# set( Eigen3_DIR  "D:/software/eigen-3.4.0")
# set( Eigen3_DIR  "D:/software/eigen-3.4.0/Eigen")
set( Eigen3_DIR  "D:/software/eigen-3.4.0/build")
include_directories(include ${Eigen3_DIR})

find_package(Eigen3 REQUIRED)


add_executable (${TARGET_NAME} main.cpp)
target_link_libraries(${TARGET_NAME} PRIVATE Eigen3::Eigen)
