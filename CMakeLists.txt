cmake_minimum_required(VERSION 3.0.0)
project(gtsam_tutorial VERSION 0.1.0)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${GTSAM_INCLUDE_DIR}
)

# Add required libraries for proper compilation
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    gtsam
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
