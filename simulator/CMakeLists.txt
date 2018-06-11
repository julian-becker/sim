cmake_minimum_required(VERSION 3.5)
project(simulator LANGUAGES CXX)

# find_package(mylibA REQUIRED)
# find_package(mylibB REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

# target_link_libraries(${PROJECT_NAME}
#   PRIVATE
#     mylibA::mylibA
#     mylibB::mylibB
# )

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
