cmake_minimum_required(VERSION 3.0)

project(
    roboclaw
)

add_library(roboclaw roboclaw.c)
install(TARGETS roboclaw DESTINATION lib)
install(FILES roboclaw.h DESTINATION include)

add_executable(roboclaw-test examples/roboclaw-test.c)
target_link_libraries(roboclaw-test roboclaw)
