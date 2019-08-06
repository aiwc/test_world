cmake_minimum_required(VERSION 3.1)
project(player_random-walk)

add_executable(player_random-walk player_random-walk.cpp ../common/player.cpp)
include_directories(../common)

target_link_libraries(player_random-walk)
set_property(TARGET player_random-walk PROPERTY CXX_STANDARD 14)
