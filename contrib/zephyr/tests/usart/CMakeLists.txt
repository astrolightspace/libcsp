cmake_minimum_required(VERSION 3.20.0)

get_filename_component(ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../../.. REALPATH)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(csp_usart_tests)

target_compile_definitions(csp PRIVATE _DEFAULT_SOURCE)
target_sources(app PRIVATE src/main.c)
