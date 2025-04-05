cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(llcc68_driver)

target_sources(app PRIVATE
  llcc68.c
  llcc68_spi.c
)
zephyr_library()
zephyr_library_sources(llcc68.c)
zephyr_include_directories(.)
