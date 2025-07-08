# The following lines of boilerplate have to be in your project's
# CMakeLists.txt, the first two are required for ESP-IDF to work
cmake_minimum_required(VERSION 3.16)

# Incluir componentes del ESP-IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Nombre del proyecto
project(vl53l0x_test)