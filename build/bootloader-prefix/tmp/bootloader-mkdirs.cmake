# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/antonio/esp/esp-idf/components/bootloader/subproject"
  "/mnt/d/pruebas_tfg/build/bootloader"
  "/mnt/d/pruebas_tfg/build/bootloader-prefix"
  "/mnt/d/pruebas_tfg/build/bootloader-prefix/tmp"
  "/mnt/d/pruebas_tfg/build/bootloader-prefix/src/bootloader-stamp"
  "/mnt/d/pruebas_tfg/build/bootloader-prefix/src"
  "/mnt/d/pruebas_tfg/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/mnt/d/pruebas_tfg/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/mnt/d/pruebas_tfg/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
