# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/razvan/esp/esp-idf/components/bootloader/subproject"
  "/home/razvan/Coding/EspProjects/Licenta/testLicenta/build/bootloader"
  "/home/razvan/Coding/EspProjects/Licenta/testLicenta/build/bootloader-prefix"
  "/home/razvan/Coding/EspProjects/Licenta/testLicenta/build/bootloader-prefix/tmp"
  "/home/razvan/Coding/EspProjects/Licenta/testLicenta/build/bootloader-prefix/src/bootloader-stamp"
  "/home/razvan/Coding/EspProjects/Licenta/testLicenta/build/bootloader-prefix/src"
  "/home/razvan/Coding/EspProjects/Licenta/testLicenta/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/razvan/Coding/EspProjects/Licenta/testLicenta/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/razvan/Coding/EspProjects/Licenta/testLicenta/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
