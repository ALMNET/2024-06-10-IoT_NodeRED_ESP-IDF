# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/ESP32/Espressif/frameworks/esp-idf-v5.0.2/components/bootloader/subproject"
  "D:/Armando/OneDrive/Desktop/ESP32_Example_Blink/blink/build/bootloader"
  "D:/Armando/OneDrive/Desktop/ESP32_Example_Blink/blink/build/bootloader-prefix"
  "D:/Armando/OneDrive/Desktop/ESP32_Example_Blink/blink/build/bootloader-prefix/tmp"
  "D:/Armando/OneDrive/Desktop/ESP32_Example_Blink/blink/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Armando/OneDrive/Desktop/ESP32_Example_Blink/blink/build/bootloader-prefix/src"
  "D:/Armando/OneDrive/Desktop/ESP32_Example_Blink/blink/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Armando/OneDrive/Desktop/ESP32_Example_Blink/blink/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Armando/OneDrive/Desktop/ESP32_Example_Blink/blink/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
