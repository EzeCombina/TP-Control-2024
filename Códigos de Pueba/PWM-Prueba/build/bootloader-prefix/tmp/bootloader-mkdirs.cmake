# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/ricar/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "D:/UNIVERSIDAD/Projects_ESP32_TDIII/Proyectos_ESPIDF/PWM-Prueba/build/bootloader"
  "D:/UNIVERSIDAD/Projects_ESP32_TDIII/Proyectos_ESPIDF/PWM-Prueba/build/bootloader-prefix"
  "D:/UNIVERSIDAD/Projects_ESP32_TDIII/Proyectos_ESPIDF/PWM-Prueba/build/bootloader-prefix/tmp"
  "D:/UNIVERSIDAD/Projects_ESP32_TDIII/Proyectos_ESPIDF/PWM-Prueba/build/bootloader-prefix/src/bootloader-stamp"
  "D:/UNIVERSIDAD/Projects_ESP32_TDIII/Proyectos_ESPIDF/PWM-Prueba/build/bootloader-prefix/src"
  "D:/UNIVERSIDAD/Projects_ESP32_TDIII/Proyectos_ESPIDF/PWM-Prueba/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/UNIVERSIDAD/Projects_ESP32_TDIII/Proyectos_ESPIDF/PWM-Prueba/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/UNIVERSIDAD/Projects_ESP32_TDIII/Proyectos_ESPIDF/PWM-Prueba/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
