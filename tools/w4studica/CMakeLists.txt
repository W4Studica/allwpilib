cmake_minimum_required(VERSION 3.25)
project(studica_runtime LANGUAGES CXX)

get_filename_component(WPILIB_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH "${WPILIB_SOURCE_DIR}/cmake/modules")
set(include_dest include)
set(WPILIB_WITH_TESTS OFF)
set(WPILIB_WITH_STUDICA ON)
set(CMAKE_BUILD_RPATH "$ORIGIN;/usr/local/lib/vmxpi")
set(CMAKE_INSTALL_RPATH "$ORIGIN;/usr/local/lib/vmxpi")
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(hal_suffix d)
endif()
add_library(hal SHARED IMPORTED GLOBAL)
set_target_properties(hal PROPERTIES
    IMPORTED_LOCATION "${WPILIB_LIBS}/libwpiHal${hal_suffix}.so"
    INTERFACE_INCLUDE_DIRECTORIES "${WPILIB_SOURCE_DIR}/hal/src/main/native/include;${WPILIB_SOURCE_DIR}/wpiutil/src/main/native/include"
    INTERFACE_LINK_LIBRARIES "${WPILIB_LIBS}/libwpiutil${hal_suffix}.so"
)
add_link_options("-Wl,--no-undefined" "-Wl,-rpath-link,${WPILIB_LIBS}")
add_subdirectory("${WPILIB_SOURCE_DIR}/studica" studica)
add_subdirectory("${WPILIB_SOURCE_DIR}/simulation/halsim_studica" halsim_studica)
add_subdirectory("${WPILIB_SOURCE_DIR}/simulation/halsim_ds_relay" halsim_ds_relay)
