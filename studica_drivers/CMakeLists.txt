project(studica_drivers)

# Studica's VMX-pi hardware library (vmxpi_hal_cpp). Only available on a VMX-pi image.
# Override the search root with -DVMXPI_ROOT=<dir> (expects include/ and lib/ below it).
find_path(
    VMXPI_INCLUDE_DIR
    VMXPi.h
    HINTS ${VMXPI_ROOT}/include ${VMXPI_ROOT}/include/vmxpi
    PATHS /usr/local/include/vmxpi
)
find_library(
    VMXPI_HAL_CPP_LIBRARY
    vmxpi_hal_cpp
    HINTS ${VMXPI_ROOT}/lib ${VMXPI_ROOT}/lib/vmxpi
    PATHS /usr/local/lib/vmxpi
)
if(NOT VMXPI_INCLUDE_DIR OR NOT VMXPI_HAL_CPP_LIBRARY)
    message(
        FATAL_ERROR
        "WPILIB_WITH_STUDICA is ON but Studica vmxpi_hal_cpp was not found. "
        "Install it on the VMX-pi or pass -DVMXPI_ROOT=<dir>, or set -DWPILIB_WITH_STUDICA=OFF."
    )
endif()

# Top-level only: examples/ is not part of the library.
file(GLOB studica_drivers_src ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(studica_drivers SHARED ${studica_drivers_src})
set_target_properties(studica_drivers PROPERTIES DEBUG_POSTFIX "d" POSITION_INDEPENDENT_CODE ON)
set_property(TARGET studica_drivers PROPERTY FOLDER "libraries")

target_compile_features(studica_drivers PUBLIC cxx_std_17)
# Third-party code (Studica, Apache-2.0): not built with WPILib's warning/-Werror flags.

find_package(Threads REQUIRED)
target_link_libraries(studica_drivers PUBLIC ${VMXPI_HAL_CPP_LIBRARY} Threads::Threads atomic rt)

target_include_directories(
    studica_drivers
    PUBLIC
        $<BUILD_INTERFACE:${VMXPI_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${VMXPI_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${include_dest}/studica_drivers>
)

install(TARGETS studica_drivers EXPORT studica_drivers)
export(TARGETS studica_drivers FILE studica_drivers.cmake NAMESPACE studica_drivers::)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION "${include_dest}/studica_drivers"
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "examples" EXCLUDE
)
install(EXPORT studica_drivers DESTINATION share/studica_drivers)
