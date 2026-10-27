project(studica)

include(CompileWarnings)
find_package(Threads REQUIRED)

add_library(studica SHARED
    src/backend/BackendFactory.cpp
    src/backend/DlBackend.cpp
    src/backend/LoopbackBackend.cpp
    src/conversion/DeviceConversions.cpp
    src/runtime/TerminationHandlers.cpp
)
wpilib_target_warnings(studica)
target_compile_features(studica PUBLIC cxx_std_20)
target_include_directories(studica PUBLIC include)
target_link_libraries(studica PRIVATE ${CMAKE_DL_LIBS} Threads::Threads)
set_property(TARGET studica PROPERTY FOLDER "libraries")
install(TARGETS studica EXPORT studica)
export(TARGETS studica FILE studica.cmake NAMESPACE studica::)
install(DIRECTORY include/ DESTINATION "${include_dest}")

# The same hardware implementation is exercised against a test SDK on the host.
set(studica_hardware_sources
    src/conversion/DeviceConversions.cpp
    src/hardware/StudicaBackend.cpp
    src/hardware/SharedVMX.cpp
    src/hardware/RtcClock.cpp
    src/hardware/devices/Digital.cpp
    src/hardware/devices/Analog.cpp
    src/hardware/devices/Encoder.cpp
    src/hardware/devices/Imu.cpp
    src/hardware/devices/Pwm.cpp
    src/hardware/devices/I2c.cpp
    src/hardware/devices/DutyCycle.cpp
    src/hardware/devices/Interrupt.cpp
)

if(WPILIB_WITH_STUDICA)
    add_subdirectory(drivers)
    add_library(studica_hardware SHARED ${studica_hardware_sources} src/titan/Titan.cpp)
    target_compile_features(studica_hardware PRIVATE cxx_std_20)
    target_include_directories(studica_hardware PUBLIC include PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../hal/src/main/native/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../wpiutil/src/main/native/include"
    )
    target_link_libraries(studica_hardware PRIVATE studica_drivers)
    set_property(TARGET studica_hardware PROPERTY FOLDER "libraries")
    install(TARGETS studica_hardware EXPORT studica_hardware)
endif()

if(WPILIB_WITH_TESTS)
    set(STUDICA_TEST_SANITIZERS "" CACHE STRING "Sanitizers for Studica hardware tests (address,undefined or thread)")
    wpilib_add_test(studica_hardware
        tests/main.cpp
        tests/hardware/HardwareBackendTest.cpp
        tests/hardware/ControlConcurrencyTest.cpp
        ${studica_hardware_sources}
    )
    target_compile_features(studica_hardware_test PRIVATE cxx_std_20)
    target_include_directories(studica_hardware_test BEFORE PRIVATE tests/hardware/fake_sdk)
    target_include_directories(studica_hardware_test PRIVATE include src/hardware)
    target_link_libraries(studica_hardware_test hal Threads::Threads)
    if(STUDICA_TEST_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(studica_hardware_test PRIVATE
            -fsanitize=${STUDICA_TEST_SANITIZERS} -fno-omit-frame-pointer)
        target_link_options(studica_hardware_test PRIVATE -fsanitize=${STUDICA_TEST_SANITIZERS})
    endif()
    foreach(variant good bad_abi incomplete)
        add_library(studica_test_plugin_${variant} SHARED
            tests/plugin/LoopbackPlugin.cpp
            src/backend/LoopbackBackend.cpp
        )
        target_compile_features(studica_test_plugin_${variant} PRIVATE cxx_std_20)
        target_include_directories(studica_test_plugin_${variant} PRIVATE include)
        set_property(TARGET studica_test_plugin_${variant} PROPERTY FOLDER "tests")
    endforeach()
    target_compile_definitions(studica_test_plugin_bad_abi PRIVATE HALSIMSTUDICA_TEST_BAD_ABI)
    target_compile_definitions(studica_test_plugin_incomplete PRIVATE HALSIMSTUDICA_TEST_INCOMPLETE)
    wpilib_add_test(studica
        tests/main.cpp
        tests/conversion/EncoderStateTest.cpp
        tests/backend/BackendPluginTest.cpp
        tests/runtime/UniqueResourceTest.cpp
        tests/runtime/TerminationHandlersTest.cpp
        tests/titan/TitanControlTest.cpp
        tests/titan/TitanEnableGuardTest.cpp
    )
    target_link_libraries(studica_test studica hal)
endif()
