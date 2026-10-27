project(studica)

include(CompileWarnings)
find_package(Threads REQUIRED)

add_library(studica SHARED
    src/BackendFactory.cpp
    src/DeviceConversions.cpp
    src/DlBackend.cpp
    src/LoopbackBackend.cpp
    src/TerminationHandlers.cpp
)
wpilib_target_warnings(studica)
target_compile_features(studica PUBLIC cxx_std_20)
target_include_directories(studica PUBLIC include)
target_link_libraries(studica PRIVATE ${CMAKE_DL_LIBS} Threads::Threads)
set_property(TARGET studica PROPERTY FOLDER "libraries")
install(TARGETS studica EXPORT studica)
export(TARGETS studica FILE studica.cmake NAMESPACE studica::)
install(DIRECTORY include/ DESTINATION "${include_dest}")

if(WPILIB_WITH_STUDICA)
    add_subdirectory(drivers)
    add_library(studica_hardware SHARED src/StudicaBackend.cpp src/DeviceConversions.cpp src/Titan.cpp)
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
    foreach(variant good bad_abi)
        add_library(studica_test_plugin_${variant} SHARED
            tests/plugin/LoopbackPlugin.cpp
            src/LoopbackBackend.cpp
        )
        target_compile_features(studica_test_plugin_${variant} PRIVATE cxx_std_20)
        target_include_directories(studica_test_plugin_${variant} PRIVATE include)
        set_property(TARGET studica_test_plugin_${variant} PROPERTY FOLDER "tests")
    endforeach()
    target_compile_definitions(studica_test_plugin_bad_abi PRIVATE HALSIMSTUDICA_TEST_BAD_ABI)
    wpilib_add_test(studica tests/main.cpp tests/TitanEnableGuardTest.cpp tests/TitanControlTest.cpp tests/TerminationHandlersTest.cpp)
    target_link_libraries(studica_test studica hal)
endif()
