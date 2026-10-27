project(halsim_studica)

include(CompileWarnings)

add_library(halsim_studica SHARED
    src/main/native/cpp/devices/I2C.cpp
    src/main/native/cpp/devices/DutyCycle.cpp
    src/main/native/cpp/devices/DigitalEdges.cpp
    src/main/native/cpp/ChannelMap.cpp
    src/main/native/cpp/HALSimStudica.cpp
    src/main/native/cpp/main.cpp
)
wpilib_target_warnings(halsim_studica)
set_target_properties(halsim_studica PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(halsim_studica PUBLIC hal studica)
target_include_directories(halsim_studica PUBLIC src/main/native/include)
set_property(TARGET halsim_studica PROPERTY FOLDER "libraries")
install(TARGETS halsim_studica EXPORT halsim_studica)
export(TARGETS halsim_studica FILE halsim_studica.cmake NAMESPACE halsim_studica::)

if(WPILIB_WITH_TESTS)
    wpilib_add_test(halsim_studica src/test/native/cpp)
    target_link_libraries(halsim_studica_test halsim_studica)
    add_dependencies(halsim_studica_test studica_test_plugin_good studica_test_plugin_bad_abi studica_test_plugin_incomplete)
    target_compile_definitions(halsim_studica_test PRIVATE
        HALSIMSTUDICA_TEST_INCOMPLETE_PLUGIN="$<TARGET_FILE:studica_test_plugin_incomplete>"
        HALSIMSTUDICA_TEST_PLUGIN="$<TARGET_FILE:studica_test_plugin_good>"
        HALSIMSTUDICA_TEST_BAD_ABI_PLUGIN="$<TARGET_FILE:studica_test_plugin_bad_abi>"
        HALSIMSTUDICA_TEST_NOT_A_PLUGIN="$<TARGET_FILE:halsim_studica>"
    )
endif()
