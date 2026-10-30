project(halsim_ds_relay)

include(CompileWarnings)

add_library(halsim_ds_relay SHARED
    src/main/native/cpp/DsRelay.cpp
    src/main/native/cpp/main.cpp
)
wpilib_target_warnings(halsim_ds_relay)
set_target_properties(halsim_ds_relay PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(halsim_ds_relay PUBLIC hal)
target_include_directories(halsim_ds_relay PUBLIC src/main/native/include)
set_property(TARGET halsim_ds_relay PROPERTY FOLDER "libraries")
install(TARGETS halsim_ds_relay EXPORT halsim_ds_relay)
export(TARGETS halsim_ds_relay FILE halsim_ds_relay.cmake NAMESPACE halsim_ds_relay::)

if(WPILIB_WITH_TESTS)
    wpilib_add_test(halsim_ds_relay src/test/native/cpp)
    target_link_libraries(halsim_ds_relay_test halsim_ds_relay)
endif()
