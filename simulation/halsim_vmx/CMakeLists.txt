project(halsim_vmx)

include(CompileWarnings)

set(halsim_vmx_src
    src/main/native/cpp/ChannelMap.cpp
    src/main/native/cpp/HALSimVMX.cpp
    src/main/native/cpp/LoopbackBackend.cpp
    src/main/native/cpp/main.cpp
)
if(WPILIB_WITH_STUDICA)
    list(APPEND halsim_vmx_src src/main/native/cpp/StudicaBackend.cpp)
else()
    list(APPEND halsim_vmx_src src/main/native/cpp/HostBackend.cpp)
endif()

add_library(halsim_vmx SHARED ${halsim_vmx_src})
wpilib_target_warnings(halsim_vmx)
set_target_properties(halsim_vmx PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(halsim_vmx PUBLIC hal)
if(WPILIB_WITH_STUDICA)
    target_link_libraries(halsim_vmx PRIVATE studica_drivers)
endif()

target_include_directories(halsim_vmx PUBLIC src/main/native/include)

set_property(TARGET halsim_vmx PROPERTY FOLDER "libraries")

install(TARGETS halsim_vmx EXPORT halsim_vmx)
export(TARGETS halsim_vmx FILE halsim_vmx.cmake NAMESPACE halsim_vmx::)

if(WPILIB_WITH_TESTS)
    wpilib_add_test(halsim_vmx src/test/native/cpp)
    target_link_libraries(halsim_vmx_test halsim_vmx)
endif()
