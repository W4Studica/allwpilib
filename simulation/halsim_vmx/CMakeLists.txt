project(halsim_vmx)

include(CompileWarnings)

set(halsim_vmx_src
    src/main/native/cpp/BackendFactory.cpp
    src/main/native/cpp/ChannelMap.cpp
    src/main/native/cpp/DlBackend.cpp
    src/main/native/cpp/HALSimVMX.cpp
    src/main/native/cpp/LoopbackBackend.cpp
    src/main/native/cpp/main.cpp
)

add_library(halsim_vmx SHARED ${halsim_vmx_src})
wpilib_target_warnings(halsim_vmx)
set_target_properties(halsim_vmx PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(halsim_vmx PUBLIC hal)
target_link_libraries(halsim_vmx PRIVATE ${CMAKE_DL_LIBS})

target_include_directories(halsim_vmx PUBLIC src/main/native/include)

set_property(TARGET halsim_vmx PROPERTY FOLDER "libraries")

install(TARGETS halsim_vmx EXPORT halsim_vmx)
export(TARGETS halsim_vmx FILE halsim_vmx.cmake NAMESPACE halsim_vmx::)

# The Studica backend is a separate plugin because it needs VMXPi.h, which exists only
# on a VMX-pi. Build it there with -DWPILIB_WITH_STUDICA=ON and point HALSIMVMX_BACKEND
# at libhalsim_vmx_studica.so. It does not link against halsim_vmx.
if(WPILIB_WITH_STUDICA)
    add_library(halsim_vmx_studica SHARED src/studica/native/cpp/StudicaBackend.cpp)
    set_target_properties(halsim_vmx_studica PROPERTIES DEBUG_POSTFIX "d")
    # studica_drivers only asks for C++17; this plugin uses C++20 library features.
    target_compile_features(halsim_vmx_studica PRIVATE cxx_std_20)
    target_include_directories(
        halsim_vmx_studica
        PUBLIC src/studica/native/include
        PRIVATE src/main/native/include
    )
    target_link_libraries(halsim_vmx_studica PRIVATE studica_drivers)
    set_property(TARGET halsim_vmx_studica PROPERTY FOLDER "libraries")
    install(TARGETS halsim_vmx_studica EXPORT halsim_vmx_studica)
endif()

if(WPILIB_WITH_TESTS)
    # Test-only backend plugins (the loopback backend behind the C ABI). They do not link
    # against halsim_vmx, like the real Studica plugin.
    foreach(variant good bad_abi)
        set(plugin_name halsim_vmx_test_plugin_${variant})
        add_library(${plugin_name} SHARED
            src/test/plugin/LoopbackPlugin.cpp
            src/main/native/cpp/LoopbackBackend.cpp
        )
        target_compile_features(${plugin_name} PRIVATE cxx_std_20)
        target_include_directories(${plugin_name} PRIVATE src/main/native/include)
        set_property(TARGET ${plugin_name} PROPERTY FOLDER "tests")
    endforeach()
    target_compile_definitions(halsim_vmx_test_plugin_bad_abi PRIVATE HALSIMVMX_TEST_BAD_ABI)

    wpilib_add_test(halsim_vmx src/test/native/cpp)
    target_link_libraries(halsim_vmx_test halsim_vmx)
    add_dependencies(halsim_vmx_test halsim_vmx_test_plugin_good halsim_vmx_test_plugin_bad_abi)
    target_compile_definitions(
        halsim_vmx_test
        PRIVATE
            HALSIMVMX_TEST_PLUGIN="$<TARGET_FILE:halsim_vmx_test_plugin_good>"
            HALSIMVMX_TEST_BAD_ABI_PLUGIN="$<TARGET_FILE:halsim_vmx_test_plugin_bad_abi>"
            HALSIMVMX_TEST_NOT_A_PLUGIN="$<TARGET_FILE:halsim_vmx>"
    )
endif()
