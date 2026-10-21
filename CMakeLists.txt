cmake_minimum_required(VERSION 3.20)
project(fpv_atv_hackrf_decoder CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if(MSVC)
    add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
    add_compile_options(/fp:fast /arch:AVX2 /utf-8)
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math")
endif()

# libhackrf: pkg-config where available (Linux/macOS), otherwise search
# directly (Windows: PothosSDR ships hackrf.h + hackrf.lib).
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(HACKRF IMPORTED_TARGET libhackrf)
endif()
if(TARGET PkgConfig::HACKRF)
    set(HACKRF_TARGET PkgConfig::HACKRF)
else()
    find_path(HACKRF_INCLUDE_DIR libhackrf/hackrf.h
              HINTS "C:/Program Files/PothosSDR/include")
    find_library(HACKRF_LIBRARY NAMES hackrf
                 HINTS "C:/Program Files/PothosSDR/lib")
    if(NOT HACKRF_INCLUDE_DIR OR NOT HACKRF_LIBRARY)
        message(FATAL_ERROR "libhackrf not found (pkg-config or PothosSDR)")
    endif()
    add_library(hackrf_import INTERFACE)
    target_include_directories(hackrf_import INTERFACE ${HACKRF_INCLUDE_DIR})
    target_link_libraries(hackrf_import INTERFACE ${HACKRF_LIBRARY})
    set(HACKRF_TARGET hackrf_import)
endif()

# SDL2: a local copy under third_party/SDL2 (Windows) is found first.
find_package(SDL2 REQUIRED CONFIG
             HINTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/SDL2")

add_library(fpv_dsp STATIC
    src/dsp/ntsc_decoder.cpp
)
target_include_directories(fpv_dsp PUBLIC src)

add_executable(fpvdec
    src/main.cpp
    src/source/hackrf_source.cpp
    src/ui/sdl_display.cpp
)
target_link_libraries(fpvdec PRIVATE fpv_dsp ${HACKRF_TARGET} SDL2::SDL2)
if(WIN32)
    # SDL2's WinMain shim is not used; keep the plain main().
    target_compile_definitions(fpvdec PRIVATE SDL_MAIN_HANDLED)
    add_custom_command(TARGET fpvdec POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:SDL2::SDL2> $<TARGET_FILE_DIR:fpvdec>)
    if(HACKRF_LIBRARY)
        get_filename_component(_hackrf_lib_dir "${HACKRF_LIBRARY}" DIRECTORY)
        # hackrf.dll pulls in libusb and pthreads from the same bin dir.
        foreach(_dll hackrf.dll libusb-1.0.dll pthreadVC2.dll)
            if(EXISTS "${_hackrf_lib_dir}/../bin/${_dll}")
                add_custom_command(TARGET fpvdec POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                            "${_hackrf_lib_dir}/../bin/${_dll}"
                            $<TARGET_FILE_DIR:fpvdec>)
            endif()
        endforeach()
    endif()
endif()

enable_testing()
add_executable(synth_fm tests/synth_fm.cpp)
target_link_libraries(synth_fm PRIVATE fpv_dsp)
add_test(NAME synth_fm COMMAND synth_fm)

add_executable(analyze_rec tests/analyze_rec.cpp)
target_link_libraries(analyze_rec PRIVATE fpv_dsp)

add_executable(test_gain_ctl tests/test_gain_ctl.cpp)
target_link_libraries(test_gain_ctl PRIVATE fpv_dsp)
add_test(NAME gain_ctl COMMAND test_gain_ctl)

add_executable(test_c5_emu tests/test_c5_emu.cpp)
target_link_libraries(test_c5_emu PRIVATE fpv_dsp)
add_test(NAME c5_emu COMMAND test_c5_emu)
