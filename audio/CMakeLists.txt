cmake_minimum_required(VERSION 3.20)
project(MagicAapSpatial LANGUAGES CXX)

include(FetchContent)
find_package(FFMPEG QUIET)
find_package(mysofa CONFIG REQUIRED)
if(TARGET mysofa::mysofa-static)
	set(MYSOFA_TARGET mysofa::mysofa-static)
else()
	set(MYSOFA_TARGET mysofa::mysofa-shared)
endif()

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/assets")
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/assets/MIT_KEMAR_normal_pinna.sofa"
	"${CMAKE_CURRENT_BINARY_DIR}/assets/MIT_KEMAR_normal_pinna.sofa"
	COPYONLY)
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/assets/THIRD_PARTY_NOTICES.md"
	"${CMAKE_CURRENT_BINARY_DIR}/assets/THIRD_PARTY_NOTICES.md"
	COPYONLY)

FetchContent_Declare(miniaudio_source
	GIT_REPOSITORY https://github.com/mackron/miniaudio.git
	GIT_TAG 0.11.25
	GIT_SHALLOW TRUE
	SOURCE_SUBDIR cmake-header-only)
FetchContent_MakeAvailable(miniaudio_source)

add_library(magic_aap_spatial_dsp spatializer.cpp)
target_compile_features(magic_aap_spatial_dsp PUBLIC cxx_std_20)
target_include_directories(magic_aap_spatial_dsp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(magic_aap_spatial_dsp PUBLIC ${MYSOFA_TARGET})

add_library(magic_aap_spatial_core ../src/core.cpp)
target_compile_features(magic_aap_spatial_core PUBLIC cxx_std_20)
target_include_directories(magic_aap_spatial_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_library(magic_aap_spatial_atmos cavern_object_stream.cpp atmos_panner.cpp)
target_compile_features(magic_aap_spatial_atmos PUBLIC cxx_std_20)
target_include_directories(magic_aap_spatial_atmos PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(magic_aap_spatial_smoke smoke.cpp)
target_link_libraries(magic_aap_spatial_smoke PRIVATE magic_aap_spatial_dsp)

add_executable(magic_aap_spatial_core_smoke ../test/core_smoke.cpp)
target_link_libraries(magic_aap_spatial_core_smoke PRIVATE magic_aap_spatial_core)

add_executable(magic_aap_spatial_atmos_smoke ../test/atmos_panner_smoke.cpp)
target_link_libraries(magic_aap_spatial_atmos_smoke PRIVATE magic_aap_spatial_atmos)

add_executable(magic-aap-spatial ../src/main.cpp ../src/windows_device.cpp media_player.cpp ffmpeg_channel_decoder.cpp)
target_link_libraries(magic-aap-spatial PRIVATE magic_aap_spatial_core magic_aap_spatial_dsp magic_aap_spatial_atmos)
target_include_directories(magic-aap-spatial PRIVATE ${miniaudio_source_SOURCE_DIR})
if(FFMPEG_FOUND)
	target_compile_definitions(magic-aap-spatial PRIVATE MAGIC_AAP_HAS_FFMPEG)
	target_include_directories(magic-aap-spatial PRIVATE ${FFMPEG_INCLUDE_DIRS})
	target_link_directories(magic-aap-spatial PRIVATE ${FFMPEG_LIBRARY_DIRS})
	target_link_libraries(magic-aap-spatial PRIVATE ${FFMPEG_LIBRARIES})
endif()
if(WIN32)
	target_link_libraries(magic-aap-spatial PRIVATE setupapi ole32 winmm avrt uuid)
	target_compile_definitions(magic-aap-spatial PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)

	add_executable(MagicAapSpatialPlayer WIN32 ../src/gui_windows.cpp ../src/head_tracker.cpp ../src/windows_device.cpp media_player.cpp ffmpeg_channel_decoder.cpp)
	target_link_libraries(MagicAapSpatialPlayer PRIVATE magic_aap_spatial_core magic_aap_spatial_dsp magic_aap_spatial_atmos setupapi comctl32 comdlg32 ole32 winmm avrt uuid)
	target_include_directories(MagicAapSpatialPlayer PRIVATE ${miniaudio_source_SOURCE_DIR})
	if(FFMPEG_FOUND)
		target_compile_definitions(MagicAapSpatialPlayer PRIVATE MAGIC_AAP_HAS_FFMPEG)
		target_include_directories(MagicAapSpatialPlayer PRIVATE ${FFMPEG_INCLUDE_DIRS})
		target_link_directories(MagicAapSpatialPlayer PRIVATE ${FFMPEG_LIBRARY_DIRS})
		target_link_libraries(MagicAapSpatialPlayer PRIVATE ${FFMPEG_LIBRARIES})
	endif()
	target_compile_definitions(MagicAapSpatialPlayer PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

enable_testing()
add_test(NAME spatializer_smoke COMMAND magic_aap_spatial_smoke)
add_test(NAME core_smoke COMMAND magic_aap_spatial_core_smoke)
add_test(NAME atmos_panner_smoke COMMAND magic_aap_spatial_atmos_smoke)