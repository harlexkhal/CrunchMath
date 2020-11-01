cmake_minimum_required(VERSION 3.8)

project(CrunchMath LANGUAGES CXX)

add_subdirectory(CrunchMath)

option(CRUNCHMATH_BUILD_SAMPLES "Build the CrunchMath TestBed2D program" ON)

if (CRUNCHMATH_BUILD_SAMPLES)

	add_subdirectory(Dependencies/glad)
	add_subdirectory(Dependencies/glfw)
	
	add_subdirectory(TestBed2D)
	add_subdirectory(UnitTest)

	# default startup project for Visual Studio
	if (MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT TestBed2D)
	endif()

endif()