project(TestBed2D LANGUAGES CXX)

set (TESTBED2D_SOURCE_FILES
	src/Main.cpp)

set (TESTBED2D_HEADER_FILES
	src/Geometry/Box.h
	src/Geometry/Circle.h
	src/Geometry/Geometry.h)

add_executable(TestBed2D ${TESTBED2D_SOURCE_FILES} ${TESTBED2D_HEADER_FILES})
target_include_directories(TestBed2D PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(TestBed2D PUBLIC CrunchMath glfw glad ${OPENGL_LIBRARIES})
