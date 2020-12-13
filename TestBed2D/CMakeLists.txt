project(TestBed2D LANGUAGES CXX)

file(GLOB TESTBED2D_SOURCE_FILES
	"src/*.*"
	"src/Geometry/*.*")

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "src" FILES ${TESTBED2D_SOURCE_FILES})
add_executable(TestBed2D ${TESTBED2D_SOURCE_FILES})
target_include_directories(TestBed2D PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(TestBed2D PUBLIC CrunchMath glfw glad ${OPENGL_LIBRARIES})