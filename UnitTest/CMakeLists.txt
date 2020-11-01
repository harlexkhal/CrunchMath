project(UnitTest LANGUAGES CXX)

set (UNITTEST_SOURCE_FILES
	src/Main.cpp)

add_executable(UnitTest ${UNITTEST_SOURCE_FILES})
target_include_directories(UnitTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(UnitTest PUBLIC CrunchMath)