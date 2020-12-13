file(GLOB CRUNCHMATH_SOURCE_FILES
	"src/*.*"
	"src/BVH/*.*")

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "src" FILES ${CRUNCHMATH_SOURCE_FILES})
add_library(CrunchMath STATIC ${CRUNCHMATH_SOURCE_FILES})
target_include_directories(CrunchMath PUBLIC /)