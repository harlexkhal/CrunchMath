file(GLOB CRUNCHMATH_SOURCE_FILES
	"src/*.*"
    "src/Physics/*.*"
	"src/Math/*.*"
	"src/Math/BVH/*.*")

file(GLOB CRUNCHMATH_INCLUDE_FILES
    "include/CrunchMath.h")

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "src" FILES ${CRUNCHMATH_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "include" FILES ${CRUNCHMATH_INCLUDE_FILES})
add_library(CrunchMath STATIC ${CRUNCHMATH_SOURCE_FILES} ${CRUNCHMATH_INCLUDE_FILES})
target_include_directories(CrunchMath PUBLIC include/)