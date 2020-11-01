set(CRUNCHMATH_SOURCE_FILES
	src/AABB.cpp
	src/Mat4x4.cpp
	src/Math_Util.cpp
	src/OBB.cpp
	src/Quaternion.cpp
	src/Sphere.cpp
	src/Vec3.cpp
	src/Vec4.cpp
	src/BVH/BVHSphere.cpp)

set(CRUNCHMATH_HEADER_FILES
	CrunchMath.h
	src/AABB.h
	src/GeometricUtility.h
	src/Mat4x4.h
	src/Math_Util.h
	src/OBB.h
	src/Quaternion.h
	src/Sphere.h
	src/Vec3.h
	src/Vec4.h
	src/BVH/BVHDS.hpp
	src/BVH/BVHSphere.h
	)

add_library(CrunchMath STATIC ${CRUNCHMATH_SOURCE_FILES} ${CRUNCHMATH_HEADER_FILES})
target_include_directories(CrunchMath PUBLIC /)