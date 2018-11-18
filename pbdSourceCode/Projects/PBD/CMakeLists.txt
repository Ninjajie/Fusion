# Find eigen in system directory and treat it as an interface library
find_package(Eigen3 REQUIRED)
add_library(eigen INTERFACE IMPORTED)

# Generate the local Makefile
if(CREATE_LOCAL_MAKEFILES)
  file(RELATIVE_PATH CISPBA_RELATIVE_PATH ${CISPBA_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(${CISPBA_SOURCE_DIR}/Scripts/Makefile.in ${CMAKE_CURRENT_SOURCE_DIR}/Makefile @ONLY)
endif()

# Configure this project
add_cispba_executable(PBD main.cpp PBDSimulator.h PBDSimulator.cpp Sphere.h Sphere.cpp Constraint.h DistanceConstraint.h DistanceConstraint.cpp BendingConstraint.h BendingConstraint.cpp CollisionConstraint.h CollisionConstraint.cpp PointConstraint.h PointConstraint.cpp GroundConstraint.h GroundConstraint.cpp TriangleFace.h TriangleMesh.h TriangleMesh.cpp OBJGenerator.h OBJGenerator.cpp Edge.h)
target_include_directories(PBD SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(PBD eigen)
target_link_libraries(PBD partio)
