cmake_minimum_required(VERSION 2.6)
PROJECT(manipulability_core)

include_directories("${EIGEN3_INCLUDE_DIR}")
include_directories("./")
include_directories("${MANIP_APP_INCLUDE}")

if ( MSVC )
	SET(CMAKE_DEBUG_POSTFIX d)
endif ( MSVC )
	
set(SOURCES
    Exports.h 
    Pi.h
    API/IkConstraintHandlerI.h
    API/JointI.h
    API/RobotI.h
    API/WorldManagerI.cpp
    API/IKSolverI.cpp
    API/PostureManagerI.h
    API/SampleVisitorI.h
    API/WorldManagerI.h
    API/IKSolverI.h
    API/TreeI.h
    IK/ForceManipulabilityConstraint.cpp  
    IK/ForceManipulabilityConstraint.h
    IK/JointConstraint.h                  IK/JointConstraint.cpp   
    IK/JointLimitConstraint.h             IK/JointLimitConstraint.cpp
    IK/IkConstraintHandler.h              IK/IkConstraintHandler.cpp
    IK/ObstacleConstraint.h               IK/ObstacleConstraint.cpp
    IK/PartialDerivativeConstraint.h      IK/PartialDerivativeConstraint.cpp
    IK/IKSolver.h                         IK/IKSolver.cpp         
    IK/PostureConstraint.h                IK/PostureConstraint.cpp
    IK/JointConstraint.h                  IK/JointConstraint.cpp
    kinematic/Com.cpp             kinematic/Robot.h
    kinematic/Com.h               kinematic/RobotVisitor_ABC.cpp
    kinematic/ComVisitor_ABC.cpp  kinematic/RobotVisitor_ABC.h
    kinematic/ComVisitor_ABC.h    kinematic/SupportPolygon.cpp
    kinematic/Enums.h             kinematic/SupportPolygon.h
    kinematic/Jacobian.cpp        kinematic/SupportPolygonVisitor_ABC.cpp
    kinematic/Jacobian.h          kinematic/SupportPolygonVisitor_ABC.h
    kinematic/Joint.cpp           kinematic/Tree.cpp
    kinematic/Joint.h             kinematic/TreeFactory.cpp
    kinematic/Robot.cpp           kinematic/TreeFactory.h
    kinematic/RobotFactory.cpp    kinematic/Tree.h
    kinematic/RobotFactory.h
    posture/PostureCriteria_ABC.cpp
    posture/PostureCriteria_ABC.h
    posture/PostureCriteriaToeOffBoundary.cpp
    posture/PostureCriteriaToeOffBoundary.h
    posture/PostureCriteriaToeOffGait.cpp
    posture/PostureCriteriaToeOffGait.h
    posture/PostureCriteriaToeOffJointLimit.cpp
    posture/PostureCriteriaToeOffJointLimit.h
    posture/PostureCriteriaToeOnAlways.cpp
    posture/PostureCriteriaToeOnAlways.h
    posture/PostureCriteriaToeOnCOM.cpp
    posture/PostureCriteriaToeOnCOM.h
    posture/PostureCriteriaToeOnGait.cpp
    posture/PostureCriteriaToeOnGait.h
    posture/PostureManagerImpl.cpp
    posture/PostureManagerImpl.h
    posture/PostureSolver.cpp
    posture/PostureSolver.h
    posture/SpiderGait.cpp
    posture/SpiderGait.h
    posture/SpiderGaitI.h
    posture/Trajectory.cpp
    posture/Trajectory.h
    sampling/Sample.cpp           sampling/SampleGeneratorVisitor_ABC.cpp
    sampling/SampleGenerator.cpp  sampling/SampleGeneratorVisitor_ABC.h
    sampling/SampleGenerator.h    sampling/Sample.h
    sampling/filters/Filter_ABC.cpp
    sampling/filters/Filter_ABC.h
    sampling/filters/FilterDistance.cpp
    sampling/filters/FilterDistance.h
    sampling/filters/FilterDistanceObstacle.cpp
    sampling/filters/FilterDistanceObstacle.h
    Trajectory/TrajectoryHandler.cpp  Trajectory/TrajectoryHandler.h
    world/CollisionHandler_ABC.cpp     world/Obstacle.cpp
    world/CollisionHandler_ABC.h       world/Obstacle.h
    world/CollisionHandlerDefault.cpp  world/ObstacleVisitor_ABC.cpp
    world/CollisionHandlerDefault.h    world/ObstacleVisitor_ABC.h
    world/Intersection.cpp             world/World.cpp
    world/Intersection.h               world/World.h
)

add_library(manipulability_core ${SOURCES})

SET_TARGET_PROPERTIES(manipulability_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET_TARGET_PROPERTIES(manipulability_core PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

