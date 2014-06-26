###############################################
# apps/holding
PROJECT(manip_app)


include_directories("${EIGEN3_INCLUDE_DIR}")
include_directories("${DS_INCLUDE_DIR}")
include_directories("${XERCES_INCLUDE_DIR}")
include_directories("${XEUMEULEU_INCLUDE_DIR}")
include_directories("${MANIP_APP_INCLUDE}")
include_directories("${MANIP_CORE}")
include_directories("${MANIP_CORE}/kinematic")
include_directories("./")

FILE(GLOB holding_srcs "*.cpp")
FILE(GLOB holding_hdrs "*.h")

set(SOURCES
MainSimulation.cpp
AutoIncline.cpp  FileHandler.h     MotionHandler.cpp      SplineTimeManager.h
TimerHandler.cpp      TimerHandler.h 
AutoIncline.h    IKSolver          MotionHandler.h        Timer.cpp
AutoRotate.cpp   InputHandler.cpp  PostureManager.cpp     Timer.h
AutoRotate.h     InputHandler.h    PostureManager.h       WorldParser
Camera.cpp       RootTrajectory.cpp     XboxMotion.cpp
Camera.h         ManipManager.cpp  RootTrajectory.h       XboxMotion.h
ManipManager.h    Simulation.cpp
MatrixDefs.cpp    Simulation.h
Draw             MatrixDefs.h      SplineTimeManager.cpp
WorldParser/WorldParser.cpp  WorldParser/WorldParserObj.cpp
WorldParser/WorldParser.h    WorldParser/WorldParserObj.h
IKSolver/IKSolver.cpp  IKSolver/IKSolver.h
Draw/DrawRobot.cpp           Draw/DrawTrajectory.cpp
Draw/DrawRobot.h             Draw/DrawTrajectory.h
Draw/DrawManager.cpp          Draw/DrawSamples.cpp         Draw/DrawTree.cpp
Draw/DrawManager.h            Draw/DrawSamples.h           Draw/DrawTree.h
Draw/DrawObstacle.cpp         Draw/DrawSpline.cpp          Draw/DrawWorld.cpp
Draw/DrawObstacle.h           Draw/DrawSpline.h            Draw/DrawWorld.h
Draw/DrawPostures.cpp         Draw/DrawSupportPolygon.cpp  Draw/glutfonts.h
Draw/DrawPostures.h           Draw/DrawSupportPolygon.h
)

include_directories("${MANIP_CORE}/API")
include_directories("${DS_INCLUDE_DIR}")

LINK_DIRECTORIES(${DS_LIBRARY_DIR})
LINK_DIRECTORIES(${XERCES_LIBRARY})
LINK_DIRECTORIES(${CMAKE_BINARY_DIR})

message(${DRAWSTUFF_LIBRARY})

ADD_EXECUTABLE(manip_app ${SOURCES})
if ( MSVC )
else ()
    TARGET_LINK_LIBRARIES(manip_app ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
	TARGET_LINK_LIBRARIES(manip_app ${DRAWSTUFF_LIBRARY})
    TARGET_LINK_LIBRARIES(manip_app ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
	TARGET_LINK_LIBRARIES(manip_app manipulability_core)
	TARGET_LINK_LIBRARIES(manip_app ${XERCES_LIBRARY})
endif ( MSVC )

SET_TARGET_PROPERTIES(manip_app PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
