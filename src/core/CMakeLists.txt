set(SOURCES
    profilertimer.cpp
    openglwindow.cpp
    trackballcameracontroller.cpp
    mainwindow.cpp
    simulationwindow.cpp
    simulator.cpp
    #boundingvolumesaspect.cpp
)

set(HEADERS
    profilertimer.h
    openglwindow.h
    trackballcameracontroller.h
    trackballhelper.h
    mainwindow.h
    simulationwindow.h
    simulator.h
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DVCG_USE_EIGEN)
add_definitions(-DEIGEN2_SUPPORT)

#add_subdirectory(vcglib)

add_library(QUtils SHARED ${SOURCES})
target_link_libraries(QUtils
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::3DCore
    Qt5::3DRender
    Qt5::3DExtras)

INSTALL(TARGETS QUtils
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(FILES ${HEADERS}
        DESTINATION "${CMAKE_INSTALL_PREFIX}/include/QUtils"
        )
