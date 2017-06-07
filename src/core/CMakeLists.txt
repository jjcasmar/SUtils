set(SUTILS_SOURCES
    profilertimer.cpp
    openglwindow.cpp
    mainwindow.cpp
    simulationwindow.cpp
    simulator.cpp
)

set(SUTILS_PUBLIC_HEADERS
    profilertimer.h
    openglwindow.h
    mainwindow.h
    simulationwindow.h
    simulator.h
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SUTILS_NAMESPACE SUtils)
set(SUTILS_INCLUDE_NAME SUtils)

set(SUTILS_LINK_LIBRARIES
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::3DCore
    Qt5::3DRender
    Qt5::3DExtras)

common_library(SUtils)
