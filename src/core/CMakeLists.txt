set(SOURCES
    profilertimer.cpp
    openglwindow.cpp
    trackballcameracontroller.cpp
)

set(HEADERS
    profilertimer.cpp
    openglwindow.h
    trackballcameracontroller.h
    trackballhelper.h
)

add_definitions(-DVCG_USE_EIGEN)
add_definitions(-DEIGEN2_SUPPORT)

#add_subdirectory(vcglib)

add_library(QUtils SHARED ${SOURCES} ${HEADERS})
target_link_libraries(QUtils
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::3DCore
    Qt5::3DRender
    Qt5::3DExtras)


