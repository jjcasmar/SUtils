set(SOURCES main.cpp)
set(HEADERS)

add_executable(3DWindow ${SOURCES} ${HEADERS})
target_link_libraries(3DWindow
    QUtils
    Qt5::Widgets)
