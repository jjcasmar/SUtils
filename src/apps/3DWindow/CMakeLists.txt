set(SOURCES
    main.cpp
    )
set(HEADERS
    hwindow.h)

add_executable(3DWindow ${SOURCES} ${HEADERS})
target_link_libraries(3DWindow
    SUtils
    Qt5::Widgets)
