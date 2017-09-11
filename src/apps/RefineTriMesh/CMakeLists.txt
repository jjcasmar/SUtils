set(SOURCES main.cpp)
set(HEADERS)

add_executable(RefineTriMesh ${SOURCES} ${HEADERS})
target_link_libraries(RefineTriMesh
    SUtils
    )
