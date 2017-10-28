set(SOURCES
    main.cpp)

add_executable(DeformationGradient ${SOURCES})
target_link_libraries(DeformationGradient SUtils)
