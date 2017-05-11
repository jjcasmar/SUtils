set(SOURCES
    main.cpp)

add_executable(ProfilerTimer ${SOURCES})
target_link_libraries(ProfilerTimer SUtils)
