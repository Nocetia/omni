add_library(app INTERFACE)

target_compile_features(app INTERFACE cxx_std_17)

target_link_libraries(app INTERFACE
    modules
)

target_include_directories(app INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/robot"
)

target_sources(app INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/app.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/control.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/robot/tasks/TaskControl.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/tasks/TaskDebug.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/tasks/TaskUI.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/tasks/TaskVision.cpp"
)
