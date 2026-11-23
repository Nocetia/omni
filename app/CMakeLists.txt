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
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/remote.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/gimbal.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/axis.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/shooter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/control/chassis.cpp"

    "${CMAKE_CURRENT_SOURCE_DIR}/robot/Task/TaskGimbal.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/Task/TaskChassis.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/Task/TaskInput.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot/Service/ServiceVision.cpp"
)
