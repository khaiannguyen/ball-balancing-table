cmake_minimum_required(VERSION 3.16)
project(balance_ball CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_compile_options(-Wall -Wextra)

include_directories(${CMAKE_SOURCE_DIR}/include)

find_package(Threads REQUIRED)

add_library(state_transport
    src/system_state.cpp
    src/can_transport.cpp
    src/task_can_rx.cpp
    src/task_can_tx.cpp
)
target_link_libraries(state_transport PUBLIC Threads::Threads)

add_executable(test_seqlock tests/test_seqlock.cpp)
target_link_libraries(test_seqlock PRIVATE Threads::Threads)

add_executable(can_bringup_test tools/can_bringup_test.cpp)
target_link_libraries(can_bringup_test PRIVATE state_transport)

add_executable(j3_test src/main_j3_test.cpp)
target_link_libraries(j3_test PRIVATE state_transport)

add_executable(j4_test src/main_j4_test.cpp)
target_link_libraries(j4_test PRIVATE state_transport)

# --- J5: Camera pipeline (GStreamer + OpenCV) ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
find_package(OpenCV REQUIRED)

add_library(camera_pipeline
    src/camera_pipeline.cpp
)
target_include_directories(camera_pipeline PUBLIC
    include
    ${GST_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(camera_pipeline PUBLIC
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    ${OpenCV_LIBS}
)
target_compile_options(camera_pipeline PUBLIC
    ${GST_CFLAGS_OTHER}
    ${GST_APP_CFLAGS_OTHER}
)

add_executable(camera_preview tools/camera_preview.cpp)
target_link_libraries(camera_preview PRIVATE camera_pipeline)

add_executable(camera_pipeline_test tools/camera_pipeline_test.cpp)
target_link_libraries(camera_pipeline_test PRIVATE camera_pipeline)

# --- J6: Camera capture ---

add_executable(j6_camera_test
    src/main_j6_camera_test.cpp
    src/task_camera_capture.cpp
    src/camera_pipeline.cpp
)
target_link_libraries(j6_camera_test PRIVATE ${OpenCV_LIBS} ${GST_LIBRARIES} ${GST_APP_LIBRARIES})
target_include_directories(j6_camera_test PRIVATE include ${GST_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})

# --- J6: Intrinsic_calib_tool ---

add_executable(intrinsic_calib_tool tools/intrinsic_calib_tool.cpp src/task_camera_capture.cpp src/camera_pipeline.cpp)
target_link_libraries(intrinsic_calib_tool PRIVATE ${OpenCV_LIBS} ${GST_LIBRARIES} ${GST_APP_LIBRARIES})
target_include_directories(intrinsic_calib_tool PRIVATE include ${GST_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})


# --- J6: Extrinsic_calib_tool ---

add_executable(extrinsic_calib_tool tools/extrinsic_calib_tool.cpp src/task_camera_capture.cpp src/camera_pipeline.cpp)
target_link_libraries(extrinsic_calib_tool PRIVATE ${OpenCV_LIBS} ${GST_LIBRARIES} ${GST_APP_LIBRARIES})
target_include_directories(extrinsic_calib_tool PRIVATE include ${GST_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})


# --- J6: Sanity_check_tool ---

add_executable(sanity_check_tool tools/sanity_check_tool.cpp src/task_camera_capture.cpp src/camera_pipeline.cpp)
target_link_libraries(sanity_check_tool PRIVATE ${OpenCV_LIBS} ${GST_LIBRARIES} ${GST_APP_LIBRARIES})
target_include_directories(sanity_check_tool PRIVATE include ${GST_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})

# --- J6: Ball_detection_test ---

add_executable(j6_ball_test
    src/main_j6_ball_test.cpp
    src/task_ball_detect.cpp
    src/task_camera_capture.cpp
    src/camera_pipeline.cpp
    src/system_state.cpp
)
target_link_libraries(j6_ball_test PRIVATE ${OpenCV_LIBS} ${GST_LIBRARIES} ${GST_APP_LIBRARIES} Threads::Threads)
target_include_directories(j6_ball_test PRIVATE include ${GST_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})

# --- J7: Control loop (PID) + full system (camera + CAN) ---
# Cần cả camera/OpenCV/GStreamer (ball detect) LẪN CAN (state_transport:
# system_state, can_transport, task_can_rx, task_can_tx). state_transport
# đã build sẵn system_state.cpp/can_transport.cpp/task_can_rx.cpp/
# task_can_tx.cpp nên KHÔNG compile lại các file đó ở đây — chỉ thêm phần
# camera + ball_detect + control_loop + main còn thiếu.
# pid_controller.hpp là header-only, không cần thêm .cpp riêng.

add_executable(j7_control_test
    src/main_j7_control_test.cpp
    src/task_control_loop.cpp
    src/task_ball_detect.cpp
    src/task_camera_capture.cpp
    src/camera_pipeline.cpp
)
target_link_libraries(j7_control_test PRIVATE
    state_transport
    ${OpenCV_LIBS}
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    Threads::Threads
)
target_include_directories(j7_control_test PRIVATE
    include
    ${GST_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
)

# --- J8: Watchdog ---

add_executable(balance_ball_main
    src/main.cpp
    src/task_watchdog.cpp
    src/task_control_loop.cpp
    src/task_ball_detect.cpp
    src/task_camera_capture.cpp
    src/camera_pipeline.cpp
    src/task_video_record.cpp
)
target_link_libraries(balance_ball_main PRIVATE
    state_transport
    ${OpenCV_LIBS}
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    Threads::Threads
)
target_include_directories(balance_ball_main PRIVATE
    include
    ${GST_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
)