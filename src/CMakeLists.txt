set(LIBRARY_SRCS
  ellipses/ellipses.cpp
  ellipses/utilities.cpp
  ellipses/ellipse_contour.cpp
  ellipses/fitting.cpp
  ellipses/ellipse_stats.cpp

  io/save_path.cpp
  io/debug.cpp

  detection/thresholds.cpp
  detection/detection_parameters.cpp
  detection/calibration_data.cpp
  detection/board.cpp
  detection/symmetries.cpp
  detection/detection.cpp
  detection/debug.cpp
  detection/board_drawing.cpp

  detection/identification/ordering.cpp
  detection/identification/identification.cpp
  detection/identification/filtering.cpp
)

add_library(EllipsesDetection ${LIBRARY_SRCS})

target_link_libraries(EllipsesDetection
  PUBLIC
    OpenCV::core
    OpenCV::imgproc
    OpenCV::imgcodecs
    Eigen3::Eigen
)

target_include_directories(EllipsesDetection
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)


add_executable(EllipsesDetectionTest
    main.cpp
)

target_link_libraries(EllipsesDetectionTest
  PUBLIC
    EllipsesDetection
    OpenCV::calib3d
)
