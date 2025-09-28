set(LIBRARY_SRCS
  ellipses.cpp
  utilities.cpp
  ellipse_contour.cpp
  fitting.cpp
  ellipse_stats.cpp
)

add_library(EllipsesDetection ${LIBRARY_SRCS})

target_link_libraries(EllipsesDetection
  PUBLIC
    OpenCV::core
    OpenCV::imgproc
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
    OpenCV::imgcodecs
    OpenCV::calib3d
)
