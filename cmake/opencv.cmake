include_guard()
include(FetchContent)

FetchContent_Declare(
    opencv
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG 4.5.3
)


FetchContent_Declare(
    opencv_contrib
    GIT_REPOSITORY https://github.com/opencv/opencv_contrib.git
    GIT_TAG 4.5.3
)


FetchContent_GetProperties(opencv)
if(NOT opencv_POPULATED)
    FetchContent_Populate(opencv)
    FetchContent_Populate(opencv_contrib)

    set(WITH_EIGEN ON)
    set(WITH_OPENEXR OFF)
    set(WITH_ITT OFF)
    set(BUILD_TESTS OFF)
    set(BUILD_PERF_TESTS OFF)
    SET(BUILD_TIFF ON)
    set(OPENCV_EXTRA_MODULES_PATH ${opencv_contrib_SOURCE_DIR}/modules CACHE PATH "Where additional OpenCV modules lies")

    add_subdirectory(${opencv_SOURCE_DIR} ${opencv_BINARY_DIR} EXCLUDE_FROM_ALL)

    foreach(the_module ${OPENCV_MODULES_BUILD})
        if(TARGET ${the_module})
            set_target_properties(
                ${the_module}
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES
                        $<TARGET_PROPERTY:${the_module},INCLUDE_DIRECTORIES>
            )
            string(REPLACE "opencv_" "" module_name ${the_module})
            add_library(OpenCV::${module_name} ALIAS ${the_module})
        endif()
    endforeach()

    get_directory_property(JPEG_LIBRARY DIRECTORY ${opencv_SOURCE_DIR} DEFINITION JPEG_LIBRARY)
    if(TARGET ${JPEG_LIBRARY})
        set_target_properties(${JPEG_LIBRARY}
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES
                    $<TARGET_PROPERTY:${JPEG_LIBRARY},INCLUDE_DIRECTORIES>
        )
        set(CMAKE_DISABLE_FIND_PACKAGE_JPEG TRUE)
        set(JPEG_FOUND TRUE)
        add_library(JPEG::JPEG ALIAS ${JPEG_LIBRARY})
    endif()

    set(CMAKE_DISABLE_FIND_PACKAGE_OpenCV TRUE)
    set(OpenCV_FOUND TRUE)

endif()

