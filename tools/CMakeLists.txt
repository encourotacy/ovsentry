cmake_minimum_required(VERSION 3.16.3)

add_library(tools OBJECT
    exiter.cpp
    extended_kalman_filter.cpp
    img_tools.cpp
    logger.cpp
    math_tools.cpp
    plotter.cpp
    ransac_sine_fitter.cpp
    recorder.cpp
    trajectory.cpp
)