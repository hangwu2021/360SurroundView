add_library(myStitch SHARED
    homography.cpp
    seam_carving.cpp
)

target_link_libraries(myStitch ${OpenCV_LIBS})
