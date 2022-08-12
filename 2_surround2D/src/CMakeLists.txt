add_library(surround2D SHARED 
    ParamSettings.cpp
    FisheysCameraModel.cpp
    CalibrateCamera.cpp
    BirdView.cpp
)

target_link_libraries(surround2D ${OpenCV_LIBS})
