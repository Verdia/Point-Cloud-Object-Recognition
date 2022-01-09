cmake_minimum_required(VERSION 3.0.2)
project(bismillahFix)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(HDF5 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  roscpp
  std_msgs
  sensor_msgs
  image_transport
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /(Fann/include)
  ${HDF5_INCLUDE_DIR}
)

link_directories(	
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_LIBRARY_DIRS}
  /(Fann/lib)
)


add_executable(pclCreateDatabase src/pclCreateDatabase.cpp)
target_link_libraries(pclCreateDatabase
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES})

add_executable(argc src/argc.cpp)
target_link_libraries(argc 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES})

add_executable(new_vfh src/new_vfh.cpp)
target_link_libraries(new_vfh 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES})

add_executable(new_correspondence src/new_correspondence.cpp)
target_link_libraries(new_correspondence 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES})

add_executable(new_correspondence_backup src/new_correspondence_backup.cpp)
target_link_libraries(new_correspondence_backup 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES})

add_executable(hypoVerification src/hypoVerification.cpp)
target_link_libraries(hypoVerification 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES})

add_executable(ism src/ism.cpp)
target_link_libraries(ism 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES})

add_executable(coba_iluminasi src/coba_iluminasi.cpp)
target_link_libraries(coba_iluminasi 
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBS})

add_executable(convert src/convert.cpp)
target_link_libraries(convert 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES})

add_executable(min_max_XYZ src/min_max_XYZ.cpp)
target_link_libraries(min_max_XYZ 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES})

add_executable(cvtColor src/cvtColor.cpp)
target_link_libraries(cvtColor 
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES})

add_executable(centroid src/centroid.cpp)
target_link_libraries(centroid 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} fann)

add_executable(main_trainning src/main_trainning.cpp)
target_link_libraries(main_trainning 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} fann)

add_executable(main_trainning_testing src/main_trainning_testing.cpp)
target_link_libraries(main_trainning_testing 
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} fann)

add_executable(pcl_write src/pcl_write.cpp)
target_link_libraries(pcl_write
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

add_executable(pcl_write_viewer src/pcl_write_viewer.cpp)
target_link_libraries(pcl_write_viewer
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

add_executable(sliderCV src/sliderCV.cpp)
target_link_libraries(sliderCV
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${OpenCV_LIBS}
)

add_executable(cobacoba src/cobacoba.cpp)
target_link_libraries(cobacoba
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${OpenCV_LIBS}
)

add_executable(normalEst src/normalEst.cpp)
target_link_libraries(normalEst
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${OpenCV_LIBS}
)

add_executable(cobaCapture src/cobaCapture.cpp)
target_link_libraries(cobaCapture
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

add_executable(cobaCaptureViewer src/cobaCaptureViewer.cpp)
target_link_libraries(cobaCaptureViewer
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

add_executable(cobaCaptureViewerRGBD src/cobaCaptureViewerRGBD.cpp)
target_link_libraries(cobaCaptureViewerRGBD
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

add_executable(sliderSub src/sliderSub.cpp)
target_link_libraries(sliderSub
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

add_executable(cobaCropPCL src/cobaCropPCL.cpp)
target_link_libraries(cobaCropPCL
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

add_executable(pclViewer src/pclViewer.cpp)
target_link_libraries(pclViewer
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

# add_executable(1._pclCapturer src/1._pclCapturer.cpp)
# target_link_libraries(1._pclCapturer
#  ${catkin_LIBRARIES} 
#  ${PCL_LIBRARIES} 
#  ${Boost_LIBRARIES} 
#  ${realsense2_LIBRARY}
#  ${OpenCV_LIBS}
# )

add_executable(2._sliderParameter src/2._sliderParameter.cpp)
target_link_libraries(2._sliderParameter
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

add_executable(3._pclObjectDetection src/3._pclObjectDetection.cpp)
target_link_libraries(3._pclObjectDetection
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

add_executable(5._Visualizer src/5._Visualizer.cpp)
target_link_libraries(5._Visualizer
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${Boost_LIBRARIES} 
 ${realsense2_LIBRARY}
 ${OpenCV_LIBS}
)

# add_executable(ikm_main_camera src/ikm_main_camera.cpp)
# target_link_libraries(ikm_main_camera
#  ${catkin_LIBRARIES} 
#  ${PCL_LIBRARIES} 
#  ${Boost_LIBRARIES} 
#  ${realsense2_LIBRARY}
#  ${OpenCV_LIBS}
# )

# add_executable(ikm_preprocesing_node src/ikm_preprocesing_node.cpp)
# target_link_libraries(ikm_preprocesing_node
#  ${catkin_LIBRARIES} 
#  ${PCL_LIBRARIES} 
#  ${Boost_LIBRARIES} 
#  ${realsense2_LIBRARY}
#  ${OpenCV_LIBS}
# )
# add_dependencies(ikm_preprocesing_node pclFix)

# add_executable(ikm_trackbar_node src/ikm_trackbar_node.cpp)
# target_link_libraries(ikm_trackbar_node
#  ${catkin_LIBRARIES} 
#  ${PCL_LIBRARIES} 
#  ${Boost_LIBRARIES} 
#  ${realsense2_LIBRARY}
#  ${OpenCV_LIBS}
# )
# add_dependencies(ikm_trackbar_node pclFix)