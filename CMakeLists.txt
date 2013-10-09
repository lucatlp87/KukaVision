cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(KukaVision)


find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${CURRENT_PROJECT_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(HDF5)
include_directories(${FLANN_INCLUDE_DIRS})
include_directories(${HDF5_INCLUDE_DIR})


add_executable (KukaVision 	main.cpp 
							TrainingStage.cpp
							NewSceneStage.cpp
							KukaVisionStage.cpp
							MaintenanceStage.cpp
							KinectAcquisition.cpp
							CloudProcessing.cpp
							OURCVFHEstimation.cpp
							DBManagement.cpp
							SceneManagement.cpp
							Object.cpp)
				
target_link_libraries (KukaVision ${PCL_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

