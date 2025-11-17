find_package(OpenMP REQUIRED)
find_package(colmap REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(OpenCV REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} 
	exe/colmap_extension.cc
	exe/calibration.cc
	estimators/bundle_adjustment.cc
	calibration/calib_rig_system.cc)
target_link_libraries(${PROJECT_NAME} PRIVATE colmap::colmap calibration_utils ${OpenCV_LIBS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

# Installation: place executable into ${CMAKE_INSTALL_BINDIR} (usually bin)
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	COMPONENT runtime)

# Install public headers (if any) so downstream packages can find them
install(FILES exe/sfm.h
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/exe)

# Install controller headers if they are intended to be public
install(FILES controllers/bundle_adjustment.h
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/controllers)
