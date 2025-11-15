find_package(Eigen3 REQUIRED) 
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(TARGET_NAME calibration_utils)

#Collect all sources recursively in this folder
file(GLOB_RECURSE CALIB_UTILS_SOURCES CONFIGURE_DEPENDS
	"${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
	"${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

#Create a library with all implementation files in this directory
add_library(${TARGET_NAME} SHARED ${CALIB_UTILS_SOURCES})
target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)
target_link_libraries(${TARGET_NAME} PUBLIC ${OpenCV_LIBS})

#Include this directory so headers like "aprilgrid/aprilgrid.h" resolve
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

#Installation
include(GNUInstallDirs) 
install(TARGETS ${TARGET_NAME} 
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} 
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} 
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} 
		COMPONENT runtime)

#Optionally install headers; install the full directory tree for convenience
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${TARGET_NAME} 
		FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
