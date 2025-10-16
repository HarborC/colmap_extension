cmake_minimum_required(VERSION 3.16)
project(colmap_extension LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE Release)

include(GNUInstallDirs)

include_directories(src)

add_subdirectory(src/colmap_extension)

# Allow users to run: cmake --install build --prefix <prefix>

# Generate uninstall target: creates a cmake script from template that reads
# install_manifest.txt and removes installed files.
if(NOT TARGET uninstall)
	configure_file(
		"${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
		"${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
		IMMEDIATE @ONLY)

	add_custom_target(uninstall
		COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake
		COMMENT "Uninstall project from CMAKE_INSTALL_PREFIX")
endif()
