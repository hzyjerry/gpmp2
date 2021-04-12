cmake_minimum_required(VERSION 2.6)
enable_testing()
project(gpmp2 CXX C)

# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

# version indicator
set(GPMP2_VERSION_MAJOR 0)
set(GPMP2_VERSION_MINOR 2)
set(GPMP2_VERSION_PATCH 0)
set(gpmp2_VERSION_STRING "${GPMP2_VERSION_MAJOR}.${GPMP2_VERSION_MINOR}.${GPMP2_VERSION_PATCH}")


# option: whether turn on Matlab toolbox
option(GPMP2_BUILD_STATIC_LIBRARY "whether build static library" OFF)
option(GPMP2_BUILD_MATLAB_TOOLBOX "whether build matlab toolbox, need shared lib" OFF)

if(GPMP2_BUILD_STATIC_LIBRARY AND GPMP2_BUILD_MATLAB_TOOLBOX)
  message(FATAL_ERROR "matlab toolbox needs shared lib")
endif()


# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)


# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(${Boost_INCLUDE_DIR})


# Generate and install config and dllexport files
configure_file("gpmp2/config.h.in" "gpmp2/config.h")
list(APPEND gpmp2_srcs "${PROJECT_BINARY_DIR}/gpmp2/config.h")
include_directories(BEFORE ${PROJECT_BINARY_DIR}) # So we can include generated config header files
install(FILES "${PROJECT_BINARY_DIR}/gpmp2/config.h" DESTINATION include/gpmp2)


# include current source folder, at the very beginning
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

# Process source subdirs
add_subdirectory(gpmp2)

# Wrapping to MATLAB
if(GPMP2_BUILD_MATLAB_TOOLBOX)
  # wrap
  include(GtsamMatlabWrap)
  wrap_and_install_library(gpmp2.h ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}" "")
  
  # install matlab functions and scripts
  add_subdirectory(matlab)
endif()


# Install config and export files
GtsamMakeConfigFile(gpmp2)
export(TARGETS ${GPMP2_EXPORTED_TARGETS} FILE gpmp2-exports.cmake)





# option: whether turn on Matlab toolbox
option(GPMP2_BUILD_PYTHON "whether build python toolbox" OFF)

# Fix cmake "NOT" "TRUE" conditional policy
if(POLICY CMP0012)
  cmake_policy(SET CMP0012 NEW)
endif()

if(GPMP2_BUILD_PYTHON)
	# ##############################################################################
	# * 6. Find the gtwrap package (either installed via GTSAM or as a standalone
	#   package) The cmake code for wrapping is included automatically.

	find_package(gtwrap REQUIRED)
	
	# ##############################################################################
	# * 7. Set the python version Load the default Python version strings and
	#   variables.
	# set(WRAP_PYTHON_VERSION "3.8.6")
	set(WRAP_PYTHON_VERSION "Default")
	message(STATUS "Outer wrap python version ${WRAP_PYTHON_VERSION}")

	gtwrap_get_python_version(${WRAP_PYTHON_VERSION})

	# ##############################################################################
	# * 8. Configure all the files for the Python module. Set the version string,
	#   needed for generating setup.py
	set(GTSAM_VERSION_STRING 0.0.1)

	# We use this as a convenience variable. It points to `python/` in this case.
	set(GTSAM_MODULE_PATH ${PROJECT_BINARY_DIR}/python)

	# Copy over the setup.py.in file. This will become `python/setup.py`.
	configure_file(${PROJECT_SOURCE_DIR}/python/wrapper/setup.py.in
	               ${GTSAM_MODULE_PATH}/setup.py)

	# Copy over the __init__.py file. This is now
	# `python/gtsam_example/__init__.py`.
	configure_file(${PROJECT_SOURCE_DIR}/python/wrapper/__init__.py.in
	               ${GTSAM_MODULE_PATH}/${PROJECT_NAME}/__init__.py)

	# Copy over the pybind .tpl file. This is now `build/gtsam_example.tpl`.
	configure_file(${PROJECT_SOURCE_DIR}/python/wrapper/pybind_wrapper.tpl.example
	               ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl)

	# Copy over C++ headers to use for wrapping
	file(COPY "${PROJECT_SOURCE_DIR}/python/wrapper/preamble.h"
	     DESTINATION "${PROJECT_BINARY_DIR}/python")

	file(COPY "${PROJECT_SOURCE_DIR}/python/wrapper/specializations.h"
	     DESTINATION "${PROJECT_BINARY_DIR}/python")

	# We print out our configuration for an easy visual check
	message("========== Configuration Options ==========")
	message(STATUS "Project:                  ${PROJECT_NAME}")
	message(STATUS "Python Version:           ${WRAP_PYTHON_VERSION}")
	message(STATUS "Python Module Path:       ${GTSAM_MODULE_PATH}")
	message("===========================================")


	# ##############################################################################
	# * 9. Build Pybind wrapper This is where the crux of the wrapping happens.

	# Ignoring the non-concrete types (type aliases and typedefs). We don't have any
	# in our current project so this is left as empty.
	set(ignore "")

	# This is the main function that generates the cpp file which Pybind11 will use.
	pybind_wrap(
	  ${PROJECT_NAME}_py # target
	  ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}.i # interface header file
	                                          # (gtsam_example.i in the root
	                                          # directory)
	  "${PROJECT_NAME}.cpp" # the generated cpp (gtsam_example.cpp)
	  "${PROJECT_NAME}" # module_name (gtsam_example)
	  "${PROJECT_NAME}" # top namespace in the cpp file (gtsam_example)
	  "${ignore}" # ignore classes
	  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl
	  ${PROJECT_NAME} # libs
	  "${PROJECT_NAME}" # dependencies, we need the library built in step 6 as the
	                    # minimum.
	  ON # we are using boost::shared_ptr not std
	)

	# We define where we wish to save the wrapped .so file once we run `make`. It
	# will be stored in `build/gtsam_example/gtsam_example` to conform to standard
	# python module structure.
	set_target_properties(
	  ${PROJECT_NAME}_py
	  PROPERTIES OUTPUT_NAME "${PROJECT_NAME}"
	             LIBRARY_OUTPUT_DIRECTORY "${GTSAM_MODULE_PATH}/${PROJECT_NAME}"
	             DEBUG_POSTFIX "" # Otherwise you will have a wrong name
	             RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
	)

	# ##############################################################################
	# * 10. Python install command Finally we add a custom make command which we can
	#   invoke to install the generated module. Simply type `make python-install`
	#   and we can now access the wrapped module as an installed library.
	add_custom_target(
	  python-install
	  COMMAND ${PYTHON_EXECUTABLE} ${GTSAM_MODULE_PATH}/setup.py install
	  DEPENDS ${PROJECT_NAME}_py
	  WORKING_DIRECTORY ${GTSAM_MODULE_PATH})

	message( ${GTSAM_MODULE_PATH} )
endif()