cmake_minimum_required(VERSION 3.12)
project(DynaSLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
  # SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_BINARY_DIR}/conan/")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/conan/")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

add_definitions("-std=c++11")

if (NOT EXISTS "${CMAKE_BINARY_DIR}/conan/")
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/conan/")
endif()
if (NOT EXISTS "${CMAKE_BINARY_DIR}/conan/conan.cmake")
  message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
  file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/master/conan.cmake"
                "${CMAKE_BINARY_DIR}/conan/conan.cmake")
endif()
include(${CMAKE_BINARY_DIR}/conan/conan.cmake)

conan_cmake_configure(
  REQUIRES
  opencv/3.4.17
  boost/1.72.0
  zlib/1.2.12
  GENERATORS
  cmake_find_package_multi
  )

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(conan_build "Debug")
else()
  set(conan_build "Release")
endif()

conan_cmake_autodetect(settings BUILD_TYPE ${conan_build})

conan_cmake_install(
  PATH_OR_REFERENCE .
  INSTALL_FOLDER ${CMAKE_BINARY_DIR}/conan
  BUILD missing
  REMOTE conancenter
  SETTINGS ${settings}
  )

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O0 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


message("PROJECT_SOURCE_DIR: " ${OpenCV_DIR})
find_package(OpenCV 2.4.11 QUIET)
if(NOT OpenCV_FOUND)
    message("OpenCV > 2.4.11 not found.")
    find_package(OpenCV 3.0 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 3.0 not found.")
    endif()
endif()

#find_package(OpenCV 3.0 QUIET)
#if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.3 QUIET)
#   if(NOT OpenCV_FOUND)
#      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#   endif()
#endif()

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Test REQUIRED)

find_package(PythonLibs 2.7 EXACT REQUIRED)
if (NOT PythonLibs_FOUND)
    message(FATAL_ERROR "PYTHON LIBS not found.")
else()
    message("PYTHON LIBS were found!")
    message("PYTHON LIBS DIRECTORY: " ${PYTHON_LIBRARY})
endif()

find_package(Boost REQUIRED)
if(Boost_FOUND)
    message("Boost was found!")
    message("Boost Headers DIRECTORY: " ${Boost_INCLUDE_DIRS})
    message("Boost LIBS DIRECTORY: " ${Boost_LIBRARY_DIRS})
    message("Found Libraries: " ${Boost_LIBRARIES})
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

list(APPEND PYTHON_INCLUDE_DIR "$ENV{HOME}/.local/lib/python2.7/site-packages/numpy/core/include/numpy")

#/usr/include/python2.7/
#/home/pecs/.local/lib/python2.7/site-packages/numpy/core/include/numpy

function(compile targetName cppFiles)

  add_executable(${targetName}
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/PnPsolver.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Initializer.cc
    src/Viewer.cc
    src/Conversion.cc
    src/MaskNet.cc
    src/Geometry.cc
    ${cppFiles})

  target_include_directories(${targetName} PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    )

  target_include_directories(${targetName} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIR}
    )

  target_link_libraries(${targetName} PUBLIC
    opencv::opencv_calib3d
    opencv::opencv_core
    opencv::opencv_features2d
    opencv::opencv_highgui
    opencv::opencv_imgproc
    opencv::opencv_imgcodecs
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    DBoW2
    g2o
    ${PYTHON_INCLUDE_DIR}
    /usr/lib/x86_64-linux-gnu/libpython2.7.so
    Boost::boost
    Boost::thread
    )

  set_target_properties(${targetName}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )

endfunction()

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)


# Build examples

compile(rgbd_tum Examples/RGB-D/rgbd_tum.cc)

compile(stereo_kitti Examples/Stereo/stereo_kitti.cc)

compile(mono_tum Examples/Monocular/mono_tum.cc)

compile(mono_kitti Examples/Monocular/mono_kitti.cc)

if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt")
  if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz")
    MESSAGE(FATAL_ERROR "ORBvoc.txt.tar.gz not found.")
  endif()
  message(STATUS "Uncompressing ORBvoc.txt.tar.gz...")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary
    )
  message(STATUS "Uncompression complete.")
endif()

# Define the path to the desired compile_commands.json file
set(COMPILE_COMMANDS_FILE "${CMAKE_BINARY_DIR}/compile_commands.json")

set(EXPECTED_SYMLINK "${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  file(REMOVE ${EXPECTED_SYMLINK})
endif()

if (NOT EXISTS "${EXPECTED_SYMLINK}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${COMPILE_COMMANDS_FILE} ${EXPECTED_SYMLINK}
    )
endif()

