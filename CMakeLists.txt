cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_C_COMPILER "/usr/bin/gcc")

project(icar)
set(CMAKE_CXX_STANDARD 17)
# 调试关闭 -flto -O2
set(CMAKE_CXX_FLAGS "-Og -g -mcpu=native")
set(CMAKE_INSTALL_PREFIX /root/nfs)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#---------------------------------------------------------------------

include_directories(${PROJECT_SOURCE_DIR}/include /usr/local/include)
#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})
#---------------------------------------------------------------------
#       [ Target  : Binary ] ==> [ Tests ]
#---------------------------------------------------------------------
add_subdirectory(${PROJECT_SOURCE_DIR}/spdlog)

# set(TEST_NAME hsv)
# add_executable(${TEST_NAME} ${PROJECT_SOURCE_DIR}/tests/hsv.cpp)
# target_precompile_headers(${TEST_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/json.hpp ${PROJECT_SOURCE_DIR}/include/controlcenter.hpp)
# target_link_libraries(
#         ${TEST_NAME}
#         ${PPNC_LIBRARIES}
#         ${ONNX_LIBRARIES}
#         ${OpenCV_LIBS}
#         ${SERIAL_LIBRARIES}
#         spdlog::spdlog
#         pthread
# )

add_executable(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_precompile_headers(${CMAKE_PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}/include/json.hpp
        ${PROJECT_SOURCE_DIR}/include/controlcenter.hpp
        ${PROJECT_SOURCE_DIR}/include/detection.hpp
        ${PROJECT_SOURCE_DIR}/include/capture.hpp
        ${PROJECT_SOURCE_DIR}/include/motion.hpp
        ${PROJECT_SOURCE_DIR}/include/preprocess.hpp
        ${PROJECT_SOURCE_DIR}/include/common.hpp
)
target_link_libraries(
        ${CMAKE_PROJECT_NAME}
        ${PPNC_LIBRARIES}
        ${ONNX_LIBRARIES}
        ${OpenCV_LIBS}
        ${SERIAL_LIBRARIES}
        spdlog::spdlog
        pthread
)

#---------------------------------------------------------------------
#       [ Install  : Binary ] ==> [ NFS/SAMBA ]
#---------------------------------------------------------------------
install(TARGETS ${TEST_NAME} ${PROJECT_NAME}
        RUNTIME DESTINATION .
        LIBRARY DESTINATION lib
)
install(FILES
        # PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_WRITE WORLD_EXECUTE
        # 此配置文件包含模型路径，自行选择是否使用其中的路径
        # ${PROJECT_SOURCE_DIR}/config/config.json
        # ppnc的推理库必须使用这两个配置文件，此文件设置了nms和deploy_paddle动态库路径
        # 设置不正确可能出现runtime error, type mismatch
        ${PROJECT_SOURCE_DIR}/config/config_ppncnms.json
        ${PROJECT_SOURCE_DIR}/config/config_ppncnna.json
        # 摄像头标定参数，Opencv使用
        ${PROJECT_SOURCE_DIR}/res/calibration/calibration.xml

        DESTINATION ${CMAKE_INSTALL_PREFIX}/config
)
# 官方模型文件，包含ppnc需要的nms和deploy_paddle，以及模型权重post.onnx
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/res/model DESTINATION ${CMAKE_INSTALL_PREFIX})
