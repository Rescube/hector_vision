cmake_minimum_required(VERSION 2.8.3)
project(hector_qrcode_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  tf
  nodelet
  hector_worldmodel_msgs)

set(ZBar0_DIR ${hector_qrcode_detection_SOURCE_DIR})
find_package(ZBar0 REQUIRED)

catkin_package(
  LIBRARIES qrcode_detection
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIR}
  ${ZBAR_INCLUDE_DIR}
)

add_library(qrcode_detection src/qrcode_detection.cpp)
target_link_libraries(qrcode_detection 
  ${ZBAR_LIBRARIES} ${catkin_LIBRARIES})

add_library(qrcode_detection_nodelet src/nodelet.cpp)
target_link_libraries(qrcode_detection_nodelet 
  qrcode_detection ${ZBAR_LIBRARIES} ${catkin_LIBRARIES})

add_executable(qrcode_detection_node src/node.cpp)
target_link_libraries(qrcode_detection_node 
  qrcode_detection ${ZBAR_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS qrcode_detection qrcode_detection_nodelet qrcode_detection_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
