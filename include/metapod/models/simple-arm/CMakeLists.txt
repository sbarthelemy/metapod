# Generate config.hh header.
SET(LIBRARY_NAME "SIMPLE_ARM")
SET(EXPORT_SYMBOL "simple_arm_EXPORTS")
CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/cmake/config.hh.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/config.hh
  @ONLY
  )

# Set header files.
SET(${PROJECT_NAME}_SIMPLE_ARM_MODEL_HEADERS
  body.hh
  joint.hh
  robot.hh
  ${CMAKE_CURRENT_BINARY_DIR}/config.hh
  )

# Set header files installation prefix.
INSTALL(FILES ${${PROJECT_NAME}_SIMPLE_ARM_MODEL_HEADERS}
  DESTINATION include/${PROJECT_NAME}/models/simple-arm
  )

ADD_LIBRARY(simple_arm SHARED
  simple_arm.cc
  ${${PROJECT_NAME}_SIMPLE_ARM_MODEL_HEADERS})
