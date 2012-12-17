# Generate config.hh header.
SET(LIBRARY_NAME "SIMPLE_ARM")
SET(EXPORT_SYMBOL "simple_arm_EXPORTS")
CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/include/metapod/config.in.hh
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

QI_CREATE_LIB(simple_arm SHARED
    simple_arm.cc
    ${${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS})
QI_USE_LIB(simple_arm metapod EIGEN3)
QI_STAGE_LIB(simple_arm)
