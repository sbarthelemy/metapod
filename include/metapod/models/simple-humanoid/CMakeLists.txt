# Generate config.hh using the same template as JRL-cmake
SET(LIBRARY_NAME "SIMPLE_HUMANOID")
SET(EXPORT_SYMBOL "simple_humanoid_EXPORTS")
CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/include/metapod/config.in.hh
  ${CMAKE_CURRENT_BINARY_DIR}/config.hh
  @ONLY)
SET(${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS
  body.hh
  joint.hh
  robot.hh
  ${CMAKE_CURRENT_BINARY_DIR}/config.hh)

QI_CREATE_LIB(simple_humanoid SHARED
    simple_humanoid.cc
    ${${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS})
QI_USE_LIB(simple_humanoid metapod EIGEN3)
QI_STAGE_LIB(simple_humanoid)
