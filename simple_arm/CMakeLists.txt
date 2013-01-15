INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/include)

# Generate config.hh header.
GENERATE_CONFIG_HEADER(
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/models/simple_arm/config.hh
  metapod_simple_arm
  )

# Set header files.
SET(METAPOD_SIMPLE_ARM_MODEL_HEADERS
  include/metapod/models/simple_arm/body.hh
  include/metapod/models/simple_arm/joint.hh
  include/metapod/models/simple_arm/robot.hh
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/models/simple_arm/config.hh
  )

QI_CREATE_LIB(metapod_simple_arm SHARED
    src/simple_arm.cc
    ${METAPOD_SIMPLE_ARM_MODEL_HEADERS})
QI_USE_LIB(metapod_simple_arm metapod EIGEN3)
QI_STAGE_LIB(metapod_simple_arm)
