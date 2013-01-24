INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set header files.
SET(METAPOD_SIMPLE_ARM_MODEL_HEADERS
  include/metapod/models/simple_arm/simple_arm.hh
  include/metapod/models/simple_arm/config.hh
  )

QI_CREATE_LIB(metapod_simple_arm SHARED
  include/metapod/models/simple_arm/simple_arm.cc
  ${METAPOD_SIMPLE_ARM_MODEL_HEADERS})
QI_USE_LIB(metapod_simple_arm metapod)
QI_STAGE_LIB(metapod_simple_arm)
