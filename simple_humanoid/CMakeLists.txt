INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/include)

# Generate config.hh header.
GENERATE_CONFIG_HEADER(
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/models/simple_humanoid/config.hh
  metapod_simple_humanoid
  )

# Set header files.
SET(METAPOD_SIMPLE_HUMANOID_MODEL_HEADERS
  include/metapod/models/simple_humanoid/body.hh
  include/metapod/models/simple_humanoid/joint.hh
  include/metapod/models/simple_humanoid/robot.hh
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/models/simple_humanoid/config.hh
  )
QI_CREATE_LIB(metapod_simple_humanoid SHARED
    src/simple_humanoid.cc
    ${METAPOD_SIMPLE_HUMANOID_MODEL_HEADERS})
QI_USE_LIB(metapod_simple_humanoid metapod EIGEN3)
QI_STAGE_LIB(metapod_simple_humanoid)
