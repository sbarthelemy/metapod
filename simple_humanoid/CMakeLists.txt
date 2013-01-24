INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set header files.
SET(METAPOD_SIMPLE_HUMANOID_MODEL_HEADERS
  include/metapod/models/simple_humanoid/simple_humanoid.hh
  include/metapod/models/simple_humanoid/config.hh
  )
QI_CREATE_LIB(metapod_simple_humanoid SHARED
  include/metapod/models/simple_humanoid/simple_humanoid.cc
  ${METAPOD_SIMPLE_HUMANOID_MODEL_HEADERS})
QI_USE_LIB(metapod_simple_humanoid metapod)
QI_STAGE_LIB(metapod_simple_humanoid)
