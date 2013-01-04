# Set header files.
SET(${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS
  body.hh
  joint.hh
  robot.hh
  config.hh
  )

# Set header files installation prefix.
INSTALL(FILES ${${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS}
  DESTINATION include/${PROJECT_NAME}/models/simple-humanoid
  )

ADD_LIBRARY(simple_humanoid SHARED
  simple_humanoid.cc
  ${${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS})
