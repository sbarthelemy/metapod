SET(${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS
  body.hh
  joint.hh
  robot.hh
  config.hh
  )

QI_CREATE_LIB(simple_humanoid SHARED
    simple_humanoid.cc
    ${${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS})
QI_USE_LIB(simple_humanoid metapod EIGEN3)
QI_STAGE_LIB(simple_humanoid)
