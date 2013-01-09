SET(${PROJECT_NAME}_SIMPLE_ARM_MODEL_HEADERS
  body.hh
  joint.hh
  robot.hh
  config.hh
  )

QI_CREATE_LIB(simple_arm SHARED
    simple_arm.cc
    ${${PROJECT_NAME}_SIMPLE_ARM_MODEL_HEADERS})
QI_USE_LIB(simple_arm metapod EIGEN3)
QI_STAGE_LIB(simple_arm)
