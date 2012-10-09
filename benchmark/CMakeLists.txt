INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/benchmark)

qi_create_lib(sample_3_dof SHARED models/sample_3_dof/sample_3_dof.cc)
qi_stage_lib(sample_3_dof)

qi_create_lib(sample_7_dof SHARED models/sample_7_dof/sample_7_dof.cc)
qi_stage_lib(sample_7_dof)

qi_create_lib(sample_15_dof SHARED models/sample_15_dof/sample_15_dof.cc)
qi_stage_lib(sample_15_dof)

qi_create_lib(sample_31_dof SHARED models/sample_31_dof/sample_31_dof.cc)
qi_stage_lib(sample_31_dof)

qi_create_lib(sample_63_dof SHARED models/sample_63_dof/sample_63_dof.cc)
qi_stage_lib(sample_63_dof)

qi_create_bin(benchmark benchmark.cc)
qi_use_lib(benchmark
  metapod
  sample_3_dof
  sample_7_dof
  sample_15_dof
  sample_31_dof
  sample_63_dof
  simple_humanoid
)

qi_create_bin(make_model make_model.cc make_model.hh)
qi_use_lib(make_model boost_filesystem)
