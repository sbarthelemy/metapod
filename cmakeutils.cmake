# Copyright 2012, 2013
#
# Sébastien Barthélémy (Aldebaran Robotics)
#
# This file is part of metapod.
# metapod is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# metapod is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with metapod.  If not, see <http://www.gnu.org/licenses/>.
# Copyright 2010, 2012, 2013

FUNCTION(FIND_GENERATOR GENERATOR_NAME)
  SET(WITH_${GENERATOR_NAME} FALSE)
  # maybe the user passed the location
  IF(${GENERATOR_NAME}_EXECUTABLE)
    SET(WITH_${GENERATOR_NAME} TRUE)
  ELSE()
    # last resort: search for it
    FIND_PACKAGE(${GENERATOR_NAME} QUIET)
    SET(WITH_${GENERATOR_NAME} ${${GENERATOR_NAME}_FOUND})
  ENDIF()
  SET(WITH_${GENERATOR_NAME} ${WITH_${GENERATOR_NAME}} PARENT_SCOPE)
ENDFUNCTION()

# GENERATE_CONFIG_HEADER
#
# Generates a configuration header for DLL API import/export
#
# OUTPUT: path (including filename) of the generated file. Usually the filename
#         is config.hh
# LIBRARY_NAME: the name of the library. It will be normalized.
FUNCTION(GENERATE_CONFIG_HEADER OUTPUT LIBRARY_NAME)
  STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_" LIBRARY_NAME "${LIBRARY_NAME}")
  STRING(TOLOWER "${LIBRARY_NAME}" "LIBRARY_NAME_LOWER")
  SET(EXPORT_SYMBOL "${LIBRARY_NAME_LOWER}_EXPORTS")
  STRING(TOUPPER "${LIBRARY_NAME}" "LIBRARY_NAME")
  # create the directory (and its parents)
  GET_FILENAME_COMPONENT(OUTPUT_DIR "${OUTPUT}" PATH)
  FILE(MAKE_DIRECTORY "${OUTPUT_DIR}")
  # Generate the header. The following variables are used in the template
  # LIBRARY_NAME: CPP symbol prefix, should match the compiled library name,
  #               usually in upper case
  # EXPORT_SYMBOL: what symbol controls the switch between symbol
  #                import/export, usually libname_EXPORTS, with libname in
  #                lower case.
  # PROJECT_VERSION: the project version. Only supported when using jrl-cmake.
  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/robotbuilder/tpl/config.in.hh
    ${OUTPUT}
    @ONLY)
  SET_SOURCE_FILES_PROPERTIES(
    ${OUTPUT}
    GENERATED)
ENDFUNCTION(GENERATE_CONFIG_HEADER)

# ADD_SAMPLEURDFMODEL
#
# Call metapodfromurdf to create one of the sample models. If not available
# use pregenerated files.
#
# NAME: the name of the model. Either simple_arm or simple_humanoid.
FUNCTION(ADD_SAMPLEURDFMODEL name)
  SET(_libname "metapod_${name}")
  SET(_urdf_file "${PROJECT_SOURCE_DIR}/data/${name}.urdf")
  SET(_config_file "${PROJECT_SOURCE_DIR}/data/${name}.config")
  SET(_license_file "${PROJECT_SOURCE_DIR}/data/metapod_license_file.txt")
  SET(_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/include/metapod/models")
  SET(_pregen_dir "${PROJECT_SOURCE_DIR}/pregeneratedmodels")
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")
  SET(_gen_sources "")
  SET(_pregen_sources "")
  SET(_sources
      config.hh
      ${name}.hh
      ${name}.cc)
  FOREACH(f ${_sources})
    LIST(APPEND _gen_sources ${_gen_dir}/${name}/${f})
    LIST(APPEND _pregen_sources ${_pregen_dir}/${name}/${f})
  ENDFOREACH()
  IF(WITH_METAPODFROMURDF)
    ADD_CUSTOM_COMMAND(
      OUTPUT ${_gen_sources}
      COMMAND ${METAPODFROMURDF_EXECUTABLE}
      --name ${name}
      --libname ${_libname}
      --directory ${_gen_dir}/${name}
      --config-file ${_config_file}
      --license-file ${_license_file}
      ${_urdf_file}
      DEPENDS ${METAPODFROMURDF_EXECUTABLE} ${_urdf_file} ${_config_file}
        ${_license_file}
      MAIN_DEPENDENCY ${_urdf_file}
      )

    FIND_PROGRAM(python_executable
      NAMES python2 python python.exe
      NO_CMAKE_FIND_ROOT_PATH)
    IF(python_executable)
      QI_ADD_TEST(
        check_pregenerated_${name}_is_up_to_date
        ${python_executable}
        ARGUMENTS ${PROJECT_SOURCE_DIR}/cmp_files.py
                  ${_gen_dir}/${name}
                  ${_pregen_dir}/${name}
                  ${_sources})
    ENDIF()

  ELSE()
    ADD_CUSTOM_COMMAND(
      OUTPUT ${_gen_sources}
      COMMAND ${CMAKE_COMMAND} -E copy_directory
              ${_pregen_dir}/${name}
              ${_gen_dir}/${name}
      DEPENDS ${_pregen_sources}
      COMMENT "metapodfromurdf is not available, copying pregenerated files instead"
      )
  ENDIF()
  SET_SOURCE_FILES_PROPERTIES(${_gen_sources} GENERATED)

  QI_CREATE_LIB(${_libname} SHARED ${_gen_sources})
  QI_USE_LIB(${_libname} metapod)
  QI_STAGE_LIB(${_libname})
ENDFUNCTION()
