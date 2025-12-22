cmake_minimum_required(VERSION 3.21)

project(
  mc_kuka_fri
  LANGUAGES CXX
  VERSION 2.0.0)

find_package(mc_rtc REQUIRED)

find_description_package(lbr_description)

find_program(XACRO xacro REQUIRED)
message(STATUS "Found xacro: ${XACRO}")

find_program(MESH_SAMPLING mesh_sampling REQUIRED)
message(STATUS "Found mesh_sampling: ${MESH_SAMPLING}")

include(GNUInstallDirs)
set(INSTALL_DATADIR ${CMAKE_INSTALL_FULL_DATADIR}/${PROJECT_NAME})

function(generate_iiwa_module MODEL)
  # Generates the URDF from the xacro files
  set(URDF_OUT ${CMAKE_CURRENT_BINARY_DIR}/urdf/${MODEL}.urdf)
  set(XACRO_IN ${LBR_DESCRIPTION_PATH}/urdf/${MODEL}/${MODEL}.xacro)
  add_custom_command(
    OUTPUT ${URDF_OUT}
    COMMAND ${XACRO} -o ${URDF_OUT} -i ${XACRO_IN}
    DEPENDS ${XACRO_IN} ${LBR_DESCRIPTION_PATH}/urdf/${MODEL}/${MODEL}.xacro
    COMMENT "Generate ${MODEL}.urdf")
  add_custom_target(generate_${MODEL}_model ALL DEPENDS ${URDF_OUT})
  install(FILES ${URDF_OUT} DESTINATION ${INSTALL_DATADIR}/urdf/)

  # Install robot module YAML file
  set(MODULE_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/")
  configure_file(aliases/iiwa_config.in.yaml
                 ${CMAKE_CURRENT_BINARY_DIR}/${MODEL}.yaml @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MODEL}.yaml
          DESTINATION ${MODULE_INSTALL_LOCATION})

  # Configure the mc_rtc alias
  set(ALIAS_OUT ${CMAKE_CURRENT_BINARY_DIR}/aliases/${MODEL}.yaml)
  configure_file(aliases/iiwa.in.yaml ${ALIAS_OUT} @ONLY)
  install(FILES ${ALIAS_OUT} DESTINATION ${MC_ROBOTS_ALIASES_DIRECTORY})

  # Generate convex files
  set(MESH_IN ${LBR_DESCRIPTION_PATH}/meshes/${MODEL}/visual)
  set(CONVEX_OUT ${INSTALL_DATADIR}/convex/${MODEL})

  add_custom_command(
    OUTPUT ${CONVEX_OUT}
    COMMAND mkdir -p ${CONVEX_OUT}
    COMMAND ${MESH_SAMPLING} --in ${MESH_IN} --convex
            ${INSTALL_DATADIR}/convex/${MODEL} --type xyz --samples 1000
    DEPENDS ${MESH_IN}
    COMMENT "Generate ${CONVEX_OUT}")

  add_custom_target(generate_${MODEL}_${LINK}_convex ALL DEPENDS ${CONVEX_OUT})
endfunction()

generate_iiwa_module(iiwa7)
generate_iiwa_module(iiwa14)

option(WITH_KUKA_FRI "Build MCKukaFRI" ON)
if(WITH_KUKA_FRI)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
  find_package(KUKA_FRI REQUIRED)
  find_package(Boost REQUIRED COMPONENTS program_options)
  add_subdirectory(src)
endif()
