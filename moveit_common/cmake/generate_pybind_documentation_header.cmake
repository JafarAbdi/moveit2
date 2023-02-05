# Copyright 2023
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# TODO: How to share this environment with other packages?
function(create_micromamba_environment MICROMAMBA_BIN)
  find_program(
    _MICROMAMBA_BIN micromamba
    PATHS ${CMAKE_CURRENT_BINARY_DIR}/micromamba_bin
  )
  if(_MICROMAMBA_BIN)
    message("Found micromamba at ${_MICROMAMBA_BIN}")
  else()
    message("Downloading micromamba")
    # TODO: handle different architectures
    file(MAKE_DIRECTORY "micromamba_bin")
    # Install micromamba
    file(DOWNLOAD "https://micro.mamba.pm/api/micromamba/linux-64/latest" micromamba_bin/micromamba.tar.bz2)
    execute_process(
      COMMAND
      bash -c "tar xjf micromamba.tar.bz2 bin/micromamba --strip-components=1"
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/micromamba_bin
    )
    set(_MICROMAMBA_BIN ${CMAKE_CURRENT_BINARY_DIR}/micromamba_bin/micromamba)
  endif()
  if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/micromamba_bin/environment)
    message("Creating micromamba environment")
    # Create an environment and install libclang + pybind11-mkdoc
    execute_process(
      COMMAND
      ${_MICROMAMBA_BIN} create -p ${CMAKE_CURRENT_BINARY_DIR}/micromamba_bin/environment --channel conda-forge libclang==14.0.6 pip --yes --quiet
    )
    execute_process(
      COMMAND
      ${_MICROMAMBA_BIN} run -p ${CMAKE_CURRENT_BINARY_DIR}/micromamba_bin/environment pip3 install git+https://github.com/pybind/pybind11_mkdoc.git
    )
  endif()
  set(MICROMAMBA_BIN ${_MICROMAMBA_BIN} PARENT_SCOPE)
endfunction()

#
# Generate a pybind doc header file using pybind11-mkdoc
#
# :param ARGN: a list of target names
# :type ARGN: list of strings
#
macro(generate_pybind_documentation_header)
  create_micromamba_environment(MICROMAMBA_BIN)

  if(${ARGC} GREATER 0)
    foreach(target ${ARGN})
      get_target_property(_source_dir ${target} SOURCE_DIR)
      file(
        GLOB_RECURSE _headers
        "${_source_dir}/*.h"
        "${_source_dir}/*.hpp"
      )

      set(_output_path ${CMAKE_CURRENT_BINARY_DIR}/${target}.hpp)

      install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${target}.hpp DESTINATION include/${PROJECT_NAME}/moveit/docstring)

      # Create the command
      add_custom_command(
        OUTPUT ${_output_path}
        COMMAND
        ${CMAKE_COMMAND} -E env LIBCLANG_PATH=${CMAKE_CURRENT_BINARY_DIR}/micromamba_bin/environment/lib/libclang.so
        ${MICROMAMBA_BIN} run -p ${CMAKE_CURRENT_BINARY_DIR}/micromamba_bin/environment
        pybind11-mkdoc
        # Docstring wrap width
        -w 80 -o "${_output_path}"
        # List of include directories
        -I $<JOIN:$<TARGET_PROPERTY:${target},INTERFACE_INCLUDE_DIRECTORIES>, -I>
        # Headers for which to generate docstrings
        "${_headers}"
        WORKING_DIRECTORY "${_source_dir}"
        COMMENT
        "Creating docstrings with pybind11-mkdoc ..."
        VERBATIM COMMAND_EXPAND_LISTS
      )

      add_custom_target(${target}_docstring ALL DEPENDS ${_output_path})
    endforeach()
  endif()
endmacro()
