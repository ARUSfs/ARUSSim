#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "arussim::main_interface_exec" for configuration ""
set_property(TARGET arussim::main_interface_exec APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(arussim::main_interface_exec PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmain_interface_exec.so"
  IMPORTED_SONAME_NOCONFIG "libmain_interface_exec.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS arussim::main_interface_exec )
list(APPEND _IMPORT_CHECK_FILES_FOR_arussim::main_interface_exec "${_IMPORT_PREFIX}/lib/libmain_interface_exec.so" )

# Import target "arussim::plot_interface_exec" for configuration ""
set_property(TARGET arussim::plot_interface_exec APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(arussim::plot_interface_exec PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libplot_interface_exec.so"
  IMPORTED_SONAME_NOCONFIG "libplot_interface_exec.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS arussim::plot_interface_exec )
list(APPEND _IMPORT_CHECK_FILES_FOR_arussim::plot_interface_exec "${_IMPORT_PREFIX}/lib/libplot_interface_exec.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
