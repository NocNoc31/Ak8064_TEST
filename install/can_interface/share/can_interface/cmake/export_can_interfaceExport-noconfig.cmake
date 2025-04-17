#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "can_interface::can_interface" for configuration ""
set_property(TARGET can_interface::can_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(can_interface::can_interface PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcan_interface.so"
  IMPORTED_SONAME_NOCONFIG "libcan_interface.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS can_interface::can_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_can_interface::can_interface "${_IMPORT_PREFIX}/lib/libcan_interface.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
