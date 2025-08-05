#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robocallee_fms::robocallee_fms__rosidl_generator_py" for configuration "Debug"
set_property(TARGET robocallee_fms::robocallee_fms__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(robocallee_fms::robocallee_fms__rosidl_generator_py PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "robocallee_fms::robocallee_fms__rosidl_generator_c;Python3::Python;robocallee_fms::robocallee_fms__rosidl_typesupport_c;service_msgs::service_msgs__rosidl_generator_c;service_msgs::service_msgs__rosidl_typesupport_fastrtps_c;service_msgs::service_msgs__rosidl_typesupport_fastrtps_cpp;service_msgs::service_msgs__rosidl_typesupport_introspection_c;service_msgs::service_msgs__rosidl_typesupport_c;service_msgs::service_msgs__rosidl_typesupport_introspection_cpp;service_msgs::service_msgs__rosidl_typesupport_cpp;service_msgs::service_msgs__rosidl_generator_py;builtin_interfaces::builtin_interfaces__rosidl_generator_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_fastrtps_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_introspection_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_fastrtps_cpp;builtin_interfaces::builtin_interfaces__rosidl_typesupport_introspection_cpp;builtin_interfaces::builtin_interfaces__rosidl_typesupport_cpp;builtin_interfaces::builtin_interfaces__rosidl_generator_py"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/librobocallee_fms__rosidl_generator_py.so"
  IMPORTED_SONAME_DEBUG "librobocallee_fms__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets robocallee_fms::robocallee_fms__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_robocallee_fms::robocallee_fms__rosidl_generator_py "${_IMPORT_PREFIX}/lib/librobocallee_fms__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
