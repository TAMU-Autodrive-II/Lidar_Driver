

set(CEPTON_SDK2_SAMPLE_SOURCES "basic.c"
"callback_task.cpp")
if (NOT WIN32)
  list(APPEND CEPTON_SDK2_SAMPLE_SOURCES "posix/direct_networking.c")
endif()

foreach(path ${CEPTON_SDK2_SAMPLE_SOURCES})
  get_filename_component(name "${path}" NAME_WE)
  add_executable(cepton_sdk2_sample_${name} "${path}")
  target_include_directories(cepton_sdk2_sample_${name}
                             PRIVATE "${CEPTON_SDK2_SOURCE_DIR}/include")
  target_link_libraries(cepton_sdk2_sample_${name} cepton_sdk2)
endforeach()

add_subdirectory("exporter")
