cmake_minimum_required(VERSION 3.10)
project(engine LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# glad
add_library(glad STATIC ${CMAKE_CURRENT_SOURCE_DIR}/include/glad/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE ENGINE_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*)

add_library(${PROJECT_NAME} STATIC ${ENGINE_SRC})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(GLFW_IMPORT_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/libglfw3dll.a)
if(NOT EXISTS ${GLFW_IMPORT_LIB})
    message(FATAL_ERROR "GLFW import library not found: ${GLFW_IMPORT_LIB}")
endif()

add_library(glfw_imported UNKNOWN IMPORTED)
set_target_properties(glfw_imported PROPERTIES
    IMPORTED_LOCATION ${GLFW_IMPORT_LIB}
)

target_link_libraries(${PROJECT_NAME} PUBLIC glfw_imported glad)