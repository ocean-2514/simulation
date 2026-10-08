cmake_minimum_required(VERSION 3.10)
project(simulation LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/engine)

file(GLOB_RECURSE SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*)

add_executable(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC engine)

set(DLL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/bin/glfw3.dll
)

foreach(DLL ${DLL_FILES})
    if(NOT EXISTS ${DLL})
        message(WARNING "DLL not found: ${DLL}")
    endif()
endforeach()

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${DLL_FILES}
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
    COMMENT "Copying DLLs to output directory for development"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE 
    SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/shader")