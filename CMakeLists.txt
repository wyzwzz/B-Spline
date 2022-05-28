cmake_minimum_required(VERSION 3.17)

project(B_Spline)

set(CMAKE_CXX_STANDARD 14)

if(WIN32)
        set(SDL_LIB SDL2 SDL2main)
elseif(UNIX AND NOT APPLE)
        set(SDL_LIB SDL2)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(PROJECT_THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR}/third_party)

#glm
set(GLM_INCLUDE_DIR ${PROJECT_THIRD_PARTY_DIR}/glm/glm)
#eigen
set(EIGEN_INCLUDE_DIR ${PROJECT_THIRD_PARTY_DIR}/eigen)
#sdl2-2.0.3
set(SDL_INCLUDE_DIR ${PROJECT_THIRD_PARTY_DIR}/SDL2-2.0.12/include)
#glad
set(GLAD_INCLUDE_DIR ${PROJECT_THIRD_PARTY_DIR}/glad/include)
set(GLAD_SRC_DIR ${PROJECT_THIRD_PARTY_DIR}/glad/src)
#gl
find_package(OpenGL REQUIRED)

aux_source_directory(${CMAKE_SOURCE_DIR}/src SRC)

add_executable(B_Spline ${SRC} ${GLAD_SRC_DIR}/glad.c)

target_include_directories(B_Spline PRIVATE
        ${GLM_INCLUDE_DIR} ${SDL_INCLUDE_DIR} ${GLAD_INCLUDE_DIR}
        ${EIGEN_INCLUDE_DIR})

target_link_directories(B_Spline PRIVATE
        ${SDL_LIB_DIR})

target_link_libraries(B_Spline PRIVATE
        ${SDL_LIB} ${CMAKE_DL_LIBS} OpenGL::GL)

add_subdirectory(${PROJECT_THIRD_PARTY_DIR}/SDL2-2.0.12)