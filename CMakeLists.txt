cmake_minimum_required(VERSION 3.17)

project(B_Spline)

set(CMAKE_CXX_STANDARD 14)

set(PROJECT_THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR}/third_party)

#glm
set(GLM_INCLUDE_DIR ${PROJECT_THIRD_PARTY_DIR}/glm/glm)
#eigen
set(EIGEN_INCLUDE_DIR ${PROJECT_THIRD_PARTY_DIR}/eigen)
#sdl2-2.0.3
set(SDL_INCLUDE_DIR ${PROJECT_THIRD_PARTY_DIR}/sdl2-2.0.3/include)
set(SDL_LIB_DIR ${PROJECT_THIRD_PARTY_DIR}/sdl2-2.0.3/bin/linux64)
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
        SDL2 ${CMAKE_DL_LIBS} OpenGL::GL)
