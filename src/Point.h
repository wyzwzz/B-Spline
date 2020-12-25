//
// Created by wyz on 20-12-25.
//

#ifndef B_SPLINE_POINT_H
#define B_SPLINE_POINT_H
#include <glad/glad.h>
#include <vector>
#include <memory>
#include "Config.h"
#include "Shader.h"
#include "Util.h"
class Point {
public:
    Point()=default;
    void setupVertex(std::vector<B_SPLINE_DATATYPE>& vertex);
    void setupModel(const glm::mat4& model);
    void setupView(const glm::mat4& view);
    void setupProjection(const glm::mat4& projection);
    void draw();
    void setupGLResource();
    void setupGLShader(const std::string& vertex_shader_path,const std::string& fragment_shader_path);
private:
    void deleteGLResource();
private:
    GLuint vao;
    GLuint vbo;
    std::vector<B_SPLINE_DATATYPE> vertex;
    std::unique_ptr<Shader> shader;
    glm::mat4 model,view,projection;
};


#endif //B_SPLINE_POINT_H
