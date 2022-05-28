//
// Created by wyz on 20-12-25.
//

#ifndef B_SPLINE_CURVE_H
#define B_SPLINE_CURVE_H
#include <glad/glad.h>
#include <vector>
#include <memory>
#include <array>
#include "Config.h"
#include "Shader.h"
class Curve {
public:
    Curve();

    void setupCurveVertex(const std::vector<B_SPLINE_DATATYPE>& vertex);

    void setupMVPMatrix(const glm::mat4& mvp);

    void setupModel(const glm::mat4& model);

    void setupView(const glm::mat4& view);

    void setupProjection(const glm::mat4& projection);

    void draw();

    void setupGLResource();

    void setupColor(std::array<float,3>& color);

    void setupGLShader(const std::string& vertex_shader_path,const std::string& fragment_shader_path);

    void setupDrawMode(GLenum mode);

    void setupPointSize(float size);
private:
    void deleteGLResource();
private:
    GLuint vao;
    GLuint vbo;
    GLuint ebo;
    std::vector<B_SPLINE_DATATYPE> vertex;
    std::vector<GLuint> index;
    std::unique_ptr<Shader> shader;
    glm::mat4 mvp;
    glm::mat4 model,view,projection;
    std::array<float,3> color;
    GLenum draw_mode;
    float point_size;
};


#endif //B_SPLINE_CURVE_H
