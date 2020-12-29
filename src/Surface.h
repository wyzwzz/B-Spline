//
// Created by wyz on 20-12-25.
//

#ifndef B_SPLINE_SURFACE_H
#define B_SPLINE_SURFACE_H
#include <glad/glad.h>
#include <vector>
#include <memory>
#include "Config.h"
#include "Shader.h"
class Surface {
public:
    Surface();

    /**
     * @brief set up surface's vertex.
     * @param pitch: number of vertex for a row.
     */
    void setupSurfaceVertex(const std::vector<B_SPLINE_DATATYPE>& vertex,uint32_t pitch);

    void setupMVPMatrix(const glm::mat4& mvp)=delete;
    void setupModel(const glm::mat4& model);
    void setupView(const glm::mat4& view);
    void setupProjection(const glm::mat4& projection);
    void draw();
    void setupGLResource();
    void setupGLShader(const std::string& vertex_shader_path,const std::string& fragment_shader_path);
    void setupDrawMode(GLenum mode);
private:
    void deleteGLResource();
private:
    GLuint vao;
    GLuint vbo;
    GLuint ebo;
    std::vector<B_SPLINE_DATATYPE> vertex;
    uint32_t row,col,triangle_num;
    std::vector<GLuint> index;
    std::unique_ptr<Shader> shader;
    glm::mat4 mvp;
    glm::mat4 model,view,projection;
    GLenum draw_mode;
};


#endif //B_SPLINE_SURFACE_H
