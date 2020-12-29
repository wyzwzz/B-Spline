//
// Created by wyz on 20-12-25.
//

#include "Point.h"

void Point::setupModel(const glm::mat4 &model)
{
    this->model=model;
}

void Point::setupView(const glm::mat4 &view)
{
    this->view=view;
}

void Point::setupProjection(const glm::mat4 &projection)
{
    this->projection=projection;
}

void Point::setupVertex(std::vector<float> &vertex)
{
    deleteGLResource();
    vertex=vertex;
    setupGLResource();
}
void Point::draw()
{
    glBindVertexArray(vao);
    shader->use();
    shader->setMat4("MVPMatrix",projection*view*model);
    glDrawArrays(GL_POINTS,0,vertex.size()/3);
    glBindVertexArray(0);
    GL_CHECK
}

void Point::setupGLResource()
{
    glGenVertexArrays(1,&vao);
    glGenBuffers(1,&vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER,vbo);
    glBufferData(GL_ARRAY_BUFFER,vertex.size()*sizeof(B_SPLINE_DATATYPE),vertex.data(),GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(B_SPLINE_DATATYPE),(void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    GL_CHECK
}

void Point::setupGLShader(const std::string &vertex_shader_path, const std::string &fragment_shader_path)
{
    shader=std::make_unique<Shader>(vertex_shader_path.c_str(),fragment_shader_path.c_str());
    GL_CHECK
}

void Point::deleteGLResource()
{
    glDeleteVertexArrays(1,&vao);
    glDeleteBuffers(1,&vbo);
    GL_CHECK
}


