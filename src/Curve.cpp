//
// Created by wyz on 20-12-25.
//

#include "Curve.h"
#include "Util.h"

Curve::Curve()
:model(glm::mat4(1.f)),draw_mode(GL_LINES)
{
    setupGLShader(CURVE_VERTEX_SHADER_PATH,CURVE_FRAGMENT_SHADER_PATH);
}
void Curve::setupMVPMatrix(const glm::mat4 &mvp)
{
    this->mvp=mvp;
}

void Curve::setupCurveVertex(const std::vector<float> &vertex)
{
    deleteGLResource();
    std::cout<<"vertex size: "<<vertex.size()<<std::endl;
    this->vertex=vertex;
    index.resize((vertex.size()/3-1)*2);
    for(size_t i=0;i<vertex.size()/3-1;i++){
        index[2*i]=i;
        index[2*i+1]=i+1;
    }
    setupGLResource();
}
void Curve::draw()
{
    glBindVertexArray(vao);
    shader->use();
    shader->setMat4("MVPMatrix",projection*view*model);

    glDrawElements(draw_mode,index.size(),GL_UNSIGNED_INT,nullptr);
    glBindVertexArray(0);
    GL_CHECK
}

void Curve::setupGLResource()
{
    glGenVertexArrays(1,&vao);
    glGenBuffers(1,&vbo);
    glGenBuffers(1,&ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER,vbo);
    glBufferData(GL_ARRAY_BUFFER,vertex.size()*sizeof(B_SPLINE_DATATYPE),vertex.data(),GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,index.size()*sizeof(GLuint),index.data(),GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(B_SPLINE_DATATYPE),(void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    GL_CHECK
}

void Curve::deleteGLResource()
{
    glDeleteVertexArrays(1,&vao);
    glDeleteBuffers(1,&vbo);
    glDeleteBuffers(1,&ebo);
    GL_CHECK
}

void Curve::setupGLShader(const std::string& vertex_shader_path, const std::string& fragment_shader_path)
{
    shader=std::make_unique<Shader>(vertex_shader_path.c_str(),fragment_shader_path.c_str());
    GL_CHECK
}

void Curve::setupModel(const glm::mat4 &model)
{
    this->model=model;
}

void Curve::setupView(const glm::mat4 &view)
{
    this->view=view;
}

void Curve::setupProjection(const glm::mat4 &projection)
{
    this->projection=projection;
}

void Curve::setupDrawMode(GLenum mode)
{
    draw_mode=mode;
}





