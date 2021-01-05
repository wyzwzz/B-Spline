//
// Created by wyz on 20-12-25.
//

#include "Surface.h"
#include "Util.h"
Surface::Surface()
:model(glm::mat4(1.f)),draw_mode(GL_TRIANGLES),
color({1.f,0.f,0.f})
{
    setupGLShader(SURFACE_VERTEX_SHADER_PATH,SURFACE_FRAGMENT_SHADER_PATH);
}

void Surface::setupSurfaceVertex(const std::vector<float> &vertex, uint32_t pitch)
{
    if(vertex.size()==0) return;
    deleteGLResource();
    this->vertex=vertex;
    this->col=pitch;
    this->row=vertex.size()/3/pitch;
    assert(col*row*3==vertex.size());
    triangle_num=(row-1)*(col-1)*2;
    index.reserve(triangle_num*3);
    //set triangle's index
    for(size_t i=0;i<row-1;i++){
        for(size_t j=0;j<col-1;j++){
            // row i, col j
            index.push_back(i*col+j);
            // row i+1, col j
            index.push_back((i+1)*col+j);
            // row i+1, col j+1
            index.push_back((i+1)*col+j+1);
            // row i, col j
            index.push_back(i*col+j);
            // row i+1, col j+1
            index.push_back((i+1)*col+j+1);
            // row i, col j+1
            index.push_back(i*col+j+1);
        }
    }
    assert(index.size()==triangle_num*3);
    setupGLResource();
}

void Surface::setupModel(const glm::mat4 &model)
{
    this->model=model;
}

void Surface::setupView(const glm::mat4 &view)
{
    this->view=view;
}

void Surface::setupProjection(const glm::mat4 &projection)
{
    this->projection=projection;
}

void Surface::draw()
{
    glBindVertexArray(vao);
    shader->use();
    shader->setMat4("MVPMatrix",projection*view*model);
    shader->setVec3("color",color[0],color[1],color[2]);
    glDrawElements(draw_mode,index.size(),GL_UNSIGNED_INT,nullptr);
    glBindVertexArray(0);
    GL_CHECK
}

void Surface::setupGLResource()
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

void Surface::setupGLShader(const std::string &vertex_shader_path, const std::string &fragment_shader_path)
{
    shader=std::make_unique<Shader>(vertex_shader_path.c_str(),fragment_shader_path.c_str());
    GL_CHECK
}

void Surface::setupDrawMode(GLenum mode)
{
    this->draw_mode=mode;
}

void Surface::deleteGLResource()
{
    glDeleteVertexArrays(1,&vao);
    glDeleteBuffers(1,&vbo);
    glDeleteBuffers(1,&ebo);
    GL_CHECK
}

void Surface::setupColor(std::array<float, 3> &color)
{
    this->color=color;
}

