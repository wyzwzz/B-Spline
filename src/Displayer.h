//
// Created by wyz on 20-12-25.
//

#ifndef B_SPLINE_DISPLAYER_H
#define B_SPLINE_DISPLAYER_H
#include <glad/glad.h>
#include <SDL.h>
#include <memory>
#include "Camera.h"
#include "Util.h"
#include "BSpline.h"
#include "Curve.h"
#include "Surface.h"
#include "Point.h"

class Displayer {
public:
    Displayer(uint32_t w, uint32_t h): w(w), h(h)
    {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
        {
            printf("%s - SDL could not initialize! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
            throw std::runtime_error("SDL could not initialize");
        }
        SDL_EXPR(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,4);)
        SDL_EXPR(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,6);)
        SDL_EXPR(SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_CORE);)
        SDL_EXPR(window = SDL_CreateWindow("B-Spline", 100, 100, w, h, SDL_WINDOW_SHOWN|SDL_WINDOW_OPENGL);)
        SDL_EXPR(gl_context=SDL_GL_CreateContext(window);)
        if(gl_context==nullptr)
        {
            throw std::runtime_error("OpenGL context could not be created!");
        }
        int flag=SDL_GL_SetSwapInterval(0);
        std::cout<<"SDL_GL_SetSwapInterval: "<<flag<<std::endl;

        //glad:load gl
        if(!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress))
        {
            throw std::runtime_error("Failed to initialize GLAD");
        }
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_MULTISAMPLE);
//        glEnable(GL_LINE_SMOOTH);
        GL_CHECK
        camera = std::make_unique<Camera>(glm::vec3(0.0f, 0.0f, 10.0f));
    }
    ~Displayer(){
        SDL_EXPR(SDL_DestroyWindow(window);)
        SDL_EXPR(SDL_Quit();)
    }

public:
    void render();

    /**
     * @brief add a new curve and return the curve's index.
     * @return return index>0 if success,else -1.
     */
    int addCurve();

    /**
     * @brief delete a curve specified by index.
     * @param index : returned by addCurve
     * @return if exits deleting curve return 0, else -1.
     * @see addCurve
     */
    int deleteCurve(uint32_t index);

    void addCurveControlPoints(uint32_t index,const std::vector<B_SPLINE_DATATYPE>& controlP);
    void clearCurveControlPoints(uint32_t index);

    int addSurface();

    int deleteSurface(uint32_t index);

    void addSurfaceControlPoints(uint32_t index,const std::vector<B_SPLINE_DATATYPE>& controlP,uint32_t pitch);
    void clearSurfaceControlPoints(uint32_t index);


private:
    SDL_GLContext gl_context;
    SDL_Window *window;
    uint32_t w;
    uint32_t h;

    std::unique_ptr<Camera> camera;

    std::unique_ptr<BSpline> bspline;

    std::unique_ptr<Curve> curve;

    std::vector<std::unique_ptr<Curve>> curves;
    std::vector<std::unique_ptr<Surface>> surfaces;

};


#endif //B_SPLINE_DISPLAYER_H
