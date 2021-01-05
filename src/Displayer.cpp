//
// Created by wyz on 20-12-25.
//

#include "Displayer.h"

void Displayer::render()
{
    SDL_StartTextInput();
    SDL_Event event;
    bool exit = false;


    auto processEvent = [&exit, &event, this]() {
        bool update_view_matrix = false;
        bool update_projection_matrix = false;

        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
                case SDL_QUIT: {
                    exit = true;
                }
                    break;
                case SDL_KEYDOWN: {
                    switch (event.key.keysym.sym)
                    {
                        case SDLK_ESCAPE: {
                            exit = true;
                        }
                            break;
                        case SDLK_f: {
                            camera->processKeyboardForArgs(Camera::CameraDefinedKey::FASTER);
                        }
                            break;
                        case SDLK_g: {
                            camera->processKeyboardForArgs(Camera::CameraDefinedKey::SLOWER);
                        }
                            break;
                        case SDLK_w: {
                            camera->processMovementByKeyboard(Camera::CameraMoveDirection::FORWARD, 0.1f);
                            update_view_matrix = true;
                        }
                            break;
                        case SDLK_s: {
                            camera->processMovementByKeyboard(Camera::CameraMoveDirection::BACKWARD, 0.1f);
                            update_view_matrix = true;
                        }
                            break;
                        case SDLK_a: {
                            camera->processMovementByKeyboard(Camera::CameraMoveDirection::LEFT, 0.1f);
                            update_view_matrix = true;
                        }
                            break;
                        case SDLK_d: {
                            camera->processMovementByKeyboard(Camera::CameraMoveDirection::RIGHT, 0.1f);
                            update_view_matrix = true;
                        }
                            break;
                        case SDLK_q: {
                            camera->processMovementByKeyboard(Camera::CameraMoveDirection::UP, 0.1f);
                            update_view_matrix = true;
                        }
                            break;
                        case SDLK_e: {
                            camera->processMovementByKeyboard(Camera::CameraMoveDirection::DOWN, 0.1f);
                            update_view_matrix = true;
                        }
                            break;
                    }
                }
                    break;
                case SDL_MOUSEWHEEL: {
                    camera->processMouseScroll(event.wheel.y);
                    update_projection_matrix = true;
                }
                    break;
                case SDL_MOUSEMOTION: {
                    if (event.button.button == 1)
                    {
                        camera->processMouseMovement(event.motion.xrel, -event.motion.yrel);
                        update_view_matrix = true;
                    }
                }
                    break;
            }
            if(update_view_matrix){
                for(auto& curve:curves)
                    curve->setupView(camera->getViewMatrix());
                for(auto& surface:surfaces)
                    surface->setupView(camera->getViewMatrix());
            }
            if(update_projection_matrix){
                for(auto& curve:curves)
                    curve->setupProjection(glm::perspective(glm::radians(camera->getZoom()), (float)w / h, 0.1f, 50.0f));
                for(auto& surface:surfaces)
                    surface->setupProjection(glm::perspective(glm::radians(camera->getZoom()), (float)w / h, 0.1f, 50.0f));
            }
        }
    };


    /**
     * render loop
     */
    auto last = std::chrono::steady_clock::now();
    while (!exit)
    {
        processEvent();

        auto current = std::chrono::steady_clock::now();
        auto t = std::chrono::duration_cast<std::chrono::milliseconds>(current - last);
        last = current;
//        std::cout << "render one frame cost time: " << t.count() << "ms" << std::endl;
        glClearColor(0.f,0.f,0.f,1.f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        for(auto& curve:curves)
            curve->draw();
        for(auto& surface:surfaces)
            surface->draw();
        glFlush();
        glFinish();
        SDL_GL_SwapWindow(window);
    }

    SDL_StopTextInput();
}

int Displayer::addCurve()
{
    try {
        curves.emplace_back(std::make_unique<Curve>());
        curves.back()->setupView(camera->getViewMatrix());
        curves.back()->setupProjection(glm::perspective(glm::radians(camera->getZoom()), (float) w / h, 0.1f, 50.0f));
    }
    catch(std::exception err){
        std::cout<<__FUNCTION__ <<" error: "<<err.what()<<std::endl;
        return -1;
    }
    return curves.size()-1;
}

int Displayer::deleteCurve(uint32_t index)
{
    if(index<curves.size()){
        curves.erase(curves.cbegin()+index);
        return 0;
    }
    return -1;
}

void Displayer::setCurveDrawMode(uint32_t index, GLenum mode)
{
    if(index<curves.size()){
        curves[index]->setupDrawMode(mode);
    }
}
void Displayer::addCurveControlPoints(uint32_t index,const std::vector<B_SPLINE_DATATYPE>& controlP)
{
    if(index<curves.size()){
        curves[index]->setupCurveVertex(controlP);
    }
}

void Displayer::clearCurveControlPoints(uint32_t index)
{
    if(index<curves.size()){
        curves[index]->setupCurveVertex({});
    }
}

int Displayer::addSurface()
{
    try {
        surfaces.emplace_back(std::make_unique<Surface>());
        surfaces.back()->setupView(camera->getViewMatrix());
        surfaces.back()->setupProjection(glm::perspective(glm::radians(camera->getZoom()), (float) w / h, 0.1f, 50.0f));
    }
    catch(std::exception err){
        std::cout<<__FUNCTION__ <<" error: "<<err.what()<<std::endl;
        return -1;
    }
    return surfaces.size()-1;
}

int Displayer::deleteSurface(uint32_t index)
{
    if(index<surfaces.size()){
        surfaces.erase(surfaces.cbegin()+index);
        return 0;
    }
    return -1;
}

void Displayer::addSurfaceControlPoints(uint32_t index, const std::vector<float> &controlP, uint32_t pitch)
{
    if(index<surfaces.size()){
        surfaces[index]->setupSurfaceVertex(controlP,pitch);
    }
}

void Displayer::clearSurfaceControlPoints(uint32_t index)
{
    if(index<surfaces.size()){
        surfaces[index]->setupSurfaceVertex({},0);
    }
}

void Displayer::setupSurfaceColor(uint32_t index,std::array<float,3> color)
{
    if(index<surfaces.size()){
        surfaces[index]->setupColor(color);
    }
}


