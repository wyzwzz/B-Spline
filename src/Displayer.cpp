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
                curve->setupView(camera->getViewMatrix());
            }
            if(update_projection_matrix){
                curve->setupProjection(glm::perspective(glm::radians(camera->getZoom()), (float)w / h, 0.1f, 50.0f));
            }
        }
    };

    std::vector<float> v{0.f,0.f,0.f,
                         5.f,5.f,5.f};

    curve->setupCurveVertex(v);
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
        std::cout << "render one frame cost time: " << t.count() << "ms" << std::endl;
        glClearColor(0.f,0.f,0.f,1.f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        curve->draw();

        glFlush();
        glFinish();
        SDL_GL_SwapWindow(window);
    }

    SDL_StopTextInput();
}
