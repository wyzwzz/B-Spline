//
// Created by wyz on 20-12-6.
//

#ifndef HIERARCHICAL_Z_BUFFER_UTIL_H
#define HIERARCHICAL_Z_BUFFER_UTIL_H

#include <SDL.h>
#include <chrono>
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <iostream>


template <typename T> T min4(T x1, T x2, T x3, T x4)
{
    T min_ = x1;
    if (x2 < min_)
        min_ = x2;
    if (x3 < min_)
        min_ = x3;
    if (x4 < min_)
        min_ = x4;
    return min_;
}
template <typename T> T max4(T x1, T x2, T x3, T x4)
{
    T max_ = x1;
    if (x2 > max_)
        max_ = x2;
    if (x3 > max_)
        max_ = x3;
    if (x4 > max_)
        max_ = x4;
    return max_;
}

template <class T>
void print(T t)
{
    std::cout<<t<<std::endl;
}
template <class T,class... Args>
void print(T t,Args... args)
{
    std::cout<<t<<" ";
    print(args...);
}


#define DEBUG
#ifdef DEBUG
#define SDL_EXPR(exec)                                                                                                 \
    exec;                                                                                                              \
    {                                                                                                                  \
        std::string str = SDL_GetError();                                                                              \
        if (str.length() > 0)                                                                                          \
        {                                                                                                              \
            std::cout << __FILE__ << "  function:" << __FUNCTION__ << "  line:" << __LINE__ << "  SDL Error: " << str  \
                      << std::endl;                                                                                    \
        }                                                                                                              \
    }

#define SDL_CHECK                                                                                                      \
    {                                                                                                                  \
        std::string str = SDL_GetError();                                                                              \
        if (str.length() > 0)                                                                                          \
        {                                                                                                              \
            std::cout << "SDL Error: " << str << " exist before line " << __LINE__ << " in file " << __FILE__          \
                      << std::endl;                                                                                    \
        }                                                                                                              \
    }

#define GL_CHECK \
    {            \
        auto v=glGetError(); \
        if(v!=0){\
            std::cout<<"GL Error: "<<v<<" in line "<<__LINE__<<" in function "<<__FUNCTION__<<" in file "<<__FILE__<<std::endl;         \
        }\
    }
#else
#define SDL_EXPR(exec) exec
#define SCL_CHECK
#endif

#define START_TIMER auto start = std::chrono::steady_clock::now();

#define END_TIMER(fmt)                                                                                                 \
    auto end = std::chrono::steady_clock::now();                                                                       \
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);                                       \
    std::cout << fmt << "cost time: " << t.count() << "ms" << std::endl;

#endif // HIERARCHICAL_Z_BUFFER_UTIL_H
