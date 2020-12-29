//
// Created by wyz on 20-12-25.
//

#include "BSplineSurface.h"
#include <iostream>
#include <cassert>
BSplineSurface::BSplineSurface()
:step_u(step),step_v(step),order_u(order),order_v(order),pitch(0)
{

}

void BSplineSurface::setupUVStep(float step_u, float step_v)
{
    this->step_u=step_u;
    this->step_v=step_v;
}

void BSplineSurface::setupUVOrder(int order_u, int order_v)
{
    this->order_u=order_u;
    this->order_v=order_v;
}

void BSplineSurface::setupPitch(int pitch)
{
    this->pitch=pitch;
}

const std::vector<B_SPLINE_DATATYPE> &BSplineSurface::getInterpolationP(std::vector<float> &controlP)
{
    if(pitch==0){
        std::cout<<"ERROR: pitch is zero!"<<std::endl;
        return {};
    }
    this->interpolationP.clear();
    //number of control points
    size_t n=controlP.size()/3;
    col=pitch;
    row=n/col;
    assert(col*row==n);
    knots_u.resize(col+order_u);
    for(size_t i=0;i<order_u;i++)
        knots_u[i]=0.f;
    for(size_t i=col;i<col+order_u;i++)
        knots_u[i]=1.f;
    for(size_t i=order_u;i<col;i++)
        knots_u[i]=(i-order_u+1)*1.f/(col-order_u+1);
    for(size_t i=0;i<knots_u.size();i++)
        std::cout<<knots_u[i]<<" ";
    std::cout<<std::endl;
    knots_v.resize(row+order_v);
    for(size_t i=0;i<order_v;i++)
        knots_v[i]=0.f;
    for(size_t i=row;i<row+order_v;i++)
        knots_v[i]=1.f;
    for(size_t i=order_v;i<row;i++)
        knots_v[i]=(i-order_v+1)*1.f/(row-order_v+1);
    for(size_t i=0;i<knots_v.size();i++)
        std::cout<<knots_v[i]<<" ";
    std::cout<<std::endl;

    return this->interpolationP;
}

const std::vector<B_SPLINE_DATATYPE> &BSplineSurface::getApproximationP(std::vector<float> &controlP)
{

    return this->interpolationP;
}
