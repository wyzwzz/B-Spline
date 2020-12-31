//
// Created by wyz on 20-12-25.
//

#include "BSplineSurface.h"
#include <iostream>
#include <cassert>
#include <Eigen/Core>
#include <Eigen/LU>
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
    //col and row is for control pts
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

    this->interpolationP.reserve(1.0/step_v*1.0/step_u*3);

    //select u-direction and get row b-spline curves(col control pts)
    std::vector<std::vector<B_SPLINE_DATATYPE>> temp_bspline_curves;
    temp_bspline_curves.resize(row);
    std::vector<B_SPLINE_DATATYPE> control_p;
    control_p.resize(order_u*3);
    for(size_t r=0;r<row;r++){
        temp_bspline_curves[r].reserve(1.0/step_u*3);
        for(float u=0.f;u<1.f;u+=step_u){
            int l;
            for(size_t i=0;i<knots_u.size()-1;i++){
                if(u>=knots_u[i] && u<knots_u[i+1]){
                    l=i;
                    break;
                }
            }
            assert(l<knots_u.size());
            for(size_t i=0;i<order_u*3;i++){
                control_p[i]=controlP[r*col*3+i+3*(l-order_u+1)];
            }
            for(int k=1;k<order;k++){
                for(int i=l;i>=l-order_u+1+k;i--){
                    if(i<1) continue;
                    int j=i-(l-order_u+1);

                    float tij=(u-knots_u[i])/(knots_u[i+order_u-k]-knots_u[i]);
                    control_p[j*3+0]=(1.f-tij)*control_p[(j-1)*3+0]
                                     +tij*control_p[j*3+0];
                    control_p[j*3+1]=(1.f-tij)*control_p[(j-1)*3+1]
                                     +tij*control_p[j*3+1];
                    control_p[j*3+2]=(1.f-tij)*control_p[(j-1)*3+2]
                                     +tij*control_p[j*3+2];
                }
            }
            temp_bspline_curves[r].push_back(control_p[(order_u-1)*3+0]);
            temp_bspline_curves[r].push_back(control_p[(order_u-1)*3+1]);
            temp_bspline_curves[r].push_back(control_p[(order_u-1)*3+2]);
        }
        std::cout<<"temp_bspline_curves size:";
        std::cout<<temp_bspline_curves[r].size()/3<<std::endl;
    }
//    assert(temp_bspline_curves.size()==row);

    control_p.clear();
    std::vector<B_SPLINE_DATATYPE> temp_controlP;
    temp_controlP.resize(row*3);
    // u-direction temp b-spline's pts number.
    for(size_t c=0;c<temp_bspline_curves[0].size()/3;c++){
        for(size_t i=0;i<row;i++){
            temp_controlP[i*3+0]=temp_bspline_curves[i][c*3+0];
            temp_controlP[i*3+1]=temp_bspline_curves[i][c*3+1];
            temp_controlP[i*3+2]=temp_bspline_curves[i][c*3+2];
        }
        for(float v=0.f;v<1.f;v+=step_v){
            int l;
            for(size_t i=0;i<knots_v.size()-1;i++){
                if(v>=knots_v[i] && v<knots_v[i+1]){
                    l=i;
                    break;
                }
            }
            assert(l<knots_v.size());
            for(size_t i=0;i<order_v*3;i++){
                control_p[i]=temp_controlP[i+3*(l-order_v+1)];
            }
            for(int k=1;k<order_v;k++){
                for(int i=l;i>=l-order_v+1+k;i--){
                    if(i<1) continue;
                    int j=i-(l-order_v+1);

                    float tij=(v-knots_v[i])/(knots_v[i+order_v-k]-knots_v[i]);
                    control_p[j*3+0]=(1.f-tij)*control_p[(j-1)*3+0]
                                     +tij*control_p[j*3+0];
                    control_p[j*3+1]=(1.f-tij)*control_p[(j-1)*3+1]
                                     +tij*control_p[j*3+1];
                    control_p[j*3+2]=(1.f-tij)*control_p[(j-1)*3+2]
                                     +tij*control_p[j*3+2];
                }
            }
            this->interpolationP.push_back(control_p[(order_v-1)*3+0]);
            this->interpolationP.push_back(control_p[(order_v-1)*3+1]);
            this->interpolationP.push_back(control_p[(order_v-1)*3+2]);

        }
    }
//    for(int i=0;i<temp_bspline_curves.size();i++){
//        for(size_t j=0;j<temp_bspline_curves[i].size();j++)
//            this->interpolationP.push_back(temp_bspline_curves[i][j]);
//    }
    std::cout<<interpolationP.size()/3<<std::endl;
    return this->interpolationP;
}

const std::vector<B_SPLINE_DATATYPE> &BSplineSurface::getApproximationP(std::vector<float> &controlP)
{

    return this->interpolationP;
}
