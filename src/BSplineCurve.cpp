//
// Created by wyz on 20-12-25.
//

#include "BSplineCurve.h"
#include <iostream>
#include <cmath>
#include <cassert>
const std::vector<B_SPLINE_DATATYPE> &BSplineCurve::getInterpolationP(std::vector<B_SPLINE_DATATYPE> &controlP)
{
//    return DeBoor_Cox(controlP);
    size_t n=controlP.size()/3;
    knots.resize(n+order);
//    for(size_t i=0;i<order;i++)
//        knots[i]=0.f;
//    for(size_t i=n;i<n+order;i++)
//        knots[i]=1.f;
//    for(size_t i=order;i<n;i++)
//        knots[i]=(i-order+1)*1.f/(n-order+1);
    for(size_t i=0;i<knots.size();i++)
        knots[i]=i*1.f/(knots.size()-1);
    for(size_t i=0;i<knots.size();i++)
        std::cout<<knots[i]<<" ";
    std::cout<<std::endl;

    std::vector<float> b_spline_base;

    // t sample at [0,1)
    this->interpolationP.reserve(1.0/step*3+3);
    for(float t=0;t<1.0f;t+=step)
    {
        float x=0.f,y=0.f,z=0.f;
        b_spline_base.clear();
        b_spline_base.assign(n+order,0.f);
        for(size_t i=0;i<b_spline_base.size()-1;i++){
            if(t>=knots[i] && t<knots[i+1]){
                b_spline_base[i]=1.f;
            }
        }
        for(int k=2;k<=order;k++){

            for(size_t i=0;i<n+order;i++){
                b_spline_base[i]=(t-knots[i])/(knots[i+k-1]-knots[i])*b_spline_base[i]
                                +(knots[i+k]-t)/(knots[i+k]-knots[i+1])*(i+1>=n+order?0.f:b_spline_base[i+1]);
            }

        }
        for(size_t i=0;i<n;i++){
            std::cout<<"b_spline_base "<<b_spline_base[i]<<std::endl;
            x+=controlP[i*3+0]*b_spline_base[i];
            y+=controlP[i*3+1]*b_spline_base[i];
            z+=controlP[i*3+2]*b_spline_base[i];
        }
        this->interpolationP.push_back(x);
        this->interpolationP.push_back(y);
        this->interpolationP.push_back(z);
        std::cout<<t<<" "<<x<<" "<<y<<" "<<z<<std::endl;
    }

    for(size_t i=0;i<interpolationP.size()/3;i++)
        std::cout<<interpolationP[i*3]<<" "
                <<interpolationP[i*3+1]<<" "
                <<interpolationP[i*3+2]<<std::endl;
    std::cout<<__FUNCTION__ <<std::endl;
    std::cout<<interpolationP.size()/3<<std::endl;
    return this->interpolationP;
}

const std::vector<B_SPLINE_DATATYPE> &BSplineCurve::getApproximationP(std::vector<B_SPLINE_DATATYPE> &controlP)
{

}

const std::vector<B_SPLINE_DATATYPE> &BSplineCurve::DeBoor_Cox(std::vector<float> &controlP)
{
    interpolationP.clear();
    size_t n=controlP.size()/3;
    knots.resize(n+order);
    for(size_t i=0;i<order;i++)
        knots[i]=0.f;
    for(size_t i=n;i<n+order;i++)
        knots[i]=1.f;
    for(size_t i=order;i<n;i++)
        knots[i]=(i-order+1)*1.f/(n-order+1);
//    for(size_t i=0;i<knots.size();i++)
//        knots[i]=i*1.f/(knots.size()-1);
    for(size_t i=0;i<knots.size();i++)
        std::cout<<knots[i]<<" ";
    std::cout<<std::endl;

    this->interpolationP.reserve(1.0/step*3+3);

    std::vector<B_SPLINE_DATATYPE> control_p;
    control_p.resize(order*3);

    for(float t=0.f;t<1.0f;t+=step)
    {
        int l;
        for(size_t i=0;i<knots.size()-1;i++){
            if(t>= knots[i] && t<knots[i+1]){
                l=i;
                break;
            }
        }
        assert(l<knots.size());
        std::cout<<"l: "<<l<<std::endl;
        for(size_t i=0;i<order*3;i++){
            control_p[i]=controlP[i+3*(l-order+1)];
        }

        // calculate order-1=degree times
        for(int k=1;k<order;k++)
        {
//            std::cout<<"k: "<<k<<std::endl;
            for(int i=l;i>=l-order+1+k;i--)
            {
                if(i-1<0) continue;
                int j=i-(l-order+1);

                float tij=(t-knots[i])/(knots[i+order-k]-knots[i]);
                std::cout<<"t_i_j: "<<tij<<std::endl;
                control_p[j*3+0]=(1.f-tij)*control_p[(j-1)*3+0]
                        +tij*control_p[j*3+0];
                control_p[j*3+1]=(1.f-tij)*control_p[(j-1)*3+1]
                                 +tij*control_p[j*3+1];
                control_p[j*3+2]=(1.f-tij)*control_p[(j-1)*3+2]
                                 +tij*control_p[j*3+2];
                std::cout<<control_p[j*3+0]<<" "
                        <<control_p[j*3+1]<<" "
                        <<control_p[j*3+2]<<std::endl;
            }
        }
//        std::cout<<t<<" ";
//        std::cout<<control_p[(order-1)*3+0]<<" "
//                <<control_p[(order-1)*3+1]<<" "
//                <<control_p[(order-1)*3+2]<<std::endl;
        this->interpolationP.push_back(control_p[(order-1)*3+0]);
        this->interpolationP.push_back(control_p[(order-1)*3+1]);
        this->interpolationP.push_back(control_p[(order-1)*3+2]);
    }

    control_p.clear();
    std::cout<<"size "<<interpolationP.size()/3<<std::endl;
    return this->interpolationP;
}
