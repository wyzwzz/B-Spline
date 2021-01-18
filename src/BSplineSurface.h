//
// Created by wyz on 20-12-25.
//

#ifndef B_SPLINE_BSPLINESURFACE_H
#define B_SPLINE_BSPLINESURFACE_H

#include "BSpline.h"

class BSplineSurface : public BSpline{
public:
    BSplineSurface();

    const std::vector<B_SPLINE_DATATYPE>& getInterpolationP(const std::vector<B_SPLINE_DATATYPE>& controlP) override;
    const std::vector<B_SPLINE_DATATYPE>& getApproximationP(std::vector<B_SPLINE_DATATYPE>& controlP) override;
    const std::vector<B_SPLINE_DATATYPE>& BasicFuncMethod(std::vector<B_SPLINE_DATATYPE>& controlP);
public:

    void setupUVStep(float step_u,float step_v);
    void setupUVOrder(int order_u,int order_v);
    void setupApproximationUVH(size_t h_u,size_t h_v);
    void setupPitch(int pitch);
    int getResultPitch() const{return this->result_pitch;}
private:
    int pitch;
    float step_u,step_v;
    int order_u,order_v;
    int row,col;
    int h_u,h_v;
    std::vector<B_SPLINE_DATATYPE> knots_u,knots_v;

    std::vector<B_SPLINE_DATATYPE> interpolationP;
    std::vector<B_SPLINE_DATATYPE> approximationP;

    int result_pitch;
};


#endif //B_SPLINE_BSPLINESURFACE_H
