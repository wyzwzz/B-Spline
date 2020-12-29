//
// Created by wyz on 20-12-25.
//

#ifndef B_SPLINE_BSPLINESURFACE_H
#define B_SPLINE_BSPLINESURFACE_H

#include "BSpline.h"

class BSplineSurface : public BSpline{
public:
    BSplineSurface();

    const std::vector<B_SPLINE_DATATYPE>& getInterpolationP(std::vector<B_SPLINE_DATATYPE>& controlP) override;
    const std::vector<B_SPLINE_DATATYPE>& getApproximationP(std::vector<B_SPLINE_DATATYPE>& controlP) override;
public:

    void setupUVStep(float step_u,float step_v);
    void setupUVOrder(int order_u,int order_v);
    void setupPitch(int pitch);
private:
    int pitch;
    float step_u,step_v;
    int order_u,order_v;
    int row,col;
    std::vector<B_SPLINE_DATATYPE> knots_u,knots_v;

    std::vector<B_SPLINE_DATATYPE> interpolationP;
    std::vector<B_SPLINE_DATATYPE> approximationP;
};


#endif //B_SPLINE_BSPLINESURFACE_H
