//
// Created by wyz on 20-12-25.
//

#ifndef B_SPLINE_BSPLINE_H
#define B_SPLINE_BSPLINE_H

#include <vector>
#include "Config.h"
class BSpline{
public:
    BSpline()=default;

    virtual const std::vector<B_SPLINE_DATATYPE>& getInterpolationP(std::vector<B_SPLINE_DATATYPE>& controlP)=0;
    virtual const std::vector<B_SPLINE_DATATYPE>& getApproximationP(std::vector<B_SPLINE_DATATYPE>& controlP)=0;
    void setupStep(float step){this->step=step;}
private:
    float step;
    int order;
    std::vector<B_SPLINE_DATATYPE> knots;
};


#endif //B_SPLINE_BSPLINE_H