//
// Created by wyz on 20-12-25.
//

#ifndef B_SPLINE_BSPLINECURVE_H
#define B_SPLINE_BSPLINECURVE_H

#include "BSpline.h"

/**
 * 3-dim B-spline curve
 */
class BSplineCurve : public BSpline{
public:
    BSplineCurve()=default;

    /**
     * @brief get interpolation points by step.
     */
    const std::vector<B_SPLINE_DATATYPE>& getInterpolationP(std::vector<B_SPLINE_DATATYPE>& controlP) override;

    const std::vector<B_SPLINE_DATATYPE>& getApproximationP(std::vector<B_SPLINE_DATATYPE>& controlP) override;

public:
    const std::vector<B_SPLINE_DATATYPE>& DeBoor_Cox(std::vector<B_SPLINE_DATATYPE>& controlP);

private:
    /**
     * x0 y0 z0 x1 y1 z1
     * size=vertex_num*3
     */
    std::vector<B_SPLINE_DATATYPE> interpolationP;
    std::vector<B_SPLINE_DATATYPE> approximationP;
    /**
     *
     */

};


#endif //B_SPLINE_BSPLINECURVE_H
