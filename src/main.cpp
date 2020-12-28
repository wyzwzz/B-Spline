#include <iostream>
#include "Displayer.h"
#include "BSplineCurve.h"
int main() {
    std::cout << "B-Spline" << std::endl;
    Displayer displayer(1024,1024);
    std::vector<float> v{5.f,1.f,0.f,
                         1.f,1.f,0.f,
                         1.f,5.f,0.f,
                         3.f,8.f,0.f,
                         5.f,3.f,0.f};
    std::vector<float> v2{
            -5.f,-1.f,0.f,
            -1.f,-1.f,0.f,
            -1.f,-5.f,0.f,
            -3.f,-8.f,0.f,
            -5.f,-3.f,0.f
    };
    int curve0=displayer.addCurve();
    BSplineCurve b_spline_curve;
    auto& v_=b_spline_curve.DeBoor_Cox(v);
    displayer.addCurveControlPoints(curve0,v_);
    int curve1=displayer.addCurve();
    auto& v_2=b_spline_curve.DeBoor_Cox(v2);
    displayer.addCurveControlPoints(curve1,v_2);
    int curve00=displayer.addCurve();
    displayer.addCurveControlPoints(curve00,v);
    int curve11=displayer.addCurve();
    displayer.addCurveControlPoints(curve11,v2);
    displayer.render();
    return 0;
}
