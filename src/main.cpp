#include <iostream>
#include "Displayer.h"
#include "BSplineCurve.h"
#include "BSplineSurface.h"
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

    std::vector<float> surface_p0{
        0.f,5.f,1.2f, 1.f,5.f,0.8f, 2.f,5.f,0.6f, 3.f,5.f,0.5f, 4.f,5.f,0.6f, 5.f,5.f,0.8f, 6.f,5.f,1.2f,
        0.f,4.f,0.9f, 1.f,4.f,0.7f, 2.f,4.f,0.5f, 3.f,4.f,0.4f, 4.f,4.f,0.5f, 5.f,4.f,0.7f, 6.f,4.f,0.9f,
        0.f,3.f,0.8f, 1.f,3.f,0.6f, 2.f,3.f,0.4f, 3.f,3.f,0.3f, 4.f,3.f,0.4f, 5.f,3.f,0.6f, 6.f,3.f,0.8f,
        0.f,2.f,0.9f, 1.f,2.f,0.7f, 2.f,2.f,0.5f, 3.f,2.f,0.4f, 4.f,2.f,0.5f, 5.f,2.f,0.7f, 6.f,2.f,0.9f,
        0.f,1.f,1.2f, 1.f,1.f,0.8f, 2.f,1.f,0.6f, 3.f,1.f,0.5f, 4.f,1.f,0.6f, 5.f,1.f,0.8f, 6.f,1.f,1.2f,
    };
//    int surface0=displayer.addSurface();
//    displayer.addSurfaceControlPoints(surface0,surface_p0,7);
    BSplineSurface b_spline_surface;
    b_spline_surface.setupPitch(7);
    b_spline_surface.setupUVStep(0.003f,0.003f);
    auto& _v_0=b_spline_surface.getInterpolationP(surface_p0);
    int curve22=displayer.addCurve();
    displayer.setCurveDrawMode(curve22,GL_POINTS);
    displayer.addCurveControlPoints(curve22,_v_0);
    displayer.render();
    return 0;
}
