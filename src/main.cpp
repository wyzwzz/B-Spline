#include <iostream>
#include "Displayer.h"
#include "BSplineCurve.h"
#include "BSplineSurface.h"
#include "Ellipsoid.h"
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
    auto& v_=b_spline_curve.getApproximationP(v);
    displayer.addCurveControlPoints(curve0,v_);
//    int curve1=displayer.addCurve();
//    auto& v_2=b_spline_curve.DeBoor_Cox(v2);
//    displayer.addCurveControlPoints(curve1,v_2);
    int curve00=displayer.addCurve();
    displayer.addCurveControlPoints(curve00,v);
//    int curve11=displayer.addCurve();
//    displayer.addCurveControlPoints(curve11,v2);

    std::vector<float> surface_p0{
        0.f,5.f,1.2f, 1.f,5.f,0.8f, 2.f,5.f,0.6f, 3.f,5.f,0.5f, 4.f,5.f,0.6f, 5.f,5.f,0.8f, 6.f,5.f,1.2f,
        0.f,4.f,0.9f, 1.f,4.f,0.7f, 2.f,4.f,0.5f, 3.f,4.f,0.4f, 4.f,4.f,0.5f, 5.f,4.f,0.7f, 6.f,4.f,0.9f,
        0.f,3.f,0.8f, 1.f,3.f,0.6f, 2.f,3.f,0.4f, 3.f,3.f,0.3f, 4.f,3.f,0.4f, 5.f,3.f,0.6f, 6.f,3.f,0.8f,
        0.f,2.f,0.9f, 1.f,2.f,0.7f, 2.f,2.f,0.5f, 3.f,2.f,0.4f, 4.f,2.f,0.5f, 5.f,2.f,0.7f, 6.f,2.f,0.9f,
        0.f,1.f,1.2f, 1.f,1.f,0.8f, 2.f,1.f,0.6f, 3.f,1.f,0.5f, 4.f,1.f,0.6f, 5.f,1.f,0.8f, 6.f,1.f,1.2f,
    };
//    int surface0=displayer.addSurface();
//    displayer.addSurfaceControlPoints(surface0,surface_p0,7);
//    displayer.setupSurfaceColor(surface0,{0.f,1.f,0.f});
    BSplineSurface b_spline_surface;
    b_spline_surface.setupPitch(7);
    b_spline_surface.setupUVStep(0.003f,0.003f);
    auto& _v_0=b_spline_surface.getApproximationP(surface_p0);
    std::cout<<"size: "<<_v_0.size()/3<<std::endl;
    int curve22=displayer.addCurve();
    displayer.setCurveDrawMode(curve22,GL_POINTS);
    displayer.addCurveControlPoints(curve22,_v_0);

    //===============================================
    Ellipsoid ellipsoid;
    int row,col;
    ellipsoid.getRowAndCol(&row,&col);
    auto control_points=ellipsoid.getControlPoints();
    for(size_t i=0;i<control_points.size()/3;i++)
        std::cout<<"point "<<i<<": "<<control_points[i*3]<<" "<<control_points[i*3+1]<<" "<<control_points[i*3+2]<<std::endl;
    BSplineSurface b_spline_surface1;
    b_spline_surface1.setupPitch(col);
    b_spline_surface1.setupUVStep(0.003f,0.003f);
    auto& solid_v=b_spline_surface1.getInterpolationP(control_points);
    std::cout<<"approxiamtion points num: "<<solid_v.size()/3<<std::endl;
    for(size_t i=0;i<solid_v.size()/3;i++)
        std::cout<<"point "<<i<<": "<<solid_v[i*3]<<" "<<solid_v[i*3+1]<<" "<<solid_v[i*3+2]<<std::endl;
    int curve33=displayer.addCurve();
    displayer.setCurveDrawMode(curve33,GL_POINTS);
    std::vector<float> test;
    int i=18;
    test.insert(test.end(),solid_v.begin()+i*303,solid_v.begin()+303*(i+1));
    displayer.addCurveControlPoints(curve33,solid_v);
    //===============================================
//    Ellipse ellipse;
//    ellipse.setSampleInterval(45);
//    auto control_pts=ellipse.getControlPoints();
//
//    int curve44=displayer.addCurve();
////    displayer.setCurveDrawMode(curve44,GL_POINTS);
//    for(int i=0;i<control_pts.size()/3;i++)
//        std::cout<<control_pts[i*3+0]<<" "
//                <<control_pts[i*3+1]<<" "
//                <<control_pts[i*3+2]<<std::endl;
//    auto& res_pts=b_spline_curve.getInterpolationP(control_pts);
//    displayer.addCurveControlPoints(curve44,res_pts);


    //===============================================
    displayer.render();
    return 0;
}
