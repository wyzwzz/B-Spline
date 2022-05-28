#include <iostream>
#include "Displayer.h"
#include "BSplineCurve.h"
#include "BSplineSurface.h"
#include "Ellipsoid.h"
int main(int argc,char** argv)
{
    Displayer displayer(1024,1024);

/*****************************************************************
 * 1.b-spline curve interpolation and approximation
 */
    std::vector<float> control_pts_0{-5.f,2.f,-0.f, -2.f,2.f,-0.f, -2.f,5.f,-0.f, -3.f,8.f,-0.f,
                         -4.f,6.f,-0.f, -5.f,5.f,-0.f, -6.f,8.f,-0.f};

    //red line for connect control_pts_0
    int control_line_0=displayer.addCurve();
    displayer.addCurveControlPoints(control_line_0,control_pts_0);
    displayer.setupCurveColor(control_line_0,{1.f,0.f,0.f});

    //green curve for interpolation
    BSplineCurve b_spline_curve;
    auto& bspline_interpolation_v0=b_spline_curve.getInterpolationP(control_pts_0);
    int b_spline_interpolation_curve0=displayer.addCurve();
    displayer.addCurveControlPoints(b_spline_interpolation_curve0,bspline_interpolation_v0);
    displayer.setupCurveColor(b_spline_interpolation_curve0,{0.f,1.f,0.f});

    //yellow curve for approximation
    auto& b_spline_approximation_v0=b_spline_curve.getApproximationP(control_pts_0);
    int b_spline_approximation_curve0=displayer.addCurve();
    displayer.addCurveControlPoints(b_spline_approximation_curve0,b_spline_approximation_v0);
    displayer.setupCurveColor(b_spline_approximation_curve0,{1.f,1.f,0.f});

/*****************************************************************
 * 2.b-spline surface interpolation and approximation
 */
    //2.1 control points for interpolation
    std::vector<float> control_pts_1{
        0.f,6.f,1.2f, 1.f,6.f,0.8f, 2.f,6.f,0.6f, 3.f,6.f,0.5f, 4.f,6.f,0.6f, 5.f,6.f,0.8f, 6.f,6.f,1.2f,
        0.f,5.f,0.9f, 1.f,5.f,0.7f, 2.f,5.f,0.5f, 3.f,5.f,0.4f, 4.f,5.f,0.5f, 5.f,5.f,0.7f, 6.f,5.f,0.9f,
        0.f,4.f,0.8f, 1.f,4.f,0.6f, 2.f,4.f,0.4f, 3.f,4.f,0.3f, 4.f,4.f,0.4f, 5.f,4.f,0.6f, 6.f,4.f,0.8f,
        0.f,3.f,0.9f, 1.f,3.f,0.7f, 2.f,3.f,0.5f, 3.f,3.f,0.4f, 4.f,3.f,0.5f, 5.f,3.f,0.7f, 6.f,3.f,0.9f,
        0.f,2.f,1.2f, 1.f,2.f,0.8f, 2.f,2.f,0.6f, 3.f,2.f,0.5f, 4.f,2.f,0.6f, 5.f,2.f,0.8f, 6.f,2.f,1.2f,
    };

    //red points for surface's control points, draw by curve mode using GL_POINTS
    int control_points_1=displayer.addCurve();
    displayer.addCurveControlPoints(control_points_1,control_pts_1);
    displayer.setCurveDrawMode(control_points_1,GL_POINTS);
    displayer.setupCurvePointSize(control_points_1,3.f);
    displayer.setupCurveColor(control_points_1,{1.f,0.f,0.f});

    //green points for b-spline surface interpolation results
    BSplineSurface b_spline_surface;
    b_spline_surface.setupPitch(7);
    b_spline_surface.setupUVStep(0.01f,0.01f);
    auto b_spline_interpolation_v1=b_spline_surface.getInterpolationP(control_pts_1);
    int b_spline_interpolation_surface0=displayer.addCurve();
    displayer.setCurveDrawMode(b_spline_interpolation_surface0,GL_POINTS);
    displayer.addCurveControlPoints(b_spline_interpolation_surface0,b_spline_interpolation_v1);
    displayer.setupCurveColor(b_spline_interpolation_surface0,{0.f,1.f,0.f});

    //2.2 control points for approximation
    std::vector<float> control_pts_2{
            0.f,-6.f,1.2f, 1.f,-6.f,0.8f, 2.f,-6.f,0.6f, 3.f,-6.f,0.5f, 4.f,-6.f,0.6f, 5.f,-6.f,0.8f, 6.f,-6.f,1.2f,
            0.f,-5.f,0.9f, 1.f,-5.f,0.7f, 2.f,-5.f,0.5f, 3.f,-5.f,0.4f, 4.f,-5.f,0.5f, 5.f,-5.f,0.7f, 6.f,-5.f,0.9f,
            0.f,-4.f,0.8f, 1.f,-4.f,0.6f, 2.f,-4.f,0.4f, 3.f,-4.f,0.3f, 4.f,-4.f,0.4f, 5.f,-4.f,0.6f, 6.f,-4.f,0.8f,
            0.f,-3.f,0.9f, 1.f,-3.f,0.7f, 2.f,-3.f,0.5f, 3.f,-3.f,0.4f, 4.f,-3.f,0.5f, 5.f,-3.f,0.7f, 6.f,-3.f,0.9f,
            0.f,-2.f,1.2f, 1.f,-2.f,0.8f, 2.f,-2.f,0.6f, 3.f,-2.f,0.5f, 4.f,-2.f,0.6f, 5.f,-2.f,0.8f, 6.f,-2.f,1.2f,
    };

    //red points for surface's control points, draw by curve mode using GL_POINTS
    int control_points_2=displayer.addCurve();
    displayer.addCurveControlPoints(control_points_2,control_pts_2);
    displayer.setCurveDrawMode(control_points_2,GL_POINTS);
    displayer.setupCurvePointSize(control_points_2,3.f);
    displayer.setupCurveColor(control_points_2,{1.f,0.f,0.f});

    //yellow points for b-spline surface approximation results
    auto b_spline_approximation_v1=b_spline_surface.getApproximationP(control_pts_2);
    int b_spline_approximation_surface0=displayer.addCurve();
    displayer.setCurveDrawMode(b_spline_approximation_surface0,GL_POINTS);
    displayer.addCurveControlPoints(b_spline_approximation_surface0,b_spline_approximation_v1);
    displayer.setupCurveColor(b_spline_approximation_surface0,{1.f,1.f,0.f});

/*****************************************************************
 * 3.b-spline surface interpolation for sphere
 */
    //get control points from sphere equation
    Ellipsoid ellipsoid;
    int row,col;
    ellipsoid.setAixsLength(1.f,1.f,1.f);
    ellipsoid.setSampleInterval(0.2,45);
    ellipsoid.getRowAndCol(&row,&col);
    auto control_points=ellipsoid.getControlPoints();

    //get interpolation points for sphere, notice drawing surface need to get normals
    BSplineSurface b_spline_surface1;
    b_spline_surface1.setupPitch(col);
    b_spline_surface1.setupUVStep(0.01f,0.01f);
    auto b_spline_interpolation_v2=b_spline_surface1.getInterpolationP(control_points);
    auto normals=ellipsoid.getNormals(b_spline_interpolation_v2);
    b_spline_interpolation_v2.insert(b_spline_interpolation_v2.cend(),normals.cbegin(),normals.cend());

    //red points for surface's control points, draw by curve mode using GL_POINTS
    int control_points_3=displayer.addCurve();
    displayer.setCurveDrawMode(control_points_3,GL_POINTS);
    displayer.addCurveControlPoints(control_points_3,control_points);
    displayer.setupCurvePointSize(control_points_3,3.f);
    displayer.setupCurveColor(control_points_3,{1.f,0.f,0.f});

    //green surface for b-spline surface interpolation for sphere
    int b_spline_interpolation_surface1=displayer.addSurface();
    displayer.addSurfaceControlPoints(b_spline_interpolation_surface1,b_spline_interpolation_v2,b_spline_surface1.getResultPitch()) ;
    displayer.setupSurfaceColor(b_spline_interpolation_surface1,{0.f,1.f,0.f});

    //calculate accuracy for approximating a sphere by using B-spline surface fitting and interpolation
    float acc1=ellipsoid.calculateAccuracy(control_points);
    std::cout<<"control points: "<<acc1<<std::endl;
    float acc2=ellipsoid.calculateAccuracy(b_spline_interpolation_v2);
    std::cout<<"interpolated points: "<<acc2<<std::endl;

/*****************************************************************
 * 4.test b-spline curve interpolation for ellipse and approximation for circle
 */
    //4.1 b-spline curve interpolation for ellipse
    Ellipse ellipse0;
    ellipse0.setSampleInterval(45);
    ellipse0.setAixsLength(2.f,1.5f);
    auto control_pts_3=ellipse0.getControlPoints();

    //red line for connect control_pts_3
    int b_spline_control_points1=displayer.addCurve();
    displayer.addCurveControlPoints(b_spline_control_points1,control_pts_3);
    displayer.setupCurveColor(b_spline_control_points1,{1.f,0.f,0.f});

    //green curve for b-spline interpolation results for ellipse
    int b_spline_interpolation_curve1=displayer.addCurve();
    auto& b_spline_interpolation_v3=b_spline_curve.getInterpolationP(control_pts_3);
    displayer.addCurveControlPoints(b_spline_interpolation_curve1,b_spline_interpolation_v3);
    displayer.setupCurveColor(b_spline_interpolation_curve1,{0.f,1.f,0.f});

    //4.2 b-spline curve approximation for circle
/*    Ellipse ellipse1;
    ellipse1.setSampleInterval(45);
    ellipse1.setAixsLength(1.8f,1.8f);
    auto control_pts_4=ellipse1.getControlPoints();

    //red line for connect control_pts_4
    int b_spline_control_points2=displayer.addCurve();
    displayer.addCurveControlPoints(b_spline_control_points2,control_pts_4);
    displayer.setupCurveColor(b_spline_control_points2,{1.f,0.f,0.f});

    //green curve for b-spline approximation results for circle
    int b_spline_approximation_curve1=displayer.addCurve();
    auto& b_spline_approximation_v3=b_spline_curve.getApproximationP(control_pts_4);
    displayer.addCurveControlPoints(b_spline_approximation_curve1,b_spline_approximation_v3);
    displayer.setupCurveColor(b_spline_approximation_curve1,{0.f,1.f,0.f});*/

/*****************************************************************
 * 4.render
 */
    displayer.render();

    return 0;
}
