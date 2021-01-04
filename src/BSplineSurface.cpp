//
// Created by wyz on 20-12-25.
//

#include "BSplineSurface.h"
#include <iostream>
#include <cassert>
#include <Eigen/Core>
#include <Eigen/LU>
#include "BSplineCurve.h"

BSplineSurface::BSplineSurface()
        : step_u(step), step_v(step), order_u(order), order_v(order), pitch(0) {

}

void BSplineSurface::setupUVStep(float step_u, float step_v) {
    this->step_u = step_u;
    this->step_v = step_v;
}

void BSplineSurface::setupUVOrder(int order_u, int order_v) {
    this->order_u = order_u;
    this->order_v = order_v;
}

void BSplineSurface::setupPitch(int pitch) {
    this->pitch = pitch;
}

const std::vector<B_SPLINE_DATATYPE> &BSplineSurface::getInterpolationP(std::vector<float> &controlP) {
    if (pitch == 0) {
        std::cout << "ERROR: pitch is zero!" << std::endl;
        return {};
    }
    this->interpolationP.clear();
    //number of control points
    size_t n = controlP.size() / 3;
    //col and row is used for control pts
    col = pitch;
    row = n / col;
    assert(col * row == n);
    knots_u.resize(col + order_u);
    for (size_t i = 0; i < order_u; i++)
        knots_u[i] = 0.f;
    for (size_t i = col; i < col + order_u; i++)
        knots_u[i] = 1.f;
    for (size_t i = order_u; i < col; i++)
        knots_u[i] = (i - order_u + 1) * 1.f / (col - order_u + 1);
    for (size_t i = 0; i < knots_u.size(); i++)
        std::cout << knots_u[i] << " ";
    std::cout << std::endl;
    knots_v.resize(row + order_v);
    for (size_t i = 0; i < order_v; i++)
        knots_v[i] = 0.f;
    for (size_t i = row; i < row + order_v; i++)
        knots_v[i] = 1.f;
    for (size_t i = order_v; i < row; i++)
        knots_v[i] = (i - order_v + 1) * 1.f / (row - order_v + 1);
    for (size_t i = 0; i < knots_v.size(); i++)
        std::cout << knots_v[i] << " ";
    std::cout << std::endl;
    this->interpolationP.reserve(1.0 / step_v * 1.0 / step_u * 3);

    std::vector<float> t_u;
    t_u.reserve(col);
    for (size_t i = 0; i < col; i++)
        t_u.push_back(i * 1.f / (col - 1));
    std::vector<float> t_v;
    t_v.reserve(row);
    for (size_t i = 0; i < row; i++)
        t_v.push_back(i * 1.f / (row - 1));

    std::vector<std::vector<B_SPLINE_DATATYPE>> temp_bspline_curves;
    temp_bspline_curves.resize(row);
    std::vector<B_SPLINE_DATATYPE> b_spline_base;
    std::vector<B_SPLINE_DATATYPE> control_pts;
    control_pts.resize(col * 3);

    BSplineCurve bspline_curve;

    for (size_t r = 0; r < row; r++) {
        temp_bspline_curves[r].reserve(1.0 / step_u * 3);
#ifndef USE_B_SPLINE_CURVE
        Eigen::MatrixXf N(col, col);
        Eigen::MatrixXf D(col, 3);
        for (size_t i = 0; i < col; i++) {
            D(i, 0) = controlP[(r * col + i) * 3 + 0];
            D(i, 1) = controlP[(r * col + i) * 3 + 1];
            D(i, 2) = controlP[(r * col + i) * 3 + 2];
        }
//        b_spline_base.resize(col+order_u,0.f);
        for (size_t t_i = 0; t_i < t_u.size(); t_i++) {
            b_spline_base.assign(col + order_u, 0.f);

            for (size_t i = 0; i < b_spline_base.size() - 1; i++) {
                if (t_u[t_i] == 1.f) {
                    if (t_u[t_i] >= knots_u[i] && t_u[t_i] <= knots_u[i + 1]) {
                        b_spline_base[i] = 1.f;
                        break;
                    }
                } else if (t_u[t_i] >= knots_u[i] && t_u[t_i] < knots_u[i + 1]) {
                    b_spline_base[i] = 1.f;
                }
            }
            for (int k = 2; k <= order_u; k++) {
                for (size_t i = 0; i < col + order_u; i++) {
                    float n_ik, n_i1k;
                    if ((knots_u[i + k - 1] - knots_u[i]) == 0.f)
                        n_ik = 0.f;
                    else
                        n_ik = i + k - 1 >= col + order_u ? 0.f : (t_u[t_i] - knots_u[i]) /
                                                                  (knots_u[i + k - 1] - knots_u[i]) * b_spline_base[i];
                    if ((knots_u[i + k] - knots_u[i + 1]) == 0.f)
                        n_i1k = 0.f;
                    else
                        n_i1k = i + k >= col + order_u ? 0.f : (knots_u[i + k] - t_u[t_i]) /
                                                               (knots_u[i + k] - knots_u[i + 1]) * b_spline_base[i + 1];
                    b_spline_base[i] = n_ik + n_i1k;
                }
            }
            for (size_t i = 0; i < col; i++)
                N(t_i, i) = b_spline_base[i];
        }
        Eigen::MatrixXf P = N.lu().solve(D);
        std::cout << "P\n" << P << std::endl;
        for (size_t i = 0; i < col; i++) {
            control_pts[i * 3 + 0] = P(i, 0);
            control_pts[i * 3 + 1] = P(i, 1);
            control_pts[i * 3 + 2] = P(i, 2);
        }
#else
        for(size_t i=0;i<col;i++){
            control_pts[i*3+0]=controlP[(r*col+i)*3+0];
            control_pts[i*3+1]=controlP[(r*col+i)*3+1];
            control_pts[i*3+2]=controlP[(r*col+i)*3+2];
        }
#endif
        auto result_pts = bspline_curve.BaseFuncMethod(control_pts);
        temp_bspline_curves[r].insert(temp_bspline_curves[r].end(),
                                      result_pts.begin(),
                                      result_pts.end());
    }
//    for(int r=0;r<2;r++) {
//        interpolationP.insert(interpolationP.end(),
//                              temp_bspline_curves[r].begin(),
//                              temp_bspline_curves[r].end());
//    }
    control_pts.resize(row * 3, 0.f);
    for (size_t c = 0; c < temp_bspline_curves[0].size() / 3; c++) {
        Eigen::MatrixXf N(row, row);
        Eigen::MatrixXf D(row, 3);
        for (size_t i = 0; i < row; i++) {
            D(i, 0) = temp_bspline_curves[i][c * 3 + 0];
            D(i, 1) = temp_bspline_curves[i][c * 3 + 1];
            D(i, 2) = temp_bspline_curves[i][c * 3 + 2];
        }
        for (size_t t_i = 0; t_i < t_v.size(); t_i++) {
            b_spline_base.assign(row + order_v, 0.f);
            for (size_t i = 0; i < b_spline_base.size() - 1; i++) {
                if (t_v[t_i] == 1.f) {
                    if (t_v[t_i] >= knots_v[i] && t_v[t_i] <= knots_v[i + 1]) {
                        b_spline_base[i] = 1.f;
                        break;
                    }
                } else if (t_v[t_i] >= knots_v[i] && t_v[t_i] < knots_v[i + 1]) {
                    b_spline_base[i] = 1.f;
                }
            }
            for (int k = 2; k <= order_v; k++) {
                for (size_t i = 0; i < row + order_v; i++) {
                    float n_ik, n_i1k;
                    if ((knots_v[i + k - 1] - knots_v[i]) == 0.f)
                        n_ik = 0.f;
                    else
                        n_ik = i + k - 1 >= row + order_v ? 0.f : (t_v[t_i] - knots_v[i]) /
                                                                  (knots_v[i + k - 1] - knots_v[i]) * b_spline_base[i];
                    if ((knots_v[i + k] - knots_v[i + 1]) == 0.f)
                        n_i1k = 0.f;
                    else
                        n_i1k = i + k >= row + order_v ? 0.f : (knots_v[i + k] - t_v[t_i]) /
                                                               (knots_v[i + k] - knots_v[i + 1]) * b_spline_base[i + 1];
                    b_spline_base[i] = n_ik + n_i1k;
                }
            }
            for (size_t i = 0; i < row; i++)
                N(t_i, i) = b_spline_base[i];
        }
        Eigen::MatrixXf P = N.lu().solve(D);
        std::cout << "P\n" << P << std::endl;
        for (size_t i = 0; i < row; i++) {
            control_pts[i * 3 + 0] = P(i, 0);
            control_pts[i * 3 + 1] = P(i, 1);
            control_pts[i * 3 + 2] = P(i, 2);
        }
        auto result_pts = bspline_curve.BaseFuncMethod(control_pts);
        this->interpolationP.insert(this->interpolationP.end(),
                                    result_pts.begin(),
                                    result_pts.end());
    }
    return this->interpolationP;
}

const std::vector<B_SPLINE_DATATYPE> &BSplineSurface::getApproximationP(std::vector<float> &controlP) {
    return this->interpolationP;
}

const std::vector<B_SPLINE_DATATYPE> &BSplineSurface::BasicFuncMethod(std::vector<float> &controlP) {
    if (pitch == 0) {
        std::cout << "ERROR: pitch is zero!" << std::endl;
        return {};
    }
    this->interpolationP.clear();
    //number of control points
    size_t n = controlP.size() / 3;
    //col and row is for control pts
    col = pitch;
    row = n / col;
    assert(col * row == n);
    knots_u.resize(col + order_u);
    for (size_t i = 0; i < order_u; i++)
        knots_u[i] = 0.f;
    for (size_t i = col; i < col + order_u; i++)
        knots_u[i] = 1.f;
    for (size_t i = order_u; i < col; i++)
        knots_u[i] = (i - order_u + 1) * 1.f / (col - order_u + 1);
    for (size_t i = 0; i < knots_u.size(); i++)
        std::cout << knots_u[i] << " ";
    std::cout << std::endl;
    knots_v.resize(row + order_v);
    for (size_t i = 0; i < order_v; i++)
        knots_v[i] = 0.f;
    for (size_t i = row; i < row + order_v; i++)
        knots_v[i] = 1.f;
    for (size_t i = order_v; i < row; i++)
        knots_v[i] = (i - order_v + 1) * 1.f / (row - order_v + 1);
    for (size_t i = 0; i < knots_v.size(); i++)
        std::cout << knots_v[i] << " ";
    std::cout << std::endl;

    this->interpolationP.reserve(1.0 / step_v * 1.0 / step_u * 3);

    //select u-direction and get row b-spline curves(col control pts)
    std::vector<std::vector<B_SPLINE_DATATYPE>> temp_bspline_curves;
    temp_bspline_curves.resize(row);
    std::vector<B_SPLINE_DATATYPE> control_p;
    control_p.resize(order_u * 3);
    for (size_t r = 0; r < row; r++) {
        temp_bspline_curves[r].reserve(1.0 / step_u * 3);
        for (float u = 0.f; u < 1.f; u += step_u) {
            int l;
            for (size_t i = 0; i < knots_u.size() - 1; i++) {
                if (u >= knots_u[i] && u < knots_u[i + 1]) {
                    l = i;
                    break;
                }
            }
            assert(l < knots_u.size());
            for (size_t i = 0; i < order_u * 3; i++) {
                control_p[i] = controlP[r * col * 3 + i + 3 * (l - order_u + 1)];
            }
            for (int k = 1; k < order; k++) {
                for (int i = l; i >= l - order_u + 1 + k; i--) {
                    if (i < 1) continue;
                    int j = i - (l - order_u + 1);

                    float tij = (u - knots_u[i]) / (knots_u[i + order_u - k] - knots_u[i]);
                    control_p[j * 3 + 0] = (1.f - tij) * control_p[(j - 1) * 3 + 0]
                                           + tij * control_p[j * 3 + 0];
                    control_p[j * 3 + 1] = (1.f - tij) * control_p[(j - 1) * 3 + 1]
                                           + tij * control_p[j * 3 + 1];
                    control_p[j * 3 + 2] = (1.f - tij) * control_p[(j - 1) * 3 + 2]
                                           + tij * control_p[j * 3 + 2];
                }
            }
            temp_bspline_curves[r].push_back(control_p[(order_u - 1) * 3 + 0]);
            temp_bspline_curves[r].push_back(control_p[(order_u - 1) * 3 + 1]);
            temp_bspline_curves[r].push_back(control_p[(order_u - 1) * 3 + 2]);
        }
        std::cout << "temp_bspline_curves size:";
        std::cout << temp_bspline_curves[r].size() / 3 << std::endl;
    }
//    assert(temp_bspline_curves.size()==row);

    control_p.clear();
    std::vector<B_SPLINE_DATATYPE> temp_controlP;
    temp_controlP.resize(row * 3);
    // u-direction temp b-spline's pts number.
    for (size_t c = 0; c < temp_bspline_curves[0].size() / 3; c++) {
        for (size_t i = 0; i < row; i++) {
            temp_controlP[i * 3 + 0] = temp_bspline_curves[i][c * 3 + 0];
            temp_controlP[i * 3 + 1] = temp_bspline_curves[i][c * 3 + 1];
            temp_controlP[i * 3 + 2] = temp_bspline_curves[i][c * 3 + 2];
        }
        for (float v = 0.f; v < 1.f; v += step_v) {
            int l;
            for (size_t i = 0; i < knots_v.size() - 1; i++) {
                if (v >= knots_v[i] && v < knots_v[i + 1]) {
                    l = i;
                    break;
                }
            }
            assert(l < knots_v.size());
            for (size_t i = 0; i < order_v * 3; i++) {
                control_p[i] = temp_controlP[i + 3 * (l - order_v + 1)];
            }
            for (int k = 1; k < order_v; k++) {
                for (int i = l; i >= l - order_v + 1 + k; i--) {
                    if (i < 1) continue;
                    int j = i - (l - order_v + 1);

                    float tij = (v - knots_v[i]) / (knots_v[i + order_v - k] - knots_v[i]);
                    control_p[j * 3 + 0] = (1.f - tij) * control_p[(j - 1) * 3 + 0]
                                           + tij * control_p[j * 3 + 0];
                    control_p[j * 3 + 1] = (1.f - tij) * control_p[(j - 1) * 3 + 1]
                                           + tij * control_p[j * 3 + 1];
                    control_p[j * 3 + 2] = (1.f - tij) * control_p[(j - 1) * 3 + 2]
                                           + tij * control_p[j * 3 + 2];
                }
            }
            this->interpolationP.push_back(control_p[(order_v - 1) * 3 + 0]);
            this->interpolationP.push_back(control_p[(order_v - 1) * 3 + 1]);
            this->interpolationP.push_back(control_p[(order_v - 1) * 3 + 2]);

        }
    }
//    for(int i=0;i<temp_bspline_curves.size();i++){
//        for(size_t j=0;j<temp_bspline_curves[i].size();j++)
//            this->interpolationP.push_back(temp_bspline_curves[i][j]);
//    }
    std::cout << interpolationP.size() / 3 << std::endl;
    return this->interpolationP;
}
