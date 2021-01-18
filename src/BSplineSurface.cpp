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

const std::vector<B_SPLINE_DATATYPE> &BSplineSurface::getInterpolationP(const std::vector<float> &controlP) {
    if (pitch == 0) {
        std::cout << "ERROR: pitch is zero!" << std::endl;
        return this->interpolationP;
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
    knots_v.resize(row + order_v);
    for (size_t i = 0; i < order_v; i++)
        knots_v[i] = 0.f;
    for (size_t i = row; i < row + order_v; i++)
        knots_v[i] = 1.f;
    for (size_t i = order_v; i < row; i++)
        knots_v[i] = (i - order_v + 1) * 1.f / (row - order_v + 1);

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
    bspline_curve.setupStep(step_u);
    for (size_t r = 0; r < row; r++) {
        temp_bspline_curves[r].reserve(1.0 / step_u * 3);
        Eigen::MatrixXf N(col, col);
        Eigen::MatrixXf D(col, 3);
        for (size_t i = 0; i < col; i++) {
            D(i, 0) = controlP[(r * col + i) * 3 + 0];
            D(i, 1) = controlP[(r * col + i) * 3 + 1];
            D(i, 2) = controlP[(r * col + i) * 3 + 2];
        }
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
                    if (i + k - 1 >= col + order_u || (knots_u[i + k - 1] - knots_u[i]) == 0.f)
                        n_ik = 0.f;
                    else
                        n_ik = (t_u[t_i] - knots_u[i]) / (knots_u[i + k - 1] - knots_u[i]) * b_spline_base[i];
                    if (i + k >= col + order_u || (knots_u[i + k] - knots_u[i + 1]) == 0.f)
                        n_i1k = 0.f;
                    else
                        n_i1k = (knots_u[i + k] - t_u[t_i]) / (knots_u[i + k] - knots_u[i + 1]) * b_spline_base[i + 1];
                    b_spline_base[i] = n_ik + n_i1k;
                }
            }
            for (size_t i = 0; i < col; i++)
                N(t_i, i) = b_spline_base[i];
        }
        Eigen::MatrixXf P = N.lu().solve(D);
        for (size_t i = 0; i < col; i++) {
            control_pts[i * 3 + 0] = P(i, 0);
            control_pts[i * 3 + 1] = P(i, 1);
            control_pts[i * 3 + 2] = P(i, 2);
        }
        auto &result_pts = bspline_curve.BaseFuncMethod(control_pts);
        temp_bspline_curves[r].insert(temp_bspline_curves[r].end(),
                                      result_pts.begin(),
                                      result_pts.end());
    }

    bspline_curve.setupStep(step_v);
    control_pts.resize(row * 3, 0.f);
    result_pitch = temp_bspline_curves[0].size() / 3;
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
                    if (i + k - 1 >= row + order_v || (knots_v[i + k - 1] - knots_v[i]) == 0.f)
                        n_ik = 0.f;
                    else
                        n_ik = (t_v[t_i] - knots_v[i]) / (knots_v[i + k - 1] - knots_v[i]) * b_spline_base[i];
                    if (i + k >= row + order_v || (knots_v[i + k] - knots_v[i + 1]) == 0.f)
                        n_i1k = 0.f;
                    else
                        n_i1k = (knots_v[i + k] - t_v[t_i]) / (knots_v[i + k] - knots_v[i + 1]) * b_spline_base[i + 1];
                    b_spline_base[i] = n_ik + n_i1k;
                }
            }
            for (size_t i = 0; i < row; i++)
                N(t_i, i) = b_spline_base[i];
        }
        Eigen::MatrixXf P = N.lu().solve(D);
        for (size_t i = 0; i < row; i++) {
            control_pts[i * 3 + 0] = P(i, 0);
            control_pts[i * 3 + 1] = P(i, 1);
            control_pts[i * 3 + 2] = P(i, 2);
        }
        auto &result_pts = bspline_curve.BaseFuncMethod(control_pts);
        this->interpolationP.insert(this->interpolationP.end(),
                                    result_pts.begin(),
                                    result_pts.end());
    }
    return this->interpolationP;
}

const std::vector<B_SPLINE_DATATYPE> &BSplineSurface::getApproximationP(std::vector<float> &controlP) {
    if (pitch == 0) {
        std::cout << "ERROR: pitch is zero!" << std::endl;
        return this->interpolationP;
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
    knots_v.resize(row + order_v);
    for (size_t i = 0; i < order_v; i++)
        knots_v[i] = 0.f;
    for (size_t i = row; i < row + order_v; i++)
        knots_v[i] = 1.f;
    for (size_t i = order_v; i < row; i++)
        knots_v[i] = (i - order_v + 1) * 1.f / (row - order_v + 1);

    this->interpolationP.reserve(1.0 / step_v * 1.0 / step_u * 3);

    std::vector<float> t_u;
    t_u.reserve(col);
    for (size_t i = 0; i < col; i++)
        t_u.push_back(i * 1.f / (col - 1));
    std::vector<float> t_v;
    t_v.reserve(row);
    for (size_t i = 0; i < row; i++)
        t_v.push_back(i * 1.f / (row - 1));

    h_u = col - 1;
    h_v = row - 1;

    std::vector<B_SPLINE_DATATYPE> b_spline_base;
    std::vector<std::vector<B_SPLINE_DATATYPE>> temp_bspline_curves;
    temp_bspline_curves.resize(row);
    BSplineCurve bspline_curve;

    for (size_t r = 0; r < row; r++) {
        Eigen::MatrixXf N(col - 2, h_u - 2);
        Eigen::MatrixXf D(col, 3);
        Eigen::MatrixXf Qk(col, 3);
        Eigen::MatrixXf Q(h_u - 2, 3);
        for (size_t i = 0; i < col; i++) {
            D(i, 0) = controlP[(r * col + i) * 3 + 0];
            D(i, 1) = controlP[(r * col + i) * 3 + 1];
            D(i, 2) = controlP[(r * col + i) * 3 + 2];
        }
        for (size_t t_i = 1; t_i < t_u.size() - 1; t_i++) {
            b_spline_base.assign(h_u + order_u, 0.f);

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
                for (size_t i = 0; i < h_u + order_u; i++) {
                    float n_ik, n_i1k;
                    if (i + k - 1 >= col + order_u || (knots_u[i + k - 1] - knots_u[i]) == 0.f)
                        n_ik = 0.f;
                    else
                        n_ik = (t_u[t_i] - knots_u[i]) / (knots_u[i + k - 1] - knots_u[i]) * b_spline_base[i];
                    if (i + k >= col + order_u || (knots_u[i + k] - knots_u[i + 1]) == 0.f)
                        n_i1k = 0.f;
                    else
                        n_i1k = (knots_u[i + k] - t_u[t_i]) / (knots_u[i + k] - knots_u[i + 1]) * b_spline_base[i + 1];
                    b_spline_base[i] = n_ik + n_i1k;
                }
            }
            for (size_t i = 1; i < h_u - 1; i++)
                N(t_i - 1, i - 1) = b_spline_base[i];
            Qk(t_i, 0) = D(t_i, 0) - b_spline_base[0] * D(0, 0) - b_spline_base[h_u - 1] * D(col - 1, 0);
            Qk(t_i, 1) = D(t_i, 1) - b_spline_base[0] * D(0, 1) - b_spline_base[h_u - 1] * D(col - 1, 1);
            Qk(t_i, 2) = D(t_i, 2) - b_spline_base[0] * D(0, 2) - b_spline_base[h_u - 1] * D(col - 1, 2);
        }
        for (size_t i = 0; i < h_u - 2; i++) {
            Q(i, 0) = Q(i, 1) = Q(i, 2) = 0.f;
            for (size_t k = 0; k < col - 2; k++) {
                Q(i, 0) += N(k, i) * Qk(k + 1, 0);
                Q(i, 1) += N(k, i) * Qk(k + 1, 1);
                Q(i, 2) += N(k, i) * Qk(k + 1, 2);
            }
        }
        auto Nt_N = N.transpose() * N;
        Eigen::MatrixX3f P = Nt_N.lu().solve(Q);
        std::vector<B_SPLINE_DATATYPE> control_points;
        control_points.reserve(h_u * 3);
        control_points.push_back(D(0, 0));
        control_points.push_back(D(0, 1));
        control_points.push_back(D(0, 2));
        for (size_t i = 0; i < P.rows(); i++) {
            control_points.push_back(P(i, 0));
            control_points.push_back(P(i, 1));
            control_points.push_back(P(i, 2));
        }
        control_points.push_back(D(col - 1, 0));
        control_points.push_back(D(col - 1, 1));
        control_points.push_back(D(col - 1, 2));

        auto &result_pts = bspline_curve.BaseFuncMethod(control_points);
        temp_bspline_curves[r].insert(temp_bspline_curves[r].end(),
                                      result_pts.begin(),
                                      result_pts.end());
    }

    for (size_t c = 0; c < temp_bspline_curves[0].size() / 3; c++) {
        Eigen::MatrixXf N(row - 2, h_v - 2);
        Eigen::MatrixXf D(row, 3);
        Eigen::MatrixXf Qk(row, 3);
        Eigen::MatrixXf Q(h_v - 2, 3);
        for (size_t i = 0; i < row; i++) {
            D(i, 0) = temp_bspline_curves[i][c * 3 + 0];
            D(i, 1) = temp_bspline_curves[i][c * 3 + 1];
            D(i, 2) = temp_bspline_curves[i][c * 3 + 2];
        }

        for (size_t t_i = 1; t_i < t_v.size() - 1; t_i++) {
            b_spline_base.assign(h_v + order_v, 0.f);
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
                for (size_t i = 0; i < h_v + order_v; i++) {
                    float n_ik, n_i1k;
                    if (i + k - 1 >= h_v + order_v || (knots_v[i + k - 1] - knots_v[i]) == 0.f)
                        n_ik = 0.f;
                    else
                        n_ik = (t_v[t_i] - knots_v[i]) / (knots_v[i + k - 1] - knots_v[i]) * b_spline_base[i];
                    if (i + k >= h_v + order_v || (knots_v[i + k] - knots_v[i + 1]) == 0.f)
                        n_i1k = 0.f;
                    else
                        n_i1k = (knots_v[i + k] - t_v[t_i]) / (knots_v[i + k] - knots_v[i + 1]) * b_spline_base[i + 1];
                    b_spline_base[i] = n_ik + n_i1k;
                }
            }
            for (size_t i = 1; i < h_v - 1; i++)
                N(t_i - 1, i - 1) = b_spline_base[i];
            Qk(t_i, 0) = D(t_i, 0) - b_spline_base[0] * D(0, 0) - b_spline_base[h_v - 1] * D(row - 1, 0);
            Qk(t_i, 1) = D(t_i, 1) - b_spline_base[0] * D(0, 1) - b_spline_base[h_v - 1] * D(row - 1, 1);
            Qk(t_i, 2) = D(t_i, 2) - b_spline_base[0] * D(0, 2) - b_spline_base[h_v - 1] * D(row - 1, 2);
        }
        for (size_t i = 0; i < h_v - 2; i++) {
            Q(i, 0) = Q(i, 1) = Q(i, 2) = 0.f;
            for (size_t k = 0; k < row - 2; k++) {
                Q(i, 0) += N(k, i) * Qk(k + 1, 0);
                Q(i, 1) += N(k, i) * Qk(k + 1, 1);
                Q(i, 2) += N(k, i) * Qk(k + 1, 2);
            }
        }
        auto Nt_N = N.transpose() * N;
        Eigen::MatrixX3f P = Nt_N.lu().solve(Q);
        std::vector<B_SPLINE_DATATYPE> control_points;
        control_points.reserve(h_v * 3);
        control_points.push_back(D(0, 0));
        control_points.push_back(D(0, 1));
        control_points.push_back(D(0, 2));
        for (size_t i = 0; i < P.rows(); i++) {
            control_points.push_back(P(i, 0));
            control_points.push_back(P(i, 1));
            control_points.push_back(P(i, 2));
        }
        control_points.push_back(D(row - 1, 0));
        control_points.push_back(D(row - 1, 1));
        control_points.push_back(D(row - 1, 2));
        auto &result_pts = bspline_curve.BaseFuncMethod(control_points);
        this->interpolationP.insert(this->interpolationP.end(),
                                    result_pts.begin(),
                                    result_pts.end());
    }
    return this->interpolationP;
}

const std::vector<B_SPLINE_DATATYPE> &BSplineSurface::BasicFuncMethod(std::vector<float> &controlP) {
    if (pitch == 0) {
        std::cout << "ERROR: pitch is zero!" << std::endl;
        return this->interpolationP;
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
    knots_v.resize(row + order_v);
    for (size_t i = 0; i < order_v; i++)
        knots_v[i] = 0.f;
    for (size_t i = row; i < row + order_v; i++)
        knots_v[i] = 1.f;
    for (size_t i = order_v; i < row; i++)
        knots_v[i] = (i - order_v + 1) * 1.f / (row - order_v + 1);

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
    }

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

    return this->interpolationP;
}

void BSplineSurface::setupApproximationUVH(size_t h_u, size_t h_v) {
    this->h_u = h_u;
    this->h_v = h_v;
}
