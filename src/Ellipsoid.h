//
// Created by wyz on 20-12-31.
//

#ifndef B_SPLINE_ELLIPSOID_H
#define B_SPLINE_ELLIPSOID_H

#include <vector>
#include <cmath>
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

class Ellipse {
public:
    Ellipse() :
            center_x(0.f), center_y(0.f), a(1.f), b(1.f), theta_interval(5.f) {}

    void setCenter(float x, float y) {
        this->center_x = x;
        this->center_y = y;
    }

    void setAixsLength(float a, float b) {
        this->a = a;
        this->b = b;
    }

    void setSampleInterval(float interval) {
        this->theta_interval = interval;
    }

    std::vector<float> getControlPoints();

    float calculateAccuracy(const std::vector<float>& data)=delete;
private:
    float center_x, center_y;
    float a, b;
    float theta_interval;
};

inline std::vector<float> Ellipse::getControlPoints() {
    std::vector<float> control_points;
    int num = 2 * glm::pi<float>() / glm::radians(theta_interval) + 1;
    for (int i = 0; i < num; i++) {
        float theta = i * theta_interval;
        float x;
        float y;
        float t;
        t = std::tan(glm::radians(theta));
        float cal_x = std::sqrt(1.f / (1.f / (a * a) + t * t / (b * b)));
        if (theta < 90) {
            x = cal_x;
            y = t * x;
        } else if (theta == 90) {
            x = 0.f;
            y = b;
        } else if (theta > 90 && theta <= 180) {
            x = -cal_x;
            y = t * x;
        } else if (theta > 180 && theta < 270) {
            x = -cal_x;
            y = t * x;
        } else if (theta == 270) {
            x = 0.f;
            y = -b;
        } else if (theta > 270 && theta <= 360) {
            x = cal_x;
            y = t * x;
        }
        control_points.push_back(x);
        control_points.push_back(y);
        control_points.push_back(0.f);
    }
    return std::move(control_points);
}



class Ellipsoid {
public:
    Ellipsoid() : center_x(0.f), center_y(0.f), center_z(0.f), a(1.f), b(1.f), c(1.f),
                  z_interval(0.05f), x_interval(5.f) {}


    void setAixsLength(float a, float b, float c) {
        this->a = a;
        this->b = b;
        this->c = c;
    }

    /**
     * @brief set sample interval
     * @param z_theta sample interval with z-axis (-c to c)
     * @param xy_theta sample interval with x-axis in xy plain in degree(0-360)
     */
    void setSampleInterval(float z_interval, float x_interval) {
        this->z_interval = z_interval;
        this->x_interval = x_interval;
    }

    void getRowAndCol(int *row, int *col) {
        if (!row || !col) {
            std::cout << "invail ptr" << std::endl;
            return;
        }
        *row = 2 * c / z_interval + 1;
        *col = 2 * glm::pi<float>() / glm::radians(x_interval) + 1;
    }

    std::vector<float> getNormals(std::vector<float> &pts);

    std::vector<float> getControlPoints();

    float calculateAccuracy(const std::vector<float> &data);
private:
    void setCenter(float x, float y, float z) {
        this->center_x = x;
        this->center_y = y;
        this->center_z = z;
    }

private:
    //(x-center_x)^2/a^2+(y-center_y)^2/b^2+(z-center_z)^2/c^2=1
    float center_x, center_y, center_z;
    float a, b, c;
    float z_interval, x_interval;
};

inline std::vector<float> Ellipsoid::getControlPoints() {
    std::vector<float> control_points;
    int row = 2 * c / z_interval + 1;
    int col = 2 * glm::pi<float>() / glm::radians(x_interval) + 1;
    control_points.reserve(row * col * 3);
    for (int i = 0; i < row; i++) {
        float z = center_z + c - i * z_interval;
        float tmp = 1 - (z - center_z) * (z - center_z) / (c * c);
        //calculate new a and b for current z's ellipse
        float _a = std::sqrt(a * a * tmp);
        float _b = std::sqrt(b * b * tmp);

        for (int j = 0; j < col; j++) {
            float theta = j * x_interval;
            float x;
            float y;
            float t;
            t = std::tan(glm::radians(theta));
            float cal_x = std::sqrt(1.f / (1.f / (_a * _a) + t * t / (_b * _b)));
            if (_a == 0.f || _b == 0.f) {
                x = y = 0.f;
                goto Add;
            }
            if (theta < 90) {
                x = cal_x;
                y = t * x;
            } else if (theta == 90) {
                x = 0.f;
                y = _b;
            } else if (theta > 90 && theta <= 180) {
                x = -cal_x;
                y = t * x;
            } else if (theta > 180 && theta < 270) {
                x = -cal_x;
                y = t * x;
            } else if (theta == 270) {
                x = 0.f;
                y = -_b;
            } else if (theta > 270 && theta <= 360) {
                x = cal_x;
                y = t * x;
            }
            Add:
            control_points.push_back(x);
            control_points.push_back(y);
            control_points.push_back(z);
        }
    }
    return std::move(control_points);
}

inline std::vector<float> Ellipsoid::getNormals(std::vector<float> &pts) {
    std::vector<float> normals;
    normals.reserve(pts.size());
    for (size_t i = 0; i < pts.size() / 3; i++) {
        auto x = pts[i * 3 + 0];
        auto y = pts[i * 3 + 1];
        auto z = pts[i * 3 + 2];
        glm::vec3 n = {(x - center_x) / (a * a), (y - center_y) / (b * b), (z - center_z) / (c * c)};
        n = glm::normalize(n);
        normals.push_back(n.x);
        normals.push_back(n.y);
        normals.push_back(n.z);
    }
    return std::move(normals);
}

float Ellipsoid::calculateAccuracy(const std::vector<float> &data) {

    float acc=0.f;
    for(size_t i=0;i<data.size()/3;i++){
        float x=data[i*3];
        float y=data[i*3+1];
        float z=data[i*3+2];
        acc+=std::fabs(-1+(x-center_x)*(x-center_x)/(a*a)+(y-center_y)*(y-center_y)/(b*b)+(z-center_z)*(z-center_z)/(c*c));
    }
    return 3*acc/data.size();
}

#endif //B_SPLINE_ELLIPSOID_H
