//
// Created by wyz on 20-12-31.
//

#ifndef B_SPLINE_ELLIPSOID_H
#define B_SPLINE_ELLIPSOID_H

#include <vector>
#include <cmath>
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

class Ellipse{
public:
    Ellipse():
    center_x(0.f),center_y(0.f),a(1.f),b(1.f),theta_interval(5.f)
    {}
    void setCenter(float x,float y){
        this->center_x=x;
        this->center_y=y;
    }
    void setAixsLength(float a,float b){
        this->a=a;
        this->b=b;
    }
    void setSampleInterval(float interval){
        this->theta_interval=interval;
    }
    std::vector<float> getControlPoints();
private:
    float center_x,center_y;
    float a,b;
    float theta_interval;
};

inline std::vector<float> Ellipse::getControlPoints()
{
    std::vector<float > control_points;
    int num=2*glm::pi<float>()/glm::radians(theta_interval)+1;
    for(int i=0;i<num;i++){
        float theta=i*theta_interval;
        float x;
        float y;
        float t;
        t=std::tan(glm::radians(theta));
        float cal_x=std::sqrt(1.f/(1.f/(a/a)+t*t/(b*b)));
        if(theta<90){
            x=cal_x;
            y=t*x;
        }
        else if(theta==90){
            x=0.f;
            y=b;
        }
        else if(theta>90 && theta<=180){
            x=-cal_x;
            y=t*x;
        }
        else if(theta>180 && theta<270){
            x=-cal_x;
            y=t*x;
        }
        else if(theta ==270){
            x=0.f;
            y=-b;
        }
        else if(theta>270 && theta<=360){
            x=cal_x;
            y=t*x;
        }
        control_points.push_back(x);
        control_points.push_back(y);
        control_points.push_back(0.f);
    }
    return std::move(control_points);
}


class Ellipsoid {
public:
    Ellipsoid():center_x(0.f),center_y(0.f),center_z(0.f),a(1.f),b(1.f),c(1.f),
                z_interval(0.05f),x_interval(5.f){}

    void setCenter(float x,float y,float z){
        this->center_x=x;
        this->center_y=y;
        this->center_z=z;
    }

    void setAixsLength(float a,float b,float c){
        this->a=a;
        this->b=b;
        this->c=c;
    }

    /**
     * @brief set sample interval
     * @param z_theta sample interval with z-axis (-c to c)
     * @param xy_theta sample interval with x-axis in xy plain in degree(0-360)
     */
    void setSampleInterval(float z_interval,float x_interval){
        this->z_interval=z_interval;
        this->x_interval=x_interval;
    }

    void getRowAndCol(int* row,int* col){
        if(!row || !col){
            std::cout<<"invail ptr"<<std::endl;
            return;
        }
        *row=2*c/z_interval+1;
        *col=2*glm::pi<float>()/glm::radians(x_interval);
        std::cout<<"row is: "<<*row<<"\ncol is: "<<*col<<std::endl;
    }

    std::vector<float> getControlPoints();
private:
    //(x-center_x)^2/a^2+(y-center_y)^2/b^2+(z-center_z)^2/c^2=1
    float center_x,center_y,center_z;
    float a,b,c;
    float z_interval,x_interval;
};

inline std::vector<float> Ellipsoid::getControlPoints()
{
    std::vector<float> control_points;
    int row=2*c/z_interval+1;
    int col=2*glm::pi<float>()/glm::radians(x_interval);
    control_points.reserve(row*col*3);
    for(int i=0;i<row;i++){
        float z=c-i*z_interval;
        float tmp=1-(z-center_z)*(z-center_z)/(c*c);
        //calculate new a and b for current z's ellipse
        float _a=std::sqrt(a*a*tmp);
        float _b=std::sqrt(b*b*tmp);
//        std::cout<<"z: "<<z<<std::endl;
        std::cout<<"_a:"<<_a<<"\t_b:"<<_b<<std::endl;
        std::vector<float> up,down;
        up.reserve(col*3);
        down.reserve(col*3);
        for(int j=0;j<18;j++){
            float theta=j*x_interval;
            std::cout<<"theta: "<<theta<<std::endl;
            float t=std::tan(glm::radians(theta));
            std::cout<<"j: "<<j<<" t: "<<t <<std::endl;
//            std::cout<<"x: "<<x<<std::endl;
            float x;
            if(theta!=90)
                x=std::sqrt(1.f/(1.f/(_a*_a)+t*t/(_b*_b)));
            else
                x=0.f;
            float x1,x2;
            if(theta<90){
                x1=x;
                x2=-x;
            }
            else{
                x1=-x;
                x2=x;
            }
            float y1;
            float y2;
            if(theta!=90){
                y1=x1*t;
                y2=x2*t;
            }
            else{
                y1=_b;
                y2=-_b;
            }
            if(_a==0.f || _b==0.f)
                x1=y1=x2=y2=0.f;
            std::cout<<"x1: "<<x1<<"\ty1: "<<y1<<std::endl;
            std::cout<<"x2: "<<x2<<"\ty2: "<<y2<<std::endl;
            up.push_back(x1);up.push_back(y1);up.push_back(z);
            down.push_back(x2);down.push_back(y2);down.push_back(z);
        }
        control_points.insert(control_points.end(),up.begin(),up.end());
//        control_points.insert(control_points.end(),down.begin(),down.end());
    }
    std::cout<<"point num is: "<<control_points.size()/3<<std::endl;
    return std::move(control_points);
}


#endif //B_SPLINE_ELLIPSOID_H
