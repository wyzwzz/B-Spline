//
// Created by wyz on 20-12-31.
//

#ifndef B_SPLINE_ELLIPSOID_H
#define B_SPLINE_ELLIPSOID_H


class Ellipsoid {
public:
    Ellipsoid()=default;
private:
    //(x-center_x)^2/a+(y-center_y)^2/b+(z-center_z)^2/c=1
    float center_x,center_y,center_z;
    float a,b,c;
};


#endif //B_SPLINE_ELLIPSOID_H
