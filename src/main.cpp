#include <iostream>
#include "Displayer.h"
int main() {
    std::cout << "B-Spline" << std::endl;
    Displayer displayer(1024,1024);
    displayer.render();
    return 0;
}
