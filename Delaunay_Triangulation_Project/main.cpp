//
//  main.cpp
//  Delaunay_Triangulation_Project
//
//  Created by Jakub Bakalarz on 13/01/2024.
//

#include <iostream>
#include "Triangulation.hpp"

int main(int argc, const char * argv[]) {
    std::vector<Point> points = {
        Point(3, 1),
        Point(6, 2),
        Point(3, 4),
        Point(5, 4),
        Point(6, 6),
        Point(4,5),
        Point(9,5),
        Point(9,3),
        Point(7,4),
        Point(8,1)
    };

    // std::vector<Point> points = {
    //     Point(3, 1),
    //     Point(6, 2),
    //     Point(3, 4),
    //     Point(5, 4)
    // };
    auto triangulation = Triangulation(200, 200, points);
    triangulation.bowyerWatson();
    auto triang = triangulation.getTriangulation();
    
    std::cout << "super:"<<std::endl;
    Triangle super = triangulation.calculateSuperTriangle(0, 0, 200, 200);
    std::cout << super.a << super.b << super.c << std::endl;
    
    std::cout << "triangulacja:"<<std::endl;
    for(auto& triangle : triang) {
        std::cout << triangle.a << triangle.b << triangle.c << std::endl;
    }
    return 0;
}
