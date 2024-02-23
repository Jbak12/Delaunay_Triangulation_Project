//
//  main.cpp
//  Delaunay_Triangulation_Project
//
//  Created by Jakub Bakalarz on 13/01/2024.
//
#include <fstream>
#include <iostream>
#include "Triangulation.hpp"

void saveTrianglesToFile(const std::set<Triangle>& triangles, const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    for (const auto& triangle : triangles) {
        outFile << triangle << std::endl;
    }

    outFile.close();
}

int main(int argc, const char * argv[]) {
//    std::vector<Point> points = {
//        Point(3, 1),
//        Point(6, 2),
//        Point(3, 4),
//        Point(5, 4),
//        Point(6, 6),
//        Point(4,5),
//        Point(9,5),
//        Point(9,3),
//        Point(7,4),
//        Point(8,1)
//    };
    std::vector<Point> points = {
        Point(6, 8),
        Point(16, 12),
        Point(18, 6),
        Point(23, 11),
        Point(6, 18)
    };

    // std::vector<Point> points = {
    //     Point(3, 1),
    //     Point(6, 2),
    //     Point(3, 4),
    //     Point(5, 4)
    // };
    auto triangulation = Triangulation(20, 20, points);
    triangulation.bowyerWatson();
    auto triang = triangulation.getTriangulation();
    saveTrianglesToFile(triang, "triangles.txt");
    
    std::cout << "super:"<<std::endl;
    Triangle super = triangulation.calculateSuperTriangle(20, 20);
    std::cout << super.a << super.b << super.c << std::endl;
    
    std::cout << "triangulacja:"<<std::endl;
    for(auto& triangle : triang) {
        std::cout <<"(" <<triangle.a <<") ("<< triangle.b <<") (" << triangle.c <<")"<< std::endl;
    }
    return 0;
}
