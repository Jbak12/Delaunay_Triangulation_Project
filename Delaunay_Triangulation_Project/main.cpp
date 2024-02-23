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
    std::vector<Point> _points = {
        Point(6, 8),
        Point(16, 12),
        Point(18, 6),
        Point(23, 11),
        Point(17, 17),
        Point(30,30),
        Point(29,11),
        Point(38,32),
        Point(10,25),
        Point(38, 20)
    };
    
    std::vector<Point> points = {
        Point(1, 2),
        Point(9, 8),
        Point(8, 2),
        Point(7, 3),
        Point(3, 5),
        Point(4,7),
        Point(3,3),
        Point(2,5),
        Point(9,1),
        Point(3, 8)
    };
    

    // std::vector<Point> points = {
    //     Point(3, 1),
    //     Point(6, 2),
    //     Point(3, 4),
    //     Point(5, 4)
    // };
    auto* triangulation = new Triangulation(10, 10, points);
    triangulation->bowyerWatson();
    auto triang = triangulation->getTriangulation();
    saveTrianglesToFile(triang, "triangles.txt");
    
    std::cout << "super:"<<std::endl;
    Triangle super = triangulation->calculateSuperTriangle(20, 20);
    std::cout << super.a << super.b << super.c << std::endl;
    
    std::cout << "triangulacja:"<<std::endl;
    for(auto& triangle : triang) {
        std::cout <<"(" <<triangle.a <<") ("<< triangle.b <<") (" << triangle.c <<")"<< std::endl;
    }
    return 0;
}
