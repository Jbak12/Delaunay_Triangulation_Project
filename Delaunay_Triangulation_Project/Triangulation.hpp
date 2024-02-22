//
//  Triangulation.hpp
//  Delaunay_Triangulation_Project
//
//  Created by Jakub Bakalarz on 13/01/2024.
//

#ifndef Triangulation_hpp
#define Triangulation_hpp

#include <stdio.h>
#include <vector>
#include <map>
#include <unordered_set>
#include <set>
#include "Structures.hpp"

class Triangulation {
private:
    Triangle superTriangle;
    std::set<Triangle> triangulation;
    std::vector<Point> points;
    std::map <Triangle, std::set<Triangle> > adjacency_list;
public:
    Triangulation(int x, int y, std::vector<Point> points) {
        for (auto point : points) {
            addPoint(point);
        }
    }
    
    void addPoint(Point p) {
        points.push_back(p);
    }

    
    
    
    void bowyerWatson() {
        Triangle superTriangle = calculateSuperTriangle(0, 0, 200, 200);
        triangulation.insert(superTriangle);
        for(auto& point : points) {
            Triangle selectedTriangle = findTriangleContainingPoint(point);
            std::vector<Triangle> badTriangles = findBadTriangles(selectedTriangle, point);
            std::vector<Edge> polygonEdges = findBoundaryEdges(badTriangles, selectedTriangle);
            removeBadTriangles(badTriangles);
            addNewTriangles(polygonEdges, point);
        }

    }
    
    void removeTriangle(const Triangle &T) {
        triangulation.erase(T);
    }
    
    void removeBadTriangles(const std::vector<Triangle>& badTriangles) {
        for(auto& badTriangle : badTriangles) {
            triangulation.erase(badTriangle);
            for(auto& triangle : adjacency_list[badTriangle]) {
                adjacency_list[triangle].erase(badTriangle);
            }
        }
    }
    
    void addNewTriangles(std::vector<Edge>& edges, Point& p) {
        for(auto& edge : edges) {
            triangulation.insert(Triangle(edge.a, edge.b, p));
        }
    }
    
    std::vector<std::pair<Edge,Triangle>> findBoundaryEdges(std::vector<Triangle>& badTriangles, Triangle centreTriangle) {
        std::vector<std::pair<Edge,Triangle>> boundaryEdges;
        std::set<Edge> centreTriangleEdges;
        for(auto triangle : badTriangles) {
            std::set<Edge> difference;
            std::set<Edge> currentTriangleEdges = triangle.generateEdges();
            std::set_difference(centreTriangleEdges.begin(),
                                centreTriangleEdges.end(),
                                currentTriangleEdges.begin(),
                                currentTriangleEdges.end(),
                                std::back_inserter(boundaryEdges));
        }
        
        return boundaryEdges;
        
    }
    
    std::vector<Triangle> findBadTriangles(Triangle& startTriangle, Point point) {
        std::vector<Triangle> badTriangles;
        badTriangles.push_back(startTriangle);
        for(auto& triangle : adjacency_list[startTriangle]) {
            if (triangle.circumCircleContainPoint(point)) {
                badTriangles.push_back(triangle);
            }
        }
        return badTriangles;
    }
    
    Triangle findTriangleContainingPoint(const Point& point) {
        if (triangulation.empty()) {
            throw std::runtime_error("Triangulation is empty.");
        }

        // Start from the first triangle in the set
        auto currentTriangleIt = triangulation.begin();
        Triangle currentTriangle = *currentTriangleIt;

        // Use a set to keep track of visited triangles to avoid loops
        std::unordered_set<Triangle> visited;

        // Continue searching until we find the triangle containing the point
        while (!currentTriangle.doesContainPoint(point)) {
            visited.insert(currentTriangle);

            // Find the neighbor closest to the point
            Triangle closestNeighbor;
            double closestDistance = std::numeric_limits<double>::max();
            bool foundNeighbor = false;

            for (const Triangle& neighbor : adjacency_list[currentTriangle]) {
                if (visited.find(neighbor) != visited.end()) {
                    continue; // Skip already visited neighbors
                }

                Point center = neighbor.circumCenter;
                double distance = point.distanceTo(center); // Assuming Point class has a method to calculate distance

                if (distance < closestDistance) {
                    closestNeighbor = neighbor;
                    closestDistance = distance;
                    foundNeighbor = true;
                }
            }

            if (!foundNeighbor) {
                throw std::runtime_error("No containing triangle found.");
            }

            // Move to the neighbor closest to the point
            currentTriangle = closestNeighbor;
        }

        return currentTriangle;
    }
    
    
    
    Triangle calculateSuperTriangle(int rectX, int rectY, int rectWidth, int rectHeight) {
        // Find the center of the rectangle
        Point rectCenter(rectX + rectWidth / 2, rectY + rectHeight / 2);

        // Calculate the half-diagonal of the rectangle
        double halfDiagonal = std::sqrt((rectWidth / 2.0) * (rectWidth / 2.0) + (rectHeight / 2.0) * (rectHeight / 2.0));

        // Calculate the vertices of the super triangle
        Point superTriangleA(rectCenter.x, rectCenter.y - static_cast<int>(2 * halfDiagonal));
        Point superTriangleB(rectCenter.x - static_cast<int>(1.5 * halfDiagonal), rectCenter.y + static_cast<int>(halfDiagonal));
        Point superTriangleC(rectCenter.x + static_cast<int>(1.5 * halfDiagonal), rectCenter.y + static_cast<int>(halfDiagonal));

        // Return the super triangle
        return Triangle(superTriangleA, superTriangleB, superTriangleC);
    }

};

#endif /* Triangulation_hpp */
