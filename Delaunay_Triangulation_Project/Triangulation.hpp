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
#include <functional>
#include <iostream>

class Triangulation {
private:
    int width;
    int height;
    Triangle _superTriangle;
    std::set<Triangle> triangulation;
    std::vector<Point> points;
    std::map <Triangle, std::set<Triangle> > adjacency_list;
public:
    Triangulation(int x, int y, std::vector<Point> _points) {
        width = x;
        height = y;
        for (auto point : _points) {
            points.push_back(point);
        }
    }
    
    void bowyerWatson() {
        Triangle superTriangle = calculateSuperTriangle(0, 0, width, height);
        _superTriangle = superTriangle;
        triangulation.insert(superTriangle);
        int i = 0;
        for(auto& point : points) {
            Triangle selectedTriangle = findTriangleContainingPoint(point);
            std::vector<Triangle> badTriangles = findBadTriangles(selectedTriangle, point, adjacency_list);
            std::cout<<"FOUND POINT"<< point << " IN "<<selectedTriangle<<std::endl;
            std::vector<Triangle> neighborsOfBadTriangles = findNeighborsOfBadTriangles(badTriangles, adjacency_list);
            removeBadTriangles(badTriangles, triangulation, adjacency_list);
            std::vector<Edge> polygonEdges = findBoundaryEdges(badTriangles, selectedTriangle);
            std::vector<Triangle> newTriangles = createTrianglesFromPolygon(polygonEdges, point, triangulation);
            std::vector<Triangle> newTrianglesAndBadAndNeighbors = concatenateVectors(newTriangles, neighborsOfBadTriangles);
            updateAdjacencyListForNewTriangles(adjacency_list, newTrianglesAndBadAndNeighbors);
            std::cout<<"i: "<< i << std::endl;
            i++;
        }
        
    }
    
    std::set<Triangle> getTriangulation() {
        return triangulation;
    }
    
    std::vector<Triangle> concatenateVectors(const std::vector<Triangle>& vec1, const std::vector<Triangle>& vec2) {
        std::vector<Triangle> concatenated;
        concatenated.reserve(vec1.size() + vec2.size());
        
        // Copy elements from vec1
        std::copy(vec1.begin(), vec1.end(), std::back_inserter(concatenated));
        
        // Copy elements from vec2
        std::copy(vec2.begin(), vec2.end(), std::back_inserter(concatenated));
        
        return concatenated;
    }
    
    std::vector<Triangle> findBadTriangles(Triangle& startTriangle, Point point, std::map <Triangle, std::set<Triangle>>& adjacencyList) {
        std::vector<Triangle> badTriangles;
        badTriangles.push_back(startTriangle);
        for(auto& triangle : adjacencyList[startTriangle]) {
            if (triangle.circumCircleContainPoint(point)) {
                badTriangles.push_back(triangle);
            }
        }
        return badTriangles;
    }
    
    void removeBadTriangles(const std::vector<Triangle>& badTriangles, std::set<Triangle>& _triangulation, std::map <Triangle, std::set<Triangle> >& adjacencyList ) {
        for(auto& badTriangle : badTriangles) {
            _triangulation.erase(badTriangle);
            adjacencyList.erase(badTriangle);
            for(auto& triangle : adjacency_list[badTriangle]) {
                adjacencyList[triangle].erase(badTriangle);
            }
        }
    }
    
//    std::generateNewTriangles(std::vector<Edge>& edges, Point& p, std::set<Triangle>& _triangulation))
    std::vector<Triangle> createTrianglesFromPolygon(std::vector<Edge>& edges, Point& p, std::set<Triangle>& _triangulation) {
        std::vector<Triangle> newTriangles;
        for(auto& edge : edges) {
            _triangulation.insert(Triangle(edge.a, edge.b, p));
            newTriangles.push_back(Triangle(edge.a, edge.b, p));
        }
        return newTriangles;
    }
    
    
    std::vector<Triangle> findNeighborsOfBadTriangles(const std::vector<Triangle>& badTriangles, const std::map<Triangle, std::set<Triangle>>& adjacencyList) {
        std::vector<Triangle> newNeighbors;
        std::map<Triangle, int> TrianglesCount;

        for (const Triangle& triangle : badTriangles) {
            for(const Triangle& adjacentTriangle : adjacencyList.at(triangle)) {
                TrianglesCount[adjacentTriangle]++;
            }
        }

        for(const auto& pair : TrianglesCount) {
            if (pair.second == 1) {
                newNeighbors.push_back(pair.first);
            }
        }

        return newNeighbors;
    }
    
    std::vector<Edge> findBoundaryEdges(const std::vector<Triangle>& triangles, const Triangle& centreTriangle) {
        std::map<Edge, int> edgeCount;
        std::vector<Edge> boundaryEdges;

        for (const Triangle& triangle : triangles) {
            std::set<Edge> edges = triangle.generateEdges();
            for (const Edge& edge : edges) {
                edgeCount[edge]++;
            }
        }

        for (const auto& pair : edgeCount) {
            if (pair.second == 1) {
                boundaryEdges.push_back(pair.first);
            }
        }

        return boundaryEdges;
    }
    
    void updateAdjacencyListForNewTriangles( std::map<Triangle, std::set<Triangle>>& adjacencyList, const std::vector<Triangle>& newTriangles) {
        
        // Temporary map to hold edges to triangles mapping for all triangles
        std::map<Edge, std::vector<Triangle>> edgeToTriangles;


        // Update the edge to triangles mapping with new triangles
        for (const Triangle& newTriangle : newTriangles) {
            std::set<Edge> edges = newTriangle.generateEdges();
            for (const Edge& edge : edges) {
                edgeToTriangles[edge].push_back(newTriangle);
            }
        }

        // Update the adjacency list for new triangles
        for (const Triangle& newTriangle : newTriangles) {
            //debug print
//            for (const auto& pair : edgeToTriangles) {
//                const std::vector<Triangle>& triangles = pair.second;
//                const Edge& edge = pair.first;
//                std::cout <<edge<<std::endl;
//                for(Triangle t : triangles) {
//                    std::cout<<t<<std::endl;
//                }
//
//            }
            
            std::set<Edge> newTriangleEdges = newTriangle.generateEdges();

            for (const Edge& edge : newTriangleEdges) {
                const std::vector<Triangle>& trianglesSharingEdge = edgeToTriangles[edge];
                for (const Triangle& adjacentTriangle : trianglesSharingEdge) {
                    if (!(newTriangle == adjacentTriangle)) {
                        // Add the adjacent triangle to the new triangle's neighbors
                        adjacencyList[newTriangle].insert(adjacentTriangle);
                        // Also add the new triangle to the adjacent triangle's neighbors
                        adjacencyList[adjacentTriangle].insert(newTriangle);
                    }
                }
            }
        }
        
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

                Point center = neighbor.getRoundedCircumCentre();
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
