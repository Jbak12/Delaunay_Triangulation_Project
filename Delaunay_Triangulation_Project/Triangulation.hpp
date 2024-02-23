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
#include <queue>


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
        Triangle superTriangle = calculateSuperTriangle(width, height);
        _superTriangle = superTriangle;
        triangulation.insert(superTriangle);
        for(auto& point : points) {
            
            Triangle selectedTriangle;
            try {
                 selectedTriangle = findTriangleContainingPoint(point);
            } catch (const std::runtime_error& e) {
                continue;
            }
           
            std::vector<Triangle> badTriangles = findBadTriangles(selectedTriangle, point, adjacency_list);
            std::vector<Triangle> neighborsOfBadTriangles = findNeighborsOfBadTriangles(badTriangles, adjacency_list);
            removeBadTriangles(badTriangles, triangulation, adjacency_list);
            std::vector<Edge> polygonEdges = findBoundaryEdges(badTriangles, selectedTriangle);
            std::vector<Triangle> newTriangles = createTrianglesFromPolygon(polygonEdges, point, triangulation);
            
            std::vector<Triangle> newTrianglesAndBadAndNeighbors = concatenateVectors(newTriangles, neighborsOfBadTriangles);
            updateAdjacencyListForNewTriangles(adjacency_list, newTrianglesAndBadAndNeighbors);
        }
//        removeConnectedToSuperTriangle(triangulation, superTriangle);
        
    }
    
    std::set<Triangle> getTriangulation() {
        return triangulation;
    }
    
    std::vector<Triangle> concatenateVectors(const std::vector<Triangle>& vec1, const std::vector<Triangle>& vec2) {
        std::vector<Triangle> concatenated;
        concatenated.reserve(vec1.size() + vec2.size());
        
        std::copy(vec1.begin(), vec1.end(), std::back_inserter(concatenated));
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
            for(auto& triangle : adjacencyList[badTriangle]) {
                adjacencyList[triangle].erase(badTriangle);
            }
            _triangulation.erase(badTriangle);
            adjacencyList.erase(badTriangle);
        }
    }
    
    std::vector<Triangle> createTrianglesFromPolygon(std::vector<Edge>& edges, Point& p, std::set<Triangle>& _triangulation) {
        std::vector<Triangle> newTriangles;
        for(auto& edge : edges) {
            _triangulation.insert(Triangle(edge.a, edge.b, p));
            newTriangles.push_back(Triangle(edge.a, edge.b, p));
        }
        return newTriangles;
    }
    
    void removeConnectedToSuperTriangle(std::set<Triangle>& triangulation, const Triangle& superTriangle) {
        std::set<Edge> triangleEdges = superTriangle.generateEdges();
        for(auto& triangle : triangulation) {
            if(triangle.hasCommonEdgeTo(triangleEdges)) {
                triangulation.erase(triangle);
                adjacency_list.erase(triangle);
                for(auto& t : adjacency_list[triangle]) {
                    adjacency_list[t].erase(triangle);
                }
            }
        }
    }

    
    
    std::vector<Triangle> findNeighborsOfBadTriangles(const std::vector<Triangle>& badTriangles, const std::map<Triangle, std::set<Triangle>>& adjacencyList) {
        std::vector <Triangle> newNeighbors;
        
        std::map<Triangle, bool> zliMapa;
        for(auto& badtriangle : badTriangles) {
            zliMapa[badtriangle] = true;
        }
        
        for (const Triangle& triangle : badTriangles) {
            for(const Triangle& adjacentTriangle : adjacencyList.at(triangle)) {
                auto it = zliMapa.find(adjacentTriangle);
                if (it == zliMapa.end()) {
                    newNeighbors.push_back(adjacentTriangle);
                    zliMapa[adjacentTriangle] = false;
                }
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

        auto compare = [&point](const Triangle& a, const Triangle& b) {
            double distanceA = point.distanceTo(a.getRoundedCircumCentre());
            double distanceB = point.distanceTo(b.getRoundedCircumCentre());
            return distanceA > distanceB; // We want the smallest distance at the top
        };

        std::priority_queue<Triangle, std::vector<Triangle>, decltype(compare)> queue(compare);

        auto currentTriangleIt = triangulation.begin();
        Triangle currentTriangle = *currentTriangleIt;
        queue.push(currentTriangle);

        std::unordered_set<Triangle> visited;

        while (!queue.empty()) {
            currentTriangle = queue.top();
            queue.pop();

            if (visited.find(currentTriangle) != visited.end()) {
                continue;
            }

            if (currentTriangle.doesContainPoint(point)) {
                return currentTriangle;
            }

            visited.insert(currentTriangle);

            for (const Triangle& neighbor : adjacency_list[currentTriangle]) {
                if (visited.find(neighbor) == visited.end()) {
                    queue.push(neighbor);
                }
            }
        }

        throw std::runtime_error("No containing triangle found.");
    }
    
    Triangle calculateSuperTriangle(int rectWidth, int rectHeight) {

        Point A = Point(0, 0);
        Point B = Point(2 * rectWidth, 0);
        Point C = Point(0, 2*rectHeight);
       
        return Triangle(A, B, C);
    }

};

#endif /* Triangulation_hpp */
