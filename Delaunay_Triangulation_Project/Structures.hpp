//
//  Structures.hpp
//  Delaunay_Triangulation_Project
//
//  Created by Jakub Bakalarz on 13/01/2024.
//

#ifndef Structures_hpp
#define Structures_hpp

#include <stdio.h>
#include <iostream>
#include <set>
#include <vector>
#include <cmath>
#include <map>

struct Point {
    int x;
    int y;
    Point(int _x, int _y) : x(_x), y(_y) {}
    Point() : x(0), y(0) {}
    bool operator == (const Point& other) const  {
        return (other.x == x && other.y == y);
    }
    bool operator<(const Point& other) const {
        return x < other.x || (x == other.x && y < other.y);
    }
    
    double distanceTo(const Point& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    friend std::ostream& operator<<(std::ostream& os, const Point& point) {
        os << point.x << " " << point.y;
        return os;
    }
};

struct Edge {
    Point a;
    Point b;
    Edge(Point _a, Point _b) {
        if (_a < _b) {
            a = _a;
            b = _b;
        } else {
            a = _b;
            b = _a;
        }
    }
    bool operator == (const Edge& other) const  {
        return ((other.a == a && other.b == b) || (other.b == a && other.a == b) );
    }
    bool operator<(const Edge& other) const {
        if (a < other.a) return true;
        if (other.a < a) return false;
        return b < other.b;
    }

    friend std::ostream& operator<<(std::ostream& os, const Edge& edge) {
        os << "EDGE: " << edge.a << ", " << edge.b;
        return os;
    }
};


class Triangle {
public:
    
    bool operator == (const Triangle& other) const {
            std::vector<Point> thisPoints = {a, b, c};
            std::vector<Point> otherPoints = {other.a, other.b, other.c};
            
            // Sort vertices of both triangles
            std::sort(thisPoints.begin(), thisPoints.end());
            std::sort(otherPoints.begin(), otherPoints.end());

            // Check for equality
            return (thisPoints[0] == otherPoints[0] && thisPoints[1] == otherPoints[1] && thisPoints[2] == otherPoints[2]);
        }

    bool operator < (const Triangle& other) const {
        if (a < other.a) return true;
        if (other.a < a) return false;
        return b < other.b;
    }
    
    Point a;
    Point b;
    Point c;
    std::pair < double, double > circumCenter;
    Point getRoundedCircumCentre() const {
        return Point(circumCenter.first, circumCenter.second);
    }
    double circumRadius;

    Triangle() : a(Point()), b(Point()), c(Point()) {
        calculateCircumCircle();
    }
    
    Triangle(Point _a, Point _b, Point _c): a(_a), b(_b), c(_c) {
        calculateCircumCircle();
    }

    
    std::pair<double, double> calculateCircumCenter() {
        double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));

        double ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) + (c.x * c.x + c.y * c.y) * (a.y - b.y)) / d;
        double uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) + (c.x * c.x + c.y * c.y) * (b.x - a.x)) / d;

        return std::pair(ux, uy);
        
        
        
    }
    double calculateCircumRadius() const {

        double dx = circumCenter.first - a.x;
        double dy = circumCenter.second - a.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    void calculateCircumCircle() {
        circumCenter = calculateCircumCenter();
        circumRadius = calculateCircumRadius();
    }
    

    bool _doesContainPoint(Point p) const  {
        double detT = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
        double alpha = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) / detT;
        double beta = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) / detT;
        double gamma = 1 - alpha - beta;

        return (alpha >= 0) && (beta >= 0) && (gamma >= 0) && (alpha <= 1) && (beta <= 1) && (gamma <= 1);
    }
    
    float area(const Point& p1, const Point& p2, const Point& p3) {
        return std::abs((p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) / 2.0);
    }

    bool doesContainPoint(const Point& p) {
        float A = area(a, b, c);
        float A1 = area(p, b, c);
        float A2 = area(a, p, c);
        float A3 = area(a, b, p);
        

        float epsilon = std::numeric_limits<float>::epsilon();
        if(!(A1 > epsilon && A2 > epsilon && A3 > epsilon)){
            return false;
        }
        return std::abs(A - (A1 + A2 + A3)) < epsilon;
    }
    
    bool circumCircleContainPoint(Point p) const {
        double dx = p.x - circumCenter.first;
        double dy = p.y - circumCenter.second;
        double distanceSquared = dx * dx + dy * dy;

        return distanceSquared <= (circumRadius * circumRadius);
    }
    std::set<Edge> generateEdges() const {
        std::set<Edge> edges;
        edges.insert(Edge(a, b));
        edges.insert(Edge(b, c));
        edges.insert(Edge(c, a));
        return edges;
    }
    
    bool hasCommonEdgeTo(const std::set<Edge>& triangleEdges) const {
        std::set<Edge> myEdges = generateEdges();
        std::map<Edge, int> edgeCount;
        for(auto& e : myEdges) {
            edgeCount[e]++;
        }
        for(auto& e : triangleEdges) {
            edgeCount[e]++;
        }
        for (const auto& pair : edgeCount) {
            if (pair.second == 1) {
                return true;
            }
        }
        return false;
    }
    
//    friend std::ostream& operator<<(std::ostream& os, const Triangle& triangle) {
//        os << "Triangle: " << triangle.a << ", " << triangle.b << ", " << triangle.c;
//        return os;
//    }
    
    friend std::ostream& operator<<(std::ostream& os, const Triangle& triangle) {
        os << triangle.a << " " << triangle.b << " " << triangle.c;
        return os;
    }

};

namespace std {
    template <>
    struct hash<Triangle> {
        size_t operator()(const Triangle& triangle) const {
            // Compute individual hash values for two data members and combine them using XOR
            // and bit shifting. This is just an example; your actual implementation may vary.
            // You might need to include other headers for hash functions for custom types.
            
            // For example, if your Triangle class has three Point members: a, b, and c
            // and Point has x and y as members, you could hash them as follows:
            return ((hash<int>()(triangle.a.x) ^ (hash<int>()(triangle.a.y) << 1)) >> 1) ^
            (hash<int>()(triangle.b.x) ^ (hash<int>()(triangle.b.y) << 1)) ^
            ((hash<int>()(triangle.c.x) ^ (hash<int>()(triangle.c.y) << 1)) << 1) ;
        }
    };
}



#endif /* Structures_hpp */
