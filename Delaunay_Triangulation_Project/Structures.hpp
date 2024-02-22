//
//  Structures.hpp
//  Delaunay_Triangulation_Project
//
//  Created by Jakub Bakalarz on 13/01/2024.
//

#ifndef Structures_hpp
#define Structures_hpp

#include <stdio.h>
#include <set>
#include <cmath>

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
};

struct Edge {
    Point a;
    Point b;
    Edge(Point _a, Point _b) : a(_a), b(_b) {}
    bool operator == (const Edge& other) const  {
        return ((other.a == a && other.b == b) || (other.b == a && other.a == b) );
    }
    bool operator<(const Edge& other) const {
       if (a < other.a) return true;
       if (other.a < a) return false;
       return b < other.b;
   }
    
};

class Triangle {
public:
    
    bool operator == (const Triangle& other) const  {
        return (a == other.a && b == other.b && c == other.c);
    }
    
    Point a;
    Point b;
    Point c;
    Point circumCenter;
    double circumRadius;

    Triangle() : a(Point()), b(Point()), c(Point()) {
        calculateCircumCircle();
    }
    
    Triangle(Point _a, Point _b, Point _c): a(_a), b(_b), c(_c) {
        calculateCircumCircle();
    }
    

    Point calculateCircumCenter() {
        Point _circumCenter;
        double detT = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
        double alpha = ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y)) / detT;
        double beta = ((c.y - a.y) * (a.x - c.x) + (a.x - c.x) * (a.y - c.y)) / detT;
        double gamma = 1 - alpha - beta;

        _circumCenter.x = static_cast<int>(alpha * a.x + beta * b.x + gamma * c.x);
        _circumCenter.y = static_cast<int>(alpha * a.y + beta * b.y + gamma * c.y);
        return _circumCenter;
    }
    double calculateCircumRadius() const {
        double dx = circumCenter.x - a.x;
        double dy = circumCenter.y - a.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    void calculateCircumCircle() {
        circumCenter = calculateCircumCenter();
        circumRadius = calculateCircumRadius();
    }
    

    bool doesContainPoint(Point p) const  {
        double detT = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
        double alpha = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) / detT;
        double beta = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) / detT;
        double gamma = 1 - alpha - beta;

        return (alpha >= 0) && (beta >= 0) && (gamma >= 0) && (alpha <= 1) && (beta <= 1) && (gamma <= 1);
    }
    
    bool circumCircleContainPoint(Point p) const {
        double dx = p.x - circumCenter.x;
        double dy = p.y - circumCenter.y;
        double distanceSquared = dx * dx + dy * dy;

        return distanceSquared <= (circumRadius * circumRadius);
    }
    std::set<Edge> generateEdges() {
        std::set<Edge> edges;
        edges.insert(Edge(a, b));
        edges.insert(Edge(b, c));
        edges.insert(Edge(c, a));
        return edges;
    }

};



#endif /* Structures_hpp */
