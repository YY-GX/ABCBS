//
// Created by Yue Yang on 2020/11/13.
//

#include "VertexConstraint.h"

std::ostream &operator<<(std::ostream &os, VertexConstraint vertexConstraint) {
    os << "(" << vertexConstraint.time << ", " << vertexConstraint.location << ")";
    return os;
}

bool VertexConstraint::operator==(const VertexConstraint & other) {
    return (this->time == other.getTime()) && (this->location == other.getLocation());
}

VertexConstraint::VertexConstraint(int time, const Location &location) : time(time), location(location) {}

VertexConstraint::VertexConstraint(const VertexConstraint &edgeConstraint) {
    this->time = edgeConstraint.time;
    this->location = edgeConstraint.location;
}

int VertexConstraint::getTime() const {
    return time;
}

void VertexConstraint::setTime(int time) {
    VertexConstraint::time = time;
}

Location VertexConstraint::getLocation() const {
    return location;
}

void VertexConstraint::setLocation(Location location) {
    VertexConstraint::location = location;
}

bool operator==(const VertexConstraint& v1, const VertexConstraint& v2) {
    return (v1.getTime() == v2.getTime()) && (v2.getLocation() == v2.getLocation());
}