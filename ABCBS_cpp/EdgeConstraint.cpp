//
// Created by Yue Yang on 2020/11/13.
//

#include "EdgeConstraint.h"


std::ostream &operator<<(std::ostream &os, EdgeConstraint vertexConstraint) {
    os << "(" << vertexConstraint.time << ", " << vertexConstraint.location_1 << ", " \
        << vertexConstraint.location_2 << ")";
    return os;
}

bool EdgeConstraint::operator==(const EdgeConstraint & other) {
    return (this->time == other.getTime()) && (this->location_1 == other.getLocation1()) \
    && (this->location_2 == other.getLocation2());
}

EdgeConstraint::EdgeConstraint(int time, const Location &location1, const Location &location2) : time(time),
                                                                                                 location_1(location1),
                                                                                                 location_2(
                                                                                                         location2) {}

EdgeConstraint::EdgeConstraint(const EdgeConstraint &edgeConstraint) {
    this->time = edgeConstraint.time;
    this->location_1 = edgeConstraint.location_1;
    this->location_2 = edgeConstraint.location_2;
}

int EdgeConstraint::getTime() const {
    return time;
}

void EdgeConstraint::setTime(int time) {
    EdgeConstraint::time = time;
}

Location EdgeConstraint::getLocation1() const {
    return location_1;
}

void EdgeConstraint::setLocation1(const Location &location1) {
    location_1 = location1;
}

Location EdgeConstraint::getLocation2() const {
    return location_2;
}

void EdgeConstraint::setLocation2(const Location &location2) {
    location_2 = location2;
}

bool operator==(const EdgeConstraint& e1, const EdgeConstraint& e2) {
    return (e1.getTime() == e2.getTime()) && (e1.getLocation1() == e2.getLocation1()) \
    && (e1.getLocation2() == e2.getLocation2());
}