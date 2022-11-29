//
// Created by Yue Yang on 2020/11/12.
//

#include "Location.h"

Location::Location(int x, int y) {
    this->x = x;
    this->y = y;
}

int Location::getX() const {
    return x;
}

void Location::setX(int x) {
    Location::x = x;
}

int Location::getY() const {
    return y;
}

void Location::setY(int y) {
    Location::y = y;
}

bool Location::operator==(const Location & other) {
    return (this->x == other.x) && (this->y == other.y);
}

std::ostream& operator<<(std::ostream & os, Location location) {
    os << "(" << location.x << ", " << location.y << ")";
    return os;
}
