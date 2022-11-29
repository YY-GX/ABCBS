//
// Created by Yue Yang on 2020/11/12.
//

#include "State.h"

State::State(int time, const Location &location) : time(time), location(location) {}

int State::getTime() const {
    return time;
}

void State::setTime(int time) {
    State::time = time;
}

const Location &State::getLocation() const {
    return location;
}

void State::setLocation(const Location &location) {
    State::location = location;
}

bool State::operator==(const State & other) {
    return (this->time == other.getTime()) && (this->location == other.getLocation());
}

bool State::is_equal_except_time(State other) {
    return this->location == other.getLocation();
}

std::ostream &operator<<(std::ostream &os, State state) {
    std::cout << "(" << state.time << ", " << state.location.getX() << ", " << state.location.getY() << ")";
    return os;
}

State::State() {
    this->time = 0;
    this->location = Location();
}

bool operator==(const State& s1, const State& s2) {
    return (s1.getTime() == s2.getTime()) and (s1.getLocation().getX() == s2.getLocation().getX()) \
    and (s1.getLocation().getY() == s2.getLocation().getY());
}

bool operator<(const State& s1, const State& s2) {
    if (s1.getTime() < s2.getTime()) {
        return true;
    }
    if (s1.getTime() > s2.getTime()) {
        return false;
    }
    if (s1.getLocation().getX() < s2.getLocation().getX()) {
        return  true;
    }
    if (s1.getLocation().getX() > s2.getLocation().getX()) {
        return false;
    }
    if (s1.getLocation().getY() < s2.getLocation().getY()) {
        return true;
    }
    if (s1.getLocation().getY() > s2.getLocation().getY()) {
        return false;
    }

//    return (s1.getTime() < s2.getTime()) and (s1.getLocation().getX() < s2.getLocation().getX()) \
//    and (s1.getLocation().getY() < s2.getLocation().getY());
}