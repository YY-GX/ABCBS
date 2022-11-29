//
// Created by Yue Yang on 2020/11/13.
//

#include "Conflict.h"

//int VERTEX = 1;
//int EDGE = 2;

std::ostream& operator<<(std::ostream & os, Conflict conflict) {
    std::cout << '(' << (conflict.time) << ", " << conflict.agent_1 << ", " << conflict.agent_2 << \
             "，" << (conflict.location_1) <<  ", " << (conflict.location_2) << ")";
    return os;
}

Conflict::Conflict() {}

int Conflict::getTime() const {
    return time;
}

void Conflict::setTime(int time) {
    Conflict::time = time;
}

int Conflict::getType() const {
    return type;
}

void Conflict::setType(int type) {
    Conflict::type = type;
}

const std::string &Conflict::getAgent1() const {
    return agent_1;
}

void Conflict::setAgent1(const std::string &agent1) {
    agent_1 = agent1;
}

const std::string &Conflict::getAgent2() const {
    return agent_2;
}

void Conflict::setAgent2(const std::string &agent2) {
    agent_2 = agent2;
}

const Location &Conflict::getLocation1() const {
    return location_1;
}

void Conflict::setLocation1(const Location &location1) {
    location_1 = location1;
}

const Location &Conflict::getLocation2() const {
    return location_2;
}

void Conflict::setLocation2(const Location &location2) {
    location_2 = location2;
}

int Conflict::getVertex() const {
    return VERTEX;
}

int Conflict::getEdge() const {
    return EDGE;
}
