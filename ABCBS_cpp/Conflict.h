//
// Created by Yue Yang on 2020/11/13.
//

#ifndef ABCBS_CPP_CONFLICT_H
#define ABCBS_CPP_CONFLICT_H

#include <iostream>
#include "Location.h"

//extern int VERTEX = 1;
//extern int EDGE = 2;

class Conflict {
    private:
        int VERTEX = 1;
        int EDGE = 2;
    public:
        int time = -1;
        int type = -1;
        std::string agent_1 = "";
        std::string agent_2 = "";
        Location location_1 = Location();
        Location location_2 = Location();

    Conflict();

    friend std::ostream& operator<<(std::ostream & os, Conflict conflict);

    int getTime() const;

    void setTime(int time);

    int getType() const;

    void setType(int type);

    const std::string &getAgent1() const;

    void setAgent1(const std::string &agent1);

    const std::string &getAgent2() const;

    void setAgent2(const std::string &agent2);

    const Location &getLocation1() const;

    void setLocation1(const Location &location1);

    const Location &getLocation2() const;

    void setLocation2(const Location &location2);

    int getVertex() const;

    int getEdge() const;
};


#endif //ABCBS_CPP_CONFLICT_H
