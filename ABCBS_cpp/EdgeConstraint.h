//
// Created by Yue Yang on 2020/11/13.
//

#ifndef ABCBS_CPP_EDGECONSTRAINT_H
#define ABCBS_CPP_EDGECONSTRAINT_H

#include "Location.h"
#include <iostream>
#include <sstream>

class EdgeConstraint {
public:
    EdgeConstraint(int time, const Location &location1, const Location &location2);

    EdgeConstraint(const EdgeConstraint &edgeConstraint);

    int getTime() const;

    void setTime(int time);

    Location getLocation1() const;

    void setLocation1(const Location &location1);

    Location getLocation2() const;

    void setLocation2(const Location &location2);

public:
    int time;
    Location location_1;
    Location location_2;
    bool operator==(const EdgeConstraint &);

    friend std::ostream& operator<<(std::ostream & os, EdgeConstraint edgeConstraint);

};

bool operator==(const EdgeConstraint& e1, const EdgeConstraint& e2);


namespace std {
    template<>
    struct hash<EdgeConstraint>
    {
        typedef EdgeConstraint argument_type;
        typedef size_t result_type;

        size_t operator()(const EdgeConstraint& edgeConstraint) const
        {
            std::stringstream ss;
            ss << edgeConstraint.time << edgeConstraint.location_1 << edgeConstraint.location_2;
            std::string s = ss.str();
            return std::stol(s);
        }
    };
}

#endif //ABCBS_CPP_EDGECONSTRAINT_H
