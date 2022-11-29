//
// Created by Yue Yang on 2020/11/13.
//

#ifndef ABCBS_CPP_VERTEXCONSTRAINT_H
#define ABCBS_CPP_VERTEXCONSTRAINT_H

#include <iostream>
#include "Location.h"
#include <sstream>

class VertexConstraint {
    public:
        int time;
        Location location;

    VertexConstraint(int time, const Location &location);

    VertexConstraint(const VertexConstraint &vertexConstraint);

    bool operator==(const VertexConstraint &);

    friend std::ostream& operator<<(std::ostream & os, VertexConstraint vertexConstraint);

    int getTime() const;

    void setTime(int time);

    Location getLocation() const;

    void setLocation(Location location);
};

bool operator==(const VertexConstraint& v1, const VertexConstraint& v2);


namespace std {
    template<>
    struct hash<VertexConstraint>
    {
        typedef VertexConstraint argument_type;
        typedef size_t result_type;

        size_t operator()(const VertexConstraint& vertexConstraint) const
        {
            std::stringstream ss;
            ss << vertexConstraint.time << vertexConstraint.location;
            std::string s = ss.str();
            return std::stol(s);
        }
    };
}

#endif //ABCBS_CPP_VERTEXCONSTRAINT_H
