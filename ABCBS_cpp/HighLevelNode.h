//
// Created by Yue Yang on 2020/11/24.
//

#ifndef ABCBS_CPP_HIGHLEVELNODE_H
#define ABCBS_CPP_HIGHLEVELNODE_H

#include <vector>
#include <map>
#include <stdlib.h>
#include <time.h>
#include <tuple>
#include <iostream>

#include "State.h"
#include "Constraints.h"

class HighLevelNode {

private:
    std::map<std::string, std::vector<State>> solution;
    std::map<std::string, Constraints> constraint_dict;
    double cost = 0;

public:

    HighLevelNode(const HighLevelNode& highLevelNode);

    HighLevelNode();

    bool operator<(const HighLevelNode& other);

    double getCost() const;

    const std::map<std::string, std::vector<State>> &getSolution() const;

    const std::map<std::string, Constraints> &getConstraintDict() const;

    void setSolution(const std::map<std::string, std::vector<State>> &solution);

    void setConstraintDict(const std::map<std::string, Constraints> &constraintDict);

    void setCost(double cost);

};

bool operator<(const HighLevelNode& h1, const HighLevelNode& h2);

bool operator==(const HighLevelNode& h1, const HighLevelNode& h2);

namespace std {
    template<>
    struct hash<HighLevelNode>
    {
        typedef HighLevelNode argument_type;
        typedef size_t result_type;

        size_t operator()(const HighLevelNode& highLevelNode) const
        {
            std::stringstream ss;
            for (auto const& cons : highLevelNode.getConstraintDict()) {
                ss << highLevelNode.getCost() << cons.second;
            }
            std::string s = ss.str();
            return std::stol(s);
        }
    };
}


#endif //ABCBS_CPP_HIGHLEVELNODE_H
