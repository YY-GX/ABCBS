//
// Created by Yue Yang on 2020/11/24.
//

#include "HighLevelNode.h"

bool HighLevelNode::operator<(const HighLevelNode &other) {
    return this->cost < other.getCost();
}

bool operator==(const HighLevelNode& h1, const HighLevelNode& h2) {
    return h1.getCost() == h2.getCost();
}

bool operator<(const HighLevelNode& h1, const HighLevelNode& h2) {
    return h1.getCost() < h2.getCost();
}

double HighLevelNode::getCost() const {
    return cost;
}

// ?: is right?
HighLevelNode::HighLevelNode(const HighLevelNode &highLevelNode) {
    for (auto const& ag_sol : highLevelNode.getSolution())  {
        std::string agent = ag_sol.first;
        std::vector<State> sol = ag_sol.second;

        std::vector<State> this_sol;
        for (auto const& state : sol) {
            this_sol.push_back(state);
        }
        this->solution[agent] = this_sol;
    }

    for (auto const& ag_cons : highLevelNode.getConstraintDict())  {
        std::string agent = ag_cons.first;
        Constraints cons = ag_cons.second;

        Constraints this_cons;
        this_cons.setEdgeConstraints(cons.getEdgeConstraints());
        this_cons.setVertexConstraints(cons.getVertexConstraints());
        this->constraint_dict[agent] = this_cons;
    }


//    this->solution = highLevelNode.getSolution();
//    this->constraint_dict = highLevelNode.getConstraintDict();
    this->cost = highLevelNode.getCost();
}

HighLevelNode::HighLevelNode() {}

void HighLevelNode::setSolution(const std::map<std::string, std::vector<State>> &solution) {
    HighLevelNode::solution = solution;
}

void HighLevelNode::setConstraintDict(const std::map<std::string, Constraints> &constraintDict) {
    constraint_dict = constraintDict;
}

void HighLevelNode::setCost(double cost) {
    HighLevelNode::cost = cost;
}

const std::map<std::string, std::vector<State>> &HighLevelNode::getSolution() const {
    return solution;
}

const std::map<std::string, Constraints> &HighLevelNode::getConstraintDict() const {
    return constraint_dict;
}
