//
// Created by Yue Yang on 2020/11/13.
//

#ifndef ABCBS_CPP_CONSTRAINTS_H
#define ABCBS_CPP_CONSTRAINTS_H

#include <unordered_set>
#include <iostream>
#include "VertexConstraint.h"
#include "EdgeConstraint.h"

class Constraints {
public:
    std::unordered_set<VertexConstraint> vertex_constraints;
    std::unordered_set<EdgeConstraint> edge_constraints;

    Constraints();

    void add_constraint(const Constraints & other);

    void addVertexConstraint(VertexConstraint & vertexConstraint);

    void addEdgeConstraint(EdgeConstraint & edgeConstraint);

    friend std::ostream& operator<<(std::ostream & os, const Constraints& constraints);

    const std::unordered_set<VertexConstraint> &getVertexConstraints() const;

    void setVertexConstraints(const std::unordered_set<VertexConstraint> &vertexConstraints);

    const std::unordered_set<EdgeConstraint> &getEdgeConstraints() const;

    void setEdgeConstraints(const std::unordered_set<EdgeConstraint> &edgeConstraints);

};


#endif //ABCBS_CPP_CONSTRAINTS_H
