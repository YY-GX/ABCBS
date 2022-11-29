//
// Created by Yue Yang on 2020/11/13.
//

#include "Constraints.h"

std::ostream &operator<<(std::ostream &os, const Constraints& constraints) {
    os << "VC: [" ;
    for (const auto& constraint : constraints.vertex_constraints) {
        os << constraint << ", ";
    }
   os << "], EC: [";

    for (const auto& constraint : constraints.edge_constraints) {
        os << constraint << ", ";
    }
    os << "]";

    return os;
}

void Constraints::add_constraint(const Constraints & other) {
    this->vertex_constraints.insert(other.vertex_constraints.begin(), other.vertex_constraints.end());
    this->edge_constraints.insert(other.edge_constraints.begin(), other.edge_constraints.end());
}

void Constraints::addVertexConstraint(VertexConstraint & vertexConstraint) {
    this->vertex_constraints.insert(vertexConstraint);
}

void Constraints::addEdgeConstraint(EdgeConstraint & edgeConstraint) {
    this->edge_constraints.insert(edgeConstraint);
}

Constraints::Constraints() {}

const std::unordered_set<VertexConstraint> &Constraints::getVertexConstraints() const {
    return vertex_constraints;
}

void Constraints::setVertexConstraints(const std::unordered_set<VertexConstraint> &vertexConstraints) {
    vertex_constraints = vertexConstraints;
}

const std::unordered_set<EdgeConstraint> &Constraints::getEdgeConstraints() const {
    return edge_constraints;
}

void Constraints::setEdgeConstraints(const std::unordered_set<EdgeConstraint> &edgeConstraints) {
    edge_constraints = edgeConstraints;
}
