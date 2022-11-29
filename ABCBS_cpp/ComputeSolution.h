//
// Created by Yue Yang on 2020/11/23.
//

#ifndef ABCBS_CPP_COMPUTESOLUTION_H
#define ABCBS_CPP_COMPUTESOLUTION_H
#include "Environment.h"
#include "AStar.h"

// Translated from the method compute_solution in Environment.py because of iterate including.
/*
 * Usage:
 * 1. Environment env = Environment(***);
 * 2. ComputeSolution computeSolution = ComputeSolution(env);
 * 3. computeSolution.compute_solution(***);
 * */
class ComputeSolution {
public:
    ComputeSolution(Environment&);

private:
    Environment env;
    std::vector<std::map<std::string, std::string>> agents; // 2nd string -> actually list. e.g., "[1, 2]" -> [1, 2]
    std::vector<int> dimension; // [32, 32]
    std::vector<std::vector<int>> obstacles; // [[1, 23], [2, 34]]
    std::map<std::string, std::map<std::string, State>> agent_dict;
    Constraints constraints;
    std::map<std::string, Constraints> constraint_dict;
    AStar a_star = AStar(Environment());

    double w_l;
    double alpha1;
    double alpha2;
    double alpha3;

public:

    std::tuple<bool, bool, bool, std::map<std::string, std::vector<State>>> compute_solution(int& start_time, \
    int& limited_time, std::string agent_name="", const std::map<std::string, std::vector<State>>* sol=NULL);

};


#endif //ABCBS_CPP_COMPUTESOLUTION_H
