//
// Created by Yue Yang on 2020/11/22.
//

#ifndef ABCBS_CPP_ASTAR_H
#define ABCBS_CPP_ASTAR_H

#include "Environment.h"
#include <unordered_set>
#include <limits>

class AStar {

private:
    std::map<std::string, std::map<std::string, State>> agent_dict;
    Environment env;

public:
    std::vector<State> reconstruct_path(std::map<State, State>& came_from, State& current);

    bool focal_compare(const State& node1, const State& node2, std::map<std::string, std::vector<State>>& pre_solution, \
    int& timestep, std::map<State, int>& f_score, double& alpha1, double& alpha2, double& alpha3);

    std::tuple<bool, bool, bool, std::vector<State>> search(std::string& agent_name, double& w_l, \
                double& alpha1, double &alpha2, double &alpha3, std::map<std::string, std::vector<State>>& pre_solution, \
                int &start_time, int &limited_time);

    AStar(const Environment &env);

    void setEnv(const Environment &env);
};


#endif //ABCBS_CPP_ASTAR_H
