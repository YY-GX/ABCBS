//
// Created by Yue Yang on 2020/11/22.
//

#ifndef ABCBS_CPP_ENVIRONMENT_H
#define ABCBS_CPP_ENVIRONMENT_H

#include <vector>
#include <map>
#include "State.h"
#include "Constraints.h"
//#include "AStar.h"
#include "Location.h"
#include "Utils.h"
#include "Conflict.h"
#include <stdlib.h>
#include <time.h>
#include <tuple>
#include <iostream>

class Environment {

private:
    std::vector<std::map<std::string, std::string>> agents; // 2nd string -> actually list. e.g., "[1, 2]" -> [1, 2]
    std::vector<int> dimension; // [32, 32]
    std::vector<std::vector<int>> obstacles; // [[1, 23], [2, 34]]
    std::map<std::string, std::map<std::string, State>> agent_dict;
    Constraints constraints;
    std::map<std::string, Constraints> constraint_dict;
//    AStar a_star;

    double w_l;
    double alpha1;
    double alpha2;
    double alpha3;

public:
    Environment(const std::vector<std::map<std::string, std::string>> &agents, const std::vector<int> &dimension,
                const std::vector<std::vector<int>> &obstacles, double wL=1.1, double alpha1=0.0, double alpha2=0.0, double alpha3=1.0);

    void make_agent_dict();

    std::vector<State> get_neighbors(const State& );

    bool state_valid(const State& );

    bool transition_valid(const State&, const State& );

    std::tuple<bool, Conflict> get_first_conflict(const std::map<std::string, std::vector<State>> & solution);

    State get_state(const std::string& agent_name, const std::map<std::string, std::vector<State>> & solution, const int t);

    std::map<std::string, Constraints> create_constraints_from_conflict(const Conflict&);

    int admissible_heuristic(const State&, const std::string agent_name);

    bool is_at_goal(State& state, const std::string agent_name);

//    1st bool: Found solution or not. 2nd bool: exceed time or not; 3rd bool: goal occupied or not.
//    std::tuple<bool, bool, bool, std::map<std::string, std::vector<State>>> compute_solution(int& start_time, \
//    int& limited_time, std::string agent_name, std::map<std::string, std::vector<State>>* sol);

    std::map<std::string, std::vector<std::map<std::string, int>>> \
    generate_plan(std::map<std::string, std::vector<State>> & solution);

    int compute_solution_cost(const std::map<std::string, std::vector<State>> & solution);











    const std::vector<std::map<std::string, std::string>> &getAgents() const;

    const std::vector<int> &getDimension() const;

    const std::vector<std::vector<int>> &getObstacles() const;

    const std::map<std::string, std::map<std::string, State>> &getAgentDict() const;

    const Constraints &getConstraints() const;

    const std::map<std::string, Constraints> &getConstraintDict() const;

    double getWL() const;

    double getAlpha1() const;

    double getAlpha2() const;

    double getAlpha3() const;

    void setConstraintDict(const std::map<std::string, Constraints> &constraintDict);

    void setConstraints(const Constraints &constraints);

    Environment();
};


#endif //ABCBS_CPP_ENVIRONMENT_H
