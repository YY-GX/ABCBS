//
// Created by Yue Yang on 2020/11/23.
//

#include "ComputeSolution.h"


std::tuple<bool, bool, bool, std::map<std::string, std::vector<State>>>  \
        ComputeSolution::compute_solution(int &start_time,
                                      int &limited_time,
                                      std::string agent_name,
                                      const std::map<std::string, std::vector<State>>* sol) {

    std::cout << "Start computing solution"  << std::endl;
    std::map<std::string, std::vector<State>> solution;
    if (sol == NULL) {

    } else {
        solution = *sol;
    }
    int start_time_ = time(NULL);
    if (agent_name == "") {
        for (auto const& agent_info : this->agent_dict)
        {
            std::string agent = agent_info.first;
//                 Get the constraint of this agent
            if ( this->constraint_dict.find(agent) == this->constraint_dict.end() ) {
                // not found
                this->constraints = Constraints();
            } else {
                // found
                this->constraints = this->constraint_dict.at(agent);
            }


//                Use A* to search ang get the solution for this agent
            std::tuple<bool, bool, bool, std::vector<State>> outcome = this->a_star.search(agent, this->w_l, \
                this->alpha1, this->alpha2, this->alpha3, solution, start_time, limited_time);


//              If occupied
            if (std::get<2>(outcome)) {
                return std::make_tuple(false, false, true, solution);
            }
//              If exceed time
            if (std::get<1>(outcome)) {
                return std::make_tuple(false, true, false, solution);
            }
//              If no solution found
            if (std::get<0>(outcome)) {
                return std::make_tuple(true, false, false, solution);
            }
//              Found solution for this agent
            solution[agent] = std::get<3>(outcome);
        }
    } else {
//                 Get the constraint of this agent
        if ( this->constraint_dict.find(agent_name) == this->constraint_dict.end() ) {
            // not found
            this->env.setConstraints(Constraints());
            this->a_star.setEnv(this->env);
        } else {
            // found
            this->env.setConstraints(this->constraint_dict.at(agent_name));
            this->a_star.setEnv(this->env);
        }

//                Use A* to search ang get the solution for this agent
        std::tuple<bool, bool, bool, std::vector<State>> outcome = this->a_star.search(agent_name, this->w_l, \
                this->alpha1, this->alpha2, this->alpha3, solution, start_time, limited_time);


//              If occupied
        if (std::get<2>(outcome)) {
            return std::make_tuple(false, false, true, solution);
        }
//              If exceed time
        if (std::get<1>(outcome)) {
            return std::make_tuple(false, true, false, solution);
        }
//              If no solution found
        if (std::get<0>(outcome)) {
            return std::make_tuple(true, false, false, solution);
        }
//              Found solution for this agent
        solution[agent_name] = std::get<3>(outcome);
    }

    int end_time_ = time(NULL);
    std::cout << "Consumed time for one solution computing: " << end_time_ - start_time_ << " s."  << std::endl;
    return std::make_tuple(false, false, false, solution);
}

ComputeSolution::ComputeSolution(Environment& environment) {
    this->env = environment;
    this->agent_dict = environment.getAgentDict();
    this->w_l = environment.getWL();
    this->alpha1 = environment.getAlpha1();
    this->alpha2 = environment.getAlpha2();
    this->alpha3 = environment.getAlpha3();
    this->constraint_dict = environment.getConstraintDict();
    this->constraints = environment.getConstraints();
    this->a_star = AStar(this->env);
}
