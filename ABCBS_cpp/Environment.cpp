//
// Created by Yue Yang on 2020/11/22.
//

#include "Environment.h"


Environment::Environment(const std::vector<std::map<std::string, std::string>> &agents,
                         const std::vector<int> &dimension, const std::vector<std::vector<int>> &obstacles, double wL,
                         double alpha1, double alpha2, double alpha3) : agents(agents), dimension(dimension),
                                                                        obstacles(obstacles), w_l(wL), alpha1(alpha1),
                                                                        alpha2(alpha2), alpha3(alpha3) {

    this->make_agent_dict();
}

void Environment::make_agent_dict() {
    for (int i = 0; i < this->agents.size(); ++i) {
        std::map<std::string, std::string> agent = agents[i];
        Utils utils = Utils();

//        YY: TEST
//        std::cout << agent["name"] << std::endl;
//        std::cout << agent["start"] << std::endl;
//        std::cout << agent["goal"] << std::endl;

        State start_state = State(0, Location(utils.string2list(agent["start"])[0],utils.string2list(agent["start"])[1]));
        State goal_state = State(0, Location(utils.string2list(agent["goal"])[0],utils.string2list(agent["goal"])[1]));
        std::map<std::string, State> agent_info = {
                {"start", start_state},
                {"goal", goal_state}
        };
        this->agent_dict[agent["name"]] = agent_info;

//         YY: TEST
//        std::cout << this->agent_dict.at(agent["name"]).at("start") << std::endl;
//        std::cout << this->agent_dict.at(agent["name"]).at("goal") << std::endl;

    }
}

bool Environment::state_valid(const State & state) {
    bool obs_eq = false;
    for (int i = 0; i < this->obstacles.size(); ++i) {
        if ((this->obstacles[i][0] == state.getLocation().getX()) and \
        (this->obstacles[i][1] == state.getLocation().getY())) {
            obs_eq = true;
            break;
        }
    }

    bool is_in_vCons = false;
    if (this->constraints.getVertexConstraints().size() >= 1) {
        for (const auto& v_cons : this->constraints.getVertexConstraints()) {
            if ((v_cons.getTime() == state.getTime()) and (v_cons.getLocation() == state.getLocation())) {
                is_in_vCons = true;
            }
        }
    }


    bool return_val = (state.getLocation().getX() >= 0) and (state.getLocation().getX() < this->dimension[0]) \
    and (state.getLocation().getY() >= 0) and (state.getLocation().getY() < this->dimension[1]) \
    and !is_in_vCons and !obs_eq;


    return return_val;
}

bool Environment::transition_valid(const State & state1, const State & state2) {
    bool return_val = this->constraints.getEdgeConstraints().find(EdgeConstraint(state1.getTime(), state1.getLocation(), \
    state2.getLocation())) == this->constraints.getEdgeConstraints().end();

//    std::cout << "trans: " << return_val  << std::endl;
    return return_val;
}

std::vector<State> Environment::get_neighbors(const State & state) {
    std::vector<State> neighbors;
    State n = State(0, Location());

//    Wait action
    n = State(state.getTime() + 1, state.getLocation());
    if (this->state_valid(n)) {
        neighbors.push_back(n);
    }

//    Up action
    n = State(state.getTime() + 1, Location(state.getLocation().getX(), state.getLocation().getY() + 1));
    if (this->state_valid(n) and this->transition_valid(state, n)) {
        neighbors.push_back(n);
    }

//    Down action
    n = State(state.getTime() + 1, Location(state.getLocation().getX(), state.getLocation().getY() - 1));
    if (this->state_valid(n) and this->transition_valid(state, n)) {
        neighbors.push_back(n);
    }

//    Left action
    n = State(state.getTime() + 1, Location(state.getLocation().getX() - 1, state.getLocation().getY()));
    if (this->state_valid(n) and this->transition_valid(state, n)) {
        neighbors.push_back(n);
    }

//    Right action
    n = State(state.getTime() + 1, Location(state.getLocation().getX() + 1, state.getLocation().getY()));
    if (this->state_valid(n) and this->transition_valid(state, n)) {
        neighbors.push_back(n);
    }

    return neighbors;
}

State Environment::get_state(const std::string &agent_name, const std::map<std::string, std::vector<State>> &solution,
                             const int t) {

//State Environment::get_state(const std::string &agent_name, const std::map<std::string, std::vector<State>> &solution,
//                             int t) {
    if (t < solution.at(agent_name).size()) {
        return solution.at(agent_name).at(t);
    } else {
        return solution.at(agent_name).at(solution.at(agent_name).size() - 1);
    }
}

std::tuple<bool, Conflict> Environment::get_first_conflict(const std::map<std::string, std::vector<State>> &solution) {
    int max_t = 0;
    Conflict result = Conflict();
    for (auto const& x : solution)
    {
        int crr_t = x.second.size();
        if (crr_t > max_t) {
            max_t = crr_t;
        }
    }

    for (int t = 0; t < max_t; ++t) {
        std::vector<std::string> agent_keys;
        for (auto const& x : solution) {
            agent_keys.push_back(x.first);
        }
//        First kind of conflict (vertex conflict)

        for (int i = 0; i < agent_keys.size(); ++i) {
            for (int j = i + 1; j < agent_keys.size(); ++j) {
                std::string agent_1 = agent_keys[i];
                std::string agent_2 = agent_keys[j];
                State state_1 = this->get_state(agent_1, solution, t);
                State state_2 = this->get_state(agent_2, solution, t);
                if (state_1.is_equal_except_time(state_2)) {
                    result.setTime(t);
                    result.setType(result.getVertex());
                    result.setLocation1(state_1.getLocation());
                    result.setAgent1(agent_1);
                    result.setAgent2(agent_2);
                    return std::make_tuple(true, result);
                }
            }
        }



//        Second kind of conflict (Edge conflict)
        for (int i = 0; i < agent_keys.size(); ++i) {
            for (int j = i + 1; j < agent_keys.size(); ++j) {
                std::string agent_1 = agent_keys[i];
                std::string agent_2 = agent_keys[j];
                State state_1a = this->get_state(agent_1, solution, t);
                State state_1b = this->get_state(agent_1, solution, t + 1);
                State state_2a = this->get_state(agent_2, solution, t);
                State state_2b = this->get_state(agent_2, solution, t + 1);
                if (state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a)) {
                    result.setTime(t);
                    result.setType(result.getEdge());
                    result.setLocation1(state_1a.getLocation());
                    result.setLocation2(state_1b.getLocation());
                    result.setAgent1(agent_1);
                    result.setAgent2(agent_2);
                    return std::make_tuple(true, result);
                }
            }
        }


    }

    return std::make_tuple(false, result);
}

std::map<std::string, Constraints> Environment::create_constraints_from_conflict(const Conflict & conflict) {
    std::map<std::string, Constraints> constraint_dict;
//    std::cout << "---- create_constraints_from_conflict ----"  << std::endl;
    if (conflict.getType() == conflict.getVertex()) {
        VertexConstraint v_constraint = VertexConstraint(conflict.getTime(), conflict.getLocation1());
        Constraints constraint = Constraints();
        constraint.addVertexConstraint(v_constraint);
        constraint_dict[conflict.getAgent1()] = constraint;
        constraint_dict[conflict.getAgent2()] = constraint;
    } else if (conflict.getType() == conflict.getEdge()) {
        Constraints constraint1 = Constraints();
        Constraints constraint2 = Constraints();
        EdgeConstraint e_constraint1 = EdgeConstraint(conflict.getTime(), conflict.getLocation1(), conflict.getLocation2());
        EdgeConstraint e_constraint2 = EdgeConstraint(conflict.getTime(), conflict.getLocation2(), conflict.getLocation1());
        constraint1.addEdgeConstraint(e_constraint1);
        constraint2.addEdgeConstraint(e_constraint2);
        constraint_dict[conflict.getAgent1()] = constraint1;
        constraint_dict[conflict.getAgent2()] = constraint2;
    }

    return constraint_dict;
}

int Environment::admissible_heuristic(const State & state, const std::string agent_name) {
    State goal = this->agent_dict.at(agent_name).at("goal");
    return abs(state.getLocation().getX() - goal.getLocation().getX()) + \
    abs(state.getLocation().getY() - goal.getLocation().getY());
}

bool Environment::is_at_goal(State &state, const std::string agent_name) {
    State goal_state = this->agent_dict.at(agent_name).at("goal");
    return state.is_equal_except_time(goal_state);
}


std::map<std::string, std::vector<std::map<std::string, int>>> Environment::generate_plan(
        std::map<std::string, std::vector<State>> &solution) {
        std::map<std::string, std::vector<std::map<std::string, int>>> plan;
        for (auto const& ag_pth : solution) {
            std::vector<std::map<std::string, int>> path_dict_list;
            std::string agent = ag_pth.first;
            std::vector<State> path = ag_pth.second;
            for (auto const& state : path) {
                std::map<std::string, int> pt = {
                        {"t", state.getTime()},
                        {"x", state.getLocation().getX()},
                        {"y", state.getLocation().getY()}
                };
                path_dict_list.push_back(pt);
            }
            plan[agent] = path_dict_list;
        }
        return plan;
    }

int Environment::compute_solution_cost(const std::map<std::string, std::vector<State>> &solution) {
    int cost = 0;
    for (auto const& ag_pth : solution) {
        std::vector<State> path = ag_pth.second;
        cost += path.size();
    }
    return cost;
}















const std::vector<std::map<std::string, std::string>> &Environment::getAgents() const {
    return agents;
}

const std::vector<int> &Environment::getDimension() const {
    return dimension;
}

const std::vector<std::vector<int>> &Environment::getObstacles() const {
    return obstacles;
}

const std::map<std::string, std::map<std::string, State>> &Environment::getAgentDict() const {
    return agent_dict;
}

const Constraints &Environment::getConstraints() const {
    return constraints;
}

const std::map<std::string, Constraints> &Environment::getConstraintDict() const {
    return constraint_dict;
}

double Environment::getWL() const {
    return w_l;
}

double Environment::getAlpha1() const {
    return alpha1;
}

double Environment::getAlpha2() const {
    return alpha2;
}

double Environment::getAlpha3() const {
    return alpha3;
}

Environment::Environment() {}

void Environment::setConstraintDict(const std::map<std::string, Constraints> &constraintDict) {
    constraint_dict = constraintDict;
}

void Environment::setConstraints(const Constraints &constraints) {
    Environment::constraints = constraints;
}







