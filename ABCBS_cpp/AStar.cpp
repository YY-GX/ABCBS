//
// Created by Yue Yang on 2020/11/22.
//

#include "AStar.h"

AStar::AStar(const Environment &env) : env(env) {
    this->agent_dict = env.getAgentDict();
}

std::vector<State> AStar::reconstruct_path(std::map<State, State> &came_from, State &current) {
    std::vector<State> total_path;
    total_path.push_back(current);
    while (came_from.find(current) != came_from.end()) {
        current = came_from.at(current);
        total_path.push_back(current);
    }


    std::reverse(total_path.begin(), total_path.end());

    return total_path;
}


bool AStar::focal_compare(const State &node1, const State &node2, std::map<std::string, std::vector<State>> &pre_solution,
                          int &timestep, std::map<State, int> &f_score, double &alpha1, double &alpha2,
                          double &alpha3) {
//    Part1: Number of conflicts with other agents
    double num_conflict_node1 = 0;
    double num_conflict_node2 = 0;
    for (const auto& agent_info : pre_solution) {
        std::string agent_name_ = agent_info.first;
//        In this timestep and for this agent, the (x, y, t) of previous solution
        if (timestep >= pre_solution.at(agent_name_).size()) {
            break;
        }
        State pre_state = pre_solution.at(agent_name_).at(timestep);
        if (pre_state == node1) {
            num_conflict_node1 += 1;
        }
        if (pre_state == node2) {
            num_conflict_node2 += 1;
        }
    }

//        Part2: Sum of distance from dynamic obstacles (Useless now)
        double distance_node1 = 0;
        double distance_node2 = 0;


//        Part3: f score
        double f_score_node1 = f_score[node1];
        double f_score_node2 = f_score[node2];

//        Part4: Calculate the formula
//        Normalize
        if ((num_conflict_node1 == 0) and (num_conflict_node2 == 0)) {
            num_conflict_node1 = 0;
            num_conflict_node2 = 0;
        } else {
            num_conflict_node1 = num_conflict_node1 / (num_conflict_node1 + num_conflict_node2);
            num_conflict_node2 = num_conflict_node2 / (num_conflict_node1 + num_conflict_node2);
        }

        if ((distance_node1 == 0) and (distance_node2 == 0)) {
            distance_node1 = 0;
            distance_node2 = 0;
        } else {
            distance_node1 = distance_node1 / (distance_node1 + distance_node2);
            distance_node2 = distance_node2 / (distance_node1 + distance_node2);
        }

//        Calculation
        double value_node1 = alpha1 * num_conflict_node1 - alpha2 * distance_node1 + alpha3 * f_score_node1;
        double value_node2 = alpha1 * num_conflict_node2 - alpha2 * distance_node2 + alpha3 * f_score_node2;


        if (value_node1 <= value_node2) {
            return true;
        } else {
            return false;
        }

}








std::tuple<bool, bool, bool, std::vector<State>> AStar::search(std::string &agent_name , double &w_l,
                                                                                      double &alpha1, double &alpha2,
                                                                                      double &alpha3,
                                                                                      std::map<std::string, std::vector<State>> &pre_solution,
                                                                                      int &start_time,
                                                                                      int &limited_time) {

    if ( this->env.getConstraintDict().find(agent_name) != this->env.getConstraintDict().end() ) {
        // found
        Constraints constraint_dict_ = this->env.getConstraintDict().at(agent_name);
//        See if the goal occupied first. If it is, return a long long route~
        for (auto const& constraint : constraint_dict_.getVertexConstraints()) {
            if ((constraint.getLocation().getX() == this->env.getAgentDict().at(agent_name).at("goal").getLocation().getX()) \
                and (constraint.getLocation().getY() == this->env.getAgentDict().at(agent_name).at("goal").getLocation().getY())) {
                std::vector<State> tmp;
                tmp.push_back(State(0, Location()));
                std::tuple<bool, bool, bool, std::vector<State>> occupy_return = std::make_tuple(false, false, true, tmp);
                return occupy_return;
            }
        }

    }
    int cnt = 0;

    State initial_state = this->agent_dict.at(agent_name).at("start");
    int step_cost = 1;
    std::unordered_set<State> closed_set;
    std::unordered_set<State> open_set;
    open_set.insert(initial_state);

    std::map<State, State> came_from;
    std::map<State, int> g_score;
    g_score[initial_state] = 0;
    std::map<State, int> f_score;
//        initial_state is the coordinate
    f_score[initial_state] = this->env.admissible_heuristic(initial_state, agent_name);

//        Focal part
    std::vector<State> focal_list;
    focal_list.push_back(initial_state);

    int timestep = 0;

    while (focal_list.size() != 0) {
        cnt ++;
//            See if exceed the time limitation

        int end_time = time(NULL);
        if ((end_time - start_time) >= limited_time) {
            std::vector<State> tmp;
            tmp.push_back(State(0, Location()));
            std::tuple<bool, bool, bool, std::vector<State>> time_return = std::make_tuple(false, true, false, tmp);
            return time_return;
        }


//            open_item is the coordinate. temp_dict.keys(): OPEN; temp_dict.values(): f
        std::map<State, int> temp_dict;
        for (const auto& open_item: open_set) {
            int score;
            if ( f_score.find(open_item) == f_score.end() ) {
                // not found
                score = std::numeric_limits<int>::max();
            } else {
                // found
                score = f_score.at(open_item);
            }
            temp_dict[open_item] = score;
        }



        State current_idx = State(0, Location());
        int min_v = std::numeric_limits<int>::max();
        for (const auto& k_v : temp_dict) {
            if (k_v.second < min_v) {
                min_v = k_v.second;
                current_idx = k_v.first;
            }
        }



//            f(head(OPEN))
        int f_min = temp_dict.at(current_idx);

//            Min node in focal list
        State current = focal_list[0];

//            Remove the node from OPEN & FOCAL. Add it to CLOSE
        open_set.erase(current);

        focal_list.erase(focal_list.begin());

        closed_set.insert(current);


//            Is goal?
        if (this->env.is_at_goal(current, agent_name)) {
            std::vector<State> pth = this->reconstruct_path(came_from, current);
            return std::make_tuple(false, false, false, pth);
        }


        timestep += 1;

        /*
         * The get_neighbors() function will return all valid directions to go to
         * what's valid is defined by constraints list
         * */
        std::vector<State> neighbor_list = this->env.get_neighbors(current);
        for (const auto& neighbor : neighbor_list) {
            bool is_add = false;
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }

            int tentative_g_score;
            if (g_score.find(current) == g_score.end()) {
                tentative_g_score = std::numeric_limits<int>::max();
            } else {
                tentative_g_score = g_score.at(current) + step_cost;
            }

            if (open_set.find(neighbor) == open_set.end()) {
                is_add = true;
                open_set.insert(neighbor);
            } else if (g_score.find(neighbor) != g_score.end()) {
                if (tentative_g_score >= g_score.at(neighbor)) {
                    continue;
                }
            }

            came_from[neighbor] = current;
            g_score[neighbor] = tentative_g_score;
            f_score[neighbor] = g_score.at(neighbor) + this->env.admissible_heuristic(neighbor, agent_name);

//                If there's new node added to OPEN, you should consider whether it should be added to FOCAL
            if (is_add) {
                if (f_score.at(neighbor) <= w_l * f_min) {
                    for (int i = 0; i < focal_list.size(); ++i) {
                        State node = focal_list.at(i);
                        bool focal_co = this->focal_compare(neighbor, node, pre_solution, timestep, f_score, alpha1, alpha2, alpha3);
                        if (focal_co) {
                            focal_list.insert(focal_list.begin() + i, neighbor);
                            break;
                        }
                        if (i == (focal_list.size() - 1)) {
                            focal_list.push_back(neighbor);
                            break;
                        }
                    }

                    if (focal_list.size() == 0) {
                        focal_list.push_back(neighbor);
                    }
                }
            }


        }



//            Update the focal list because lower bound of open set is changed, sth may come into the open set~
        std::map<State, int> temp_dict_;
        for (const auto& open_item : open_set) {
            if (f_score.find(open_item) == f_score.end()) {
                temp_dict_[open_item] = std::numeric_limits<int>::max();
            } else {
                temp_dict_[open_item] = f_score.at(open_item);
            }
        }

        if (temp_dict_.size() == 0) {
            std::vector<State> tmp;
            tmp.push_back(State(0, Location()));
            std::tuple<bool, bool, bool, std::vector<State>> not_found_return = std::make_tuple(true, false, false, tmp);
            return not_found_return;
        }



        State current_idx_ = State(0, Location());
        int min_v_ = std::numeric_limits<int>::max();
        for (const auto& k_v : temp_dict_) {
            if (k_v.second < min_v_) {
                min_v_ = k_v.second;
                current_idx_ = k_v.first;
            }
        }


        int f_min_new = temp_dict_.at(current_idx_);
        if ((open_set.size() != 0) and (f_min < f_min_new)) {
//                updateLowerBound part
            double old_bound = w_l * f_min;
            double new_bound = w_l * f_min_new;
            for (const auto& new_node : open_set) {
                if ((f_score.at(new_node) > old_bound) and (f_score.at(new_node) < new_bound)) {
                    for (int i = 0; i < focal_list.size(); ++i) {
                        State node = focal_list.at(i);
                        if (focal_compare(new_node, node, pre_solution, timestep, f_score, alpha1, alpha2, alpha3)) {
                            focal_list.insert(focal_list.begin() + i, new_node);
                            break;
                        }
                        if (i == (focal_list.size() - 1)) {
                            focal_list.push_back(new_node);
                            break;
                        }
                    }

                    if (focal_list.size() == 0) {
                        focal_list.push_back(new_node);
                    }
                }

            }
        }




    }

    std::cout << agent_name << " END, it doesn't find its path :(" << std::endl;
    std::vector<State> tmp;
    tmp.push_back(State(0, Location()));
    std::tuple<bool, bool, bool, std::vector<State>> not_found_return = std::make_tuple(true, false, false, tmp);
    return not_found_return;
}

void AStar::setEnv(const Environment &env) {
    AStar::env = env;
}
