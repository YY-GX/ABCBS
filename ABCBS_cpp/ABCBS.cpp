//
// Created by Yue Yang on 2020/11/24.
//

#include "ABCBS.h"

ABCBS::ABCBS(const Environment &env, bool isAnytime, int limitedTime, double gamma, double wH, double wL) : env(env),
                                                                                                            is_anytime(
                                                                                                                    isAnytime),
                                                                                                            limited_time(
                                                                                                                    limitedTime),
                                                                                                            gamma(gamma),
                                                                                                            w_h(wH),
                                                                                                            w_l(wL) {
    this->w_h = w_h / gamma;
}

void ABCBS::updateFocalBound(double bound) {
    for (int i = 0; i < this->focal_list.size(); ++i) {
        HighLevelNode node = HighLevelNode();
        try {
                node = this->focal_list.at(i);
            }
            catch (std::out_of_range &exc) {
                std::cerr << exc.what() << " Line:" << __LINE__ << " File:" << __FILE__  << std::endl;
            }

        if (node.getCost() > bound) {
            this->focal_list.erase(this->focal_list.begin() + i);
        }
    }
}

void ABCBS::updateLowerBound(double old_bound, double new_bound) {
    for (auto const& new_node : this->open_set) {
        if ((new_node.getCost() > old_bound) and (new_node.getCost() < new_bound)) {
            for (int i = 0; i < this->focal_list.size(); ++i) {
                HighLevelNode node = HighLevelNode();
                try {
                        node = this->focal_list.at(i);
                    }
                    catch (std::out_of_range &exc) {
                        std::cerr << exc.what() << " Line:" << __LINE__ << " File:" << __FILE__  << std::endl;
                    }

                if (new_node.getConstraintDict().size() <= node.getConstraintDict().size()) {
                    this->focal_list.insert(this->focal_list.begin() + i, new_node);
                    break;
                }
                if (i == (this->focal_list.size() - 1)) {
                    this->focal_list.push_back(new_node);
                    break;
                }
            }

            if (this->focal_list.size() == 0) {
                this->focal_list.push_back(new_node);
            }
        }
    }
}

std::tuple<bool, double> ABCBS::getNextBound() {
    double return_thing = this->w_h * this->gamma;
    if (return_thing < 1.001) {
        return std::make_tuple(true, 0);
    }
    return std::make_tuple(false, return_thing);
}

std::tuple<bool, std::map<std::string, std::vector<std::map<std::string, int>>>> ABCBS::open_focal_initialization(
        std::vector<std::map<std::string, std::vector<std::map<std::string, int>>>>& solutions) {
//    Construct start node in high level
    HighLevelNode start = HighLevelNode();

//    1. Node.constraint_dict initialization
    std::map<std::string, Constraints> constraint_dict;
    for (auto const& agent_info : this->env.getAgentDict()) {
        std::string agent = agent_info.first;
        try {
                constraint_dict[agent] = Constraints();
            }
            catch (std::out_of_range &exc) {
                std::cerr << exc.what() << " Line:" << __LINE__ << " File:" << __FILE__  << std::endl;
            }

        start.setConstraintDict(constraint_dict);
    }

//    2. Node.solution initialization
    ComputeSolution computeSolution = ComputeSolution(this->env);
    std::tuple<bool, bool, bool, std::map<std::string, std::vector<State>>>  outcome = \
    computeSolution.compute_solution(this->start_time, this->limited_time);
    start.setSolution(std::get<3>(outcome));

//    Time exceeded, go outside ~
    if (std::get<1>(outcome)) {
        if (solutions.size() == 0) {
            std::cout << "Time exeeded. No solution found." << std::endl;
            std::map<std::string, std::vector<std::map<std::string, int>>> tmp;
            return std::make_tuple(true, tmp);
        } else {
            try {
                    return std::make_tuple(true, solutions.at(solutions.size() - 1));
                }
                catch (std::out_of_range &exc) {
                    std::cerr << exc.what() << " Line:" << __LINE__ << " File:" << __FILE__  << std::endl;
                }
        }
    }

    if (std::get<0>(outcome)) {
        std::map<std::string, std::vector<std::map<std::string, int>>> tmp;
        return std::make_tuple(true, tmp);
    }



//    3. Node.cost initialization
    start.setCost(this->env.compute_solution_cost(start.getSolution()));

//    OPEN & FOCAL initialization
    this->open_set.insert(start);
    this->focal_list.push_back(start);
    std::map<std::string, std::vector<std::map<std::string, int>>> tmp;

    return std::make_tuple(false, tmp);
}



std::map<std::string, std::vector<std::map<std::string, int>>> ABCBS::generate_plan(
        const std::map<std::string, std::vector<State>> &solution) {
    std::map<std::string, std::vector<std::map<std::string, int>>> plan;
    for (auto const& ag_pth : solution) {
        std::string agent = ag_pth.first;
        std::vector<State> path = ag_pth.second;
        std::vector<std::map<std::string, int>> path_dict_list;
        for (auto const& state : path) {
            std::map<std::string, int> pt = {
                    {"t", state.getTime()},
                    {"x", state.getLocation().getX()},
                    {"y", state.getLocation().getY()}
            };
            path_dict_list.push_back(pt);
        }
        try {
                plan[agent] = path_dict_list;
            }
            catch (std::out_of_range &exc) {
                std::cerr << exc.what() << " Line:" << __LINE__ << " File:" << __FILE__  << std::endl;
            }

    }
    return plan;
}




std::tuple<bool, std::map<std::string, std::vector<std::map<std::string, int>>>> ABCBS::search() {
//    Calc consumed time for the first success
    bool is_first_succ = true;
    int succ_start_time = time(NULL);

//    Find how many solutions
    int num_sol = 1;

//    All found solutions
    std::vector<std::map<std::string, std::vector<std::map<std::string, int>>>> solutions;

//    Time limitation iteration
    setStartTime(time(NULL));
//    int start_time = time(NULL);

//    Initialize OPEN & FOCAL
    std::tuple<bool, std::map<std::string, std::vector<std::map<std::string, int>>>>  msg_sol = \
    this->open_focal_initialization(solutions);

    if (std::get<0>(msg_sol)) {
        return std::make_tuple(false, std::get<1>(msg_sol));
    }

//    Keep running if anytime limitation is not over
    while (true) {

//        If there's no node, initialize again
        if ((this->open_set.size() == 0) and (this->focal_list.size() == 0)) {
            std::cout << "One new iteration for a new Solution" << std::endl;
            std::tuple<bool, std::map<std::string, std::vector<std::map<std::string, int>>>> msg_sol = \
        this->open_focal_initialization(solutions);

        }


        if (std::get<0>(msg_sol)) {
            return std::make_tuple(false, std::get<1>(msg_sol));
        }
//        Tighten the bound
        std::tuple<bool, double> w_h_info = this->getNextBound();

        if (std::get<0>(w_h_info)) {
            if (solutions.size() == 0) {
                std::map<std::string, std::vector<std::map<std::string, int>>> tmp;
                return std::make_tuple(true, tmp);
            } else {
                std::cout << "Optimal solution found."  << std::endl;
                try {
                        return std::make_tuple(false, solutions.at(solutions.size() - 1));
                    }
                    catch (std::out_of_range &exc) {
                        std::cerr << exc.what() << " Line:" << __LINE__ << " File:" << __FILE__  << std::endl;
                    }

            }
        }


//        Update Focal list
        double f_min = std::numeric_limits<int>::max();
        for (auto const& node : this->open_set) {
            if (node.getCost() < f_min) {
                f_min = node.getCost();
            }
        }


        this->updateFocalBound(w_h * f_min); // Here, f_min is the f(head(OPEN))

//        Has found one solution?
        bool has_found_sol = false;

        while (this->focal_list.size() != 0) {
            std::cout << "Length of focal list: " << this->focal_list.size() << std::endl;

            double f_min = std::numeric_limits<int>::max();

            for (auto const& node : this->open_set) {
                if (node.getCost() < f_min) {
                    f_min = node.getCost();
                }
            }

//            Order Focal list if not efficiently reusable (Here, it is. So do nothing)

//            Get the expanded state from FOCAL
            HighLevelNode P = HighLevelNode();
            try {
                    P = this->focal_list.at(0);
                }
                catch (std::out_of_range &exc) {
                    std::cerr << exc.what() << " Line:" << __LINE__ << " File:" << __FILE__  << std::endl;
                }

//            Delete the node from both OPEN & FOCAL
            this->open_set.erase(P);
            this->focal_list.erase(this->focal_list.begin());

//            Update environment
            this->env.setConstraintDict(P.getConstraintDict());

//            Get the first conflict from all agents
            std::tuple<bool, Conflict> first_conf = this->env.get_first_conflict(P.getSolution());

//            No conflict found
            if (!std::get<0>(first_conf)) {
                std::cout << ">>> " << num_sol << " solution(s) are found." << std::endl;
                std::cout << ">>> Cost of success node: " << P.getCost() << std::endl;

                if (is_first_succ) {
                    is_first_succ = !is_first_succ;
                    int succ_end_time = time(NULL);

                    std::cout << ">>> Success! Consumed time: " << succ_end_time - succ_start_time  << std::endl;

//                    If not anytime, return the first solution immediately, else continue compute one new solution
                    if (! this->is_anytime) {
                        return std::make_tuple(false, this->generate_plan(P.getSolution()));
                    }
                }

                num_sol += 1;
                has_found_sol = !has_found_sol;
                solutions.push_back(this->generate_plan(P.getSolution()));
                this->open_set.clear();
                this->focal_list.clear();
            }

//            Conflict found
            if (has_found_sol) {
                break;
            }

//            Have conflict, generate constraint dict
            std::map<std::string, Constraints> constraint_dict = \
            this->env.create_constraints_from_conflict(std::get<1>(first_conf));

//            Generate 2 son nodes
            for (auto const& agent_info : constraint_dict) {
                std::string agent = agent_info.first;

//                1. Extend constraint dict from father node
                HighLevelNode new_node = HighLevelNode(P);


//                2. Add new constrant to constraint dict according to which agent you choose
                try {
                        std::map<std::string, Constraints> constraintDict = new_node.getConstraintDict();
                        Constraints constraints = constraintDict.at(agent);
                        constraints.add_constraint(constraint_dict.at(agent));
                        constraintDict[agent] = constraints;
                        new_node.setConstraintDict(constraintDict);
                    }
                    catch (std::out_of_range &exc) {
                        std::cerr << exc.what() << " Line:" << __LINE__ << " File:" << __FILE__  << std::endl;
                    }


//                Update environment constraint
                this->env.setConstraintDict(new_node.getConstraintDict()); // A*就是根据self.env.constraint_dict来得到solution的


                /*
                 * 3. Update solution with new constraints added. Now we have the third param (agent),
                 * it can make the compute_solution only compute path for this agent
                 * */
                ComputeSolution computeSolution = ComputeSolution(this->env);
                std::tuple<bool, bool, bool, std::map<std::string, std::vector<State>>> compute_result = \
                computeSolution.compute_solution(this->start_time, this->limited_time, agent, &P.getSolution());

                if (std::get<2>(compute_result)) {
                    continue;
                }
                if (std::get<1>(compute_result)) {
                    if (solutions.size() == 0) {
                        std::cout << "Time exceeded. No solution found."  << std::endl;
                        std::map<std::string, std::vector<std::map<std::string, int>>> tmp;
                        return std::make_tuple(true, tmp);
                    } else {
                        return std::make_tuple(false, solutions.at(solutions.size() - 1));
                    }
                }

                if (std::get<0>(compute_result)) {
                    continue;
                }


                new_node.setSolution(std::get<3>(compute_result));

//                4. Add cost
                new_node.setCost(this->env.compute_solution_cost(new_node.getSolution()));

//                Add the son node to open set
                this->open_set.insert(new_node);

//                If it's conform to the standard(cost <= w * f_min), insert it to the focal list by the conflict number
                if (new_node.getCost() <= w_h * f_min) {
                    for (int i = 0; i < this->focal_list.size(); ++i) {
                        HighLevelNode node = this->focal_list.at(i);
                        if (new_node.getConstraintDict().size() <= node.getConstraintDict().size()) {
                            this->focal_list.insert(this->focal_list.begin() + i, new_node);
                            break;
                        }
                    }
                    if (this->focal_list.size() == 0) {
                        this->focal_list.push_back(new_node);
                    }
                }


//                Update the focal list because lower bound of open set is changed, sth may come into the open set~
                if (this->open_set.size() != 0) {
                    double f_min_new = std::numeric_limits<int>::max();
                    for (auto const& node : this->open_set) {
                        if (node.getCost() < f_min_new) {
                            f_min_new = node.getCost();
                        }
                    }

                    if ((this->open_set.size() != 0) and (this->w_h * f_min < this->w_h * f_min_new)) {
                        this->updateLowerBound(f_min, f_min_new);
                    }

                }
            }

        }


    }

    std::map<std::string, std::vector<std::map<std::string, int>>> tmp;
    return std::make_tuple(true, tmp);
}

void ABCBS::setStartTime(int startTime) {
    start_time = startTime;
}

