
def search(self, N=0, agent_num=0, obstacle_prob=0):
    # Calc consumed time for the first success
    is_first_succ = True
    succ_start_time = time.time()
    # Find how many solutions
    num_sol = 1
    # All found solutions
    solutions = []
    # Time limitation iteration
    self.start_time = time.time()
    # Initialize OPEN & FOCAL
    msg, return_data = self.open_focal_initialization(solutions)
    if msg == "OVER":
        return return_data
    # Keep running if anytime limitation is not over
    while True:
        # If there's no node, initialize again
        if len(self.open_set) == 0 or len(self.focal_list) == 0:
            self.logger.info("One new iteration for a new Solution")
            msg, return_data = self.open_focal_initialization(solutions)

        if msg == "OVER":
            return return_data
        # Tighten the bound
        w_h = self.getNextBound()
        if w_h == "OPTIMAL":
            if len(solutions) == 0:
                return {}
            else:
                self.logger.info("Optimal solution found.")
                return solutions[len(solutions) - 1]
        # Update Focal list
        f_min = min(self.open_set).cost
        self.updateFocalBound(w_h * f_min) # Here, f_min is the f(head(OPEN))
        # Has found one solution?
        has_found_sol = False
        while self.focal_list:
            self.logger.debug("Length of focal list: ", len(self.focal_list))
            f_min = min(self.open_set).cost
            # Get the expanded state from FOCAL
            P = self.focal_list[0]
            # Delete the node from both OPEN & FOCAL
            self.open_set -= {P}
            self.focal_list.pop(0)
            # Update environment
            self.env.constraint_dict = P.constraint_dict
            # Get the first conflict from all agents
            conflict_dict = self.env.get_first_conflict(P.solution)
            self.logger.debug("Conflict dict: " + str(conflict_dict))
            # If there's no conflict for all paths of agents, solution if found!
            if not conflict_dict:
                self.logger.info(str(num_sol) + " solution(s) are found.")
                self.logger.info("Cost of success nude: " + str(P.cost))
                if is_first_succ:
                    is_first_succ = not is_first_succ
                    succ_end_time = time.time()
                    self.logger.info("Success! Consumed time: " + str(succ_end_time - succ_start_time))
                    # If not anytime, return the first solution immediately, else continue compute one new solution
                    if not self.is_anytime:
                        return self.generate_plan(P.solution)
                num_sol += 1
                has_found_sol = not has_found_sol
                solutions.append(self.generate_plan(P.solution))
                self.open_set = set()
                self.focal_list = []
            if has_found_sol:
                break
            # Have conflict, generate constraint dict
            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)
            # Generate 2 son nodes
            for agent in constraint_dict.keys():
                self.logger.debug("Solve conflict from: " + str(agent))
                # 1. Extend constraint dict from father node
                new_node = deepcopy(P)
                # 2. Add new constrant to constraint dict according to which agent you choose
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])
                # Update environment constraint
                self.env.constraint_dict = new_node.constraint_dict # A*就是根据self.env.constraint_dict来得到solution的
                # 3. Update solution with new constraints added. Now we have the third param (agent),
                #       it can make the compute_solution only compute path for this agent
                new_node.solution = self.env.compute_solution(self.start_time, self.limited_time, agent, deepcopy(P.solution))
                if new_node.solution == "OCCUPY":
                    continue
                if new_node.solution == "TIME":
                    if len(solutions) == 0:
                        self.logger.warning("Time exceeded. No solution found.")
                        return {}
                    else:
                        return solutions[len(solutions) - 1]
                if not new_node.solution:
                    continue
                # 4. Add cost
                new_node.cost = self.env.compute_solution_cost(new_node.solution)
                # Add the son node to open set
                self.open_set |= {new_node}
                # If it's conform to the standard(cost <= w * f_min), insert it to the focal list by the conflict number
                self.logger.debug("New_node.cost: " + str(new_node.cost))
                self.logger.debug("w_h * f_min: " + str(w_h * f_min))
                if new_node.cost <= w_h * f_min:
                    for i in range(len(self.focal_list)):
                        node = self.focal_list[i]
                        if len(new_node.constraint_dict) <= len(node.constraint_dict):
                            self.focal_list.insert(i, new_node)
                            break
                        if i == len(self.focal_list) - 1:
                            self.focal_list.append(new_node)
                            break
                    if len(self.focal_list) == 0:
                        self.focal_list.append(new_node)
            # Update the focal list because lower bound of open set is changed, sth may come into the open set~
            if len(self.open_set) != 0:
                f_min_new = min(self.open_set).cost
                if (len(self.open_set) != 0) and (self.w_h * f_min < self.w_h * f_min_new):
                    self.updateLowerBound(f_min, f_min_new)
    return {}