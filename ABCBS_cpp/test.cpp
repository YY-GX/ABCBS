//
// Created by Yue Yang on 2020/11/28.
//

#include "test.h"

void test::test_modules() {
    std::cout << "Hello, World!" << std::endl;

    std::cout << "----- TEST1: Location -----" << std::endl;
    Location loc1 = Location(1, 2);
    Location loc2 = Location(1, 2);
    Location loc3 = Location(1, 3);
    std::cout << loc1;
    std::cout << (loc1 == loc2) << std::endl;
    std::cout << (loc1 == loc3) << std::endl;

    std::cout << "----- TEST2: State -----" << std::endl;
    State state1 = State(1, loc1);
    State state2 = State(2, loc1);
    State state3 = State(1, loc1);
    std::cout << (state1 == state2) << std::endl;
    std::cout << (state1 == state3) << std::endl;
    std::cout << (state1.is_equal_except_time(state2)) << std::endl;

    std::cout << "----- TEST3: Conflict -----" << std::endl;
    Conflict conflict = Conflict();
    std::cout << conflict << std::endl;
    std::cout << conflict.getVertex()  << std::endl;
    std::cout << conflict.getEdge()  << std::endl;

    std::cout << "----- TEST3: Conflict -----" << std::endl;
    VertexConstraint vertexConstraint1 = VertexConstraint(1, loc1);
    VertexConstraint vertexConstraint2 = VertexConstraint(2, loc1);
    VertexConstraint vertexConstraint3 = VertexConstraint(1, loc1);
    std::cout << vertexConstraint1  << std::endl;
    std::cout << vertexConstraint2  << std::endl;
    std::cout << vertexConstraint3  << std::endl;
    std::cout << (vertexConstraint1 == vertexConstraint2)  << std::endl;
    std::cout << (vertexConstraint1 == vertexConstraint3)  << std::endl;

    EdgeConstraint edgeConstraint1 = EdgeConstraint(1, loc1, loc2);
    EdgeConstraint edgeConstraint2 = EdgeConstraint(1, loc1, loc3);
    EdgeConstraint edgeConstraint3 = EdgeConstraint(1, loc1, loc2);
    std::cout << edgeConstraint1  << std::endl;
    std::cout << edgeConstraint2  << std::endl;
    std::cout << edgeConstraint3  << std::endl;
    std::cout << (edgeConstraint3 == edgeConstraint2)  << std::endl;
    std::cout << (edgeConstraint1 == edgeConstraint3)  << std::endl;

    std::cout << "----- TEST4: Constraints -----" << std::endl;

    Constraints constraints1 = Constraints();
    Constraints constraints2 = Constraints();
    constraints1.addVertexConstraint(vertexConstraint1);
    constraints2.addVertexConstraint(vertexConstraint2);
    constraints2.addVertexConstraint(vertexConstraint3);

    constraints1.addEdgeConstraint(edgeConstraint1);
    constraints2.addEdgeConstraint(edgeConstraint2);
    constraints2.addEdgeConstraint(edgeConstraint3);

    std::cout << constraints1  << std::endl;
    std::cout << constraints2  << std::endl;
    constraints1.add_constraint(constraints2);
    std::cout << constraints1  << std::endl;

    std::cout << "----- TEST5: Utils -----" << std::endl;
    Utils utils = Utils();
    for (int i = 0; i < 2; ++i) {
        std::cout << utils.string2list("[8799, 7]")[i] << std::endl;;
    }

    std::cout << "----- TEST6: Environment -----" << std::endl;
    // agents initialization
    std::vector<std::map<std::string, std::string>> agents;
    std::map<std::string, std::string> agent1 = {
            {"name", "agent0"},
            {"start", "[0, 2]"},
            {"goal", "[1, 3]"}
    };
    std::map<std::string, std::string> agent2 = {
            {"name", "agent1"},
            {"start", "[0, 0]"},
            {"goal", "[1, 12]"}
    };
    agents.push_back(agent1);
    agents.push_back(agent2);

    // dimention initialization
    std::vector<int> dimention = {32, 32};

    // obstacles initialization
    std::vector<std::vector<int>> obstacles = {{1, 2}, {2, 4}, {12, 31}};

    // Other initialization
    double w_l;
    double alpha1;
    double alpha2;
    double alpha3;

    // Initialization Test -> To see test outcomes, de commented-out codes in Environment::make_agent_dict
    Environment environment = Environment(agents, dimention, obstacles, w_l, alpha1, alpha2, alpha3);

}