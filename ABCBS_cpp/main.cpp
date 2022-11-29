#include <iostream>
#include <string>
#include <fstream>

#include "Location.h"
#include "State.h"
#include "Conflict.h"
#include "VertexConstraint.h"
#include "Constraints.h"
#include "Environment.h"
#include "Utils.h"
#include "ABCBS.h"

#include "yaml-cpp/include/yaml-cpp/yaml.h"

int main() {
//    Read data from file (.yaml)
    YAML::Node input = YAML::LoadFile("/Users/yygx/Documents/codes/robotics/mapf/multi_agent_path_planning_mine/ABCBS_cpp/inputs/input.yaml");
    std::vector<std::map<std::string, std::string>> agents_;
    for (auto const& agent_info : input["agents"]) {
        std::map<std::string, std::string> agent_item = {
                {"start", agent_info["start"].as<std::string>()},
                {"goal", agent_info["goal"].as<std::string>()},
                {"name", agent_info["name"].as<std::string>()}
        };
        agents_.push_back(agent_item);
    }

    std::vector<int> dimension_;
    Utils utils1 = Utils();
    dimension_.push_back(utils1.string2list(input["map"]["dimensions"].as<std::string>())[0]);
    dimension_.push_back(utils1.string2list(input["map"]["dimensions"].as<std::string>())[1]);

    std::vector<std::vector<int>> obstacles_;
    for (auto const& obs_info : input["map"]["obstacles"]) {
        std::vector<int> obs_item;
        obs_item.push_back(utils1.string2list(obs_info.as<std::string>())[0]);
        obs_item.push_back(utils1.string2list(obs_info.as<std::string>())[1]);
        obstacles_.push_back(obs_item);
    }

//    Set Environment for MAPF
    Environment environment_ = Environment(agents_, dimension_, obstacles_);
    ABCBS abcbs = ABCBS(environment_);

    std::tuple<bool, std::map<std::string, std::vector<std::map<std::string, int>>>> solution;
    std::map<std::string, std::vector<std::map<std::string, int>>> result;

//    Search for solution
    std::cout << "--------------  ABCBS Starts to run  --------------"  << std::endl;
    solution = abcbs.search();
    if (std::get<0>(solution)) {
        std::cout << "Solution Not Found!"  << std::endl;
        std::cout << "--------------     ABCBS END     --------------"  << std::endl;
    } else {
        std::cout << "Solution Found!"  << std::endl;
        std::cout << "--------------     ABCBS END     --------------"  << std::endl;
        result = std::get<1>(solution);
        std::cout << "Write to yaml file ..."  << std::endl;

        std::ofstream fout("/Users/yygx/Documents/codes/robotics/mapf/multi_agent_path_planning_mine/ABCBS_cpp/outputs/output.yaml");

        YAML::Node node;
        YAML::Node node_;
        for (auto const& ag_pth : result) {
            std::string agent = ag_pth.first;
            std::vector<std::map<std::string, int>> pth = ag_pth.second;
            YAML::Node node_ag;
            for (auto const& pt : pth) {
                node_ag.push_back(pt);
            }
            node_[agent] = node_ag;
        }
        node["schedule"]= node_;
        fout << node <<std::endl;

        std::cout << "~~~ Writing finished ~~~"  << std::endl;
    }

    return 0;
}
