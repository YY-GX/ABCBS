//
// Created by Yue Yang on 2020/11/24.
//

#ifndef ABCBS_CPP_ABCBS_H
#define ABCBS_CPP_ABCBS_H

#include "Environment.h"
#include "HighLevelNode.h"
#include "ComputeSolution.h"

class ABCBS {

private:
    Environment env;
    bool is_anytime = false;
    std::unordered_set<HighLevelNode> open_set;

//   Anytime things
    std::vector<HighLevelNode> focal_list;
    int limited_time;
    double gamma;
    double w_h;
    double w_l;

    int start_time;

public:
    ABCBS(const Environment &env, bool isAnytime = false, int limitedTime = 30, double gamma = 0.985, \
    double wH = 1.1, double wL = 1.1);

    void updateFocalBound(double bound);

    void updateLowerBound(double old_bound, double new_bound);

//    bool: is OPTIMAL
    std::tuple<bool, double> getNextBound();

//    bool: OVER
    std::tuple<bool, std::map<std::string, std::vector<std::map<std::string, int>>>> \
    open_focal_initialization(std::vector<std::map<std::string, std::vector<std::map<std::string, int>>>>& solution);

    std::tuple<bool, std::map<std::string, std::vector<std::map<std::string, int>>>> search();

    std::map<std::string, std::vector<std::map<std::string, int>>> \
    generate_plan(const std::map<std::string, std::vector<State>>& solution);


    void setStartTime(int startTime);
};


#endif //ABCBS_CPP_ABCBS_H
