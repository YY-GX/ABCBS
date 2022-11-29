//
// Created by Yue Yang on 2020/11/22.
//

#include "Utils.h"
#include <vector>

// Input: "[1, 2]"
std::vector<int> Utils::string2list(std::string string_ls) {
//    int list[2] = {0, 0};
    std::vector<int> list;
    std::string delimiter = ", ";
    list.push_back(std::stoi(string_ls.substr(1, string_ls.find(delimiter) - 1)));
    std::string str = string_ls.substr(string_ls.find(delimiter) + 2, string_ls.length() - string_ls.find(delimiter) - 3);
    list.push_back(std::stoi(str));

    return list;
}