//
// Created by Yue Yang on 2020/11/12.
//

#ifndef ABCBS_CPP_STATE_H
#define ABCBS_CPP_STATE_H

#include "Location.h"
#include <sstream>

class State {
    private:
        int time;
        Location location;

    public:
    State();

    State(int time, const Location &location);

        bool operator==(const State &);

        bool is_equal_except_time(State);

        int getTime() const;

        void setTime(int time);

        const Location &getLocation() const;

        void setLocation(const Location &location);

        friend std::ostream& operator<<(std::ostream & os, State state);



};

bool operator==(const State& s1, const State& s2);

bool operator<(const State& s1, const State& s2);


namespace std {
    template<>
    struct hash<State>
    {
        typedef State argument_type;
        typedef size_t result_type;

        size_t operator()(const State& state) const
        {
            std::stringstream ss;
            ss << state.getTime() << state.getLocation().getX() << state.getLocation().getY();
            std::string s = ss.str();
            return std::stol(s);
        }
    };
}


#endif //ABCBS_CPP_STATE_H
