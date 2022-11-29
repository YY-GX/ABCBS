//
// Created by Yue Yang on 2020/11/12.
//

#ifndef ABCBS_CPP_LOCATION_H
#define ABCBS_CPP_LOCATION_H

#include <iostream>

class Location {
    private:
        int x;
        int y;

    public:
        Location(int x=-1, int y=-1);
        bool operator==(const Location &);

    friend std::ostream& operator<<(std::ostream & os, Location location);

    int getX() const;

    void setX(int x);

    int getY() const;

    void setY(int y);


};


#endif //ABCBS_CPP_LOCATION_H
