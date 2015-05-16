#ifndef ADAM_H
#define ADAM_H

#include "BFS.h"


class Direction
{
public:
    static const Direction ipos;
    static const Direction jpos;
    static const Direction ineg;
    static const Direction jneg;
    
    Direction& operator++();
    Direction& operator--();
    Direction operator++(int);
    Direction operator--(int);
    bool operator==(const Direction& other) const;
    bool operator!=(const Direction& other) const;

private:
    Direction(int dir) : dir(dir) {}
    int dir;
};

struct Gps
{
    Node current;
    Direction heading;
};

Gps drive2cell(Gps position, Node target);

void updateMaze(Gps);

#endif
