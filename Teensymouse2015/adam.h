#ifndef ADAM_H
#define ADAM_H

struct Gps
{
    Node current;
    Direction heading;
};

enum movement
{
    forward,
    backward,
    left,
    right
};

movement pos2move(Gps position, Node target)
{
    i=target.i-position.current.i;
    j=target.j-position.current.j;
    
    switch(position.heading)
    {
        case Direction::jpos
            break;
        case Direction::jneg
            
    

Gps drive2(Gps position, Node target)
{
    
    
    
    