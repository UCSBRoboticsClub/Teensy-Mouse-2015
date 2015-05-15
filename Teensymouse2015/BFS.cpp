#include "BFS.h"
#include "BitArray2D.h"
#include <cstdlib>


void permute(int* order);


bool bfs(const Maze<16, 16>& maze,
         Node start,
         Node goal,
         NodeStack& bfsPath)
{
    BitArray2D<16, 16> goals;
    goals.set(goal.i, goal.j, true);
    return bfs(maze, start, goals, bfsPath);
}


bool bfs(const Maze<16, 16>& maze,
         Node start,
         const BitArray2D<16, 16>& goals,
         NodeStack& bfsPath)  
{
    bfsPath.clear();

    if (goals.get(start.i, start.j))
    {
        bfsPath.push(start);
        return true;
    }
    
    NodeQueue q;
    BitArray2D<16, 16> nodeMarks;
    EdgeStack edges;
    
    q.push(start);
    
    while (!q.empty())
    {
        auto v = q.pop();
        auto cw = maze.getCellWalls(v.i, v.j);

        int order[4] = {0, 1, 2, 3};
        //permute(order);
        
        for (int i = 0; i < 4; ++i)
        {
            int wall = order[i];
            
            if (cw[wall])
                continue;

            auto u = v;
            if (0 == wall)
                ++u.i;
            else if (1 == wall)
                ++u.j;
            else if (2 == wall)
                --u.i;
            else if (3 == wall)
                --u.j;

            if (!nodeMarks.get(u.i, u.j))
            {
                nodeMarks.set(u.i, u.j, true);
                edges.push({v, u});
                q.push(u);
            }

            if (goals.get(u.i, u.j))
            {
                auto e = edges.pop();
                bfsPath.push(e.b);
                bfsPath.push(e.a);
                
                while (!edges.empty())
                {
                    e = edges.pop();
                    if (e.b == bfsPath.peek())
                        bfsPath.push(e.a);
                }

                return true;
            }      
        }
    }

    return false;
}


void permute(int* order)
{
    order[0] = std::rand() % 4;
        
    do
        order[1] = std::rand() % 4;
    while (order[1] == order[0]);
        
    do
        order[2] = std::rand() % 4;
    while (order[2] == order[0] || order[2] == order[1]);

    order[3] = (0 + 1 + 2 + 3) - order[0] - order[1] - order[2];
}
