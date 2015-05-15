#ifndef MAZE_HPP
#define MAZE_HPP

#undef min
#undef max
#include <array>
#include <cstdlib>
#include "BitArray2D.h"


/* The get/setCellsWalls functions take and return arrays of booleans
 * representing the state of each wall around a cell.
 *     cw[0] is the wall in the +i direction
 *     cw[1] is the wall in the +j direction
 *     cw[2] is the wall in the -i direction
 *     cw[3] is the wall in the -j direction
 * The borders of the maze are assumed to always have walls. Trying to set a
 * wall along the border to false will fail, but other walls set in the same
 * operation will succeed and the function will return false to signal the
 * invalid wall setting.
 *
 * The maze is drawn using matrix convention for indices and directions:
 * +-------+-------+---> j
 * | (0,0) | (0,1) |
 * +-------+-------+
 * | (1,0) | (1,1) |
 * +-------+-------+
 * |
 * v
 * i
 */


template<int m, int n>
class Maze
{
public:
    Maze();

    std::array<bool, 4> getCellWalls(int i, int j) const;
    bool setCellWalls(int i, int j, std::array<bool, 4> cw);
    
    void fill();
    void clear();
    void randomize();

//private:
    BitArray2D<m - 1, n> mWalls;
    BitArray2D<m, n - 1> nWalls;
};


template<int m, int n>
Maze<m, n>::Maze()
{
    clear();
}


template<int m, int n>
std::array<bool, 4> Maze<m, n>::getCellWalls(int i, int j) const
{
    if (i < 0 || j < 0 || i >= m || j >= n)
        return {false, false, false, false};

    std::array<bool, 4> cw = {
        (m - 1 == i) || mWalls.get(i, j),
        (n - 1 == j) || nWalls.get(i, j),
        (0 == i) || mWalls.get(i - 1, j),
        (0 == j) || nWalls.get(i, j - 1) };

    return cw;
}


template<int m, int n>
bool Maze<m, n>::setCellWalls(int i, int j, std::array<bool, 4> cw)
{
    if (i < 0 || j < 0 || i >= m || j >= n)
        return false;

    bool error = ((m - 1 == i && !cw[0]) ||
                  (n - 1 == j && !cw[1]) ||
                  (0 == i && !cw[2]) ||
                  (0 == j && !cw[3]));

    if (i < m - 1)
        mWalls.set(i, j, cw[0]);

    if (j < n - 1)
        nWalls.set(i, j, cw[1]);

    if (i > 0)
        mWalls.set(i - 1, j, cw[2]);

    if (j > 0)
        nWalls.set(i, j - 1, cw[3]);

    // Returns false if the caller tried to set the boundary walls to false
    return !error;
}


template<int m, int n>
void Maze<m, n>::clear()
{
    mWalls.setAll(false);
    nWalls.setAll(false);
}


template<int m, int n>
void Maze<m, n>::fill()
{
    mWalls.setAll(true);
    nWalls.setAll(true);
}


template<int m, int n>
void Maze<m, n>::randomize()
{
    for (int i = 0; i < mWalls.size(); ++i)
        mWalls[i] = std::rand() % 256;
    
    for (int i = 0; i < nWalls.size(); ++i)
        nWalls[i] = std::rand() % 256;
}


#endif // MAZE_HPP
