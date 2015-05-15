#ifndef BFS_HPP
#define BFS_HPP

#include "Maze.h"
#include "BitArray2D.h"

#define MAX_NODES (16*16)


struct Node
{
    int i;
    int j;

    bool operator==(const Node& other) const
    {
        return i == other.i && j == other.j;
    }
    bool operator!=(const Node& other) const
    {
        return !(*this == other);
    }
};

    
struct Edge
{
    Node a;
    Node b;
};


class EdgeStack
{
public:
    void push(Edge edge)
    {
        data[head++] = ((edge.a.i & 0xf) << 0 |
                        (edge.a.j & 0xf) << 4 |
                        (edge.b.i & 0xf) << 8 |
                        (edge.b.j & 0xf) << 12);
    }
    Edge pop()
    {
        auto e = data[--head];
        return {{(e >> 0) & 0xf, (e >>  4) & 0xf},
            {(e >> 8) & 0xf, (e >> 12) & 0xf}};
    }
    bool empty() { return head <= 0; }
    
private:
    unsigned short data[MAX_NODES];
    int head = 0;
};

    
class NodeStack
{
public:
    void push(Node node)
    {
        data[head++] = ((node.i & 0xf) << 0 | (node.j & 0xf) << 4);
    }
    Node pop()
    {
        auto n = data[--head];
        return {(n >> 0) & 0xf, (n >> 4) & 0xf};
    }
    Node peek() const
    {
        auto n = data[head - 1];
        return {(n >> 0) & 0xf, (n >> 4) & 0xf};
    }
    void clear() { head = 0; }
    bool empty() { return head <= 0; }
    Node operator[](int i) const
    {
        auto n = data[head - 1 - i];
        return {(n >> 0) & 0xf, (n >> 4) & 0xf};
    }
    int size() { return head; }
    
private:
    unsigned char data[MAX_NODES];
    int head = 0;
};


class NodeQueue
{
public:
    void push(Node node)
    {
        data[tail] = ((node.i & 0xf) << 0 | (node.j & 0xf) << 4);
        tail = (tail + 1) % MAX_NODES;
    }
    Node pop()
    {
        auto n = data[head];
        head = (head + 1) % MAX_NODES;
        return {(n >> 0) & 0xf, (n >> 4) & 0xf};
    }
    bool empty() { return head == tail; }

private:
    unsigned char data[MAX_NODES];
    int head = 0;
    int tail = 0;
};


bool bfs(const Maze<16, 16>& maze,
         Node start,
         Node goal,
         NodeStack& bfsPath);

bool bfs(const Maze<16, 16>& maze,
         Node start,
         const BitArray2D<16, 16>& goals,
         NodeStack& bfsPath);

#endif // BFS_HPP
