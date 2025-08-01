#pragma once
#include <vector>
#include <utility>
#include <queue>
#include <unordered_map>
#include <string>

class CBSSolver {
public:
    CBSSolver(const std::vector<std::vector<bool>>& map,
              const std::vector<Position>& starts,
              const std::vector<Position>& goals);

    std::vector<std::vector<Position>> findSolution(bool disjoint);

private:
    std::vector<std::vector<bool>> map_;
    std::vector<Position> starts_;
    std::vector<Position> goals_;
    std::vector<std::vector<std::vector<int>>> heuristics_;

    std::vector<std::vector<int>> computeHeuristic(const Position& goal);
    std::vector<Conflict> detectCollisions(const std::vector<std::vector<Position>>& paths) const;
    Conflict detectCollision(const std::vector<Position>& path1,
                             const std::vector<Position>& path2);

    std::vector<Constraint> standardSplitting(const Conflict& conflict) const;
    std::vector<Constraint> disjointSplitting(const Conflict& conflict) const;
    std::vector<int> pathsViolateConstraint(const Constraint& c,
                                            const std::vector<std::vector<Position>>& paths);
};


class AStarPlanner {
public:
    AStarPlanner(const std::vector<std::vector<bool>>& map); // 선언만!

    bool isConstrained(
        const Position& curr,
        const Position& next,
        int timestep,
        int agent,
        const std::vector<Constraint>& constraints
    ) const;

    // heuristics: 각 위치에 대한 휴리스틱 맵 [y][x]
    // agent_id: 이 에이전트의 번호
    // constraints: (다중 에이전트 MAPF용) 제약조건
    std::vector<Position> findPath(
        const Position& start,
        const Position& goal,
        const std::vector<std::vector<int>>& heuristics,
        int agent_id,
        const std::vector<Constraint>& constraints
    );
    
private:
    std::vector<std::vector<bool>> map_;
    std::vector<Position> directions_;
};



using Position = std::pair<int, int>;

struct Constraint {
    int agent;
    int timestep;
    std::vector<Position> loc;  // size=1이면 vertex constraint, size=2이면 edge constraint
};

struct Conflict {
    int agent1, agent2;
    int timestep;
    std::vector<Position> loc;  // 충돌 위치 (vertex 또는 edge)
};

struct Node {
    Position pos;
    int g_val;
    int h_val;
    int timestep;
    Node* parent;

    int f_val() const { return g_val + h_val; }
};
