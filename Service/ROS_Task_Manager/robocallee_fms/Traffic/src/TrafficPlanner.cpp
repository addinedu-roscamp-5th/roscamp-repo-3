#include "TrafficPlanner.hpp"
using namespace Commondefine;
using namespace Integrated;
using namespace traffic;
// AStarPlanner 구현
AStarPlanner::AStarPlanner(const std::vector<std::vector<bool>>& map) : map_(map)
{
    directions_ = {{0,1},{1,0},{0,-1},{-1,0},{0,0}};
}
bool AStarPlanner::isConstrained(
    const Position& curr,
    const Position& next,
    int timestep,
    int agent,
    const std::vector<Constraint>& constraints
) const
{
    for (const auto& c : constraints)
    {
        if (c.agent != agent) continue;
        if (c.timestep != timestep) continue;
        if (c.loc.size() == 1 && c.loc[0] == next) return true;
        if (c.loc.size() == 2 && c.loc[0] == curr && c.loc[1] == next) return true;
    }
    return false;
}
std::vector<Position> AStarPlanner::findPath(
    const Position& start,
    const Position& goal,
    const std::vector<std::vector<int>>& heuristics,
    int agent_id,
    const std::vector<Constraint>& constraints
)

{   
    // 경계 검사 추가
    if (start.x < 0 || start.y < 0 || 
        static_cast<size_t>(start.y) >= heuristics.size() ||      // start.y가 행 인덱스
        static_cast<size_t>(start.x) >= heuristics[0].size()) {   // start.x가 열 인덱스

        printf("[ERROR] 잘못된 시작점: (%d, %d), 휴리스틱 맵 크기: %zux%zu\n", 
        start.x, start.y, heuristics.size(), 
        heuristics.empty() ? 0 : heuristics[0].size());
        return {}; // 잘못된 시작점
    }

    auto cmp = [](Node* a, Node* b) { return a->f_val() > b->f_val(); };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_list(cmp);
    std::unordered_map<std::string, Node*> visited;
    auto to_key = [](const Position& p, int t)
    {
        return std::to_string(p.x) + "," + std::to_string(p.y) + "," + std::to_string(t);
    };
    Node* root = new Node{start, 0, heuristics[start.y][start.x], 0, nullptr};

    open_list.push(root);
    while (!open_list.empty())
    {
        Node* curr = open_list.top();
        open_list.pop();
        if (curr->pos == goal && !isConstrained(curr->pos, curr->pos, curr->timestep + 1, agent_id, constraints))
        {
            std::vector<Position> path;
            while (curr)
            {
                path.push_back(curr->pos);
                curr = curr->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        std::string key = to_key(curr->pos, curr->timestep);
        if (visited.count(key)) continue;
        visited[key] = curr;
        for (const auto& dir : directions_)
        {
            Position next = {curr->pos.x + dir.x, curr->pos.y + dir.y};
            if (next.x < 0 || next.y < 0 ||
                static_cast<size_t>(next.y) >= map_.size() ||      // next.y가 행 인덱스
                static_cast<size_t>(next.x) >= map_[0].size()) continue;  // next.x가 열 인덱스
            if (map_[next.y][next.x]) continue;  // [행][열] = [y][x]
            if (isConstrained(curr->pos, next, curr->timestep + 1, agent_id, constraints)) continue;
            int g = curr->g_val + 1;
            int h = heuristics[next.y][next.x];  // [행][열] = [y][x]
            Node* child = new Node{next, g, h, curr->timestep + 1, curr};
            open_list.push(child);
        }
    }
    return {};
}
// TrafficPlanner 구현
TrafficPlanner::TrafficPlanner(const std::vector<std::vector<bool>>& map, Log::Logger::s_ptr log)
    : map_(map), log_(log)
{
    std::string msg = "TrafficPlanner 초기화 완료. 맵 크기: " + std::to_string(map_.size()) + "X : " + std::to_string(map_[0].size());
    log_->Log(INFO,msg);
}
std::vector<std::vector<Position>> TrafficPlanner::planPaths(
    const std::vector<Position>& starts,
    const std::vector<Position>& goals,
    bool use_disjoint_splitting
) {
    if (starts.size() != goals.size())
    {
        log_->Log(ERROR, "시작점과 목표점의 개수가 일치하지 않습니다!");
        return {};
    }
    std::string msg = "CBS 경로 계획 시작 - " + std::to_string( starts.size()) + "개 에이전트";
    log_->Log(INFO,msg);
    AStarPlanner planner(map_);
    std::priority_queue<CBSNode, std::vector<CBSNode>, std::greater<>> open;
    CBSNode root;
    root.cost = 0;
    root.id = 0;
    // 초기 경로 계산
    for (size_t i = 0; i < starts.size(); ++i)
    {
        std::vector<Constraint> empty;
        auto heuristics = calculateManhattanHeuristics(goals[i]);
        auto path = planner.findPath(starts[i], goals[i], heuristics, i, empty);
        if (path.empty())
        {
            std::string msg = "에이전트 " + std::to_string(i) + "의 초기 경로를 찾을 수 없습니다!";
            log_->Log(ERROR,msg);
            return {};
        }
        root.paths.push_back(path);
        root.cost += path.size();
    }
    root.conflicts = detectCollisions(root.paths);
    open.push(root);
    int node_id = 1;
    int iteration = 0;
    while (!open.empty())
    {
        CBSNode curr = open.top();
        open.pop();
        iteration++;
        if (iteration % 100 == 0)
        {
            std::string msg = "CBS 반복 "  + std::to_string(iteration) + ", 충돌 개수: " + std::to_string(curr.conflicts.size());
            log_->Log(ERROR,msg);
        }
        if (curr.conflicts.empty())
        {
            std::string msg = "CBS 성공! "  + std::to_string(iteration) + "번의 반복으로 해결";
            log_->Log(INFO,msg);
            return curr.paths;
        }
        Conflict conflict = curr.conflicts[0];
        auto constraints = use_disjoint_splitting ?
                          disjointSplitting(conflict) :
                          standardSplitting(conflict);
        for (const auto& constr : constraints)
        {
            CBSNode child = curr;
            child.id = node_id++;
            child.constraints.push_back(constr);
            auto heuristics = calculateManhattanHeuristics(goals[constr.agent]);
            auto path = planner.findPath(
                starts[constr.agent],
                goals[constr.agent],
                heuristics,
                constr.agent,
                child.constraints
            );
            if (path.empty()) continue;
            child.paths[constr.agent] = path;
            child.cost = 0;
            for (const auto& p : child.paths) child.cost += p.size();
            child.conflicts = detectCollisions(child.paths);
            open.push(child);
        }
    }
    log_->Log(ERROR,"CBS 실패 - 해결책을 찾을 수 없습니다!");
    return {};
}
void TrafficPlanner::updateMap(const std::vector<std::vector<bool>>& new_map)
{
    map_ = new_map;
    log_->Log(INFO,"맵 업데이트 완료");
}
bool TrafficPlanner::validatePaths(const std::vector<std::vector<Position>>& paths) const
{
    // 경로 유효성 검사 로직
    auto conflicts = detectCollisions(paths);
    return conflicts.empty();
}
std::vector<Conflict> TrafficPlanner::detectCollisions(const std::vector<std::vector<Position>>& paths) const
{
    std::vector<Conflict> conflicts;
    size_t max_t = 0;
    for (const auto& p : paths)
    {
        if (p.size() > max_t) max_t = p.size();
    }
    for (size_t t = 0; t < max_t; ++t)
    {
        for (size_t i = 0; i < paths.size(); ++i)
        {
            Position pi = (t < paths[i].size()) ? paths[i][t] : paths[i].back();
            for (size_t j = i + 1; j < paths.size(); ++j)
            {
                Position pj = (t < paths[j].size()) ? paths[j][t] : paths[j].back();
                // Vertex conflict
                if (pi == pj)
                {
                    conflicts.push_back(Conflict{static_cast<int>(i), static_cast<int>(j),
                                               static_cast<int>(t), {pi}});
                }
                // Edge conflict
                if (t > 0)
                {
                    Position pi_prev = (t-1 < paths[i].size()) ? paths[i][t-1] : paths[i].back();
                    Position pj_prev = (t-1 < paths[j].size()) ? paths[j][t-1] : paths[j].back();
                    if (pi_prev == pj && pj_prev == pi) {
                        conflicts.push_back(Conflict{static_cast<int>(i), static_cast<int>(j),
                                                   static_cast<int>(t), {pi_prev, pi}});
                    }
                }
            }
        }
    }
    return conflicts;
}
std::vector<Constraint> TrafficPlanner::standardSplitting(const Conflict& conflict) const
{
    std::vector<Constraint> constraints;
    // Vertex conflict: loc.size() == 1
    if (conflict.loc.size() == 1) {
        constraints.push_back(Constraint{conflict.agent1, conflict.timestep, conflict.loc});
        constraints.push_back(Constraint{conflict.agent2, conflict.timestep, conflict.loc});
    }
    // Edge conflict: loc.size() == 2
    else if (conflict.loc.size() == 2) {
        constraints.push_back(Constraint{conflict.agent1, conflict.timestep,
                                       {conflict.loc[0], conflict.loc[1]}});
        constraints.push_back(Constraint{conflict.agent2, conflict.timestep,
                                       {conflict.loc[1], conflict.loc[0]}});
    }
    return constraints;
}
std::vector<Constraint> TrafficPlanner::disjointSplitting(const Conflict& conflict) const
{
    // 간단하게 표준 분할과 동일하게 구현 (실제로는 더 복잡한 로직 가능)
    return standardSplitting(conflict);
}
std::vector<std::vector<int>> TrafficPlanner::calculateManhattanHeuristics(const Position& goal) const
{
    std::vector<std::vector<int>> heuristics(map_.size(), std::vector<int>(map_[0].size(), 0));
    for (size_t row = 0; row < map_.size(); ++row) {        // 행 순회 (y좌표)
        for (size_t col = 0; col < map_[0].size(); ++col) { // 열 순회 (x좌표)  
            heuristics[row][col] = std::abs(goal.y - static_cast<int>(row))   // goal.y와 row 비교
                                 + std::abs(goal.x - static_cast<int>(col));  // goal.x와 col 비교
        }
    }
    return heuristics;
}