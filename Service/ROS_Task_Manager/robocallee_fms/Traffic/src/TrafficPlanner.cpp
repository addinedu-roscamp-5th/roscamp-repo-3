#include "TrafficPlanner.hpp"
using namespace Commondefine;
using namespace Integrated;
using namespace traffic;

// 개선된 AStarPlanner 구현
class ImprovedAStarPlanner {
private:
    struct Node {
        Position pos;
        int g_val;
        int h_val;
        int timestep;
        std::shared_ptr<Node> parent;
        
        int f_val() const { return g_val + h_val; }
        
        Node(Position p, int g, int h, int t, std::shared_ptr<Node> par = nullptr)
            : pos(p), g_val(g), h_val(h), timestep(t), parent(par) {}
    };
    
    std::vector<std::vector<bool>> map_;
    std::vector<Position> directions_;
    
public:
    ImprovedAStarPlanner(const std::vector<std::vector<bool>>& map) 
        : map_(map), directions_({{0,1},{1,0},{0,-1},{-1,0},{0,0}}) {}
    
    std::vector<Position> findPath(
        const Position& start,
        const Position& goal,
        const std::vector<std::vector<int>>& heuristics,
        int agent_id,
        const std::vector<Constraint>& constraints,
        int max_timesteps = 1000  // 타임스텝 제한 추가
    ) {
        // 입력 유효성 검사
        if (!isValidPosition(start, heuristics) || !isValidPosition(goal, heuristics)) {
            return {};
        }
        
        auto cmp = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { 
            return a->f_val() > b->f_val(); 
        };
        std::priority_queue<std::shared_ptr<Node>, 
                           std::vector<std::shared_ptr<Node>>, 
                           decltype(cmp)> open_list(cmp);
        
        std::unordered_set<std::string> visited;
        
        auto to_key = [](const Position& p, int t) {
            return std::to_string(p.x) + "," + std::to_string(p.y) + "," + std::to_string(t);
        };
        
        auto root = std::make_shared<Node>(start, 0, heuristics[start.y][start.x], 0);
        open_list.push(root);
        
        while (!open_list.empty()) {
            auto curr = open_list.top();
            open_list.pop();
            
            // 타임스텝 제한 검사
            if (curr->timestep > max_timesteps) {
                continue;
            }
            
            // 목표 도달 검사
            if (curr->pos == goal && !isConstrained(curr->pos, curr->pos, 
                                                   curr->timestep + 1, agent_id, constraints)) {
                return reconstructPath(curr);
            }
            
            std::string key = to_key(curr->pos, curr->timestep);
            if (visited.count(key)) continue;
            visited.insert(key);
            
            // 이웃 노드 탐색
            for (const auto& dir : directions_) {
                Position next = {curr->pos.x + dir.x, curr->pos.y + dir.y};
                
                if (!isValidPosition(next, map_) || map_[next.y][next.x]) continue;
                if (isConstrained(curr->pos, next, curr->timestep + 1, agent_id, constraints)) continue;
                
                int g = curr->g_val + (dir.x == 0 && dir.y == 0 ? 0 : 1);  // 대기 비용 0
                int h = heuristics[next.y][next.x];
                
                auto child = std::make_shared<Node>(next, g, h, curr->timestep + 1, curr);
                open_list.push(child);
            }
        }
        
        return {};  // 경로를 찾을 수 없음
    }

private:
    bool isValidPosition(const Position& pos, const std::vector<std::vector<int>>& grid) const {
        return pos.x >= 0 && pos.y >= 0 && 
               static_cast<size_t>(pos.y) < grid.size() && 
               static_cast<size_t>(pos.x) < grid[0].size();
    }
    
    bool isValidPosition(const Position& pos, const std::vector<std::vector<bool>>& grid) const {
        return pos.x >= 0 && pos.y >= 0 && 
               static_cast<size_t>(pos.y) < grid.size() && 
               static_cast<size_t>(pos.x) < grid[0].size();
    }
    
    std::vector<Position> reconstructPath(std::shared_ptr<Node> node) const {
        std::vector<Position> path;
        while (node) {
            path.push_back(node->pos);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    bool isConstrained(const Position& curr, const Position& next, int timestep, 
                      int agent, const std::vector<Constraint>& constraints) const {
        for (const auto& c : constraints) {
            if (c.agent != agent || c.timestep != timestep) continue;
            
            // Vertex constraint
            if (c.loc.size() == 1 && c.loc[0] == next) return true;
            
            // Edge constraint
            if (c.loc.size() == 2 && c.loc[0] == curr && c.loc[1] == next) return true;
        }
        return false;
    }
};

// 개선된 TrafficPlanner
class ImprovedTrafficPlanner {
private:
    std::vector<std::vector<bool>> map_;
    Log::Logger::s_ptr log_;
    
    // 설정 가능한 파라미터들
    static constexpr int MAX_CBS_ITERATIONS = 10000;
    static constexpr int MAX_ASTAR_TIMESTEPS = 1000;
    static constexpr int LOG_INTERVAL = 500;

public:
    ImprovedTrafficPlanner(const std::vector<std::vector<bool>>& map, Log::Logger::s_ptr log)
        : map_(map), log_(log) {
        log_->Log(INFO, "개선된 TrafficPlanner 초기화 완료. 맵 크기: " + 
                 std::to_string(map_.size()) + "x" + std::to_string(map_[0].size()));
    }
    
    std::vector<std::vector<Position>> planPaths(
        const std::vector<Position>& starts,
        const std::vector<Position>& goals,
        bool use_disjoint_splitting = false
    ) {
        // 입력 유효성 검사
        if (!validateInput(starts, goals)) {
            return {};
        }
        
        log_->Log(INFO, "CBS 경로 계획 시작 - " + std::to_string(starts.size()) + "개 에이전트");
        
        try {
            return runCBS(starts, goals, use_disjoint_splitting);
        } catch (const std::exception& e) {
            log_->Log(ERROR, "CBS 실행 중 예외 발생: " + std::string(e.what()));
            return {};
        }
    }

private:
    bool validateInput(const std::vector<Position>& starts, const std::vector<Position>& goals) {
        if (starts.size() != goals.size()) {
            log_->Log(ERROR, "시작점과 목표점의 개수가 일치하지 않습니다!");
            return false;
        }
        
        if (starts.empty()) {
            log_->Log(WARNING, "에이전트가 없습니다.");
            return false;
        }
        
        // 시작점과 목표점이 유효한 위치인지 확인
        for (size_t i = 0; i < starts.size(); ++i) {
            if (!isValidPosition(starts[i]) || !isValidPosition(goals[i])) {
                log_->Log(ERROR, "에이전트 " + std::to_string(i) + "의 시작점 또는 목표점이 유효하지 않습니다.");
                return false;
            }
            
            if (map_[starts[i].y][starts[i].x] || map_[goals[i].y][goals[i].x]) {
                log_->Log(ERROR, "에이전트 " + std::to_string(i) + "의 시작점 또는 목표점이 장애물입니다.");
                return false;
            }
        }
        
        return true;
    }
    
    bool isValidPosition(const Position& pos) const {
        return pos.x >= 0 && pos.y >= 0 && 
               static_cast<size_t>(pos.y) < map_.size() && 
               static_cast<size_t>(pos.x) < map_[0].size();
    }
    
    std::vector<std::vector<Position>> runCBS(
        const std::vector<Position>& starts,
        const std::vector<Position>& goals,
        bool use_disjoint_splitting
    ) {
        ImprovedAStarPlanner planner(map_);
        std::priority_queue<CBSNode, std::vector<CBSNode>, std::greater<>> open;
        
        // 루트 노드 생성 및 초기 경로 계산
        CBSNode root = createRootNode(starts, goals, planner);
        if (root.paths.empty()) {
            return {};  // 초기 경로 생성 실패
        }
        
        open.push(root);
        int iteration = 0;
        
        while (!open.empty() && iteration < MAX_CBS_ITERATIONS) {
            CBSNode curr = open.top();
            open.pop();
            iteration++;
            
            // 주기적 로그 출력
            if (iteration % LOG_INTERVAL == 0) {
                log_->Log(INFO, "CBS 반복 " + std::to_string(iteration) + 
                         ", 충돌 개수: " + std::to_string(curr.conflicts.size()) +
                         ", 현재 비용: " + std::to_string(curr.cost));
            }
            
            // 해결책 발견
            if (curr.conflicts.empty()) {
                log_->Log(INFO, "CBS 성공! " + std::to_string(iteration) + "번의 반복으로 해결");
                return curr.paths;
            }
            
            // 충돌 해결
            if (!expandNode(curr, open, starts, goals, planner, use_disjoint_splitting)) {
                log_->Log(WARNING, "노드 확장 실패");
            }
        }
        
        if (iteration >= MAX_CBS_ITERATIONS) {
            log_->Log(ERROR, "CBS 최대 반복 횟수(" + std::to_string(MAX_CBS_ITERATIONS) + ") 초과");
        } else {
            log_->Log(ERROR, "CBS 실패 - 해결책을 찾을 수 없습니다!");
        }
        
        return {};
    }
    
    CBSNode createRootNode(const std::vector<Position>& starts, 
                          const std::vector<Position>& goals,
                          ImprovedAStarPlanner& planner) {
        CBSNode root;
        root.cost = 0;
        root.id = 0;
        
        for (size_t i = 0; i < starts.size(); ++i) {
            try {
                auto heuristics = calculateManhattanHeuristics(goals[i]);
                std::vector<Constraint> empty;
                auto path = planner.findPath(starts[i], goals[i], heuristics, i, empty, MAX_ASTAR_TIMESTEPS);
                
                if (path.empty()) {
                    log_->Log(ERROR, "에이전트 " + std::to_string(i) + "의 초기 경로를 찾을 수 없습니다!");
                    return CBSNode{};  // 빈 노드 반환
                }
                
                root.paths.push_back(path);
                root.cost += static_cast<int>(path.size());
                
            } catch (const std::exception& e) {
                log_->Log(ERROR, "에이전트 " + std::to_string(i) + " 경로 계산 중 예외: " + e.what());
                return CBSNode{};
            }
        }
        
        root.conflicts = detectCollisions(root.paths);
        return root;
    }
    
    bool expandNode(const CBSNode& curr, 
                   std::priority_queue<CBSNode, std::vector<CBSNode>, std::greater<>>& open,
                   const std::vector<Position>& starts,
                   const std::vector<Position>& goals,
                   ImprovedAStarPlanner& planner,
                   bool use_disjoint_splitting) {
        
        if (curr.conflicts.empty()) return true;
        
        const Conflict& conflict = curr.conflicts[0];
        auto constraints = use_disjoint_splitting ? 
                          disjointSplitting(conflict) : 
                          standardSplitting(conflict);
        
        static int node_id = 1;
        bool success = false;
        
        for (const auto& constr : constraints) {
            try {
                CBSNode child = curr;
                child.id = node_id++;
                child.constraints.push_back(constr);
                
                // 제약된 에이전트의 경로 재계산
                auto heuristics = calculateManhattanHeuristics(goals[constr.agent]);
                auto path = planner.findPath(
                    starts[constr.agent],
                    goals[constr.agent], 
                    heuristics,
                    constr.agent,
                    child.constraints,
                    MAX_ASTAR_TIMESTEPS
                );
                
                if (path.empty()) {
                    continue;  // 이 제약조건으로는 경로를 찾을 수 없음
                }
                
                // 경로 업데이트 및 비용 재계산
                child.paths[constr.agent] = path;
                child.cost = calculateTotalCost(child.paths);
                child.conflicts = detectCollisions(child.paths);
                
                // 유효한 자식 노드 생성됨
                open.push(child);
                success = true;
                
            } catch (const std::exception& e) {
                log_->Log(WARNING, "자식 노드 생성 중 예외: " + std::string(e.what()));
                continue;
            }
        }
        
        return success;
    }
    
    int calculateTotalCost(const std::vector<std::vector<Position>>& paths) const {
        int total = 0;
        for (const auto& path : paths) {
            total += static_cast<int>(path.size());
        }
        return total;
    }
    
    std::vector<Conflict> detectCollisions(const std::vector<std::vector<Position>>& paths) const {
        std::vector<Conflict> conflicts;
        if (paths.empty()) return conflicts;
        
        // 최대 타임스텝 계산
        size_t max_t = 0;
        for (const auto& path : paths) {
            max_t = std::max(max_t, path.size());
        }
        
        // 각 타임스텝에서 충돌 검사
        for (size_t t = 0; t < max_t; ++t) {
            for (size_t i = 0; i < paths.size(); ++i) {
                Position pi = (t < paths[i].size()) ? paths[i][t] : paths[i].back();
                
                for (size_t j = i + 1; j < paths.size(); ++j) {
                    Position pj = (t < paths[j].size()) ? paths[j][t] : paths[j].back();
                    
                    // Vertex conflict (같은 위치)
                    if (pi == pj) {
                        conflicts.push_back(Conflict{
                            static_cast<int>(i), static_cast<int>(j),
                            static_cast<int>(t), {pi}
                        });
                    }
                    
                    // Edge conflict (서로 교차)
                    if (t > 0) {
                        Position pi_prev = (t-1 < paths[i].size()) ? paths[i][t-1] : paths[i].back();
                        Position pj_prev = (t-1 < paths[j].size()) ? paths[j][t-1] : paths[j].back();
                        
                        if (pi_prev == pj && pj_prev == pi) {
                            conflicts.push_back(Conflict{
                                static_cast<int>(i), static_cast<int>(j),
                                static_cast<int>(t), {pi_prev, pi}
                            });
                        }
                    }
                }
            }
        }
        
        return conflicts;
    }
    
    std::vector<Constraint> standardSplitting(const Conflict& conflict) const {
        std::vector<Constraint> constraints;
        
        if (conflict.loc.size() == 1) {
            // Vertex conflict
            constraints.push_back(Constraint{conflict.agent1, conflict.timestep, conflict.loc});
            constraints.push_back(Constraint{conflict.agent2, conflict.timestep, conflict.loc});
        } else if (conflict.loc.size() == 2) {
            // Edge conflict
            constraints.push_back(Constraint{conflict.agent1, conflict.timestep, 
                                           {conflict.loc[0], conflict.loc[1]}});
            constraints.push_back(Constraint{conflict.agent2, conflict.timestep,
                                           {conflict.loc[1], conflict.loc[0]}});
        }
        
        return constraints;
    }
    
    std::vector<Constraint> disjointSplitting(const Conflict& conflict) const {
        // 향후 더 정교한 disjoint splitting 구현 가능
        return standardSplitting(conflict);
    }
    
    std::vector<std::vector<int>> calculateManhattanHeuristics(const Position& goal) const {
        std::vector<std::vector<int>> heuristics(map_.size(), 
                                                std::vector<int>(map_[0].size(), 0));
        
        for (size_t row = 0; row < map_.size(); ++row) {
            for (size_t col = 0; col < map_[0].size(); ++col) {
                heuristics[row][col] = std::abs(goal.y - static_cast<int>(row)) + 
                                     std::abs(goal.x - static_cast<int>(col));
            }
        }
        
        return heuristics;
    }

public:
    // 공개 인터페이스
    void updateMap(const std::vector<std::vector<bool>>& new_map) {
        if (new_map.empty() || new_map[0].empty()) {
            log_->Log(ERROR, "유효하지 않은 맵 데이터");
            return;
        }
        
        map_ = new_map;
        log_->Log(INFO, "맵 업데이트 완료: " + std::to_string(map_.size()) + "x" + 
                 std::to_string(map_[0].size()));
    }
    
    const std::vector<std::vector<bool>>& getMap() const { return map_; }
    
    bool validatePaths(const std::vector<std::vector<Position>>& paths) const {
        auto conflicts = detectCollisions(paths);
        if (!conflicts.empty()) {
            log_->Log(WARNING, "경로 검증 실패: " + std::to_string(conflicts.size()) + "개의 충돌 발견");
            return false;
        }
        return true;
    }
};