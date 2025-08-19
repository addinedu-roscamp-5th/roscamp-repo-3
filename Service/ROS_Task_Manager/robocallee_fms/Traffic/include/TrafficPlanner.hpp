#pragma once
#include "Commondefine.hpp"
#include "Integrated.hpp"
#include <utility>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <cmath>
#include <memory>

namespace traffic
{
    // AStarPlanner 클래스 (개선된 버전)
    class AStarPlanner
    {
    private:
        struct Node
        {
            Commondefine::Position pos;
            int g_val;
            int h_val;
            int timestep;
            std::shared_ptr<Node> parent;
            
            int f_val() const { return g_val + h_val; }
            
            Node(Commondefine::Position p, int g, int h, int t, std::shared_ptr<Node> par = nullptr)
                : pos(p), g_val(g), h_val(h), timestep(t), parent(par) {}
        };
        
        std::vector<std::vector<bool>> map_;
        std::vector<Commondefine::Position> directions_;
        
        // 새로 추가된 private 함수들
        bool isValidPosition(const Commondefine::Position& pos, const std::vector<std::vector<int>>& grid) const;
        bool isValidPosition(const Commondefine::Position& pos, const std::vector<std::vector<bool>>& grid) const;
        std::vector<Commondefine::Position> reconstructPath(std::shared_ptr<Node> node) const;
        bool isConstrained(
            const Commondefine::Position& curr,
            const Commondefine::Position& next,
            int timestep,
            int agent,
            const std::vector<Commondefine::Constraint>& constraints
        ) const;

    public:
        AStarPlanner(const std::vector<std::vector<bool>>& map);
        
        // 개선된 findPath 함수 (타임스텝 제한 추가)
        std::vector<Commondefine::Position> findPath(
            const Commondefine::Position& start,
            const Commondefine::Position& goal,
            const std::vector<std::vector<int>>& heuristics,
            int agent_id,
            const std::vector<Commondefine::Constraint>& constraints,
            int max_timesteps = 1000
        );
    };

    // TrafficPlanner 클래스 (개선된 버전)
    class TrafficPlanner
    {
    private:
        std::vector<std::vector<bool>> map_;
        Log::Logger::s_ptr log_;
        
        // 설정 가능한 파라미터들 (새로 추가)
        static constexpr int MAX_CBS_ITERATIONS = 10000;
        static constexpr int MAX_ASTAR_TIMESTEPS = 1000;
        static constexpr int LOG_INTERVAL = 500;

        // 새로 추가된 private 함수들
        bool validateInput(const std::vector<Commondefine::Position>& starts, 
                          const std::vector<Commondefine::Position>& goals);
        bool isValidPosition(const Commondefine::Position& pos) const;
        
        std::vector<std::vector<Commondefine::Position>> runCBS(
            const std::vector<Commondefine::Position>& starts,
            const std::vector<Commondefine::Position>& goals,
            bool use_disjoint_splitting
        );
        
        Commondefine::CBSNode createRootNode(
            const std::vector<Commondefine::Position>& starts, 
            const std::vector<Commondefine::Position>& goals,
            AStarPlanner& planner
        );
        
        bool expandNode(
            const Commondefine::CBSNode& curr, 
            std::priority_queue<Commondefine::CBSNode, std::vector<Commondefine::CBSNode>, std::greater<>>& open,
            const std::vector<Commondefine::Position>& starts,
            const std::vector<Commondefine::Position>& goals,
            AStarPlanner& planner,
            bool use_disjoint_splitting
        );
        
        int calculateTotalCost(const std::vector<std::vector<Commondefine::Position>>& paths) const;
        
        // 기존 private 함수들
        std::vector<Commondefine::Conflict> detectCollisions(
            const std::vector<std::vector<Commondefine::Position>>& paths
        ) const;
        
        std::vector<Commondefine::Constraint> standardSplitting(
            const Commondefine::Conflict& conflict
        ) const;
        
        std::vector<Commondefine::Constraint> disjointSplitting(
            const Commondefine::Conflict& conflict
        ) const;
        
        std::vector<std::vector<int>> calculateManhattanHeuristics(
            const Commondefine::Position& goal
        ) const;

    public:
        // 생성자
        TrafficPlanner(const std::vector<std::vector<bool>>& map, Log::Logger::s_ptr log);
        
        // 메인 경로 계획 함수
        std::vector<std::vector<Commondefine::Position>> planPaths(
            const std::vector<Commondefine::Position>& starts,
            const std::vector<Commondefine::Position>& goals,
            bool use_disjoint_splitting = false
        );
        
        // 맵 관리 함수들
        void updateMap(const std::vector<std::vector<bool>>& new_map);
        const std::vector<std::vector<bool>>& getMap() const { return map_; }
        
        // 경로 유효성 검사
        bool validatePaths(const std::vector<std::vector<Commondefine::Position>>& paths) const;
        
        // 설정값 접근 함수들 (새로 추가)
        static constexpr int getMaxCBSIterations() { return MAX_CBS_ITERATIONS; }
        static constexpr int getMaxAStarTimesteps() { return MAX_ASTAR_TIMESTEPS; }
        static constexpr int getLogInterval() { return LOG_INTERVAL; }
    };
}