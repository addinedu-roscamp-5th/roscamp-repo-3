#pragma once

#include "Commondefine.hpp"
#include "Integrated.hpp"

#include <utility>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <cmath>

// CBS 노드 구조체

namespace traffic
{
    class AStarPlanner
    {
    public:
        AStarPlanner(const std::vector<std::vector<bool>>& map);

        std::vector<Commondefine::Position> findPath(
            const Commondefine::Position& start,
            const Commondefine::Position& goal,
            const std::vector<std::vector<int>>& heuristics,
            int agent_id,
            const std::vector<Commondefine::Constraint>& constraints
        );

    private:
        std::vector<std::vector<bool>> map_;
        std::vector<Commondefine::Position> directions_;

        bool isConstrained(
            const Commondefine::Position& curr,
            const Commondefine::Position& next,
            int timestep,
            int agent,
            const std::vector<Commondefine::Constraint>& constraints
        ) const;
    };

    class TrafficPlanner
    {
    public:
        TrafficPlanner(const std::vector<std::vector<bool>>& map, Log::Logger::s_ptr log);
        
        // 메인 경로 계획 함수
        std::vector<std::vector<Commondefine::Position>> planPaths(
            const std::vector<Commondefine::Position>& starts,
            const std::vector<Commondefine::Position>& goals,
            bool use_disjoint_splitting = true
        );
        
        // 맵 업데이트
        void updateMap(const std::vector<std::vector<bool>>& new_map);
        
        // 맵 정보 가져오기
        const std::vector<std::vector<bool>>& getMap() const { return map_; }
        
        // 경로 유효성 검사
        bool validatePaths(const std::vector<std::vector<Commondefine::Position>>& paths) const;

    private:
        std::vector<std::vector<bool>>      map_;
        Logger::s_ptr                       log_;
        
        // CBS 내부 구현
        std::vector<Commondefine::Conflict> detectCollisions(const std::vector<std::vector<Commondefine::Position>>& paths) const;
        std::vector<Commondefine::Constraint> standardSplitting(const Commondefine::Conflict& conflict) const;
        std::vector<Commondefine::Constraint> disjointSplitting(const Commondefine::Conflict& conflict) const;
        
        // 휴리스틱 계산
        std::vector<std::vector<int>> calculateManhattanHeuristics(const Commondefine::Position& goal) const;
    };
};
