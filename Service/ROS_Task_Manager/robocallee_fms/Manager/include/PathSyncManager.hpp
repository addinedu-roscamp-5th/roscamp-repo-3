#pragma once

#include "Integrated.hpp"
#include "Commondefine.hpp"

#include "ICore.hpp"



namespace Manager
{
    class PathSyncManager
    {
    private:
        Integrated::w_ptr<core::ICore>                      icore_;
        Logger::s_ptr                                       log_;
        bool                                                requestNewPath_{false};
        int                                                 expected_{0};
        int                                                 remaining_{0};
        std::vector<bool>                                   arrived_in_window_;
        std::mutex                                          mtx_;
    
        void open_impl();
        
        void arrive_impl(int robot_id);
        
        void shrink_impl(int new_expected);
        
        void replan_and_close_impl();
    
    public:
        using u_ptr = Integrated::u_ptr<PathSyncManager>;

        PathSyncManager(Integrated::w_ptr<core::ICore> icore, int max_robot, Logger::s_ptr log);
        ~PathSyncManager();
        
        void OpenSyncWindow();
        
        void ArriveAtSyncOnce(int robot_id);
        
        void ShrinkExpectedAsync(int new_expected);

        bool checkSyncWindow();

        bool IsSyncOpen();

    };

};