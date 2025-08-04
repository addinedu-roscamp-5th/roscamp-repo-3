#pragma once

#include "Integrated.hpp"
#include "Commondefine.hpp"

namespace core
{
    class ICore
    {
    private:

    public:
        ICore() = default; 
        virtual ~ICore() = default;

        virtual bool SetAmrNextStep(int idx, Commondefine::AmrStep step) = 0;

        // virtual bool SetRobotArmNextStep(Commondefine::RobotArmStep step) = 0;
        virtual bool SetRobotArmNextStep(Commondefine::RobotArmStep step , Commondefine::shoesproperty shoe_info , int pinky_num ) = 0 ;

        virtual bool ArmRequestMakeCall(int arm_num, int shelf_num, int pinky_num) = 0;

        virtual int RequestCallback(const Commondefine::GUIRequest& request) = 0;

        virtual bool DoneCallback(const std::string& requester, const int& customer_id) = 0;

        virtual Commondefine::RobotState GetAmrState(int idx) = 0;

        virtual int GetAmrBattery(int idx) = 0;

        virtual int GetAmrCustID(int idx) = 0;
        
        virtual int GetAmrVecSize() = 0;

        virtual void SetTaskInfo(int idx, const Commondefine::GUIRequest& request) = 0;

        // virtual void SetAmrState(int idx) = 0;

    };
};

