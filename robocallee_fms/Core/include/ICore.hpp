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

        virtual bool SetAmrNextStep(Commondefine::AmrStep step) = 0;

        virtual bool SetRobotArmNextStep(Commondefine::RobotArmStep step) = 0;
        
        virtual bool RequestCallback(const Commondefine::GUIRequest& request) = 0;

        virtual bool DoneCallback(const std::string& requester) = 0;

        virtual Commondefine::RobotState GetAmrState(int index) const = 0;

        virtual void SetAmrState(int index, const Commondefine::RobotState& state) = 0;

        virtual int GetAmrVecSize() = 0;
    };

};

