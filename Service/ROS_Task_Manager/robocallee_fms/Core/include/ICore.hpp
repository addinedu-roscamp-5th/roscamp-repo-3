#pragma once

#include <string>
#include "Integrated.hpp"
#include "Commondefine.hpp"

namespace core {

class ICore {
public:
    ICore() = default;
    virtual ~ICore() = default;

    // AMR 다음 스텝 설정
    virtual bool SetAmrNextStep(int idx, Commondefine::AmrStep step) = 0;

    // 로봇팔 요청
    virtual bool SetRobotArmNextStep(Commondefine::RobotArmStep step,
                                     Commondefine::shoesproperty shoe_info,
                                     int pinky_num) = 0;

    // 팔 서비스 호출
    virtual bool ArmRequestMakeCall(int arm_num,
                                    int shelf_num,
                                    int pinky_num) = 0;

<<<<<<< Updated upstream:Service/ROS_Task_Manager/robocallee_fms/Core/include/ICore.hpp
        virtual int RequestCallback(const Commondefine::GUIRequest& request) = 0;
=======
    // 위치 콜백 (Aruco 또는 LM 위치)
    virtual bool PoseCallback(const Commondefine::pose2f &pos,
                              int pinky_id) = 0;

    // 요청 및 완료 콜백
    virtual bool RequestCallback(const Commondefine::GUIRequest &request) = 0;
    virtual bool DoneCallback(const std::string &requester,
                              const int &customer_id) = 0;

    // 상태 조회
    virtual Commondefine::RobotState GetAmrState(int idx) = 0;
    virtual int GetAmrBattery(int idx) = 0;
    virtual int GetAmrCustID(int idx) = 0;
    virtual int GetAmrVecSize() = 0;
>>>>>>> Stashed changes:robocallee_fms/Core/include/ICore.hpp

    // 작업 정보 저장
    virtual void SetTaskInfo(int idx,
                             const Commondefine::GUIRequest &request) = 0;
};

} // namespace core
