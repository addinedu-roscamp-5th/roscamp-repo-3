#pragma once

#include <string>
#include "Integrated.hpp"
#include "Commondefine.hpp"

namespace core {

class ICore {
public:
    ICore() = default;
    virtual ~ICore() = default;

    virtual bool assignTask(int idx, Commondefine::AmrStep step) = 0;

    virtual bool assignTask(Commondefine::RobotArmStep step) = 0;

    virtual bool assignTask(Integrated::Task task) = 0;

    // 팔 서비스 호출
    virtual bool ArmRequestMakeCall(Commondefine::RobotArm arm, int shelf_num, int robot_id, std::string action) = 0;

    // 위치 콜백 (Aruco 또는 LM 위치)
    virtual bool PoseCallback(const std::vector<Commondefine::pose2f> &pos) =0;

    // 요청 및 완료 콜백
    virtual int RequestCallback(const Commondefine::GUIRequest &request) = 0;

    // WayPoint를 를 실제 통신으로 내보내주는 Code
    virtual bool publishNavGoal(int idx, const Commondefine::Position wp) = 0;

    virtual bool DoneCallback(const std::string &requester, const int &customer_id) = 0;

    virtual bool ArmDoneCallback(Commondefine::ArmRequest request) = 0;

    //--------------------------AMR STATE--------------------------
    virtual Commondefine::RobotState GetAmrState(int idx) = 0;

    virtual void UpdateBattery(int idx, float percent) = 0;

    virtual int GetAmrBattery(int idx) = 0;

    virtual int GetAmrCustID(int idx) = 0;

    virtual int GetAmrVecSize() = 0;

    // 작업 정보 저장
    virtual void SetTaskInfo(int idx, const Commondefine::GUIRequest &request) = 0;

    virtual bool waitNewPath(std::chrono::milliseconds ms) = 0;

    virtual void assignPlanPaths() = 0;

    //Request 에서 일을 할당 할때 호출
    virtual void assignWork(int amr, Commondefine::GUIRequest r) = 0;

    virtual void SetRequestNewPath(bool Request) = 0;

    virtual bool GetRequestNewPath() = 0;

    virtual void assignBestRobotSelector() = 0;

    //--------------------------StorageManager--------------------------
    virtual bool setStorageRequest(Commondefine::StorageRequest& Request) = 0;
    
    virtual bool findStorage(Commondefine::ContainerType Container , Commondefine::StorageRequest& Request) = 0;

    virtual bool setStorage(Commondefine::ContainerType Container , Commondefine::StorageRequest Request) = 0;

    virtual bool getStorage(Commondefine::ContainerType Container , Commondefine::StorageRequest& Request) = 0;

    virtual int findEmptyStorage(Commondefine::ContainerType Container) = 0;

    virtual bool waitCriticalSection(std::chrono::milliseconds ms) = 0;

    //--------------------------PathSyncManager--------------------------

    virtual int CurrentActiveRobotCount() = 0;

    virtual void ReplanAndBroadcast() = 0;

    virtual bool IsSyncOpen() = 0;

    virtual void ArriveAtSyncOnce(int robot_id) = 0;

    virtual void OpenSyncWindow() = 0;

};

} // namespace core
