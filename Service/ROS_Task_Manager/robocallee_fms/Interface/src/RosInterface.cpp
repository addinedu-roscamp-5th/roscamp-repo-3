#include "RosInterface.hpp"

using namespace interface;
using namespace Integrated;

RosInterface::RosInterface(Logger::s_ptr log)
: Node(_ROS_NODE_NAME_), log_(log)
{

}

RosInterface::~RosInterface(){}

bool RosInterface::Initialize(Integrated::w_ptr<core::ICore> Icore)
{
    Icore_ = Icore;

    req_service_ = create_service<ReqServiceType>("request_service", std::bind(&RosInterface::cbRequestService, this, std::placeholders::_1, std::placeholders::_2));
    done_service_ = create_service<DoneServiceType>("done_service", std::bind(&RosInterface::cbDoneService, this, std::placeholders::_1, std::placeholders::_2));
    
    // arm1 한테 request 보내는
    arm1_client_ = create_client<ArmServiceType>("arm1_service");

    // 서비스가 뜰 때까지 기다리기
    while (!arm1_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_INFO(this->get_logger(), "서비스 대기 중...");
    }



    return true;
}


// int32 shelf_num
// int32 pinky_num

void RosInterface::arm1_send_request(int shelf_num, int pinky_num )
{
    log_->Log(Log::INFO, "arm1_send_request 진입");

    auto request = std::make_shared<ArmServiceType::Request>();
    request->shelf_num = shelf_num;
    request->pinky_num = pinky_num;

    // 응답 도착 시, 아래 콜백이 자동 호출됨
    arm1_client_->async_send_request(request,
        std::bind(&RosInterface::cbArmService, this, std::placeholders::_1));
}




// 로봇팔한테 request 보내고 response 받는 부분
void RosInterface::cbArmService(rclcpp::Client<ArmServiceType>::SharedFuture future)
{
        log_->Log(Log::INFO, "cbArmService 진입");

    auto res = future.get();

    // if(res->accepted)
    if(res->success)

        RCLCPP_INFO(this->get_logger(), "팔 요청 성공 !\n");
    else
        RCLCPP_ERROR(this->get_logger(), "팔 요청 실패 ㅠㅠ \n");

}


void RosInterface::cbRequestService(const std::shared_ptr<ReqServiceType::Request> request, std::shared_ptr<ReqServiceType::Response> response)
{
    Commondefine::GUIRequest r;
    r.requester = request->requester;
    r.shoes_property.size = request->size;
    r.shoes_property.model = request->model;
    r.shoes_property.color = request->color;
    r.dest2.x = request->x;
    r.dest2.y = request->y;
    r.customer_id = request->customer_id;

    auto icore = Icore_.lock();
    if(icore == nullptr)
    {
        log_->Log(Log::LogLevel::INFO, "ICore expired");

        //error
        response->wait_list = -1;

        return;
    }

    // bool wait_list = icore->RequestCallback(r);
    int wait_list = icore->RequestCallback(r);
    response->wait_list = wait_list;
}

void RosInterface::cbDoneService(const std::shared_ptr<DoneServiceType::Request> request,std::shared_ptr<DoneServiceType::Response> response)
{
    std::string requester = request->requester;
    int customer_id = request->customer_id;
    
    log_->Log(Log::LogLevel::INFO, "cbDoneService() 진입");

    if (auto icore = Icore_.lock())
    {
        log_->Log(Log::LogLevel::INFO, "DoneCallback() 호출 전");

        bool accepted = icore->DoneCallback(requester, customer_id);
        response->accepted = accepted;

        log_->Log(Log::LogLevel::INFO, "DoneCallback() 호출 후");

    }
    else
    {
        log_->Log(Log::LogLevel::INFO, "ICore expired");
        response->accepted = false;
    }
}