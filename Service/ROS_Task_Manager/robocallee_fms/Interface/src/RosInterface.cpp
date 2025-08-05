// RosInterface.cpp
#include "RosInterface.hpp"

using namespace interface;

RosInterface::RosInterface(Logger::s_ptr log)
: Node(_ROS_NODE_NAME_), log_(log)
{

}

RosInterface::~RosInterface() {}

bool RosInterface::Initialize(Integrated::w_ptr<core::ICore> Icore)
{
  Icore_ = Icore;

  req_service_ = create_service<ReqServiceType>("request_service", std::bind(&RosInterface::cbRequestService, this, std::placeholders::_1, std::placeholders::_2));
  done_service_ = create_service<DoneServiceType>("done_service", std::bind(&RosInterface::cbDoneService, this, std::placeholders::_1, std::placeholders::_2));
  
  arm1_client_ = create_client<ArmServiceType>("arm1_service");

  for (int i = 0; i < _AMR_NUM_; ++i)
  {
    auto topic = "/aruco_pose" + std::to_string(i+1);
    aruco_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(topic, 10, std::bind(&RosInterface::arucoPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
    log_->Log(INFO, "Subscribed to " + topic.c_str());
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
  
  if(res->success) log_->Log(Log::INFO, "Arm 요청 성공 !");

  else log_->Log(Log::INFO, "Arm 요청 실패 !");
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
  r.customer_id = request->customer_i
  
  auto icore = Icore_.lock();
  if(icore == nullptr)
  {
    log_->Log(Log::LogLevel::INFO, "ICore expire")
    response->wait_list = -1;
    
    return;
  }
    
  int wait_list = icore->RequestCallback(r);
  response->wait_list = wait_list;

  return;
}

void RosInterface::lmArrayCallback(
  const LmPoseMsg::ConstSharedPtr & msg,
  int pinky_id)
{
  if (msg->data.size() < 2) {
    RCLCPP_WARN(get_logger(), "lmArrayCallback: data size < 2");
    return;
  }

  // (1) pose2f 로 통일
  Commondefine::pose2f p;
  p.x = static_cast<float>(msg->data[0]);
  p.y = static_cast<float>(msg->data[1]);

  onArucoPose(pinky_id, p);
}

void RosInterface::arucoPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg, int pinky_id)
{
  std::lock_guard<std::mutex> lk(pose_mutex_);
  
  Commondefine::pose2f p;
  p.x = static_cast<float>(msg->pose.position.x);
  p.y = static_cast<float>(msg->pose.position.y);

  if (auto icore = Icore_.lock())
  {
    icore->PoseCallback(p, pinky_id);
  }
}

void RosInterface::cbDoneService(
  const std::shared_ptr<DoneServiceType::Request>  request,
  std::shared_ptr<DoneServiceType::Response>       response)
{
  if (auto icore = Icore_.lock())
  {
    response->accepted = icore->DoneCallback(request->requester,request->customer_id);
  }
  else 
  {
    log_->Log(Log::LogLevel::INFO, "ICore expired");
    response->accepted = false;
  }
}
