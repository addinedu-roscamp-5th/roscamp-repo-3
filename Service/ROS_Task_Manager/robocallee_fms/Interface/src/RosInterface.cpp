// RosInterface.cpp
#include "RosInterface.hpp"

using namespace interface;

RosInterface::RosInterface(Logger::s_ptr log)
: Node(_ROS_NODE_NAME_), log_(log)
{
  pose_subs_.resize(NUM_PINKY);
  aruco_subs_.resize(NUM_PINKY);
  last_poses_.resize(NUM_PINKY);
}

RosInterface::~RosInterface() {}

bool RosInterface::Initialize(Integrated::w_ptr<core::ICore> Icore)
{
  Icore_ = Icore;

  // 1) Float32MultiArray 구독 (/lm_poseN)
  for (int i = 0; i < NUM_PINKY; ++i) {
    auto topic = "/lm_pose" + std::to_string(i+1);
    pose_subs_[i] = create_subscription<LmPoseMsg>(
      topic, 10,
      [this, i](const LmPoseMsg::ConstSharedPtr & msg) {
        lmArrayCallback(msg, i);
      }
    );
    RCLCPP_INFO(get_logger(), "Subscribed to %s", topic.c_str());
  }

  // 2) Aruco PoseStamped 구독 (/aruco_poseN)
  for (int i = 0; i < NUM_PINKY; ++i) {
    auto topic = "/aruco_pose" + std::to_string(i+1);
    aruco_subs_[i] = create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, 10,
      [this, i](const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg) {
        arucoPoseCallback(msg, i);
      }
    );
    RCLCPP_INFO(get_logger(), "Subscribed to %s", topic.c_str());
  }

<<<<<<< Updated upstream:Service/ROS_Task_Manager/robocallee_fms/Interface/src/RosInterface.cpp


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
=======
  // 3) 서비스 초기화
  req_service_ = create_service<ReqServiceType>(
    "process_request",
    [this](const std::shared_ptr<ReqServiceType::Request>  req,
           std::shared_ptr<ReqServiceType::Response>       res)
    {
      cbRequestService(req, res);
>>>>>>> Stashed changes:robocallee_fms/Interface/src/RosInterface.cpp
    }
  );
  done_service_ = create_service<DoneServiceType>(
    "process_done",
    [this](const std::shared_ptr<DoneServiceType::Request>  req,
           std::shared_ptr<DoneServiceType::Response>       res)
    {
      cbDoneService(req, res);
    }
  );

<<<<<<< Updated upstream:Service/ROS_Task_Manager/robocallee_fms/Interface/src/RosInterface.cpp
    // bool wait_list = icore->RequestCallback(r);
    int wait_list = icore->RequestCallback(r);
    response->wait_list = wait_list;
=======
  arm1_client_ = create_client<ArmServiceType>("robot_arm_request");

  return true;
>>>>>>> Stashed changes:robocallee_fms/Interface/src/RosInterface.cpp
}

void RosInterface::lmArrayCallback(
  const LmPoseMsg::ConstSharedPtr & msg,
  int pinky_id)
{
<<<<<<< Updated upstream:Service/ROS_Task_Manager/robocallee_fms/Interface/src/RosInterface.cpp
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
=======
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
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg,
  int pinky_id)
{
  Commondefine::pose2f p;
  p.x = static_cast<float>(msg->pose.position.x);
  p.y = static_cast<float>(msg->pose.position.y);

  // (2) 바로 onArucoPose() 로 전달
  onArucoPose(pinky_id, p);
}

void RosInterface::onArucoPose(
  int pinky_id,
  const Commondefine::pose2f &p)
{
  {
    // (3) last_poses_ 에 안전하게 저장
    std::lock_guard<std::mutex> lk(pose_mutex_);
    last_poses_[pinky_id] = p;
  }
  // (4) Core 로 전달
  if (auto icore = Icore_.lock()) {
    icore->PoseCallback(p, pinky_id);
  }
}

void RosInterface::cbRequestService(
  const std::shared_ptr<ReqServiceType::Request>  request,
  std::shared_ptr<ReqServiceType::Response>       response)
{
  Commondefine::GUIRequest r;
  r.requester            = request->requester;
  r.shoes_property.size  = request->size;
  r.shoes_property.model = request->model;
  r.shoes_property.color = request->color;
  r.dest2.x              = request->x;
  r.dest2.y              = request->y;
  r.customer_id          = request->customer_id;

  if (auto icore = Icore_.lock()) {
    response->accepted = icore->RequestCallback(r);
  } else {
    log_->Log(Log::LogLevel::INFO, "ICore expired");
    response->accepted = false;
  }
}

void RosInterface::cbDoneService(
  const std::shared_ptr<DoneServiceType::Request>  request,
  std::shared_ptr<DoneServiceType::Response>       response)
{
  if (auto icore = Icore_.lock()) {
    response->accepted = icore->DoneCallback(request->requester,
                                             request->customer_id);
  } else {
    log_->Log(Log::LogLevel::INFO, "ICore expired");
    response->accepted = false;
  }
}

void RosInterface::arm1_send_request(int shelf_num, int pinky_num)
{
  if (auto icore = Icore_.lock()) {
    icore->ArmRequestMakeCall(1, shelf_num, pinky_num);
  }
}
>>>>>>> Stashed changes:robocallee_fms/Interface/src/RosInterface.cpp
