#include "RosInterface.hpp"
using namespace interface;
using namespace Integrated;
using namespace Commondefine;
RosInterface::RosInterface(Logger::s_ptr log)
: Node(_ROS_NODE_NAME_), log_(log) , server_wait_is_Running(true)
{
  server_wait_thread_ = std::thread(&RosInterface::async_server_wait, this);
}
RosInterface::~RosInterface()
{
  server_wait_is_Running = false;
  client_cv_.notify_all();
  if(server_wait_thread_.joinable())
  {
    server_wait_thread_.join();
    log_->Log(INFO, "async_server_wait thread 정상 종료");
  }
}
<<<<<<< Updated upstream
=======


>>>>>>> Stashed changes
bool RosInterface::Initialize(Integrated::w_ptr<core::ICore> Icore)
{
  Icore_ = Icore;
  customer_service_ = create_service<CustomerServiceType>("customer_service", std::bind(&RosInterface::cbCustomerRequest, this, std::placeholders::_1, std::placeholders::_2));
  employee_service_ = create_service<EmployeeServiceType>("employee_service", std::bind(&RosInterface::cbEmployeeRequest, this, std::placeholders::_1, std::placeholders::_2));
  aurco_array_sub_  = create_subscription<ArucoPoseArray>("/aruco_pose_array", 10, std::bind(&RosInterface::cbarucoPoseArray, this, std::placeholders::_1));
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
  for(int i = 0 ; i < RobotArm::RobotArmNum; ++i)
  {
    auto client_name = "arm" + std::to_string(i+1) + "_service";
    auto client = create_client<ArmServiceType>(client_name);
    arm_clients_.push_back(client);
    //모든 클라이언트는 서버 연결을 기다려야 하기 때문에 생성 후에 AddWaitClient함수를 통해 대기 Map에 넣어두고 thread 에서 각각 client가 서버에 연결되길 기다린다.
    AddWaitClient(client_name, client);
  }
<<<<<<< Updated upstream
  odom_pubs_.clear();
=======

  odom_pubs_.clear();
  nav_goal_pubs_.clear();
  battery_subs_.clear();

>>>>>>> Stashed changes
  for (int i = 0; i < _AMR_NUM_; ++i)
  {
    std::string topic = "/odom_" + std::to_string(i+1);
    odom_pubs_.push_back(create_publisher<nav_msgs::msg::Odometry>(topic, 10));
    log_->Log(Log::LogLevel::INFO, "Created odom for: " + topic);
<<<<<<< Updated upstream
=======
  
>>>>>>> Stashed changes
    topic = "goalpose" + std::to_string(i+1);
    nav_goal_pubs_.push_back( create_publisher<geometry_msgs::msg::PoseStamped>(topic,10));
    log_->Log(Log::LogLevel::INFO, "Created publisher for: " + topic);

    topic = "/pinky" + std::to_string(i+1) + "/pinky_battery_present";
    auto func = [this, i](const std_msgs::msg::Float32::SharedPtr msg) {this->cbBattery(i, msg);};
    battery_subs_.push_back(create_subscription<std_msgs::msg::Float32>(topic, 10, func));
    log_->Log(Log::LogLevel::INFO, "Subscribed battery for:" + topic);
  }

  return true;
}
void RosInterface::async_server_wait()
{
  log_->Log(INFO, "async_server_wait thread init");
  while (server_wait_is_Running)
  {
      std::unique_lock<std::mutex> lock(client_mutex_);
      // 대기중인 client 가 없거나 종료 요청 받으면 대기 풀림
      client_cv_.wait(lock, [this]() { return !wait_clients_.empty() || !server_wait_is_Running;});
      // 대기 중 종료 요청 받았으면 종료
      if (!server_wait_is_Running)
          break;
      for (auto it = wait_clients_.begin(); it != wait_clients_.end();)
      {
        if (it->second->wait_for_service(std::chrono::seconds(1)))
        {
          log_->Log(INFO, "클라이언트 [" + it->first + "] 연결 성공");
          it = wait_clients_.erase(it);
        }
        else
        {
          log_->Log(INFO, "클라이언트 [" + it->first  + "] 연결 대기 중...");
          ++it;
        }
      }
      // 락 해제 후 주기 대기 (optional)
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  log_->Log(INFO, "async_server_wait thread return");
  return;
}
// int32 shelf_num
// int32 robot_id
void RosInterface::arm1_send_request(int shelf_num, int robot_id , std::string action )
{
    log_->Log(Log::INFO, "arm1_send_request 진입. action: " + action);
    auto request = std::make_shared<ArmServiceType::Request>();
    request->robot_id = robot_id;
    request->action = action;
    request->shelf_num = shelf_num;
    // 응답 도착 시, 아래 콜백이 자동 호출됨
    arm_clients_[Commondefine::RobotArm1]->async_send_request(request,
        std::bind(&RosInterface::cbArmService, this, std::placeholders::_1));
}
void RosInterface::arm2_send_request(int robot_id , std::string action )
{
    log_->Log(Log::INFO, "arm2_send_request 진입. action: " + action);
    auto request = std::make_shared<ArmServiceType::Request>();
    request->robot_id = robot_id;
    request->action = action;
    request->shelf_num = -1;
    // 응답 도착 시, 아래 콜백이 자동 호출됨
    arm_clients_[Commondefine::RobotArm2]->async_send_request(request,
        std::bind(&RosInterface::cbArmService, this, std::placeholders::_1));
}
// 로봇팔한테 request 보내고 response 받는 부분
void RosInterface::cbArmService(rclcpp::Client<ArmServiceType>::SharedFuture future)
{
  log_->Log(Log::INFO, "cbArmService 진입");
  auto res = future.get();
  auto icore = Icore_.lock();
  if(icore == nullptr)
  {
      log_->Log(Log::LogLevel::INFO, "ICore expired");
      //error
      return;
  }
  if(res->success){
    // 다시 서랍에 넣는 신발 정보 업뎃
    if(res->action == "buffer_to_shelf"){
      Commondefine::shoesproperty incoming_shoe;
      incoming_shoe.size = res->size ;
      incoming_shoe.model = res->model ;
      incoming_shoe.color = res->color ;
      int shelf_num = res->shelf_num ;
      icore->UpdateShelfInfo(incoming_shoe, shelf_num);
    }
    log_->Log(Log::INFO, "RobotArm " + res->action + " 요청 성공 !");
  }
  else log_->Log(Log::INFO, "RobotArm 요청 실패 !");
}
void RosInterface::cbCustomerRequest(const std::shared_ptr<CustomerServiceType::Request> request, std::shared_ptr<CustomerServiceType::Response> response)
{
    Commondefine::GUIRequest r;
    r.requester = request->requester;
    r.action = request->action;
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
        response->success = false;
        response->action = r.action;
        return;
    }
    if (r.action == "come_here")
    {
        int wait_list = icore->RequestCallback(r);
        response->wait_list = wait_list;
        response->success = true;
        response->action = r.action;
    }
    else if (r.action == "done")
    {
        log_->Log(Log::LogLevel::INFO, "cbCustomerRequest DoneCallback() 호출 전");
        bool success = icore->DoneCallback(r.requester, r.customer_id);
        response->wait_list = 0;
        response->success = success;
        response->action = r.action;
        log_->Log(Log::LogLevel::INFO, "cbCustomerRequest DoneCallback() 호출 후");
    }
}
void RosInterface::cbEmployeeRequest(const std::shared_ptr<EmployeeServiceType::Request> request,std::shared_ptr<EmployeeServiceType::Response> response)
{
    Commondefine::GUIRequest r;
    r.requester = request->requester;
    r.action = request->action;
    r.customer_id = -1;
    auto icore = Icore_.lock();
    if(icore == nullptr)
    {
        log_->Log(Log::LogLevel::INFO, "ICore expired");
        //error
        response->wait_list = -1;
        response->success = false;
        response->action = r.action;
        return;
    }
    if (r.action == "come_here")
    {
        int wait_list = icore->RequestCallback(r);
        response->wait_list = wait_list;
        response->success = true;
        response->action = r.action;
    }
    else if (r.action == "done")
    {
        log_->Log(Log::LogLevel::INFO, "cbEmployeeRequest DoneCallback() 호출 전");
        bool success = icore->DoneCallback(r.requester, r.customer_id);
        response->wait_list = 0;
        response->success = success;
        response->action = r.action;
        log_->Log(Log::LogLevel::INFO, "cbEmployeeRequest DoneCallback() 호출 후");
    }
    else
    {
        log_->Log(Log::LogLevel::INFO, "ICore expired");
        response->success = false;
        response->action = r.action;
        response->wait_list = -1;
    }
}
<<<<<<< Updated upstream
=======

void RosInterface::cbBattery(const int idx, const std_msgs::msg::Float32::SharedPtr msg)
{
  updatebattery(idx, msg->data);
}


void RosInterface::updatebattery(int idx, float percent)
{
  if (percent < 0.f) percent = 0.f;
  if (percent > 100.f) percent = 100.f;

  if (auto icore = Icore_.lock())
  {
    icore->UpdateBattery(idx, percent);
  }

  log_->Log(Log::LogLevel::INFO, (std::ostringstream{} << "[BAT] pinky" << (idx+1) << " = "
                                  << std::fixed << std::setprecision(1)
                                  << percent << "%").str());
}
>>>>>>> Stashed changes
void RosInterface::cbarucoPoseArray(const ArucoPoseArray::ConstSharedPtr & msg)
{
  {
    std::lock_guard<std::mutex> lk(pose_mutex_);
<<<<<<< Updated upstream
    std::vector<Commondefine::pose2f> pos(_AMR_NUM_, Commondefine::pose2f{-1.f, -1.f});
=======
    
    std::vector<Commondefine::pose2f> pos(_AMR_NUM_, Commondefine::pose2f{-1.f, -1.f});

>>>>>>> Stashed changes
    for (const auto & ap : msg->poses)
    {
      int odom_i = ap.id;
      if (odom_i < 1 || odom_i > _AMR_NUM_) continue;
      int idx = odom_i -1 ;
<<<<<<< Updated upstream
      Commondefine::pose2f p;
      pos[idx].x = static_cast<float>(ap.x);
      pos[idx].y = static_cast<float>(ap.y);
=======

      Commondefine::pose2f p;
      pos[idx].x = static_cast<float>(ap.x);
      pos[idx].y = static_cast<float>(ap.y);

>>>>>>> Stashed changes
      nav_msgs::msg::Odometry odom;
      odom.header.stamp       = msg->header.stamp;
      odom.header.frame_id    = "map";
      odom.child_frame_id     = "pinky_" + std::to_string(odom_i);
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
      odom.pose.pose.position.x = ap.x;
      odom.pose.pose.position.y = ap.y;
      odom.pose.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, ap.yaw);
      odom.pose.pose.orientation = tf2::toMsg(q);
      odom.twist.twist = geometry_msgs::msg::Twist();
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
      if (idx >= 0 && idx < static_cast<int>(odom_pubs_.size()))
        odom_pubs_[idx]->publish(odom);
    }
    if (auto icore = Icore_.lock())
    {
      if(pos.empty()) return;
      icore->PoseCallback(pos);
    }
  }
}
void RosInterface::publishNavGoal(int idx, const Commondefine::Position wp)
{
    if (idx < 0 || idx >= static_cast<int>(nav_goal_pubs_.size()))
    {
        log_->Log(Log::LogLevel::ERROR, "publishNavGoal: invalid robot_id " + std::to_string(idx));
        return;
    }
    Commondefine::Quaternion q = Commondefine::toQuaternion(wp);
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = rclcpp::Clock().now();
    ps.header.frame_id = "map";
    ps.pose.position.x = ((wp.x - 1) * _MAP_RESOLUTION_) + (_MAP_RESOLUTION_ / 2.0);
    ps.pose.position.y = ((wp.y - 1) * _MAP_RESOLUTION_) + (_MAP_RESOLUTION_ / 2.0);
    ps.pose.position.z = 0.0;
    ps.pose.orientation.x = q.x;
    ps.pose.orientation.y = q.y;
    ps.pose.orientation.z = q.z;
    ps.pose.orientation.w = q.w;
    nav_goal_pubs_[idx]->publish(ps);
    log_->Log(Log::LogLevel::INFO,
              "Published wp to goalpose" + std::to_string(idx+1) +
              // ": (" + std::to_string(wp.pose.position.x) +
              // ", " + std::to_string(wp.pose.position.y) + ")");
              ": (" + std::to_string(wp.x) +
              ", " + std::to_string(wp.y) + ")");
}
