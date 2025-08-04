#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mapf_planner/cbs.hpp"

#include <vector>
#include <cmath>
#include <map>
#include <string>

using namespace std::chrono_literals;

struct WaypointWithYaw {
  double x, y, yaw;
};

struct RobotData {
  std::vector<WaypointWithYaw> waypoints;
  size_t waypoint_index{0};
  geometry_msgs::msg::PoseStamped current_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  rclcpp::TimerBase::SharedPtr timeout_timer;
  bool goal_sent{false};

  static constexpr double THRESHOLD = 0.04;
  static constexpr double THRESH_SQ = THRESHOLD * THRESHOLD;
};

class CBSPlannerNode : public rclcpp::Node {
public:
  CBSPlannerNode() : Node("cbs_planner_node_pose_pub") {
    RCLCPP_INFO(get_logger(), "📌 CBS Planner Pose Publisher Started");

    // 파라미터 선언 및 기본값 설정
    declare_parameter("num_robots", 2);  // 기본값을 2로 변경
    declare_parameter("timeout_seconds", 800.0);
    declare_parameter("resolution", 0.1);
    declare_parameter("start_positions", std::vector<double>{});
    declare_parameter("goal_positions", std::vector<double>{});

    num_robots_ = get_parameter("num_robots").as_int();
    timeout_seconds_ = get_parameter("timeout_seconds").as_double();
    resolution_ = get_parameter("resolution").as_double();

    RCLCPP_INFO(get_logger(), "🤖 로봇 수: %d", num_robots_);

    if (num_robots_ <= 0) {
      RCLCPP_ERROR(get_logger(), "❌ 로봇 수는 1 이상이어야 합니다");
      return;
    }

    std::vector<std::vector<bool>> map = {
      {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
      {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
      {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };

    // 시작점과 목표점 생성
    auto starts = generate_start_positions();
    auto goals = generate_goal_positions();

    if (starts.size() != static_cast<size_t>(num_robots_) || 
        goals.size() != static_cast<size_t>(num_robots_)) {
      RCLCPP_ERROR(get_logger(), "❌ 시작점 또는 목표점 수가 로봇 수와 일치하지 않음");
      RCLCPP_ERROR(get_logger(), "   시작점 수: %zu, 목표점 수: %zu, 로봇 수: %d", 
                   starts.size(), goals.size(), num_robots_);
      return;
    }

    RCLCPP_INFO(get_logger(), "🚀 CBS 솔버 시작 - 로봇 %d개", num_robots_);
    
    // 시작점과 목표점 출력
    for (int i = 0; i < num_robots_; ++i) {
      RCLCPP_INFO(get_logger(), "로봇 %d: 시작(%d,%d) → 목표(%d,%d)", 
                  i+1, starts[i].first, starts[i].second, goals[i].first, goals[i].second);
    }

    CBSSolver solver(map, starts, goals);
    auto paths = solver.findSolution(true);

    // 경로 생성 결과 확인 (로봇 수와 정확히 일치해야 함)
    if (paths.size() != static_cast<size_t>(num_robots_)) {
      RCLCPP_ERROR(get_logger(), "❌ CBS 경로 생성 실패 또는 일부 로봇 경로 없음");
      RCLCPP_ERROR(get_logger(), "   생성된 경로 수: %zu, 예상 로봇 수: %d", paths.size(), num_robots_);
      return;
    }

    RCLCPP_INFO(get_logger(), "✅ CBS 경로 생성 성공! %zu개 경로 생성됨", paths.size());

    // 각 로봇에 대해 초기화 (동적으로 num_robots_ 만큼)
    for (int id = 1; id <= num_robots_; ++id) {
      auto &rd = robots_[id];

      // waypoints 생성
      for (const auto &cell : paths[id - 1]) {
        double x = (cell.second - 1) * resolution_ + resolution_ / 2.0;
        double y = (cell.first - 1)  * resolution_ + resolution_ / 2.0;
        double yaw = M_PI / 2.0;
        rd.waypoints.push_back({x, y, yaw});
      }

      RCLCPP_INFO(get_logger(), "[%d] waypoints 생성 완료: %zu개", id, rd.waypoints.size());

      // pose subscription
      std::string pose_topic = "/robot" + std::to_string(id) + "/pose";
      rd.pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, 10,
        [this, id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          handle_pose(id, msg);
        });

      // goal publisher
      std::string goal_topic = "/goalpose" + std::to_string(id);
      rd.goal_pub = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

      // path publisher
      std::string path_topic = "/path" + std::to_string(id);
      rd.path_pub = create_publisher<nav_msgs::msg::Path>(path_topic, 10);

      // 경로 즉시 publish
      publish_path(id);
    }

    RCLCPP_INFO(get_logger(), "🎉 %d개 로봇에 대한 CBS 플래너 초기화 완료", num_robots_);
  }

private:
  std::vector<Position> generate_start_positions() {
    // 파라미터에서 커스텀 시작점 읽기
    auto custom_starts = get_parameter("start_positions").as_double_array();
    
    if (custom_starts.size() == static_cast<size_t>(num_robots_ * 2)) {
      std::vector<Position> starts;
      for (int i = 0; i < num_robots_; ++i) {
        int row = static_cast<int>(custom_starts[i * 2]);
        int col = static_cast<int>(custom_starts[i * 2 + 1]);
        starts.push_back({row, col});
      }
      RCLCPP_INFO(get_logger(), "📍 커스텀 시작점 사용");
      return starts;
    }

    // 기본 시작점 생성 (우측 가장자리에 세로로 배치)
    std::vector<Position> starts;
    int start_row = 2;  // 시작 행
    int start_col = 19; // 우측 가장자리
    
    for (int i = 0; i < num_robots_; ++i) {
      starts.push_back({start_row + i * 2, start_col});
    }
    
    RCLCPP_INFO(get_logger(), "📍 기본 시작점 생성 완료 (%d개)", num_robots_);
    return starts;
  }

  std::vector<Position> generate_goal_positions() {
    // 파라미터에서 커스텀 목표점 읽기
    auto custom_goals = get_parameter("goal_positions").as_double_array();
    
    if (custom_goals.size() == static_cast<size_t>(num_robots_ * 2)) {
      std::vector<Position> goals;
      for (int i = 0; i < num_robots_; ++i) {
        int row = static_cast<int>(custom_goals[i * 2]);
        int col = static_cast<int>(custom_goals[i * 2 + 1]);
        goals.push_back({row, col});
      }
      RCLCPP_INFO(get_logger(), "🎯 커스텀 목표점 사용");
      return goals;
    }

    // 기본 목표점 생성 (중앙 영역에 세로로 배치)
    std::vector<Position> goals;
    int goal_row = 3;   // 시작 행
    int goal_col = 11;  // 중앙 영역
    
    for (int i = 0; i < num_robots_; ++i) {
      goals.push_back({goal_row + i * 2, goal_col});
    }
    
    RCLCPP_INFO(get_logger(), "🎯 기본 목표점 생성 완료 (%d개)", num_robots_);
    return goals;
  }

  void publish_path(int id) {
    auto &rd = robots_[id];
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = now();

    // waypoints를 Path 메시지로 변환
    for (const auto &wp : rd.waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = now();
      pose.pose.position.x = wp.x;
      pose.pose.position.y = wp.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
      pose.pose.orientation.w = std::cos(wp.yaw / 2.0);
      
      path_msg.poses.push_back(pose);
    }

    rd.path_pub->publish(path_msg);
    RCLCPP_INFO(get_logger(), "[%d] 🛤️  경로 publish 완료 (%zu waypoints)", id, rd.waypoints.size());
  }

  void handle_pose(int id, geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    auto &rd = robots_[id];
    rd.current_pose = *msg;

    if (rd.waypoint_index >= rd.waypoints.size()) return;

    const auto &wp = rd.waypoints[rd.waypoint_index];
    double dx = rd.current_pose.pose.position.x - wp.x;
    double dy = rd.current_pose.pose.position.y - wp.y;

    RCLCPP_INFO(get_logger(),
      "[%d] 현재 위치: (%.2f, %.2f) → 목표: (%.2f, %.2f)",
      id, rd.current_pose.pose.position.x, rd.current_pose.pose.position.y, wp.x, wp.y);

    if (!rd.goal_sent) {
      send_goal_pose(id);
      start_timeout(id);
      return;
    }

    if (dx * dx + dy * dy < RobotData::THRESH_SQ) {
      RCLCPP_INFO(get_logger(), "✅ [%d] wp[%zu] 도달!", id, rd.waypoint_index);
      if (rd.timeout_timer) rd.timeout_timer->cancel();
      advance_waypoint(id);
    }
  }

  void send_goal_pose(int id) {
    auto &rd = robots_[id];
    const auto &wp = rd.waypoints[rd.waypoint_index];

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = now();
    goal.pose.position.x = wp.x;
    goal.pose.position.y = wp.y;
    goal.pose.orientation.z = std::sin(wp.yaw / 2.0);
    goal.pose.orientation.w = std::cos(wp.yaw / 2.0);

    rd.goal_pub->publish(goal);
    rd.goal_sent = true;

    RCLCPP_INFO(get_logger(),
      "[%d] 🟢 goalpose publish → wp[%zu] (%.2f, %.2f)", id, rd.waypoint_index, wp.x, wp.y);
  }

  void start_timeout(int id) {
    auto &rd = robots_[id];
    if (rd.timeout_timer) rd.timeout_timer->cancel();

    rd.timeout_timer = create_wall_timer(
      std::chrono::duration<double>(timeout_seconds_),
      [this, id]() {
        RCLCPP_WARN(get_logger(), "[%d] ⏰ %.0f초 timeout. 다음 waypoint로 이동", id, timeout_seconds_);
        robots_[id].timeout_timer->cancel();
        advance_waypoint(id);
      });
  }

  void advance_waypoint(int id) {
    auto &rd = robots_[id];
    rd.waypoint_index++;
    rd.goal_sent = false;

    if (rd.waypoint_index < rd.waypoints.size()) {
      send_goal_pose(id);
      start_timeout(id);
    } else {
      RCLCPP_INFO(get_logger(), "🎉 [%d] 모든 waypoint 완료", id);
    }
  }

  // 멤버 변수
  std::map<int, RobotData> robots_;
  int num_robots_;
  double timeout_seconds_;
  double resolution_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CBSPlannerNode>());
  rclcpp::shutdown();
  return 0;
}