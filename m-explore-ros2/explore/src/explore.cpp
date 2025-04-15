// Modified to never blacklist and continuously try to navigate
#include <explore/explore.h>
#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  move_base_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, ACTION_NAME);
  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(), potential_scale_, gain_scale_, min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/frontiers", 10);
  }

  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
    "explore/resume", 10, std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");
  RCLCPP_INFO(logger_, "Using DFS-based frontier exploration");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s", map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)), [this]() { makePlan(); });
  makePlan();
}

Explore::~Explore() { stop(); }

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) resume();
  else stop();
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{// removed to try and explore forever
  // constexpr static size_t tolerace = 5;
  // auto* costmap2d = costmap_client_.getCostmap();
  // for (const auto& g : frontier_blacklist_) {
  //   double dx = fabs(goal.x - g.x), dy = fabs(goal.y - g.y);
  //   if (dx < tolerace * costmap2d->getResolution() && dy < tolerace * costmap2d->getResolution()) return true;
  // }
  return false;
}

bool Explore::goalAlreadyVisited(const geometry_msgs::msg::Point& goal)
{
  constexpr double visit_threshold_m = 1.0;  // meters
  for (const auto& g : visited_frontiers_) {
    double distance = std::hypot(goal.x - g.x, goal.y - g.y);
    if (distance < visit_threshold_m)
      return true;
  }
  return false;
}

void Explore::visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0.0; blue.g = 0.0; blue.b = 1.0; blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0; red.g = 0.0; red.b = 0.0; red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0.0; green.g = 1.0; green.b = 0.0; green.a = 1.0;
  std_msgs::msg::ColorRGBA yellow;
  yellow.r = 1.0; yellow.g = 1.0; yellow.b = 0.0; yellow.a = 1.0;

  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.frame_locked = true;
  m.action = visualization_msgs::msg::Marker::ADD;

  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;
  size_t id = 0;
  for (const auto& frontier : frontiers) {
    m.id = int(id++);
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.scale.x = m.scale.y = m.scale.z = 0.1;
    m.points = frontier.points;
    m.color = goalOnBlacklist(frontier.centroid) ? red : (frontier.dfs_path ? yellow : blue);
    markers.push_back(m);

    m.id = int(id++);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose.position = frontier.initial;
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = m.scale.y = m.scale.z = scale;
    m.points.clear();
    m.color = green;
    markers.push_back(m);
  }

  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }
  last_markers_count_ = markers.size();
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  auto pose = costmap_client_.getRobotPose();
  auto frontiers = search_.searchFrom(pose.position);

  if (visualize_) visualizeFrontiers(frontiers);
  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    return;
  }

  auto it = std::find_if(frontiers.begin(), frontiers.end(), [this](const auto& f) {
    return !goalOnBlacklist(f.centroid) && !goalAlreadyVisited(f.centroid);
  });

  if (it == frontiers.end()) {
    RCLCPP_INFO(logger_, "All frontiers visited or blacklisted.");
    return;
  }

  geometry_msgs::msg::Point target = it->centroid;
  bool same_goal = same_point(prev_goal_, target);
  prev_goal_ = target;

  if (!same_goal || prev_distance_ > it->min_distance) {
    last_progress_ = this->now();
    prev_distance_ = it->min_distance;
  }

  if ((this->now() - last_progress_ > tf2::durationFromSec(progress_timeout_)) && !resuming_) {
    frontier_blacklist_.push_back(target);
    makePlan();
    return;
  }

  if (resuming_) resuming_ = false;
  if (same_goal) return;

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target;
  goal.pose.pose.orientation.w = 1.0;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  options.result_callback = [this, target](const auto& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger_, "Goal reached.");
        visited_frontiers_.push_back(target);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(logger_, "Goal aborted.");
        frontier_blacklist_.push_back(target);
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger_, "Goal canceled.");
        return;
      default:
        RCLCPP_WARN(logger_, "Unknown result code.");
        break;
    }
    makePlan();
  };

  move_base_client_->async_send_goal(goal, options);
}

void Explore::stop(bool /*finished*/)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  exploring_timer_->reset();
  makePlan();
}
}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explore::Explore>());
  rclcpp::shutdown();
  return 0;
}