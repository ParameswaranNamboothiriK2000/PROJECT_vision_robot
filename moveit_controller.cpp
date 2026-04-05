// =============================================================
//  PART 3 — moveit_controller.cpp
//  Project : Autonomous Vision-to-Motion Robotic Inspection
//  Node    : MoveIt 2 Controller (C++)
//  Role    : Subscribes to /target_pose from Python vision node
//            → Solves IK → Moves Panda 7-DOF arm to target
//  Subscribes : /target_pose     (geometry_msgs/Point)
//  Subscribes : /vision_status   (std_msgs/String)
//  Robot      : Franka Panda 7-DOF (panda_arm planning group)
// =============================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>

// MoveIt 2 core headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

// TF2 for quaternion math
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

// C++ standard library
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

// =============================================================
//  CONFIGURATION
// =============================================================

// Panda planning group name (defined in panda.srdf)
const std::string PLANNING_GROUP = "panda_arm";

// End-effector link name for Panda
const std::string EEF_LINK = "panda_hand";

// Cooldown between movements (seconds) — prevents command flooding
const double MOVE_COOLDOWN_SEC = 2.0;

// Minimum distance (meters) to trigger a new movement
// Avoids re-planning for tiny jitter movements
const double MIN_MOVE_DISTANCE = 0.03;

// MoveIt planning time limit (seconds)
const double PLANNING_TIME_SEC = 5.0;

// Maximum velocity and acceleration scaling (0.0 to 1.0)
// Keep low for safe simulation movement
const double MAX_VELOCITY_SCALE     = 0.3;
const double MAX_ACCELERATION_SCALE = 0.3;

// Number of IK planning retries on failure
const int MAX_PLAN_RETRIES = 3;

// Workspace safety bounds (must match vision_node.py)
const double WS_X_MIN = 0.2,  WS_X_MAX = 0.7;
const double WS_Y_MIN = -0.4, WS_Y_MAX = 0.4;
const double WS_Z_MIN = 0.1,  WS_Z_MAX = 0.8;

// =============================================================


class MoveitController : public rclcpp::Node
{
public:
  // ----------------------------------------------------------
  //  CONSTRUCTOR
  // ----------------------------------------------------------
  MoveitController()
  : Node("moveit_controller",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    is_moving_(false),
    total_moves_(0),
    failed_plans_(0)
  {
    // --- Subscribers ---
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/target_pose",
      10,
      std::bind(&MoveitController::target_pose_callback, this, _1)
    );

    vision_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vision_status",
      10,
      std::bind(&MoveitController::vision_status_callback, this, _1)
    );

    // Initialize last move time to allow immediate first move
    last_move_time_ = this->now() - rclcpp::Duration::from_seconds(MOVE_COOLDOWN_SEC + 1.0);

    // Initialize last target position to far-away sentinel value
    last_target_.x = -999.0;
    last_target_.y = -999.0;
    last_target_.z = -999.0;

    RCLCPP_INFO(this->get_logger(),
      "\n"
      "╔══════════════════════════════════════════╗\n"
      "║  MoveIt Controller Node READY           ║\n"
      "║  Planning group : panda_arm             ║\n"
      "║  Waiting for   : /target_pose topic     ║\n"
      "║  Cooldown       : %.1f seconds           ║\n"
      "╚══════════════════════════════════════════╝",
      MOVE_COOLDOWN_SEC
    );
  }

  // ----------------------------------------------------------
  //  INITIALIZE MOVEIT (called after node is spinning)
  //  Must run AFTER rclcpp::spin starts — MoveIt needs
  //  the node executor to be active to connect to move_group
  // ----------------------------------------------------------
  void init_moveit()
  {
    RCLCPP_INFO(this->get_logger(), "Connecting to MoveIt 2 move_group server...");

    // Create MoveGroupInterface — connects to move_group action server
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP
    );

    // Configure planning parameters
    move_group_->setPlanningTime(PLANNING_TIME_SEC);
    move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);
    move_group_->setNumPlanningAttempts(MAX_PLAN_RETRIES);

    // Set planner — OMPL RRTConnect is fast and reliable
    move_group_->setPlannerId("RRTConnect");

    // Create planning scene interface for collision objects
    planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Add a table collision object to prevent arm hitting the floor
    add_table_collision_object();

    // Move arm to safe home position on startup
    go_to_home_position();

    RCLCPP_INFO(this->get_logger(),
      "MoveIt 2 connected! Planning group: %s | End-effector: %s",
      PLANNING_GROUP.c_str(), EEF_LINK.c_str()
    );

    moveit_ready_.store(true);
  }

private:
  // ----------------------------------------------------------
  //  CALLBACK: /target_pose — main control loop
  // ----------------------------------------------------------
  void target_pose_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    // Gate 1: MoveIt must be initialized
    if (!moveit_ready_.load()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "MoveIt not ready yet — waiting for init_moveit()");
      return;
    }

    // Gate 2: Don't interrupt an ongoing movement
    if (is_moving_.load()) {
      RCLCPP_DEBUG(this->get_logger(), "Movement in progress — skipping new target");
      return;
    }

    // Gate 3: Enforce cooldown timer
    auto now = this->now();
    auto elapsed = (now - last_move_time_).seconds();
    if (elapsed < MOVE_COOLDOWN_SEC) {
      return;  // Silently skip — no log spam
    }

    // Gate 4: Check if target moved enough to justify replanning
    double dx = msg->x - last_target_.x;
    double dy = msg->y - last_target_.y;
    double dz = msg->z - last_target_.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist < MIN_MOVE_DISTANCE && last_target_.x != -999.0) {
      RCLCPP_DEBUG(this->get_logger(),
        "Target moved %.3fm — below threshold %.3fm, skipping",
        dist, MIN_MOVE_DISTANCE
      );
      return;
    }

    // Gate 5: Safety bounds check
    if (!is_within_workspace(msg->x, msg->y, msg->z)) {
      RCLCPP_WARN(this->get_logger(),
        "Target (%.3f, %.3f, %.3f) is OUTSIDE workspace bounds — rejected!",
        msg->x, msg->y, msg->z
      );
      return;
    }

    // All gates passed — execute movement in a separate thread
    // This prevents blocking the ROS 2 callback executor
    last_target_ = *msg;
    last_move_time_ = now;
    is_moving_.store(true);

    RCLCPP_INFO(this->get_logger(),
      "→ Moving to target: x=%.3f  y=%.3f  z=%.3f  (dist_from_last=%.3fm)",
      msg->x, msg->y, msg->z, dist
    );

    // Launch movement in detached thread so callbacks keep running
    std::thread([this, target = *msg]() {
      execute_motion(target);
      is_moving_.store(false);
    }).detach();
  }

  // ----------------------------------------------------------
  //  CALLBACK: /vision_status — log vision node status
  // ----------------------------------------------------------
  void vision_status_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Only log status changes to avoid log spam
    if (msg->data != last_vision_status_) {
      last_vision_status_ = msg->data;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[Vision] %s", msg->data.c_str()
      );
    }
  }

  // ----------------------------------------------------------
  //  EXECUTE MOTION — plan and move arm to target position
  // ----------------------------------------------------------
  void execute_motion(const geometry_msgs::msg::Point target)
  {
    // Build a Pose from the target Point
    // Orientation: end-effector pointing straight DOWN (gripper faces floor)
    // This is the standard pick-and-place approach orientation
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target.x;
    target_pose.position.y = target.y;
    target_pose.position.z = target.z;

    // Quaternion for downward-pointing end-effector
    // Roll=PI rotates gripper to face down, Pitch=0, Yaw=0
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);  // End-effector pointing down
    q.normalize();
    target_pose.orientation = tf2::toMsg(q);

    // Set the target pose for the end-effector
    move_group_->setPoseTarget(target_pose, EEF_LINK);

    // Plan trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = false;

    for (int attempt = 1; attempt <= MAX_PLAN_RETRIES; ++attempt) {
      auto result = move_group_->plan(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        plan_success = true;
        RCLCPP_INFO(this->get_logger(),
          "✅ Plan found on attempt %d/%d | Trajectory: %.2fs",
          attempt, MAX_PLAN_RETRIES,
          plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
          plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9
        );
        break;
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Plan attempt %d/%d failed — retrying...", attempt, MAX_PLAN_RETRIES
        );
        std::this_thread::sleep_for(200ms);
      }
    }

    if (!plan_success) {
      failed_plans_++;
      RCLCPP_ERROR(this->get_logger(),
        "❌ All %d planning attempts failed for target (%.3f, %.3f, %.3f) "
        "| Total failures: %d",
        MAX_PLAN_RETRIES, target.x, target.y, target.z, failed_plans_
      );
      move_group_->clearPoseTargets();
      return;
    }

    // Execute the planned trajectory
    auto exec_result = move_group_->execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      total_moves_++;
      RCLCPP_INFO(this->get_logger(),
        "🤖 Arm moved! Total successful moves: %d", total_moves_
      );
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Execution failed after successful plan!");
    }

    // Always clear pose targets after attempt
    move_group_->clearPoseTargets();
  }

  // ----------------------------------------------------------
  //  HOME POSITION — safe starting pose on startup
  // ----------------------------------------------------------
  void go_to_home_position()
  {
    RCLCPP_INFO(this->get_logger(), "Moving arm to home position...");

    // Named target "ready" is defined in Panda's SRDF
    move_group_->setNamedTarget("ready");

    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (move_group_->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(home_plan);
      RCLCPP_INFO(this->get_logger(), "✅ Arm at home (ready) position");
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not plan to home — arm stays at current pose");
    }
  }

  // ----------------------------------------------------------
  //  ADD TABLE COLLISION OBJECT
  //  Prevents arm from planning through the floor/table
  // ----------------------------------------------------------
  void add_table_collision_object()
  {
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group_->getPlanningFrame();
    table.id = "table";

    // Table as a box shape
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {1.5, 1.5, 0.05};  // 1.5m x 1.5m x 5cm thick

    // Position table just below the robot base
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.4;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.05;  // Just below z=0
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(box);
    table.primitive_poses.push_back(table_pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_->applyCollisionObject(table);
    RCLCPP_INFO(this->get_logger(), "✅ Table collision object added to planning scene");
  }

  // ----------------------------------------------------------
  //  WORKSPACE BOUNDS CHECK
  // ----------------------------------------------------------
  bool is_within_workspace(double x, double y, double z)
  {
    return (x >= WS_X_MIN && x <= WS_X_MAX &&
            y >= WS_Y_MIN && y <= WS_Y_MAX &&
            z >= WS_Z_MIN && z <= WS_Z_MAX);
  }

  // ----------------------------------------------------------
  //  MEMBER VARIABLES
  // ----------------------------------------------------------
  // ROS 2 subscribers
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_status_sub_;

  // MoveIt 2 interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  // Thread-safe state flags
  std::atomic<bool> is_moving_{false};      // True while arm is executing
  std::atomic<bool> moveit_ready_{false};   // True after init_moveit() completes

  // Timing and motion state
  rclcpp::Time last_move_time_;
  geometry_msgs::msg::Point last_target_;
  std::string last_vision_status_;

  // Statistics
  int total_moves_;
  int failed_plans_;
};


// =============================================================
//  MAIN — MultiThreadedExecutor for concurrent callbacks
// =============================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // MultiThreadedExecutor allows:
  //   - target_pose callback to run while MoveIt plans
  //   - Vision status callback to run independently
  //   - Movement thread to execute without blocking callbacks
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  auto node = std::make_shared<MoveitController>();
  executor->add_node(node);

  // Spin in a background thread so init_moveit() can run
  // (init_moveit() needs the node spinning to connect to MoveIt)
  std::thread spin_thread([&executor]() {
    executor->spin();
  });

  // Give executor a moment to start up, then initialize MoveIt
  std::this_thread::sleep_for(500ms);
  node->init_moveit();

  RCLCPP_INFO(node->get_logger(),
    "\n"
    "╔══════════════════════════════════════════╗\n"
    "║  System FULLY OPERATIONAL               ║\n"
    "║  Vision node → MoveIt controller active ║\n"
    "║  Panda arm tracking target object...    ║\n"
    "╚══════════════════════════════════════════╝"
  );

  // Wait for spin thread to finish (runs until Ctrl+C)
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}
