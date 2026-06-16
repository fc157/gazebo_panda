/**
 * moveit_control_gui.cpp
 *
 * Qt5-based graphical control interface for the Franka Emika Panda robot
 * using MoveIt 2. Provides joint-space, Cartesian-space, named-pose,
 * gripper, and Cartesian-path control via an easy-to-use GUI.
 *
 * Usage:
 *   # Terminal 1: start the full demo (Gazebo + MoveIt + RViz)
 *   ros2 launch panda_moveit_config demo.launch.py
 *
 *   # Terminal 2 (after demo is fully up):
 *   ros2 launch moveit_control moveit_control.launch.py
 *   # or directly:
 *   ros2 run moveit_control moveit_control_gui
 */

#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <functional>

#include <QApplication>
#include <QMainWindow>
#include <QTabWidget>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTextEdit>
#include <QStatusBar>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "moveit_control_gui/main_window.hpp"

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr int    UPDATE_MS        = 2000;   // state refresh period
static constexpr double PI               = 3.14159265358979323846;
static constexpr double DEG2RAD          = PI / 180.0;
static constexpr double RAD2DEG          = 180.0 / PI;

// Joint limits in degrees (approximate)
static constexpr double JOINT_LIMITS[7][2] = {
    {-166.0, 166.0},   // joint1
    {-101.0, 101.0},   // joint2
    {-166.0, 166.0},   // joint3
    {-176.0,  -4.0},   // joint4
    {-166.0, 166.0},   // joint5
    {  -1.0, 215.0},   // joint6
    {-166.0, 166.0},   // joint7
};

static const char* JOINT_NAMES[7] = {
    "Joint1 (panda_joint1)",
    "Joint2 (panda_joint2)",
    "Joint3 (panda_joint3)",
    "Joint4 (panda_joint4)",
    "Joint5 (panda_joint5)",
    "Joint6 (panda_joint6)",
    "Joint7 (panda_joint7)",
};

static const double GRIPPER_MIN = 0.0;
static const double GRIPPER_MAX = 0.04;

// ---------------------------------------------------------------------------
// MoveItControlNode  – ROS 2 node + MoveIt interfaces
//
// NOTE: We do NOT inherit from rclcpp::Node here because MoveGroupInterface
// requires a shared_ptr<Node> to be passed. If we inherited from Node and
// tried to call shared_from_this() in the constructor, we'd get
// std::bad_weak_ptr (the shared_ptr isn't set up yet). Instead we hold a
// member rclcpp::Node::SharedPtr and pass it to MoveGroupInterface.
// ---------------------------------------------------------------------------
class MoveItControlNode
{
public:
    explicit MoveItControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    {
        node_ = std::make_shared<rclcpp::Node>("moveit_control_gui", options);

        RCLCPP_INFO(node_->get_logger(), "MoveIt control node initializing...");

        // Wait a bit for move_group to be up
        using namespace std::chrono_literals;
        rclcpp::sleep_for(1s);

        // Create the MoveIt interfaces
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                              node_, "panda_arm");
        move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                              node_, "gripper");
        planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Configure arm group
        move_group_arm_->setPlanningTime(10.0);
        move_group_arm_->setNumPlanningAttempts(10);
        move_group_arm_->setPlannerId("RRTConnectkConfigDefault");
        move_group_arm_->setMaxVelocityScalingFactor(0.5);
        move_group_arm_->setMaxAccelerationScalingFactor(0.5);
        move_group_arm_->allowReplanning(true);

        // Configure gripper group
        move_group_gripper_->setPlanningTime(5.0);
        move_group_gripper_->setNumPlanningAttempts(5);

        RCLCPP_INFO(node_->get_logger(), "MoveIt control node ready.");
        RCLCPP_INFO(node_->get_logger(), "  Planning frame: %s",
                    move_group_arm_->getPlanningFrame().c_str());
        RCLCPP_INFO(node_->get_logger(), "  End-effector  : %s",
                    move_group_arm_->getEndEffectorLink().c_str());
    }

    rclcpp::Node::SharedPtr getNode() const { return node_; }

    // ---- accessors --------------------------------------------------------
    std::vector<double> getJointValues()
    {
        try {
            return move_group_arm_->getCurrentJointValues();
        } catch (...) {
            return std::vector<double>(7, 0.0);
        }
    }

    std::vector<std::string> getJointNames()
    {
        try {
            return move_group_arm_->getJointNames();
        } catch (...) {
            return {"panda_joint1", "panda_joint2", "panda_joint3",
                    "panda_joint4", "panda_joint5", "panda_joint6",
                    "panda_joint7"};
        }
    }

    geometry_msgs::msg::PoseStamped getCurrentPose()
    {
        try {
            return move_group_arm_->getCurrentPose();
        } catch (...) {
            return geometry_msgs::msg::PoseStamped();
        }
    }

    std::vector<double> getGripperValues()
    {
        try {
            return move_group_gripper_->getCurrentJointValues();
        } catch (...) {
            return std::vector<double>(2, 0.0);
        }
    }

    std::string getStateString()
    {
        std::ostringstream oss;
        oss << "=== Robot State ===\n";

        // Joints
        try {
            auto vals = move_group_arm_->getCurrentJointValues();
            auto names = getJointNames();
            for (size_t i = 0; i < vals.size() && i < names.size(); ++i) {
                oss << "  " << names[i] << ": "
                    << std::fixed << std::setprecision(4) << vals[i] << " rad  ("
                    << std::fixed << std::setprecision(1) << vals[i] * RAD2DEG << "\260)\n";
            }
        } catch (const std::exception& e) {
            oss << "  Joints: " << e.what() << "\n";
        }

        // Pose
        try {
            auto pose = move_group_arm_->getCurrentPose();
            oss << "  EE Position: x=" << std::fixed << std::setprecision(4)
                << pose.pose.position.x << ", y=" << pose.pose.position.y
                << ", z=" << pose.pose.position.z << "\n";
            oss << "  EE Orientation: x=" << pose.pose.orientation.x
                << ", y=" << pose.pose.orientation.y
                << ", z=" << pose.pose.orientation.z
                << ", w=" << pose.pose.orientation.w << "\n";
        } catch (const std::exception& e) {
            oss << "  EE Pose: " << e.what() << "\n";
        }

        // Gripper
        try {
            auto gvals = move_group_gripper_->getCurrentJointValues();
            oss << "  Gripper: " << std::fixed << std::setprecision(4)
                << gvals[0] << ", " << gvals[1] << "\n";
        } catch (...) {}

        return oss.str();
    }

    // ---- movement commands -------------------------------------------------
    bool planAndExecuteJointGoal(const std::vector<double>& joint_goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Planning to joint target...");
        move_group_arm_->setJointValueTarget(joint_goal);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR(node_->get_logger(), "Joint-space planning failed");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "Plan succeeded, executing...");
        auto result = move_group_arm_->execute(plan);
        move_group_arm_->stop();
        move_group_arm_->clearPoseTargets();
        return (result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    bool planAndExecutePoseGoal(const geometry_msgs::msg::PoseStamped& target)
    {
        RCLCPP_INFO(node_->get_logger(), "Planning to pose target...");
        move_group_arm_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR(node_->get_logger(), "Pose-space planning failed");
            move_group_arm_->clearPoseTargets();
            return false;
        }



        // 拿到完整轨迹：moveit_msgs::msg::RobotTrajectory
        const moveit_msgs::msg::RobotTrajectory& traj = plan.trajectory_;

        // 遍历轨迹点（每个点 = 一组关节位置/速度/时间）
        for (const auto& point : traj.joint_trajectory.points)
        {
            // 关节位置
            for (double pos : point.positions)
            {
                // do something
                RCLCPP_INFO(node_->get_logger(), "Pose plan pos = %f", pos);
            }
            // 关节速度
            for (double vel : point.velocities)
            {
                // do something
                RCLCPP_INFO(node_->get_logger(), "Pose plan vel = %f", vel);
            }
            // 相对起始时间
            //double time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
        }




        RCLCPP_INFO(node_->get_logger(), "Pose plan succeeded, executing...");
        auto result = move_group_arm_->execute(plan);
        move_group_arm_->stop();
        move_group_arm_->clearPoseTargets();
        return (result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    bool goToNamedPose(const std::string& name)
    {
        RCLCPP_INFO(node_->get_logger(), "Going to named pose '%s'", name.c_str());
        move_group_arm_->setNamedTarget(name);
        auto result = move_group_arm_->move();
        move_group_arm_->stop();
        move_group_arm_->clearPoseTargets();
        return (result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    bool controlGripper(double position)
    {
        RCLCPP_INFO(node_->get_logger(), "Setting gripper to %.4f", position);
        std::vector<std::string> gripper_joints = {"panda_finger_joint1", "panda_finger_joint2"};
        std::vector<double> gripper_values = {position, position};
        move_group_gripper_->setJointValueTarget(gripper_joints, gripper_values);
        auto result = move_group_gripper_->move();
        move_group_gripper_->stop();
        return (result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    bool planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                           double eef_step = 0.01, double jump_threshold = 0.0)
    {
        RCLCPP_INFO(node_->get_logger(), "Planning Cartesian path with %zu waypoints",
                    waypoints.size());

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_arm_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(node_->get_logger(), "Cartesian path: %.1f%% coverage", fraction * 100.0);

        if (fraction < 0.8) {
            RCLCPP_WARN(node_->get_logger(), "Low Cartesian path coverage");
            return false;
        }

        auto result = move_group_arm_->execute(trajectory);
        move_group_arm_->stop();
        return (result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
};

// ===========================================================================
// MainWindow implementation
// ===========================================================================

MainWindow::MainWindow(std::shared_ptr<MoveItControlNode> mc_node,
                       QWidget* parent)
    : QMainWindow(parent), mc_node_(mc_node)
{
    setWindowTitle("MoveIt Panda Control (C++ / Qt5)");
    resize(1000, 750);

    buildUI();

    // Start the periodic state updater (Qt timer, runs on main thread)
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateState);
    update_timer_->start(UPDATE_MS);

    statusBar()->showMessage("Ready");
}

// ---- Slots: Joint Control ------------------------------------------------
void MainWindow::onPlanJointGoal()
{
    std::vector<double> goal(7);
    for (int i = 0; i < 7; ++i)
        goal[i] = joint_spinboxes_[i]->value() * DEG2RAD;

    statusBar()->showMessage("Planning joint-space goal...");
    runAsync([this, goal]() {
        bool ok = mc_node_->planAndExecuteJointGoal(goal);
        QMetaObject::invokeMethod(this, [this, ok]() {
            statusBar()->showMessage(ok ? "Joint goal succeeded" : "Joint goal FAILED", 5000);
        });
    });
}

void MainWindow::onReadJoints()
{
    auto vals = mc_node_->getJointValues();
    for (int i = 0; i < 7 && i < (int)vals.size(); ++i)
        joint_spinboxes_[i]->setValue(vals[i] * RAD2DEG);
    statusBar()->showMessage("Read current joint values", 3000);
}

// ---- Slots: Pose Control -------------------------------------------------
void MainWindow::onPlanPose()
{
    double x = pose_x_->value();
    double y = pose_y_->value();
    double z = pose_z_->value();
    double roll  = pose_roll_->value()  * DEG2RAD;
    double pitch = pose_pitch_->value() * DEG2RAD;
    double yaw   = pose_yaw_->value()   * DEG2RAD;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = mc_node_->getCurrentPose().header.frame_id;
    if (target.header.frame_id.empty())
        target.header.frame_id = "panda_link0";
    target.header.stamp = mc_node_->getNode()->now();
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    target.pose.orientation.x = q.x();
    target.pose.orientation.y = q.y();
    target.pose.orientation.z = q.z();
    target.pose.orientation.w = q.w();

    statusBar()->showMessage("Planning pose goal...");
    runAsync([this, target]() {
        bool ok = mc_node_->planAndExecutePoseGoal(target);
        QMetaObject::invokeMethod(this, [this, ok]() {
            statusBar()->showMessage(ok ? "Pose goal succeeded" : "Pose goal FAILED", 5000);
        });
    });
}

void MainWindow::onReadPose()
{
    auto pose = mc_node_->getCurrentPose();
    pose_x_->setValue(pose.pose.position.x);
    pose_y_->setValue(pose.pose.position.y);
    pose_z_->setValue(pose.pose.position.z);

    // Convert quaternion to RPY for display
    tf2::Quaternion q(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    pose_roll_->setValue(r * RAD2DEG);
    pose_pitch_->setValue(p * RAD2DEG);
    pose_yaw_->setValue(y * RAD2DEG);

    statusBar()->showMessage("Read current pose", 3000);
}

// ---- Slots: Named Poses --------------------------------------------------
void MainWindow::onNamedPose(const std::string& name)
{
    statusBar()->showMessage(QString("Going to '%1'...").arg(name.c_str()));
    if (name == "home") {
        runAsync([this]() {
            bool ok = mc_node_->planAndExecuteJointGoal({0,0,0,0,0,0,0});
            QMetaObject::invokeMethod(this, [this, ok]() {
                statusBar()->showMessage(ok ? "Home succeeded" : "Home FAILED", 5000);
            });
        });
    } else {
        std::string name_copy = name;
        runAsync([this, name_copy]() {
            bool ok = mc_node_->goToNamedPose(name_copy);
            QMetaObject::invokeMethod(this, [this, ok, name_copy]() {
                statusBar()->showMessage(
                    QString("Named pose '%1' %2")
                        .arg(name_copy.c_str(), ok ? "succeeded" : "FAILED").toStdString().c_str(),
                    5000);
            });
        });
    }
}

// ---- Slots: Gripper ------------------------------------------------------
void MainWindow::onGripperOpen()   { onGripperSet(0.035); }
void MainWindow::onGripperClose()  { onGripperSet(0.0); }
void MainWindow::onGripperCustom() { onGripperSet(gripper_spin_->value()); }

void MainWindow::onGripperSet(double pos)
{
    statusBar()->showMessage("Setting gripper...");
    runAsync([this, pos]() {
        bool ok = mc_node_->controlGripper(pos);
        QMetaObject::invokeMethod(this, [this, ok]() {
            statusBar()->showMessage(ok ? "Gripper succeeded" : "Gripper FAILED", 5000);
        });
    });
}

// ---- Slots: Cartesian ----------------------------------------------------
void MainWindow::onCartesianUp()
{
    auto current = mc_node_->getCurrentPose();
    if (current.header.frame_id.empty()) {
        statusBar()->showMessage("Cannot get current pose", 3000);
        return;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    auto p1 = current.pose;
    p1.position.z += 0.1;
    waypoints.push_back(p1);
    waypoints.push_back(current.pose);  // back down

    statusBar()->showMessage("Planning Cartesian up/down...");
    runAsync([this, waypoints]() {
        bool ok = mc_node_->planCartesianPath(waypoints);
        QMetaObject::invokeMethod(this, [this, ok]() {
            statusBar()->showMessage(ok ? "Cartesian succeeded" : "Cartesian FAILED", 5000);
        });
    });
}

void MainWindow::onCartesianCircle()
{
    auto current = mc_node_->getCurrentPose();
    if (current.header.frame_id.empty()) {
        statusBar()->showMessage("Cannot get current pose", 3000);
        return;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    const double radius = 0.05;
    const int npts = 16;

    for (int i = 0; i <= npts; ++i) {
        geometry_msgs::msg::Pose p = current.pose;
        double angle = 2.0 * PI * i / npts;
        p.position.x = current.pose.position.x + radius * std::cos(angle);
        p.position.y = current.pose.position.y + radius * std::sin(angle);
        waypoints.push_back(p);
    }

    statusBar()->showMessage("Planning Cartesian circle...");
    runAsync([this, waypoints]() {
        bool ok = mc_node_->planCartesianPath(waypoints, 0.005);
        QMetaObject::invokeMethod(this, [this, ok]() {
            statusBar()->showMessage(ok ? "Circle succeeded" : "Circle FAILED", 5000);
        });
    });
}

// ---- State Update --------------------------------------------------------
void MainWindow::updateState()
{
    if (!mc_node_) return;

    // Spawn a short async read to avoid blocking the GUI thread
    std::thread([this]() {
        std::string state = mc_node_->getStateString();
        auto gvals = mc_node_->getGripperValues();

        QMetaObject::invokeMethod(this, [this, state, gvals]() {
            state_display_->setText(state.c_str());

            if (gvals.size() >= 2) {
                gripper_state_label_->setText(
                    QString("Gripper: %1, %2")
                        .arg(gvals[0], 0, 'f', 4)
                        .arg(gvals[1], 0, 'f', 4));
            }
        });
    }).detach();
}

// ---- UI Construction -----------------------------------------------------
void MainWindow::buildUI()
{
    QWidget* central = new QWidget(this);
    setCentralWidget(central);

    QHBoxLayout* main_layout = new QHBoxLayout(central);

    // Left panel: tabbed control
    QTabWidget* tabs = new QTabWidget();
    main_layout->addWidget(tabs, 3);

    buildJointTab(tabs);
    buildPoseTab(tabs);
    buildNamedTab(tabs);
    buildGripperTab(tabs);
    buildCartesianTab(tabs);

    // Right panel: state display
    QWidget* right_panel = new QWidget();
    QVBoxLayout* right_layout = new QVBoxLayout(right_panel);
    main_layout->addWidget(right_panel, 2);

    // State display
    QGroupBox* state_box = new QGroupBox("Robot State");
    QVBoxLayout* state_layout = new QVBoxLayout(state_box);
    state_display_ = new QTextEdit();
    state_display_->setReadOnly(true);
    state_display_->setFont(QFont("Courier", 9));
    state_display_->setMinimumWidth(350);
    state_layout->addWidget(state_display_);
    right_layout->addWidget(state_box);

    // Gripper state
    QGroupBox* grip_box = new QGroupBox("Gripper Status");
    QVBoxLayout* grip_layout = new QVBoxLayout(grip_box);
    gripper_state_label_ = new QLabel("Gripper: Unknown");
    gripper_state_label_->setFont(QFont("Courier", 10));
    grip_layout->addWidget(gripper_state_label_);
    right_layout->addWidget(grip_box);

    right_layout->addStretch();
}

void MainWindow::buildJointTab(QTabWidget* tabs)
{
    QWidget* tab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(tab);

    QGroupBox* box = new QGroupBox("Joint Position Control (degrees)");
    QVBoxLayout* box_layout = new QVBoxLayout(box);

    for (int i = 0; i < 7; ++i) {
        QHBoxLayout* row = new QHBoxLayout();

        QLabel* lbl = new QLabel(JOINT_NAMES[i]);
        lbl->setMinimumWidth(180);
        row->addWidget(lbl);

        QDoubleSpinBox* sb = new QDoubleSpinBox();
        sb->setRange(JOINT_LIMITS[i][0], JOINT_LIMITS[i][1]);
        sb->setDecimals(1);
        sb->setSingleStep(5.0);
        sb->setValue(0.0);
        sb->setMinimumWidth(80);
        joint_spinboxes_.push_back(sb);
        row->addWidget(sb);

        row->addStretch();
        box_layout->addLayout(row);
    }

    // Buttons
    QHBoxLayout* btn_row = new QHBoxLayout();
    QPushButton* plan_btn = new QPushButton("Plan && Execute Joint Goal");
    QPushButton* read_btn = new QPushButton("Read Current Joints");
    btn_row->addWidget(plan_btn);
    btn_row->addWidget(read_btn);
    btn_row->addStretch();
    box_layout->addLayout(btn_row);

    layout->addWidget(box);
    layout->addStretch();
    tabs->addTab(tab, "Joint Control");

    connect(plan_btn, &QPushButton::clicked, this, &MainWindow::onPlanJointGoal);
    connect(read_btn, &QPushButton::clicked, this, &MainWindow::onReadJoints);
}

void MainWindow::buildPoseTab(QTabWidget* tabs)
{
    QWidget* tab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(tab);

    // Position group
    QGroupBox* pos_box = new QGroupBox("End-Effector Position (meters)");
    QFormLayout* pos_form = new QFormLayout(pos_box);

    pose_x_ = addSpinBox(pos_form, "X", 0.307, -1.0, 1.0, 0.01);
    pose_y_ = addSpinBox(pos_form, "Y", 0.0,   -1.0, 1.0, 0.01);
    pose_z_ = addSpinBox(pos_form, "Z", 0.5,   -0.5, 1.5, 0.01);
    layout->addWidget(pos_box);

    // Orientation group
    QGroupBox* ori_box = new QGroupBox("Orientation (degrees)");
    QFormLayout* ori_form = new QFormLayout(ori_box);

    pose_roll_  = addSpinBox(ori_form, "Roll",  0.0, -180, 180, 1.0);
    pose_pitch_ = addSpinBox(ori_form, "Pitch", 0.0, -180, 180, 1.0);
    pose_yaw_   = addSpinBox(ori_form, "Yaw",   0.0, -180, 180, 1.0);
    layout->addWidget(ori_box);

    // Buttons
    QHBoxLayout* btn_row = new QHBoxLayout();
    QPushButton* plan_btn = new QPushButton("Plan && Execute Pose");
    QPushButton* read_btn = new QPushButton("Read Current Pose");
    btn_row->addWidget(plan_btn);
    btn_row->addWidget(read_btn);
    btn_row->addStretch();
    layout->addLayout(btn_row);

    layout->addStretch();
    tabs->addTab(tab, "Pose Control");

    connect(plan_btn, &QPushButton::clicked, this, &MainWindow::onPlanPose);
    connect(read_btn, &QPushButton::clicked, this, &MainWindow::onReadPose);
}

void MainWindow::buildNamedTab(QTabWidget* tabs)
{
    QWidget* tab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(tab);

    QGroupBox* box = new QGroupBox("Predefined Poses");
    QVBoxLayout* box_layout = new QVBoxLayout(box);

    struct { const char* label; const char* name; const char* desc; } poses[] = {
        {"Ready",      "ready",     "Initial folded pose (default)"},
        {"Extended",   "extended",  "Straight arm pose"},
        {"Transport",  "transport", "Transport configuration"},
        {"Home",       "home",      "All joints at 0\260 (all zeros)"},
    };

    for (auto& p : poses) {
        QHBoxLayout* row = new QHBoxLayout();
        QPushButton* btn = new QPushButton(p.label);
        btn->setFixedWidth(120);
        row->addWidget(btn);

        QLabel* desc_lbl = new QLabel(p.desc);
        row->addWidget(desc_lbl);
        row->addStretch();
        box_layout->addLayout(row);

        connect(btn, &QPushButton::clicked, this, [this, name = std::string(p.name)]() {
            onNamedPose(name);
        });
    }

    layout->addWidget(box);
    layout->addStretch();
    tabs->addTab(tab, "Named Poses");
}

void MainWindow::buildGripperTab(QTabWidget* tabs)
{
    QWidget* tab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(tab);

    QGroupBox* box = new QGroupBox("Gripper Control");
    QVBoxLayout* box_layout = new QVBoxLayout(box);

    QHBoxLayout* pos_row = new QHBoxLayout();
    pos_row->addWidget(new QLabel("Gripper Position:"));
    gripper_spin_ = new QDoubleSpinBox();
    gripper_spin_->setRange(GRIPPER_MIN, GRIPPER_MAX);
    gripper_spin_->setDecimals(4);
    gripper_spin_->setSingleStep(0.002);
    gripper_spin_->setValue(0.035);
    gripper_spin_->setMinimumWidth(100);
    pos_row->addWidget(gripper_spin_);
    pos_row->addStretch();
    box_layout->addLayout(pos_row);

    QHBoxLayout* btn_row = new QHBoxLayout();
    QPushButton* open_btn = new QPushButton("Open (0.035)");
    QPushButton* close_btn = new QPushButton("Close (0.0)");
    QPushButton* set_btn = new QPushButton("Set Custom");
    btn_row->addWidget(open_btn);
    btn_row->addWidget(close_btn);
    btn_row->addWidget(set_btn);
    btn_row->addStretch();
    box_layout->addLayout(btn_row);

    layout->addWidget(box);
    layout->addStretch();
    tabs->addTab(tab, "Gripper");

    connect(open_btn, &QPushButton::clicked, this, &MainWindow::onGripperOpen);
    connect(close_btn, &QPushButton::clicked, this, &MainWindow::onGripperClose);
    connect(set_btn,  &QPushButton::clicked, this, &MainWindow::onGripperCustom);
}

void MainWindow::buildCartesianTab(QTabWidget* tabs)
{
    QWidget* tab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(tab);

    QGroupBox* box = new QGroupBox("Cartesian Path Control");
    QVBoxLayout* box_layout = new QVBoxLayout(box);

    QLabel* info = new QLabel(
        "Execute pre-defined Cartesian paths:\n"
        " - Move Up: lifts the end-effector 10cm in Z then returns.\n"
        " - Draw Circle: traces a 5cm-radius circle in the XY plane.");
    info->setWordWrap(true);
    box_layout->addWidget(info);

    QPushButton* up_btn = new QPushButton("Move Up 10cm && Back");
    QPushButton* circle_btn = new QPushButton("Draw Circle (approx)");
    box_layout->addWidget(up_btn);
    box_layout->addWidget(circle_btn);

    layout->addWidget(box);
    layout->addStretch();
    tabs->addTab(tab, "Cartesian");

    connect(up_btn, &QPushButton::clicked, this, &MainWindow::onCartesianUp);
    connect(circle_btn, &QPushButton::clicked, this, &MainWindow::onCartesianCircle);
}

// ---- Helpers -------------------------------------------------------------
QDoubleSpinBox* MainWindow::addSpinBox(QFormLayout* form, const QString& label,
                                       double def, double lo, double hi, double step)
{
    QDoubleSpinBox* sb = new QDoubleSpinBox();
    sb->setRange(lo, hi);
    sb->setDecimals(3);
    sb->setSingleStep(step);
    sb->setValue(def);
    sb->setMinimumWidth(100);
    form->addRow(label, sb);
    return sb;
}

void MainWindow::runAsync(std::function<void()> func)
{
    std::thread(func).detach();
}

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char* argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the control object (contains its own rclcpp::Node internally)
    auto mc_node = std::make_shared<MoveItControlNode>();

    // Spin ROS in a background thread
    std::thread spin_thread([mc_node]() {
        rclcpp::spin(mc_node->getNode());
    });

    // Initialize Qt
    QApplication app(argc, argv);
    MainWindow window(mc_node);
    window.show();

    int result = app.exec();

    // Cleanup
    rclcpp::shutdown();
    spin_thread.join();

    return result;
}