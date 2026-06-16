#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <functional>

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
#include <QTimer>
#include <QStatusBar>
#include <QString>

// Forward declare the control node
class MoveItControlNode;

// ---------------------------------------------------------------------------
// MainWindow – the Qt5 GUI
// ---------------------------------------------------------------------------
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<MoveItControlNode> mc_node,
                        QWidget* parent = nullptr);
    ~MainWindow() override = default;

private slots:
    void onPlanJointGoal();
    void onReadJoints();
    void onPlanPose();
    void onReadPose();
    void onNamedPose(const std::string& name);
    void onGripperOpen();
    void onGripperClose();
    void onGripperCustom();
    void onGripperSet(double pos);
    void onCartesianUp();
    void onCartesianCircle();
    void updateState();

private:
    void buildUI();
    void buildJointTab(QTabWidget* tabs);
    void buildPoseTab(QTabWidget* tabs);
    void buildNamedTab(QTabWidget* tabs);
    void buildGripperTab(QTabWidget* tabs);
    void buildCartesianTab(QTabWidget* tabs);
    QDoubleSpinBox* addSpinBox(QFormLayout* form, const QString& label,
                               double def, double lo, double hi, double step);
    void runAsync(std::function<void()> func);

    std::shared_ptr<MoveItControlNode> mc_node_;
    QTimer* update_timer_ = nullptr;

    // Joint control
    std::vector<QDoubleSpinBox*> joint_spinboxes_;

    // Pose control
    QDoubleSpinBox *pose_x_ = nullptr, *pose_y_ = nullptr, *pose_z_ = nullptr;
    QDoubleSpinBox *pose_roll_ = nullptr, *pose_pitch_ = nullptr, *pose_yaw_ = nullptr;

    // Gripper
    QDoubleSpinBox* gripper_spin_ = nullptr;

    // State display
    QTextEdit* state_display_ = nullptr;
    QLabel* gripper_state_label_ = nullptr;
    QTextEdit* log_display_ = nullptr;
};

#endif // MAIN_WINDOW_HPP