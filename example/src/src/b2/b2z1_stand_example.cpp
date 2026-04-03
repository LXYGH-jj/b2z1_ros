#include <chrono>
// [新增] 为 Z1 轨迹插值和夹爪比例限制增加标准库支持
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <iostream>
#include <thread>
// [新增] 为 Z1 控制对象使用智能指针
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "b2/b2_motion_switch_client.hpp"
#include "motor_crc.h"
#ifdef WITH_Z1_SDK
// [新增] Z1 SDK 头文件
#include "unitree_arm_sdk/control/unitreeArm.h"
#endif

#define TOPIC_LOWCMD "/lowcmd"
#define TOPIC_LOWSTATE "/lowstate"

class Custom : public rclcpp::Node {
 public:
  explicit Custom() : Node("low_level_cmd_node"), msc_(this) {
    Init();
    Start();
  }
  // [新增] 在节点析构时安全回收 Z1 控制状态
  ~Custom() override {
#ifdef WITH_Z1_SDK
    ShutdownZ1();
#endif
  }

  void Init();
  void Start();

 private:
  void InitLowCmd();
  void LowStateMessageHandler(unitree_go::msg::LowState::SharedPtr msg);
  void LowCmdWrite();
  int queryMotionStatus();
  std::string queryServiceName(std::string form, std::string name);
#ifdef WITH_Z1_SDK
  // [新增] Z1 初始化、联动控制和退出收尾接口
  void InitZ1();
  void SyncZ1Motion();
  void ShutdownZ1();
#endif

  float kp_ = 1000.0;
  float kd_ = 10.0;
  double time_consume_ = 0;
  int rate_count_ = 0;
  int sin_count_ = 0;
  int motiontime_ = 0;
  float dt_ = 0.002;  // 0.001~0.01

  unitree::robot::b2::MotionSwitchClient msc_;
  unitree_go::msg::LowCmd low_cmd_;
  unitree_go::msg::LowState low_state_;

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_publisher;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscriber;
  rclcpp::TimerBase::SharedPtr lowCmdWriteThreadPtr;

  float target_pos_1_[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

  float target_pos_2_[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                              0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

  float target_pos_3_[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              0.0, 1.36, -2.65, 0.0, 1.36, -2.65};

  float target_pos_4_[12] = {-0.5, 1.36, -2.65, 0.5, 1.36, -2.65,
                              -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

  float start_pos_[12]{};
  float duration_1_ = 500;
  float duration_2_ = 900;
  float duration_3_ = 1000;
  float duration_4_ = 1100;
  float duration_5_ = 500;
  float percent_1_ = 0;
  float percent_2_ = 0;
  float percent_3_ = 0;
  float percent_4_ = 0;
  float percent_5_ = 0;

  bool first_run_ = true;
  bool done_ = false;
#ifdef WITH_Z1_SDK
  // [新增] Z1 控制状态与轨迹目标
  bool z1_ready_ = false;
  bool z1_has_gripper_ = true;
  std::unique_ptr<UNITREE_ARM::unitreeArm> z1_arm_;
  Vec6 z1_init_q_ = Vec6::Zero();
  Vec6 z1_pose_a_ = (Vec6() << 0.0, 1.0, -0.9, -0.4, 0.0, 0.2).finished();
  Vec6 z1_pose_b_ = (Vec6() << 0.0, 1.3, -1.1, -0.7, 0.0, -0.2).finished();
#endif
};

void Custom::Init() {
  InitLowCmd();

  lowcmd_publisher = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
  lowstate_subscriber = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, [this](const unitree_go::msg::LowState::SharedPtr msg) {
        LowStateMessageHandler(msg);
      });
  
  while(queryMotionStatus()) {
      std::cout << "Try to deactivate the motion control-related service." << std::endl;
      int32_t ret = msc_.ReleaseMode(); 
      if (ret == 0) {
          std::cout << "ReleaseMode succeeded." << std::endl;
      } else {
          std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
      }
     std::this_thread::sleep_for(std::chrono::seconds(5));
  }
#ifdef WITH_Z1_SDK
  // [修改] 在原 B2 初始化流程末尾接入 Z1 初始化
  InitZ1();
#endif
}

void Custom::InitLowCmd() {
  low_cmd_.head[0] = 0xFE;
  low_cmd_.head[1] = 0xEF;
  low_cmd_.level_flag = 0xFF;
  low_cmd_.gpio = 0;

  for (int i = 0; i < 20; i++) {
    low_cmd_.motor_cmd[i].mode = (0x0A);  // motor switch to servo (PMSM) mode
    low_cmd_.motor_cmd[i].q = (PosStopF);
    low_cmd_.motor_cmd[i].kp = (0);
    low_cmd_.motor_cmd[i].dq = (VelStopF);
    low_cmd_.motor_cmd[i].kd = (0);
    low_cmd_.motor_cmd[i].tau = (0);
  }
}

int Custom::queryMotionStatus()
{
    std::string robotForm, motionName;
    int motionStatus;
    int32_t ret = msc_.CheckMode(robotForm, motionName);
    if (ret == 0) {
        std::cout << "CheckMode succeeded." << std::endl;
    } else {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }

    if(motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = queryServiceName(robotForm, motionName);
        std::cout << "Service: "<< serviceName<< " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

std::string Custom::queryServiceName(std::string form, std::string name)
{
    if(form == "0")
    {
        if(name == "normal" ) return "sport_mode"; 
        if(name == "ai" ) return "ai_sport"; 
        if(name == "advanced" ) return "advanced_sport"; 
    }
    else
    {
        if(name == "ai-w" ) return "wheeled_sport(go2W)"; 
        if(name == "normal-w" ) return "wheeled_sport(b2W)";
    }
    return "";
}

void Custom::Start() {
  /*loop publishing thread*/
  lowCmdWriteThreadPtr = this->create_wall_timer(std::chrono::milliseconds(2), [this] {
    LowCmdWrite();
  });
}

void Custom::LowStateMessageHandler(
    const unitree_go::msg::LowState::SharedPtr msg) {
  low_state_ = *msg;
}

void Custom::LowCmdWrite() {
  if (percent_5_ < 1) {
    std::cout << "Read sensor data example: " << std::endl;
    std::cout << "Joint 0 pos: " << low_state_.motor_state[0].q << std::endl;
    std::cout << "Imu accelerometer : " << "x: "
              << low_state_.imu_state.accelerometer[0]
              << " y: " << low_state_.imu_state.accelerometer[1]
              << " z: " << low_state_.imu_state.accelerometer[2] << std::endl;
    std::cout << "Foot force " << low_state_.foot_force[0] << std::endl;
    std::cout << std::endl;
  }
  if ((percent_5_ == 1) && (!done_)) {
    std::cout << "The example is done! " << std::endl;
    std::cout << std::endl;
    done_ = true;
  }

  motiontime_++;
  if (motiontime_ >= 500) {
    if (first_run_) {
      for (int i = 0; i < 12; i++) {
        start_pos_[i] = low_state_.motor_state[i].q;
      }
      first_run_ = false;
    }

    percent_1_ += static_cast<float>(1) / duration_1_;
    percent_1_ = percent_1_ > 1 ? 1 : percent_1_;
    if (percent_1_ < 1) {
      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - percent_1_) * start_pos_[j] + percent_1_ * target_pos_1_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }
    if ((percent_1_ == 1) && (percent_2_ < 1)) {
      percent_2_ += static_cast<float>(1) / duration_2_;
      percent_2_ = percent_2_ > 1 ? 1 : percent_2_;

      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - percent_2_) * target_pos_1_[j] + percent_2_ * target_pos_2_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }

    if ((percent_1_ == 1) && (percent_2_ == 1) && (percent_3_ < 1)) {
      percent_3_ += static_cast<float>(1) / duration_3_;
      percent_3_ = percent_3_ > 1 ? 1 : percent_3_;

      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q = target_pos_2_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }
    
    if ((percent_1_ == 1) && (percent_2_ == 1) && (percent_3_ == 1) && (percent_4_ < 1)) {
      percent_4_ += static_cast<float>(1) / duration_4_;
      percent_4_ = percent_4_ > 1 ? 1 : percent_4_;
      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - percent_4_) * target_pos_2_[j] + percent_4_ * target_pos_3_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }

    if ((percent_1_ == 1) && (percent_2_ == 1) && (percent_3_ == 1) && (percent_4_ == 1) && ((percent_5_ <= 1))) {
      percent_5_ += static_cast<float>(1) / duration_5_;
      percent_5_ = percent_5_ > 1 ? 1 : percent_5_;
      for (int j = 0; j < 12; j++) {
        low_cmd_.motor_cmd[j].q =
            (1 - percent_5_) * target_pos_3_[j] + percent_5_ * target_pos_4_[j];
        low_cmd_.motor_cmd[j].dq = 0;
        low_cmd_.motor_cmd[j].kp = kp_;
        low_cmd_.motor_cmd[j].kd = kd_;
        low_cmd_.motor_cmd[j].tau = 0;
      }
    }
    
    get_crc(low_cmd_); 
    
    lowcmd_publisher->publish(low_cmd_);
#ifdef WITH_Z1_SDK
    // [修改] 在原 B2 lowcmd 发布后，同步下发 Z1 控制命令
    SyncZ1Motion();
#endif
  }
}

#ifdef WITH_Z1_SDK
// [新增] Z1 初始化：建链路、切状态机到 LOWCMD、读取初始关节角
// [官方参考] z1_sdk/examples/lowcmd_development.cpp (sendRecvThread/start, backToStart, setFsm, setControlGain, getQ)
void Custom::InitZ1() {
  try {
    z1_arm_ = std::make_unique<UNITREE_ARM::unitreeArm>(z1_has_gripper_);
    z1_arm_->sendRecvThread->start();
    z1_arm_->backToStart();
    z1_arm_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    z1_arm_->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);

    std::vector<double> kp = z1_arm_->_ctrlComp->lowcmd->kp;
    std::vector<double> kd = z1_arm_->_ctrlComp->lowcmd->kd;
    z1_arm_->_ctrlComp->lowcmd->setControlGain(kp, kd);
    z1_arm_->sendRecvThread->shutdown();

    z1_init_q_ = z1_arm_->lowstate->getQ();
    z1_ready_ = true;
    std::cout << "Z1 controller initialized and switched to LOWCMD." << std::endl;
  } catch (const std::exception& e) {
    z1_ready_ = false;
    std::cout << "Failed to initialize Z1 controller: " << e.what() << std::endl;
  } catch (...) {
    z1_ready_ = false;
    std::cout << "Failed to initialize Z1 controller: unknown error." << std::endl;
  }
}

// [新增] Z1 联动轨迹：按 B2 percent_1~5 分段同步机械臂和夹爪
// [官方参考] z1_sdk/examples/lowcmd_development.cpp (线性插值 q/qd + inverseDynamics + setArmCmd/setGripperCmd/sendRecv)
void Custom::SyncZ1Motion() {
  if (!z1_ready_ || !z1_arm_) {
    return;
  }

  Vec6 q_des = z1_init_q_;
  Vec6 qd_des = Vec6::Zero();
  Vec6 tau_des = Vec6::Zero();

  const double seg_1 = static_cast<double>(duration_1_) * static_cast<double>(dt_);
  const double seg_2 = static_cast<double>(duration_2_) * static_cast<double>(dt_);
  const double seg_4 = static_cast<double>(duration_4_) * static_cast<double>(dt_);
  const double seg_5 = static_cast<double>(duration_5_) * static_cast<double>(dt_);

  if (percent_1_ < 1.0f) {
    const double r = static_cast<double>(percent_1_);
    q_des = z1_init_q_ * (1.0 - r) + z1_pose_a_ * r;
    qd_des = (z1_pose_a_ - z1_init_q_) / seg_1;
  } else if (percent_2_ < 1.0f) {
    const double r = static_cast<double>(percent_2_);
    q_des = z1_pose_a_ * (1.0 - r) + z1_pose_b_ * r;
    qd_des = (z1_pose_b_ - z1_pose_a_) / seg_2;
  } else if (percent_3_ < 1.0f) {
    q_des = z1_pose_b_;
    qd_des.setZero();
  } else if (percent_4_ < 1.0f) {
    const double r = static_cast<double>(percent_4_);
    q_des = z1_pose_b_ * (1.0 - r) + z1_pose_a_ * r;
    qd_des = (z1_pose_a_ - z1_pose_b_) / seg_4;
  } else {
    const double r = static_cast<double>(percent_5_);
    q_des = z1_pose_a_ * (1.0 - r) + z1_init_q_ * r;
    qd_des = (z1_init_q_ - z1_pose_a_) / seg_5;
  }

  tau_des = z1_arm_->_ctrlComp->armModel->inverseDynamics(
      q_des, qd_des, Vec6::Zero(), Vec6::Zero());

  z1_arm_->setArmCmd(q_des, qd_des, tau_des);

  if (z1_has_gripper_) {
    const double p = std::clamp(
        static_cast<double>(percent_1_ + percent_2_ + percent_3_ + percent_4_ + percent_5_) / 5.0,
        0.0, 1.0);
    const double gripper_q = -p;
    z1_arm_->setGripperCmd(gripper_q, 0.0, 0.0);
  }

  z1_arm_->sendRecv();
}

// [新增] Z1 退出收尾：回初始位并切回 PASSIVE
// [官方参考] z1_sdk/examples/lowcmd_development.cpp (JOINTCTRL -> backToStart -> PASSIVE)
void Custom::ShutdownZ1() {
  if (!z1_ready_ || !z1_arm_) {
    return;
  }
  try {
    z1_arm_->sendRecvThread->start();
    z1_arm_->setFsm(UNITREE_ARM::ArmFSMState::JOINTCTRL);
    z1_arm_->backToStart();
    z1_arm_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    z1_arm_->sendRecvThread->shutdown();
  } catch (...) {
    // ignore shutdown exceptions to avoid blocking process exit
  }
  z1_ready_ = false;
}
#endif

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Custom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
