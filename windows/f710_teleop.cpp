// f710_teleop — native Windows C++ client (ADR-0017).
//
// Reads a Logitech F710 (XInput mode, back switch = X) via SDL2 and publishes
// joint targets to the Jetson's control container over DDS:
//   subscribe: /joint_states  (sensor_msgs/msg/JointState) — arm feedback
//   publish:   /joint_targets (std_msgs/msg/Float64MultiArray, 6 = pan, lift,
//                              elbow, wrist_flex, wrist_roll, gripper)
// Joint order matches the LeRobot SO-100 config (ADR-0006/0009).
//
// Modes (ADR-0017):
//   joint-step     (default) D-pad L/R selects joint 0..5; left stick Y steps
//                  it; LB/RB = gripper close/open.
//   cartesian-feel START toggles: left X/Y -> pan/lift, right X/Y ->
//                  elbow/wrist, triggers -> gripper.
//   record/replay  X starts/stops recording joint targets; B plays the
//                  recorded trajectory back.
//
// Safety: targets are absolute angles integrated from the last /joint_states;
// until the first message arrives nothing is published. Per-tick deltas are
// clamped; the Jetson bridge enforces the real limits/arm state.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include <SDL.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace {

constexpr int kDoF = 6;
constexpr double kTick = 0.02;            // 50 Hz
constexpr double kStepMax = 0.012;        // rad per tick in step mode
constexpr double kFeelMax = 0.006;        // rad per tick at full stick
constexpr double kGripStep = 1.5;         // % per tick
constexpr double kDeadZone = 0.15;        // stick deadzone
constexpr std::array<double, kDoF> kLimitLo{-170.0, -170.0, -170.0, -170.0, -170.0, 0.0};
constexpr std::array<double, kDoF> kLimitHi{170.0, 170.0, 170.0, 170.0, 170.0, 100.0};

double axis(SDL_GameController* pad, SDL_GameControllerAxis a) {
  double v = SDL_GameControllerGetAxis(pad, a) / 32768.0;
  if (std::abs(v) < kDeadZone) return 0.0;
  return v;
}
bool btn(SDL_GameController* pad, SDL_GameControllerButton b) {
  return SDL_GameControllerGetButton(pad, b) != 0;
}
bool pressed(SDL_GameController* pad, SDL_GameControllerButton b, Uint8* prev) {
  Uint8 now = SDL_GameControllerGetButton(pad, b);
  bool p = now && !*prev;
  *prev = now;
  return p;
}
double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

class F710Teleop : public rclcpp::Node {
 public:
  F710Teleop() : Node("f710_teleop") {
    targets_.fill(0.0);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr m) { on_state(std::move(m)); });
    target_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/joint_targets", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "f710_teleop ready. Modes: joint-step (default) | START = cartesian-feel | "
                "X = record | B = replay. Waiting for /joint_states...");
  }

 private:
  void on_state(const sensor_msgs::msg::JointState::SharedPtr m) {
    if (m->position.size() < kDoF) return;
    if (!have_state_) {
      for (int i = 0; i < kDoF; ++i) targets_[i] = m->position[i];
      have_state_ = true;
      RCLCPP_INFO(get_logger(), "state received — publishing current targets");
    } else {
      for (int i = 0; i < kDoF; ++i) state_[i] = m->position[i];
    }
  }

  void publish_targets() {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.assign(targets_.begin(), targets_.end());
    target_pub_->publish(msg);
    if (recording_) record_.push_back(targets_);
  }

  void tick() {
    if (!pad_) {
      if (SDL_NumJoysticks() > 0) {
        pad_ = SDL_GameControllerOpen(0);
        if (pad_) RCLCPP_INFO(get_logger(), "F710 opened: %s", SDL_GameControllerName(pad_));
      }
      if (!pad_) return;
    }
    SDL_GameControllerUpdate();

    // Mode toggles + record/replay (edge-triggered).
    if (pressed(pad_, SDL_CONTROLLER_BUTTON_START, &prev_start_)) {
      cartesian_ = !cartesian_;
      RCLCPP_INFO(get_logger(), "mode: %s", cartesian_ ? "cartesian-feel" : "joint-step");
    }
    if (pressed(pad_, SDL_CONTROLLER_BUTTON_X, &prev_x_)) {
      recording_ = !recording_;
      if (recording_) {
        record_.clear();
        RCLCPP_INFO(get_logger(), "RECORDING started");
      } else {
        RCLCPP_INFO(get_logger(), "RECORDING stopped — %zu points", record_.size());
      }
    }
    if (pressed(pad_, SDL_CONTROLLER_BUTTON_B, &prev_b_) && !record_.empty()) {
      replaying_ = true;
      replay_i_ = 0;
      RCLCPP_INFO(get_logger(), "REPLAY of %zu points", record_.size());
    }

    if (replaying_) {
      if (replay_i_ >= record_.size()) {
        replaying_ = false;
        RCLCPP_INFO(get_logger(), "replay finished");
      } else {
        targets_ = record_[replay_i_++];
        if (have_state_) publish_targets();
      }
      return;
    }

    if (!have_state_) return;  // never command before knowing where the arm is

    if (cartesian_) {
      targets_[0] += axis(pad_, SDL_CONTROLLER_AXIS_LEFTX) * kFeelMax;
      targets_[1] += -axis(pad_, SDL_CONTROLLER_AXIS_LEFTY) * kFeelMax;
      targets_[2] += axis(pad_, SDL_CONTROLLER_AXIS_RIGHTX) * kFeelMax;
      targets_[4] += -axis(pad_, SDL_CONTROLLER_AXIS_RIGHTY) * kFeelMax;
      targets_[5] += (axis(pad_, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) -
                      axis(pad_, SDL_CONTROLLER_AXIS_TRIGGERLEFT)) * kGripStep;
    } else {
      if (pressed(pad_, SDL_CONTROLLER_BUTTON_DPAD_LEFT, &prev_dl_)) joint_sel_ = (joint_sel_ + 5) % 6;
      if (pressed(pad_, SDL_CONTROLLER_BUTTON_DPAD_RIGHT, &prev_dr_)) joint_sel_ = (joint_sel_ + 1) % 6;
      targets_[joint_sel_] += -axis(pad_, SDL_CONTROLLER_AXIS_LEFTY) * kStepMax;
      if (btn(pad_, SDL_CONTROLLER_BUTTON_LEFTSHOULDER)) targets_[5] -= kGripStep;
      if (btn(pad_, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)) targets_[5] += kGripStep;
    }

    for (int i = 0; i < kDoF; ++i) targets_[i] = clamp(targets_[i], kLimitLo[i], kLimitHi[i]);
    publish_targets();
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  SDL_GameController* pad_ = nullptr;
  bool have_state_ = false;
  std::array<double, kDoF> targets_{};
  std::array<double, kDoF> state_{};

  bool cartesian_ = false;
  int joint_sel_ = 0;
  Uint8 prev_start_ = 0, prev_x_ = 0, prev_b_ = 0, prev_dl_ = 0, prev_dr_ = 0;

  bool recording_ = false;
  std::deque<std::array<double, kDoF>> record_;
  bool replaying_ = false;
  size_t replay_i_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("f710_teleop"), "SDL_Init failed: %s", SDL_GetError());
    return 1;
  }
  auto node = std::make_shared<F710Teleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  SDL_Quit();
  return 0;
}
