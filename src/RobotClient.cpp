#include "RobotClient.h"

namespace mc_kuka_fri
{

RobotClient::RobotClient(AppState & state, const std::string & name)
: state_(state), app_(connection_, *this), name_(name), torques_measured_(7, 0.0), joints_measured_(7, 0.0),
  torques_command_(7, 0.0), joints_command_(7, 0.0), vel_estimated_(7, 0.0), joints_measured_prev_(7, 0.0)
{
  begin = std::chrono::steady_clock::now();
  auto config = state.gc.configuration().config("KukaFRI")(name);
  std::string host = config("host");
  int port = config("port", 30200);
  if(!app_.connect(port, host.c_str()))
  {
    mc_rtc::log::error_and_throw("[MCKukaFRI] Connection to {}:{} failed", host, port);
  }
  else { mc_rtc::log::success("[MCKukaFRI] Connection to {}:{} OK!", host, port); }
  updateMcRtcInputs();
}

void RobotClient::waitForCommand()
{
  LBRClient::waitForCommand();

  std::lock_guard<std::mutex> lck(state_.gc_mutex);
  updateMcRtcInputs();
  if(robotState().getClientCommandMode() == KUKA::FRI::TORQUE) { robotCommand().setTorque(torques_command_.data()); }
}

void RobotClient::updateMcRtcInputs()
{
  joints_measured_prev_ = joints_measured_;
  const auto & state = robotState();
  std::memcpy(joints_measured_.data(), state.getMeasuredJointPosition(), 7 * sizeof(double));
  std::memcpy(torques_measured_.data(), state.getMeasuredTorque(), 7 * sizeof(double));
  state_.gc.setEncoderValues(name_, joints_measured_);
  state_.gc.setJointTorques(name_, torques_measured_);

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000000.0;
  mc_rtc::log::info("[RobotClient] loop duration {}", duration);
  for(size_t i = 0; i < 7; i++)
  {
    if(duration < 0.001 || duration > 0.9)
    { // avoid inf value at the start
      vel_estimated_[i] = 0;
    }
    else
    {
      vel_estimated_[i] =
          0.2 * (joints_measured_[i] - joints_measured_prev_[i]) / (duration) + (1 - 0.2) * vel_estimated_[i];
    }
    begin = std::chrono::steady_clock::now();
  }
  state_.gc.setEncoderVelocities(name_, vel_estimated_);
}

void RobotClient::command()
{
  LBRClient::command();

  std::lock_guard<std::mutex> lck(state_.gc_mutex);
  updateKukaCommand();
}

void RobotClient::updateKukaCommand()
{
  const auto & robot = state_.gc.robot(name_);
  for(size_t i = 0; i < 7; ++i)
  {
    auto mbcIdx = robot.jointIndexInMBC(i);
    torques_command_[i] = robot.jointTorque()[mbcIdx][0];
    joints_command_[i] = robot.q()[mbcIdx][0];
  }
  robotCommand().setJointPosition(joints_command_.data());
  if(robotState().getClientCommandMode() == KUKA::FRI::TORQUE) { robotCommand().setTorque(torques_command_.data()); }
}

void RobotClient::startControlThread()
{
  control_thread_ = std::thread(
      [this]()
      {
        bool ok = true;
        while(ok) { ok = app_.step(); }
      });
}

void RobotClient::joinControlThread()
{
  if(!control_thread_.joinable()) { return; }
  control_thread_.join();
  app_.disconnect();
}

void MainRobotClient::command()
{
  LBRClient::command();

  std::lock_guard<std::mutex> lck(state_.gc_mutex);
  updateMcRtcInputs(); // update measured encoder positions and joint torques
  state_.gc.run();
  updateKukaCommand();
}

} // namespace mc_kuka_fri
