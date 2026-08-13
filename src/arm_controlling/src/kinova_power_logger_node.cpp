#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "BaseCyclicClientRpc.h"
#include "RouterClient.h"
#include "SessionManager.h"
#include "TransportClientUdp.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace k_api = Kinova::Api;
using namespace std::chrono_literals;

struct PowerSample
{
  double voltage_v;
  double current_a;
  double power_w;
};

class KinovaPowerLogger : public rclcpp::Node
{
public:
  KinovaPowerLogger()
  : Node("kinova_power_logger"), state_start_time_(now())
  {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.1.10");
    username_ = declare_parameter<std::string>("username", "admin");
    password_ = declare_parameter<std::string>("password", "admin");
    log_path_ = declare_parameter<std::string>(
      "log_path", "/home/thiwa/CEAbot/power_logs/arm_power_log.csv");
    sample_period_s_ = declare_parameter<double>("sample_period_s", 1.0);
    reconnect_period_s_ = declare_parameter<double>("reconnect_period_s", 10.0);

    if (sample_period_s_ <= 0.0) {
      throw std::invalid_argument("sample_period_s must be greater than zero");
    }

    initialize_log();

    state_subscription_ = create_subscription<std_msgs::msg::String>(
      "/auto_state", rclcpp::QoS(10).transient_local(),
      std::bind(&KinovaPowerLogger::state_callback, this, std::placeholders::_1));
    power_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("/arm_power", 10);
    sample_timer_ = create_wall_timer(
      std::chrono::duration<double>(sample_period_s_),
      std::bind(&KinovaPowerLogger::sample_power, this));

    RCLCPP_INFO(
      get_logger(), "Logging Kinova arm power by /auto_state to %s", log_path_.c_str());
  }

  ~KinovaPowerLogger() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    save_state_locked(auto_state_);
    disconnect_locked();
  }

private:
  void initialize_log()
  {
    const std::filesystem::path path(log_path_);
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }

    if (!std::filesystem::exists(path) || std::filesystem::file_size(path) == 0) {
      std::ofstream file(log_path_, std::ios::app);
      if (!file) {
        throw std::runtime_error("Could not open power log: " + log_path_);
      }
      file << "timestamp,auto_state,duration_s,sample_count,avg_arm_voltage_v,"
              "avg_arm_current_a,avg_arm_power_w,min_arm_power_w,max_arm_power_w,energy_wh\n";
    }
  }

  bool connect_locked()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (last_connect_attempt_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(steady_now - last_connect_attempt_).count() < reconnect_period_s_)
    {
      return false;
    }
    last_connect_attempt_ = steady_now;

    try {
      transport_ = std::make_unique<k_api::TransportClientUdp>();
      if (!transport_->connect(robot_ip_, 10001)) {
        throw std::runtime_error("UDP connection failed");
      }

      router_ = std::make_unique<k_api::RouterClient>(
        transport_.get(), [this](k_api::KError error) {
          RCLCPP_ERROR(this->get_logger(), "Kortex router error: %s", error.toString().c_str());
        });
      session_ = std::make_unique<k_api::SessionManager>(router_.get());

      k_api::Session::CreateSessionInfo session_info;
      session_info.set_username(username_);
      session_info.set_password(password_);
      session_info.set_session_inactivity_timeout(60000);
      session_info.set_connection_inactivity_timeout(2000);
      session_->CreateSession(session_info);
      cyclic_client_ = std::make_unique<k_api::BaseCyclic::BaseCyclicClient>(router_.get());

      connected_ = true;
      connection_error_reported_ = false;
      RCLCPP_INFO(get_logger(), "Connected to Kinova cyclic feedback at %s:10001", robot_ip_.c_str());
      return true;
    } catch (const std::exception & error) {
      disconnect_locked();
      if (!connection_error_reported_) {
        RCLCPP_WARN(get_logger(), "Kinova power connection unavailable: %s", error.what());
        connection_error_reported_ = true;
      }
      return false;
    }
  }

  void disconnect_locked()
  {
    cyclic_client_.reset();
    if (session_) {
      try {
        session_->CloseSession();
      } catch (const std::exception &) {
      }
    }
    session_.reset();
    router_.reset();
    if (transport_) {
      transport_->disconnect();
    }
    transport_.reset();
    connected_ = false;
  }

  void state_callback(const std_msgs::msg::String::SharedPtr message)
  {
    std::string next_state = message->data;
    next_state.erase(next_state.begin(), std::find_if(
      next_state.begin(), next_state.end(), [](unsigned char c) {return !std::isspace(c);}));
    next_state.erase(std::find_if(
      next_state.rbegin(), next_state.rend(), [](unsigned char c) {return !std::isspace(c);}).base(),
      next_state.end());
    std::transform(
      next_state.begin(), next_state.end(), next_state.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    if (next_state.empty()) {
      next_state = "unknown";
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (next_state == auto_state_) {
      return;
    }
    save_state_locked(auto_state_);
    auto_state_ = next_state;
  }

  void sample_power()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_ && !connect_locked()) {
      return;
    }

    try {
      const auto feedback = cyclic_client_->RefreshFeedback();
      const double voltage_v = feedback.base().arm_voltage();
      const double current_a = feedback.base().arm_current();
      const double power_w = voltage_v * current_a;

      if (!std::isfinite(voltage_v) || !std::isfinite(current_a) || !std::isfinite(power_w)) {
        RCLCPP_WARN(get_logger(), "Discarding non-finite Kinova power feedback");
        return;
      }

      samples_.push_back({voltage_v, current_a, power_w});

      std_msgs::msg::Float32MultiArray message;
      // Same convention as /drive_power: power first, followed by voltage and current.
      message.data = {
        static_cast<float>(power_w), static_cast<float>(voltage_v), static_cast<float>(current_a)};
      power_publisher_->publish(message);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Could not read Kinova power feedback: %s", error.what());
      disconnect_locked();
    }
  }

  static std::string wall_timestamp()
  {
    const auto system_now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(system_now);
    std::tm local_time{};
    localtime_r(&time, &local_time);
    std::ostringstream output;
    output << std::put_time(&local_time, "%Y-%m-%dT%H:%M:%S");
    return output.str();
  }

  void save_state_locked(const std::string & state)
  {
    const auto end_time = now();
    if (samples_.empty()) {
      state_start_time_ = end_time;
      return;
    }

    double voltage_sum = 0.0;
    double current_sum = 0.0;
    double power_sum = 0.0;
    double min_power = std::numeric_limits<double>::infinity();
    double max_power = -std::numeric_limits<double>::infinity();
    for (const auto & sample : samples_) {
      voltage_sum += sample.voltage_v;
      current_sum += sample.current_a;
      power_sum += sample.power_w;
      min_power = std::min(min_power, sample.power_w);
      max_power = std::max(max_power, sample.power_w);
    }

    const double count = static_cast<double>(samples_.size());
    const double duration_s = (end_time - state_start_time_).seconds();
    const double average_power_w = power_sum / count;
    const double energy_wh = average_power_w * duration_s / 3600.0;

    std::ofstream file(log_path_, std::ios::app);
    if (!file) {
      RCLCPP_ERROR(get_logger(), "Could not append to %s", log_path_.c_str());
      return;
    }
    file << wall_timestamp() << ',' << state << ',' << std::fixed << std::setprecision(3)
         << duration_s << ',' << samples_.size() << ',' << voltage_sum / count << ','
         << current_sum / count << ',' << average_power_w << ',' << min_power << ','
         << max_power << ',' << std::setprecision(6) << energy_wh << '\n';
    file.flush();

    samples_.clear();
    state_start_time_ = end_time;
  }

  std::mutex mutex_;
  std::string robot_ip_;
  std::string username_;
  std::string password_;
  std::string log_path_;
  double sample_period_s_;
  double reconnect_period_s_;
  std::string auto_state_{"manual"};
  rclcpp::Time state_start_time_;
  std::vector<PowerSample> samples_;
  bool connected_{false};
  bool connection_error_reported_{false};
  std::chrono::steady_clock::time_point last_connect_attempt_{};

  std::unique_ptr<k_api::TransportClientUdp> transport_;
  std::unique_ptr<k_api::RouterClient> router_;
  std::unique_ptr<k_api::SessionManager> session_;
  std::unique_ptr<k_api::BaseCyclic::BaseCyclicClient> cyclic_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr power_publisher_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KinovaPowerLogger>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("kinova_power_logger"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
