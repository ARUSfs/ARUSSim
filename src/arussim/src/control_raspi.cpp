#include <arussim/control_raspi.hpp>

#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <thread>

#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace control_raspi
{

namespace
{
constexpr const char * kLaunchTopic = "/arussim/launch";
constexpr const char * kResetTopic = "/arussim/reset";

// Relative path from the ControlRaspi executable to the Control-RaspPi executable
// Change this path if doesn't match your workspace structure
constexpr const char * kControlRaspRelativePath = "../../../../../ws_raspi/src/Control-RaspPi/build/ControlRaspi";

/**
 * @brief Get the directory of the current executable
 * 
 */
std::string get_executable_dir()
{
  std::error_code error;
  fs::path exe_path = fs::read_symlink("/proc/self/exe", error);
  if (error) {
    return std::string{};
  }
  return exe_path.parent_path().string();
}

/**
 * @brief Guard class to reset the stopping flag when going out of scope
 * 
 */
class StoppingGuard
{
public:
  explicit StoppingGuard(std::atomic<bool> & stopping) : stopping_(stopping) {}
  ~StoppingGuard() { stopping_.store(false); }

private:
  std::atomic<bool> & stopping_;
};

}  // namespace

/**
 * @brief ControlRaspi class that manages the Control-RaspPi process
 * 
 * It listens to launch and reset topics to start and stop the Control-RaspPi process accordingly.
 */
ControlRaspi::ControlRaspi(const rclcpp::NodeOptions & options)
: rclcpp::Node("control_raspi", options)
{
  launch_patata_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    kLaunchTopic,
    rclcpp::QoS(1),
    std::bind(&ControlRaspi::launch_callback, this, std::placeholders::_1));

  reset_patata_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    kResetTopic,
    rclcpp::QoS(1),
    std::bind(&ControlRaspi::reset_callback, this, std::placeholders::_1));
}

/**
 * @brief Callback function for the launch topic
 * 
 */
void ControlRaspi::launch_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg || !msg->data) {
    return;
  }
  start_control_rasp();
}

/**
 * @brief Callback function for the reset topic
 * 
 */
void ControlRaspi::reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg || !msg->data) {
    return;
  }
  stop_control_rasp();
}


/**
 * @brief Start the Control-RaspPi process
 * 
 * This function checks if the process is already running, resolves the path to the Control-RaspPi
 * executable and its working directory, and then starts the process using fork and exec.
 * 
 * It also handles errors and logs appropriate messages.
 */
void ControlRaspi::start_control_rasp()
{
  if (control_rasp_pid_.has_value()) {
    int status = 0;
    pid_t wait_result = waitpid(control_rasp_pid_.value(), &status, WNOHANG);
    if (wait_result == 0) {
      RCLCPP_INFO(this->get_logger(), "ControlRaspi already running (pid=%d)", static_cast<int>(control_rasp_pid_.value()));
      return;
    }
    control_rasp_pid_.reset();
  }

  const std::string control_rasp_path = resolve_control_rasp_path();
  if (control_rasp_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to resolve ControlRaspi path");
    return;
  }

  const std::string control_rasp_working_dir = resolve_control_rasp_working_dir(control_rasp_path);
  if (control_rasp_working_dir.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to resolve Control-RaspPi working directory");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Starting ControlRaspi: %s", control_rasp_path.c_str());
  pid_t pid = fork();
  if (pid < 0) {
    RCLCPP_ERROR(this->get_logger(), "fork() failed: %s", std::strerror(errno));
    return;
  }

  if (pid == 0) {
    if (chdir(control_rasp_working_dir.c_str()) != 0) {
      std::fprintf(stderr, "chdir() failed: %s\n", std::strerror(errno));
      _exit(127);
    }
    const char * argv[] = {control_rasp_path.c_str(), nullptr};
    execv(argv[0], const_cast<char * const *>(argv));
    // If exec returns, it failed.
    std::fprintf(stderr, "execv() failed: %s\n", std::strerror(errno));
    _exit(127);
  }

  control_rasp_pid_ = pid;
}

/**
 * @brief Stop the Control-RaspPi process
 * 
 * Firstly tries to stop it by sending SIGINT, then SIGTERM and finally SIGKILL
 * to force stop it if it doesn't stop.
 */
void ControlRaspi::stop_control_rasp()
{
  if (stopping_.exchange(true)) {
    return;
  }

  StoppingGuard stopping_guard(stopping_);

  if (!control_rasp_pid_.has_value()) {
    return;
  }

  const pid_t pid = control_rasp_pid_.value();

  auto try_wait_for_exit = [&](std::chrono::milliseconds timeout) -> bool {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      int status = 0;
      pid_t wait_result = waitpid(pid, &status, WNOHANG);
      if (wait_result == pid) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  };

  RCLCPP_INFO(this->get_logger(), "Stopping ControlRaspi (pid=%d)", static_cast<int>(pid));

  (void)kill(pid, SIGINT);
  if (try_wait_for_exit(std::chrono::milliseconds(1500))) {
    control_rasp_pid_.reset();
    return;
  }

  (void)kill(pid, SIGTERM);
  if (try_wait_for_exit(std::chrono::milliseconds(1500))) {
    control_rasp_pid_.reset();
    return;
  }

  (void)kill(pid, SIGKILL);
  (void)try_wait_for_exit(std::chrono::milliseconds(1500));
  control_rasp_pid_.reset();
}

/**
 * @brief Get the directory of the current executable
 * 
 */
std::string ControlRaspi::resolve_control_rasp_path() const
{
  const std::string exe_dir = get_executable_dir();
  if (exe_dir.empty()) {
    return std::string{};
  }

  std::error_code error;
  fs::path candidate = fs::path(exe_dir) / kControlRaspRelativePath;
  fs::path normalized = fs::weakly_canonical(candidate, error);
  if (error) {
    normalized = candidate;
  }

  if (!fs::exists(normalized)) {
    RCLCPP_ERROR(this->get_logger(), "ControlRaspi not found: %s", normalized.string().c_str());
    return std::string{};
  }

  if (access(normalized.c_str(), X_OK) != 0) {
    RCLCPP_ERROR(this->get_logger(), "ControlRaspi not executable: %s", normalized.string().c_str());
    return std::string{};
  }

  return normalized.string();
}

/**
 * @brief Resolve the working directory for Control-RaspPi
 * 
 * This function assumes that the working directory is the parent directory of the Control-RaspPi executable.
 * It checks if the directory exists and contains a parameters.yaml file to validate it.
 */
std::string ControlRaspi::resolve_control_rasp_working_dir(const std::string & control_rasp_path) const
{
  std::error_code error;
  fs::path control_rasp = fs::path(control_rasp_path);
  fs::path build_dir = control_rasp.parent_path();
  fs::path repo_dir = build_dir.parent_path();

  fs::path canonical_repo_dir = fs::weakly_canonical(repo_dir, error);
  if (!error) {
    repo_dir = canonical_repo_dir;
  }

  if (!fs::exists(repo_dir)) {
    RCLCPP_ERROR(this->get_logger(), "Control-RaspPi repo dir not found: %s", repo_dir.string().c_str());
    return std::string{};
  }

  fs::path parameters_yaml = repo_dir / "config" / "parameters.yaml";
  if (!fs::exists(parameters_yaml)) {
    RCLCPP_ERROR(this->get_logger(), "parameters.yaml not found at: %s", parameters_yaml.string().c_str());
    return std::string{};
  }

  return repo_dir.string();
}

}  // namespace control_raspi

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_raspi::ControlRaspi>());
  rclcpp::shutdown();
  return 0;
}
