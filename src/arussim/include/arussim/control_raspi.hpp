#pragma once

#include <rclcpp/rclcpp.hpp> // Necesario para rclcpp::Logger
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <thread>
#include <atomic>
#include <optional>
#include <string>

namespace fs = std::filesystem;

namespace control_raspi
{
  namespace
  {
    // Relative path from the ControlRaspi executable to the Control-RaspPi executable
    // Change this path if doesn't match your workspace structure
    constexpr const char *kControlRaspRelativePath = "../../../../../ws_raspi/src/Control-RaspPi/build/ControlRaspi";

    /**
     * @brief Get the directory of the current executable
     *
     */
    inline std::string get_executable_dir()
    {
      std::error_code error;
      fs::path exe_path = fs::read_symlink("/proc/self/exe", error);
      if (error)
      {
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
      explicit StoppingGuard(std::atomic<bool> &stopping) : stopping_(stopping) {}
      ~StoppingGuard() { stopping_.store(false); }

    private:
      std::atomic<bool> &stopping_;
    };
  }

  class ControlRaspi
  {
  public:
    ControlRaspi() = default;

    ~ControlRaspi()
    {
      if (control_rasp_pid_.has_value())
      {
        kill(control_rasp_pid_.value(), SIGKILL);
      }
    }

    void start_control_rasp(rclcpp::Logger logger)
    {
      if (control_rasp_pid_.has_value())
      {
        int status = 0;
        pid_t wait_result = waitpid(control_rasp_pid_.value(), &status, WNOHANG);
        if (wait_result == 0)
        {
          RCLCPP_INFO(logger, "ControlRaspi already running (pid=%d)", static_cast<int>(control_rasp_pid_.value()));
          return;
        }
        control_rasp_pid_.reset();
      }

      const std::string control_rasp_path = resolve_control_rasp_path(logger);
      if (control_rasp_path.empty())
      {
        RCLCPP_ERROR(logger, "Failed to resolve ControlRaspi path");
        return;
      }

      const std::string control_rasp_working_dir = resolve_control_rasp_working_dir(control_rasp_path, logger);
      if (control_rasp_working_dir.empty())
      {
        RCLCPP_ERROR(logger, "Failed to resolve ControlRaspi working directory");
        return;
      }

      RCLCPP_INFO(logger, "Starting ControlRaspi: %s", control_rasp_path.c_str());
      pid_t pid = fork();
      if (pid < 0)
      {
        RCLCPP_ERROR(logger, "fork() failed: %s", std::strerror(errno));
        return;
      }

      if (pid == 0)
      {
        if (chdir(control_rasp_working_dir.c_str()) != 0)
        {
          std::fprintf(stderr, "chdir() failed: %s\n", std::strerror(errno));
          _exit(127);
        }
        const char *argv[] = {control_rasp_path.c_str(), nullptr};
        execv(argv[0], const_cast<char *const *>(argv));
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
     *
     * IMPORTANT: Sometimes Control-RaspPi doesn't stop instantly, just wait or spam Ctrl+C
     */
    void stop_control_rasp(rclcpp::Logger logger)
    {
      if (stopping_.exchange(true))
      {
        return;
      }

      StoppingGuard stopping_guard(stopping_);

      if (!control_rasp_pid_.has_value())
      {
        return;
      }

      const pid_t pid = control_rasp_pid_.value();

      auto try_wait_for_exit = [&](std::chrono::milliseconds timeout) -> bool
      {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
          int status = 0;
          pid_t wait_result = waitpid(pid, &status, WNOHANG);
          if (wait_result == pid)
          {
            return true;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return false;
      };

      RCLCPP_INFO(logger, "Stopping ControlRaspi (pid=%d)", static_cast<int>(pid));

      (void)kill(pid, SIGINT);
      if (try_wait_for_exit(std::chrono::milliseconds(1500)))
      {
        control_rasp_pid_.reset();
        return;
      }

      (void)kill(pid, SIGTERM);
      if (try_wait_for_exit(std::chrono::milliseconds(1500)))
      {
        control_rasp_pid_.reset();
        return;
      }

      (void)kill(pid, SIGKILL);
      (void)try_wait_for_exit(std::chrono::milliseconds(1500));
      control_rasp_pid_.reset();
    }

  private:
    /**
     * @brief Get the directory of the current executable
     *
     * Not found or Not executable are related to a feilure to resolve the relative path
     *
     */
    std::string resolve_control_rasp_path(rclcpp::Logger logger) const
    {
      const std::string exe_dir = get_executable_dir();
      if (exe_dir.empty())
        return std::string{};

      std::error_code error;
      fs::path candidate = fs::path(exe_dir) / kControlRaspRelativePath;
      fs::path normalized = fs::weakly_canonical(candidate, error);
      if (error)
        normalized = candidate;

      if (!fs::exists(normalized))
      {
        RCLCPP_ERROR(logger, "ControlRaspi not found: %s", normalized.string().c_str());
        return std::string{};
      }

      if (access(normalized.c_str(), X_OK) != 0)
      {
        RCLCPP_ERROR(logger, "ControlRaspi not executable: %s", normalized.string().c_str());
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
    std::string resolve_control_rasp_working_dir(const std::string &control_rasp_path, rclcpp::Logger logger) const
    {
      std::error_code error;
      fs::path control_rasp = fs::path(control_rasp_path);
      fs::path build_dir = control_rasp.parent_path();
      fs::path repo_dir = build_dir.parent_path();

      fs::path canonical_repo_dir = fs::weakly_canonical(repo_dir, error);
      if (!error)
      {
        repo_dir = canonical_repo_dir;
      }

      if (!fs::exists(repo_dir))
      {
        RCLCPP_ERROR(logger, "Control-RaspPi repo dir not found: %s", repo_dir.string().c_str());
        return std::string{};
      }

      fs::path parameters_yaml = repo_dir / "config" / "parameters.yaml";
      if (!fs::exists(parameters_yaml))
      {
        RCLCPP_ERROR(logger, "parameters.yaml not found at: %s", parameters_yaml.string().c_str());
        return std::string{};
      }

      return repo_dir.string();
    }

    std::optional<pid_t> control_rasp_pid_;
    std::atomic<bool> stopping_{false};
  };

} // namespace control_raspi

/*#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sys/types.h>

#include <atomic>
#include <optional>
#include <string>

namespace control_raspi
{

  class ControlRaspi : public rclcpp::Node
  {
  public:
    explicit ControlRaspi(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void launch_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void reset_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void start_control_rasp();
    void stop_control_rasp();

    std::string resolve_control_rasp_path() const;
    std::string resolve_control_rasp_working_dir(const std::string &control_rasp_path) const;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr launch_raspi_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_raspi_;

    std::optional<pid_t> control_rasp_pid_;
    std::atomic<bool> stopping_{false};
  };

} // namespace control_raspi
*/