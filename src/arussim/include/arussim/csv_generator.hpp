/**
 * @file csv_generator.hpp
 * @author Rafael Guil Valero (rafaguilvalero@gmail.com)
 * @version 0.1
 * @date 2024-12-29
 * 
 */
#include <vector>
#include <string>
#include <fstream>
#include <filesystem>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cstdlib>

/**
 * @class CSVGenerator
 * 
 */
class CSVGenerator
{
public:
    CSVGenerator(const std::string &mode)
    {
        csv_mode_ = mode;
        // Registrar la creación del CSVGenerator
        std::cout << "Initializing CSVGenerator" << std::endl;
    
        home_dir_ = std::string(std::getenv("HOME"));
        csv_dir_ = std::filesystem::path(home_dir_) / "Arus_logs" / "csv";
        
        if (!std::filesystem::exists(csv_dir_)) {
            std::filesystem::create_directories(csv_dir_);
            std::cout << "CSV directory created: " << csv_dir_ << std::endl;
        } else {
            std::cout << "CSV directory already exists: " << csv_dir_ << std::endl;
        }
    
        now_ = time(nullptr);
        tm *time_info = localtime(&now_);
    
        year_ = time_info->tm_year + 1900;
        month_ = time_info->tm_mon + 1;
        day_ = time_info->tm_mday;
        hour_ = time_info->tm_hour;
        minute_ = time_info->tm_min;
        second_ = time_info->tm_sec;
    
        filename_ = (csv_dir_ / (csv_mode_ + "_" 
                                + std::to_string(day_) + "-" 
                                + std::to_string(month_) + "-" 
                                + std::to_string(year_) + "_"
                                + std::to_string(hour_) + ":"
                                + std::to_string(minute_) + ":"
                                + std::to_string(second_)
                                + ".csv")).string();
    
        out_file_.open(filename_);
        if (out_file_.is_open()) {
            std::cout << "CSV file created: " << filename_ << std::endl;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create CSV file: %s", filename_.c_str());
        }
    }

    void write_row(const std::string &first_row, const std::vector<std::string> &values)
    {
        if (!out_file_.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "CSV file is not open for writing.");
            return;
        }
    
        if (!header_written_) {
            out_file_ << first_row << "\n";
            header_written_ = true;
        }
    
        for (size_t i = 0; i < values.size(); i++)
        {
            out_file_ << values[i];
            if (i < values.size() - 1)
            {
                out_file_ << ",";
            }
        }
        out_file_ << "\n";
        out_file_.flush();
    }

private:
    std::filesystem::path csv_dir_;
    std::string home_dir_;

    std::ofstream out_file_;
    bool header_written_ = false;
    std::string csv_mode_;

    time_t now_;

    std::string filename_;
    int year_;
    int month_;
    int day_;
    int hour_;
    int minute_;
    int second_;
};