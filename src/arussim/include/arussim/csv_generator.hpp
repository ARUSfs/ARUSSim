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
#pragma once

/**
 * @class CSVGenerator
 * 
 * @brief Class to generate CSV files for logging data.
 */
class CSVGenerator
{
public:
    CSVGenerator(const std::string &csv_mode)
    {
        std::cout << "Initializing CSVGenerator" << std::endl;
    
        std::string home_dir = std::string(std::getenv("HOME"));
        std::filesystem::path csv_dir = std::filesystem::path(home_dir) / "ARUS_logs" / "csv";
        
        if (!std::filesystem::exists(csv_dir)) {
            std::filesystem::create_directories(csv_dir);
            std::cout << "CSV directory created: " << csv_dir << std::endl;
        } else {
            std::cout << "CSV directory already exists: " << csv_dir << std::endl;
        }
    
        time_t now_ = time(nullptr);
        tm *time_info = localtime(&now_);
    
        int year = time_info->tm_year + 1900;
        int month = time_info->tm_mon + 1;
        int day = time_info->tm_mday;
        int hour = time_info->tm_hour;
        int minute = time_info->tm_min;
        int second = time_info->tm_sec;
    
        std::string filename = (csv_dir / (csv_mode + "_" 
                                + std::to_string(day) + "-" 
                                + std::to_string(month) + "-" 
                                + std::to_string(year) + "_"
                                + std::to_string(hour) + ":"
                                + std::to_string(minute) + ":"
                                + std::to_string(second)
                                + ".csv")).string();
    
        out_file_.open(filename);
        if (out_file_.is_open()) {
            std::cout << "CSV file created: " << filename << std::endl;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create CSV file: %s", filename.c_str());
        }
    }

    /**
     * @brief Algorithm to write a row in the CSV file.
     * 
     * @param first_row First row of the CSV file.
     * @param values Rest of the values to be written in each row.
     */
    void write_row(const std::string &first_row, const std::vector<std::string> &values)
    {
        if (!out_file_.is_open()) {
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

    std::ofstream out_file_;
    bool header_written_ = false;
};