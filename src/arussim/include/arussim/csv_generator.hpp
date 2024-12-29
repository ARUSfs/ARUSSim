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
    
        // Ensure the csv directory exists
        if (!std::filesystem::exists("ARUSSim/src/arussim/resources/csv")) {
            std::filesystem::create_directories("ARUSSim/src/arussim/resources/csv");
            std::cout << "CSV directory created" << std::endl;
        } else {
            std::cout << "CSV directory already exists" << std::endl;
        }
    
        time_t now = time(nullptr);
        tm *time_info = localtime(&now);
    
        int year = time_info->tm_year + 1900;
        int month = time_info->tm_mon + 1;
        int day = time_info->tm_mday;
        int hour = time_info->tm_hour;
        int minute = time_info->tm_min;
        int second = time_info->tm_sec;
    
        std::string filename = "ARUSSim/src/arussim/resources/csv/" 
                                + csv_mode_ + "_"
                                + std::to_string(day) + "-" 
                                + std::to_string(month) + "-" 
                                + std::to_string(year) + "_"
                                + std::to_string(hour) + ":"
                                + std::to_string(minute) + ":"
                                + std::to_string(second)
                                + ".csv";
    
        out_file_.open(filename);
        if (out_file_.is_open()) {
            std::cout << "CSV file created: " << filename << std::endl;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create CSV file: %s", filename.c_str());
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
    
        // Registrar la escritura de una fila
        std::cout << "Row written to CSV." << std::endl;
    }

private:
    std::ofstream out_file_;
    bool header_written_ = false;
    std::string csv_mode_;
};