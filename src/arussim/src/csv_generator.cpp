/**
 * @file csv_generator.cpp
 * @author Rafael Guil Valero (rafaguilvalero@gmail.com)
 * @version 0.1
 * @date 2024-12-29
 * 
 */
#include "arussim/csv_generator.hpp"
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Constructor for the CSVGenerator class
 * 
 */
CSVGenerator::CSVGenerator()
{
    // Registrar la creación del CSVGenerator
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inicializando CSVGenerator");

    // Ensure the csv directory exists
    if (!std::filesystem::exists("ARUSSim/src/arussim/resources/csv")) {
        std::filesystem::create_directories("ARUSSim/src/arussim/resources/csv");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Directorio csv creado");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Directorio csv ya existe");
    }

    time_t now = time(nullptr);
    tm *time_info = localtime(&now);

    int year = time_info->tm_year + 1900;
    int month = time_info->tm_mon + 1;
    int day = time_info->tm_mday;
    int hour = time_info->tm_hour;
    int minute = time_info->tm_min;
    int second = time_info->tm_sec;

    std::string filename = "ARUSSim/src/arussim/resources/csv/csv_" + std::to_string(day) + "-" 
                            + std::to_string(month) + "-" 
                            + std::to_string(year) + "_"
                            + std::to_string(hour) + "-"
                            + std::to_string(minute) + "-"
                            + std::to_string(second)
                            + ".csv";

    out_file_.open(filename);
    if (out_file_.is_open()) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Archivo CSV creado: %s", filename.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No se pudo crear el archivo CSV: %s", filename.c_str());
    }
    header_written_ = false;
}

/**
 * @brief CSV generator
 * 
 * @param mode param to specify the first row of the csv
 * @param values params to write in the csv
 */
void CSVGenerator::write_row(std::string mode, const std::vector<std::string> &values)
{
    if (mode == "supervisor" && !header_written_) {
        out_file_ << "time per lap,hit_conos_acumulados\n";
        header_written_ = true;
    }

    if (!out_file_.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "El archivo CSV no está abierto para escritura.");
        return;
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fila escrita en CSV.");
}

