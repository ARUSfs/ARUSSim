/**
 * @file car_parameters.hpp
 * @brief Load the ART-Parameters CSV of the simulated car. Shared by the simulator and sensors nodes
 */

#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace arussim
{

/**
 * @brief Resolves the path to resources/parameters/<csv_filename> (ART-Parameters submodule)
 */
inline std::string get_csv_path(const std::string &csv_filename)
{
    return ament_index_cpp::get_package_share_directory("arussim") + "/resources/parameters/" + csv_filename;
}

/**
 * @brief Load car parameters from a CSV file and return them as a map.
 * @param filepath The path to the CSV file containing the car parameters.
 * @param logger Logger of the node that loads the parameters.
 */
inline std::map<std::string, double> load_car_parameters(const std::string &filepath, const rclcpp::Logger &logger)
{
    std::map<std::string, double> parameters_map;
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        RCLCPP_ERROR(logger, "Could not open CSV parameter file: %s", filepath.c_str());
        return parameters_map;
    }

    std::string line;

    // Skip the first line (header)
    std::getline(file, line);

    // Read the file line by line
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string key;
        std::string value_str;

        // Separate the line by commas
        if (std::getline(ss, key, ',') && std::getline(ss, value_str, ','))
        {
            try
            {
                parameters_map[key] = std::stod(value_str);
            }
            catch (const std::invalid_argument &e)
            {
                RCLCPP_WARN(logger, "Warning: Parameter '%s' is not a number: '%s'", key.c_str(), value_str.c_str());
            }
        }
    }

    return parameters_map;
}

/**
 * @brief Get a parameter that the node cannot run without. Throws if it is missing, so the node
 * does not start with a made-up value.
 */
inline double get_required_parameter(const std::map<std::string, double> &parameters_map,
                                     const std::string &key, const std::string &csv_filename)
{
    auto it = parameters_map.find(key);
    if (it == parameters_map.end())
    {
        throw std::runtime_error("Missing parameter '" + key + "' in " + csv_filename);
    }
    return it->second;
}

}  // namespace arussim
