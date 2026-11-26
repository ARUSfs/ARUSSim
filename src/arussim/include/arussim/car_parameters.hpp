/**
 * @file car_parameters.hpp
 * @brief Read a single parameter from the ART-Parameters CSV of the simulated car
 */

#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace arussim
{

/**
 * @brief Read a parameter from resources/parameters/<car>.csv (ART-Parameters submodule).
 * Throws if the file cannot be opened or the parameter is missing, so the simulation does not
 * start with a made-up value.
 */
inline double get_car_parameter(const std::string &car, const std::string &key)
{
    const std::string path = ament_index_cpp::get_package_share_directory("arussim")
        + "/resources/parameters/" + car + ".csv";

    std::ifstream file(path);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open car parameters CSV: " + path);
    }

    std::string line;
    std::getline(file, line); // header

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string name, value;
        if (std::getline(ss, name, ',') && std::getline(ss, value, ',') && name == key)
        {
            return std::stod(value);
        }
    }

    throw std::runtime_error("Missing parameter '" + key + "' in " + path);
}

}  // namespace arussim
