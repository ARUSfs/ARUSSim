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

/**
 * @class CSVGenerator
 * 
 */
class CSVGenerator
{
public:
    CSVGenerator();

    void write_row(std::string mode, const std::vector<std::string> &values);

private:
    std::ofstream out_file_;
    bool header_written_ = false;
};