#include <vector>
#include <string>
#include <fstream>


class CSVGenerator
{
public:
    CSVGenerator();

    void write_row(std::string mode, const std::vector<std::string> &values);

private:
    std::ofstream out_file_;
    bool header_written_ = false;
};