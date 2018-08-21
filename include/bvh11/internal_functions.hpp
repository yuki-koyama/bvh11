#ifndef INTERNAL_FUNCTIONS_HPP_
#define INTERNAL_FUNCTIONS_HPP_

#include <vector>
#include <string>
#include <fstream>
#include <regex>
#include <cassert>
#include <Eigen/Core>

namespace bvh11
{
    namespace internal
    {
        inline std::vector<std::string> split(const std::string& sequence, const std::string& pattern)
        {
            const std::regex regex(pattern);
            const std::sregex_token_iterator first{ sequence.begin(), sequence.end(), regex, -1 };
            const std::sregex_token_iterator last {};
            return std::vector<std::string>{ first->str().empty() ? std::next(first) : first, last };
        }
        
        inline std::vector<std::string> tokenize_next_line(std::ifstream& ifs)
        {
            std::string line;
            if (std::getline(ifs, line))
            {
                return internal::split(line, R"([\t\s]+)");
            }
            else
            {
                assert(false && "Failed to read the next line");
                return {};
            }
        }
        
        inline Eigen::Vector3d read_offset(const std::vector<std::string>& tokens)
        {
            assert(tokens.size() == 4 && tokens[0] == "OFFSET");
            const double offset_x = std::stod(tokens[1]);
            const double offset_y = std::stod(tokens[2]);
            const double offset_z = std::stod(tokens[3]);
            return Eigen::Vector3d(offset_x, offset_y, offset_z);
        }
    }
}

#endif
