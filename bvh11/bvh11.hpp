#ifndef BVH11_HPP_
#define BVH11_HPP_

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <regex>
#include <cassert>
#include <Eigen/Core>
#include "Joint.hpp"
#include "Channel.hpp"

namespace bvh11
{
    inline std::vector<std::string> split(const std::string& sequence, const std::string& pattern)
    {
        const std::regex regex(pattern);
        const std::sregex_token_iterator first{ sequence.begin(), sequence.end(), regex, -1 };
        const std::sregex_token_iterator last {};
        return std::vector<std::string>{ first->str().empty() ? std::next(first) : first, last };
    }
    
    class BvhObject
    {
    public:
        BvhObject(const std::string& file_path)
        {
            ReadBvhFile(file_path);
        }
        
        int    frames()     const { return frames_;     }
        double frame_time() const { return frame_time_; }
        
    private:
        int                    frames_;
        double                 frame_time_;
        std::vector<Channel>   channels_;
        Eigen::MatrixXd        motion_;
        std::shared_ptr<Joint> root_joint_;
        
        void ReadBvhFile(const std::string& file_path)
        {
            // Open the input file
            std::ifstream ifs(file_path);
            assert(ifs.is_open() && "Failed to open the input file.");
            
            // Read the file line by line
            std::string line;
            while (std::getline(ifs, line))
            {
                // Split the line into tokens
                const std::vector<std::string> tokens = split(line, R"([\t\s]+)");
            }
        }
    };
}

#endif
