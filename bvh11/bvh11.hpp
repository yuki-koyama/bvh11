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

#ifdef BVH11_VERBOSE
#include <iostream>
#endif

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
            std::vector<std::shared_ptr<Joint>> stack;
            while (std::getline(ifs, line))
            {
                // Split the line into tokens
                const std::vector<std::string> tokens = internal::split(line, R"([\t\s]+)");
                
                // Ignore empty lines
                if (tokens.empty())
                {
                    continue;
                }
                // Start to create a new joint
                else if (tokens[0] == "ROOT" || tokens[0] == "JOINT")
                {
                    assert(tokens.size() == 2 && "Failed to find a joint name");
                    
                    const std::string& joint_name = tokens[1];
                    
#ifdef BVH11_VERBOSE
                    for (int i = 0; i < stack.size(); ++ i) { std::cout << "  "; }
                    std::cout << joint_name << std::endl;
#endif
                    
                    const std::shared_ptr<Joint> parent = stack.empty() ? nullptr : stack.back();
                    
                    std::shared_ptr<Joint> new_joint = std::make_shared<Joint>(joint_name, parent);
                    
                    if (parent) { parent->AddChild(new_joint); }
                    
                    stack.push_back(new_joint);
                    
                    // Read the next line, which should be "{"
                    const std::vector<std::string> tokens_begin_block = internal::tokenize_next_line(ifs);
                    assert(tokens_begin_block.size() == 1 && "Found two or more tokens");
                    assert(tokens_begin_block[0] == "{" && "Could not find an expected '{'");
                }
                // Read an offset value
                else if (tokens[0] == "OFFSET")
                {
                    assert(tokens.size() == 4);
                    const std::shared_ptr<Joint> current_joint = stack.back();
                    current_joint->offset() = internal::read_offset(tokens);
                }
                // Read an end site
                else if (tokens[0] == "End")
                {
                    assert(tokens.size() == 2 && tokens[1] == "Site");
                    
                    const std::shared_ptr<Joint> current_joint = stack.back();
                    current_joint->has_end_site() = true;
                    
                    // Read the next line, which should be "{"
                    const std::vector<std::string> tokens_begin_block = internal::tokenize_next_line(ifs);
                    assert(tokens_begin_block.size() == 1 && "Found two or more tokens");
                    assert(tokens_begin_block[0] == "{" && "Could not find an expected '{'");
                    
                    // Read the next line, which should state an offset
                    const std::vector<std::string> tokens_offset = internal::tokenize_next_line(ifs);
                    current_joint->end_site() = internal::read_offset(tokens_offset);
                    
                    // Read the next line, which should be "{"
                    const std::vector<std::string> tokens_end_block = internal::tokenize_next_line(ifs);
                    assert(tokens_end_block.size() == 1 && "Found two or more tokens");
                    assert(tokens_end_block[0] == "}" && "Could not find an expected '}'");
                }
                // Finish to create a joint
                else if (tokens[0] == "}")
                {
                    assert(!stack.empty());
                    stack.pop_back();
                }
            }
        }
    };
}

#endif
