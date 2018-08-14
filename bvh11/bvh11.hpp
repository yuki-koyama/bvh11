#ifndef BVH11_HPP_
#define BVH11_HPP_

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Core>
#include "Joint.hpp"
#include "Channel.hpp"

namespace bvh11
{
    class BvhObject
    {
    public:
        BvhObject(const std::string& file_path) {}
        
        const int& frames() const { return frames_; }
        const double& frame_time() const { return frame_time_; }
        
    private:
        int                    frames_;
        double                 frame_time_;
        std::vector<Channel>   channels_;
        Eigen::MatrixXd        motion_;
        std::shared_ptr<Joint> root_joint_;
    };
}

#endif
