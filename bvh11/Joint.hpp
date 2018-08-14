#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <list>
#include <memory>
#include <string>
#include <Eigen/Core>

namespace bvh11
{
    class Joint
    {
    public:
        Joint(const std::string& name, const Eigen::Vector3d& offset) : name_(name), offset_(offset) {}
        
        void AddChild(std::shared_ptr<Joint> child)
        {
            children_.push_back(child);
        }
        
    private:
        const std::string name_;
        const Eigen::Vector3d offset_;
        
        std::list<std::shared_ptr<Joint>> children_;
    };
}

#endif
