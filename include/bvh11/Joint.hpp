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
        Joint(const std::string& name, std::shared_ptr<Joint> parent) : name_(name), parent_(parent) {}
        
        const Eigen::Vector3d& offset() const { return offset_; }
        Eigen::Vector3d& offset() { return offset_; }
        
        bool has_end_site() const { return has_end_site_; }
        bool& has_end_site() { return has_end_site_; }
        
        const Eigen::Vector3d& end_site() const { return end_site_; }
        Eigen::Vector3d& end_site() { return end_site_; }
        
        const std::string& name() const { return name_; }
        
        const std::list<std::shared_ptr<Joint>>& children() const { return children_; }
        
        std::shared_ptr<Joint> parent() const { return parent_; }
        
        void AddChild(std::shared_ptr<Joint> child)
        {
            children_.push_back(child);
        }
        
    private:
        const std::string            name_;
        const std::shared_ptr<Joint> parent_;
        
        bool                              has_end_site_ = false;
        Eigen::Vector3d                   end_site_;
        Eigen::Vector3d                   offset_;
        std::list<std::shared_ptr<Joint>> children_;
    };
}

#endif
