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
        const std::list<int>& associated_channels_indices() const { return associated_channels_indices_; }
        
        std::shared_ptr<Joint> parent() const { return parent_; }
        
        void AddChild(std::shared_ptr<Joint> child) { children_.push_back(child); }
        void AssociateChannel(int channel_index) { associated_channels_indices_.push_back(channel_index); }
        
    private:
        const std::string            name_;
        const std::shared_ptr<Joint> parent_;
        
        bool                              has_end_site_ = false;
        Eigen::Vector3d                   end_site_;
        Eigen::Vector3d                   offset_;
        std::list<std::shared_ptr<Joint>> children_;
        std::list<int>                    associated_channels_indices_;
    };
}

#endif
