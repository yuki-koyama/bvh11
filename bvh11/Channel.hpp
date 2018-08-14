#ifndef CHANNEL_HPP_
#define CHANNEL_HPP_

#include <memory>

namespace bvh11
{
    class Joint;
    
    class Channel
    {
    public:
        enum class Type
        {
            x_position, y_position, z_position,
            z_rotation, x_rotation, y_rotation
        };
        
        Channel(Type type, std::shared_ptr<Joint> target_joint) {}
        
        const Type& type() const { return type_; }
        std::shared_ptr<Joint> target_joint() const { return target_joint_; }
        
    private:
        Type type_;
        std::shared_ptr<Joint> target_joint_;
    };
}

#endif
