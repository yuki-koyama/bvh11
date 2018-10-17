#ifndef CHANNEL_HPP_
#define CHANNEL_HPP_

#include <memory>
#include <iostream>

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
        
        Channel(Type type, std::shared_ptr<Joint> target_joint) : type_(type), target_joint_(target_joint) {}
        
        const Type& type() const { return type_; }
        std::shared_ptr<Joint> target_joint() const { return target_joint_; }
        
    private:
        const Type type_;
        const std::shared_ptr<Joint> target_joint_;
    };
    
    inline std::ostream& operator<<(std::ostream& os, const Channel::Type& type)
    {
        switch (type) {
            case Channel::Type::x_position:
                os << "Xposition";
                break;
            case Channel::Type::y_position:
                os << "Yposition";
                break;
            case Channel::Type::z_position:
                os << "Zposition";
                break;
            case Channel::Type::x_rotation:
                os << "Xrotation";
                break;
            case Channel::Type::y_rotation:
                os << "Yrotation";
                break;
            case Channel::Type::z_rotation:
                os << "Zrotation";
                break;
        }
        return os;
    }
}

#endif
