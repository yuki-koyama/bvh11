#include "Widget.hpp"
#include <three-dim-util/gl-wrapper.hpp>
#include <three-dim-util/draw-functions.hpp>
#include <igl/opengl2/draw_floor.h>

Widget::Widget(const bvh11::BvhObject& bvh, QWidget *parent) :
threedimutil::TrackballWidget(parent),
bvh_(bvh)
{
    camera().position() = Eigen::Vector3d(50.0, 10.0, 80.0);
    camera().target()   = Eigen::Vector3d( 0.0, 20.0,  0.0);
    camera().up()       = Eigen::Vector3d( 0.0,  1.0,  0.0);
    
    near_clip() = 1.0;
    far_clip()  = 500.0;
    
    time_point_ = std::chrono::steady_clock::now();
    
    timer_ = std::make_shared<QTimer>(this);
    connect(timer_.get(), SIGNAL(timeout()), this, SLOT(advance()));
    timer_->start();
}

void Widget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    setProjectionMatrix();
    setModelViewMatrix();
    
    glPushMatrix();
    glScaled(5.0, 1.0, 5.0);
    igl::opengl2::draw_floor();
    glPopMatrix();
    
    glColor3d(0.5, 0.1, 0.1);
    drawJointSubHierarchy(frame_, bvh_.root_joint());
}

void Widget::advance()
{
    const auto current_time_point = std::chrono::steady_clock::now();
    const auto elapsed_duration   = std::chrono::duration_cast<std::chrono::milliseconds>(current_time_point - time_point_);
    
    frame_ = static_cast<int>((static_cast<double>(elapsed_duration.count()) / 1000.0) / bvh_.frame_time()) % bvh_.frames();
    
    camera().RotateAroundTarget(1e-03);
    
    update();
}

void Widget::drawJointSubHierarchy(int frame, std::shared_ptr<const bvh11::Joint> joint) const
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    threedimutil::translate(joint->offset());
    
    for (int channel_index : joint->associated_channels_indices())
    {
        const bvh11::Channel& channel = bvh_.channels()[channel_index];
        const double value = bvh_.motion()(frame, channel_index);
        switch (channel.type())
        {
            case bvh11::Channel::Type::x_position:
                threedimutil::translate(Eigen::Vector3d(value, 0.0, 0.0));
                break;
            case bvh11::Channel::Type::y_position:
                threedimutil::translate(Eigen::Vector3d(0.0, value, 0.0));
                break;
            case bvh11::Channel::Type::z_position:
                threedimutil::translate(Eigen::Vector3d(0.0, 0.0, value));
                break;
            case bvh11::Channel::Type::x_rotation:
                threedimutil::rotate(value, Eigen::Vector3d(1.0, 0.0, 0.0));
                break;
            case bvh11::Channel::Type::y_rotation:
                threedimutil::rotate(value, Eigen::Vector3d(0.0, 1.0, 0.0));
                break;
            case bvh11::Channel::Type::z_rotation:
                threedimutil::rotate(value, Eigen::Vector3d(0.0, 0.0, 1.0));
                break;
        }
    }
    
    threedimutil::draw_point(Eigen::Vector3d::Zero());
    
    for (auto child : joint->children()) { drawJointSubHierarchy(frame, child); }
    
    glPopMatrix();
}
