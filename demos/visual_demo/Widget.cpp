#include "Widget.hpp"
#include <three-dim-util/opengl2/draw-functions.hpp>
#include <three-dim-util/opengl2/gl-wrappers.hpp>

Widget::Widget(const bvh11::BvhObject& bvh, QWidget* parent) : threedimutil::TrackballWidget(parent), bvh_(bvh)
{
    camera().position() = Eigen::Vector3d(0.0, 0.8, 4.0);
    camera().target()   = Eigen::Vector3d(0.0, 0.8, 0.0);
    camera().up()       = Eigen::Vector3d(0.0, 1.0, 0.0);

    near_clip() = 1.0;
    far_clip()  = 100.0;

    time_point_ = std::chrono::steady_clock::now();

    timer_ = std::make_shared<QTimer>(this);
    connect(timer_.get(), SIGNAL(timeout()), this, SLOT(advance()));
    timer_->start();
}

void Widget::paintGL()
{
    this->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    setProjectionMatrix();
    setModelViewMatrix();

    this->glEnable(GL_COLOR_MATERIAL);
    this->glEnable(GL_LIGHTING);
    this->glEnable(GL_LIGHT0);

    this->glLightfv(GL_LIGHT0, GL_POSITION, std::vector<GLfloat>{+1.0, 2.0, +3.0, 0.0}.data());

    threedimutil::draw_floor(10.0, 10);

    this->glColor3d(0.7, 0.2, 0.2);
    drawJointSubHierarchy(frame_, bvh_.root_joint());

    // Draw pseudo-shadow
    this->glPushMatrix();
    this->glScaled(1.0, 0.01, 1.0);
    this->glColor3d(0.2, 0.2, 0.2);
    drawJointSubHierarchy(frame_, bvh_.root_joint());
    this->glPopMatrix();
}

void Widget::advance()
{
    const auto current_time_point = std::chrono::steady_clock::now();
    const auto elapsed_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(current_time_point - time_point_);

    frame_ =
        static_cast<int>((static_cast<double>(elapsed_duration.count()) / 1000.0) / bvh_.frame_time()) % bvh_.frames();

    camera().RotateAroundTarget(1e-03);

    update();
}

void Widget::drawJointSubHierarchy(int frame, std::shared_ptr<const bvh11::Joint> joint)
{
    constexpr double radius = 0.025;

    this->glMatrixMode(GL_MODELVIEW);
    this->glPushMatrix();

    const Eigen::Matrix4d transform = bvh_.GetTransformationRelativeToParent(joint, frame).matrix();

    threedimutil::mult_matrix(transform);

    threedimutil::draw_sphere(radius, Eigen::Vector3d::Zero());

    for (auto child : joint->children())
    {
        threedimutil::draw_cylinder(radius, Eigen::Vector3d::Zero(), child->offset());

        drawJointSubHierarchy(frame, child);
    }

    if (joint->has_end_site())
    {
        threedimutil::draw_cylinder(radius, Eigen::Vector3d::Zero(), joint->end_site());
        threedimutil::draw_sphere(radius, joint->end_site());
    }

    this->glPopMatrix();
}
