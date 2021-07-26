#ifndef WIDGET_HPP_
#define WIDGET_HPP_

#include <QTimer>
#include <bvh11.hpp>
#include <chrono>
#include <three-dim-util/opengl2/widgets/trackball-widget.hpp>

class Widget : public threedimutil::TrackballWidget
{
    Q_OBJECT
public:
    explicit Widget(const bvh11::BvhObject& bvh, QWidget* parent = nullptr);

protected:
    void paintGL();

protected slots:
    void advance();

private:
    const bvh11::BvhObject bvh_;

    std::chrono::steady_clock::time_point time_point_;
    int                                   frame_ = 0;
    std::shared_ptr<QTimer>               timer_;

    void drawJointSubHierarchy(int frame, std::shared_ptr<const bvh11::Joint> joint);
};

#endif
