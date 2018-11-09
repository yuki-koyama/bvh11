#ifndef WIDGET_HPP_
#define WIDGET_HPP_

#include <chrono>
#include <bvh11.hpp>
#include <three-dim-util/trackball-widget.hpp>
#include <QTimer>

class Widget : public threedimutil::TrackballWidget
{
    Q_OBJECT
public:
    explicit Widget(const bvh11::BvhObject& bvh, QWidget *parent = nullptr);
    
protected:
    void paintGL();
    
protected slots:
    void advance();
    
private:
    const bvh11::BvhObject bvh_;
    
    std::chrono::steady_clock::time_point time_point_;
    int frame_ = 0;
    std::shared_ptr<QTimer> timer_;
    
    void drawJointSubHierarchy(int frame, std::shared_ptr<const bvh11::Joint> joint) const;
};

#endif
