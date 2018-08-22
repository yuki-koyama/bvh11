#include <iostream>
#include <bvh11/bvh11.hpp>
#include <three-dim-util/trackball-widget.hpp>
#include <QApplication>

class Widget : public threedimutil::TrackballWidget
{
public:
    Widget(QWidget *parent = nullptr) : threedimutil::TrackballWidget(parent) {}
    
protected:
    void paintGL()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        setProjectionMatrix();
        setModelViewMatrix();
    }
};

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: simpe_demo [BVH_PATH]" << std::endl;
        exit(1);
    }
    
    bvh11::BvhObject bvh(argv[1]);
    
    QApplication app(argc, argv);
    Widget widget;
    widget.show();
    return app.exec();
}
