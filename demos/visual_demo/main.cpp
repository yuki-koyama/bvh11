#include <iostream>
#include <bvh11/bvh11.hpp>
#include <QApplication>
#include "Widget.hpp"

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: simpe_demo [BVH_PATH]" << std::endl;
        exit(1);
    }
    
    // Instantiate a BVH object
    bvh11::BvhObject bvh(argv[1]);
    
    // Instantiate a widget for visualizing BVH object
    QApplication app(argc, argv);
    Widget widget(bvh);
    widget.show();
    return app.exec();
}
