#include <iostream>
#include <bvh11/bvh11.hpp>
#include <QApplication>
#include "Widget.hpp"

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: visual_demo [BVH_PATH]" << std::endl;
        std::cout << "Loading a default BVH file..." << std::endl;
    }

    const std::string bvh_file_path = (argc >= 2) ? argv[1] : "131_03.bvh";
    
    // Instantiate a BVH object
    bvh11::BvhObject bvh(bvh_file_path);
    
    // Instantiate a widget for visualizing BVH object
    QApplication app(argc, argv);
    Widget widget(bvh);
    widget.show();
    return app.exec();
}
