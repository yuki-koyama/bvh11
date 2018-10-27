#include <iostream>
#include <bvh11.hpp>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: simpe_demo [BVH_PATH]" << std::endl;
        std::cout << "Loading a default BVH file..." << std::endl;
    }
    
    const std::string bvh_file_path = (argc >= 2) ? argv[1] : "131_03.bvh";

    bvh11::BvhObject bvh(bvh_file_path);
    
    std::cout << "#Channels       : " << bvh.channels().size() << std::endl;
    std::cout << "#Frames         : " << bvh.frames()          << std::endl;
    std::cout << "Frame time      : " << bvh.frame_time()      << std::endl;
    std::cout << "Joint hierarchy : " << std::endl;
    bvh.PrintJointHierarchy();
    
    return 0;
}
