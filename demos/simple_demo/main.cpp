#include <iostream>
#include <bvh11/bvh11.hpp>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: simpe_demo [BVH_PATH]" << std::endl;
        exit(1);
    }
    
    bvh11::BvhObject bvh(argv[1]);
    
    std::cout << "#Channels       : " << bvh.channels().size() << std::endl;
    std::cout << "#Frames         : " << bvh.frames()          << std::endl;
    std::cout << "Frame time      : " << bvh.frame_time()      << std::endl;
    std::cout << "Joint hierarchy : " << std::endl;
    bvh.PrintJointHierarchy();
    
    return 0;
}
