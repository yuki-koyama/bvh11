#include <bvh11.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: simpe_demo [BVH_PATH]" << std::endl;
        std::cout << "Loading a default BVH file..." << std::endl;
    }

    const std::string bvh_file_path = (argc >= 2) ? argv[1] : "131_03.bvh";

    // Import data from a BVH file
    bvh11::BvhObject bvh(bvh_file_path);

    // Display basic info
    std::cout << "#Channels       : " << bvh.channels().size() << std::endl;
    std::cout << "#Frames         : " << bvh.frames() << std::endl;
    std::cout << "Frame time      : " << bvh.frame_time() << std::endl;
    std::cout << "Joint hierarchy : " << std::endl;
    bvh.PrintJointHierarchy();

    // Edit the motion data (here, the number of frames is reduced into the half)
    bvh.ResizeFrames(bvh.frames() / 2);
 
    // Export data to a BVH file
    bvh.WriteBvhFile("./out.bvh");

    return 0;
}
