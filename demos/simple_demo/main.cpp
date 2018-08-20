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

    return 0;
}
