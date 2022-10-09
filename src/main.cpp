#include <iostream>
#include <posegraph.hpp>
#include "common.hpp"


int main()
{
    std::cout << "Welcome to GRAPH SLAM" << std::endl;
    std::string path = "./data/p1.txt";

    PoseGraph pg;
    pg.load_data(path);

    pg.optimize(5);

    system("pause");

    return EXIT_SUCCESS;
}

