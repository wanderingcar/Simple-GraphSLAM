#pragma once
#include "common.hpp"
#include <Eigen/Core>


Mat3 v2t(Vec3 trans);

Vec3 t2v(Mat3 mat);

std::vector<std::string> split(std::string input, char delimiter);