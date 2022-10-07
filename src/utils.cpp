#include <Eigen/Core>
#include "utils.hpp"
#include "common.hpp"

Mat3 v2t(Vec3 trans)
{
	Mat3 ret;
	double c = cos(trans[2]);
	double s = sin(trans[2]);

	ret << c, -s, trans[0],
		s, c, trans[1],
		0, 0, 1;

	return ret;
}

Vec3 t2v(Mat3 mat)
{
	Vec3 ret;
	ret << mat(0, 2), mat(1, 2), atan2(mat(1, 0), mat(0, 0));

	return ret;
}

