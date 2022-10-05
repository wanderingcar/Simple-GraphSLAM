#include "common.hpp"
#include <unordered_map>

class PoseEdge
{
private:
	size_t from = -1;
	size_t to = -1;
	Vec3 odometry;
	Mat3 information_matrix;

public:
	void set_edge(size_t i, size_t j, Vec3 odom, Mat3 infm)
	{
		from = i;
		to = j;
		odometry = odom;
		information_matrix = infm;
	}

	size_t get_from()
	{
		return from;
	}

	size_t get_to()
	{
		return to;
	}

	Vec3 get_odometry()
	{
		return odometry;
	}

	Mat3 get_information_matrix()
	{
		return information_matrix;
	}
};

class PoseGraph
{
private:
	std::unordered_map<size_t, Vec3> nodes; // <index, (x, y, theta)>
	std::vector<PoseEdge> edges;
	Eigen::MatrixXd H;
	Eigen::VectorXd b; // H @ x = b

public:
	void set_graph()
	{

	}

	void print_graph()
	{

	}

	void optimize()
	{

	}

	void linearize()
	{

	}

	void solve() // solve linear system, update poses
	{

	}
};