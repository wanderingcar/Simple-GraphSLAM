#include "common.hpp"
#include <unordered_map>
#include "utils.hpp"

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
	std::unordered_map<size_t, Vec2> landmarks; // <index, (x, y)>
	std::vector<PoseEdge> edges;
	Eigen::MatrixXd H;
	Eigen::VectorXd b;


public:
	void add_node(Vec3 node)
	{
		size_t index = nodes.size();
		nodes.insert({ index, node });
	}

	void add_landmark(Vec2 pose)
	{
		size_t index = landmarks.size();
		landmarks.insert({ index, pose });
	}

	void add_edge(PoseEdge edge)
	{
		edges.push_back(edge);
	}

	void print_graph()
	{
		std::cout << "NODES" << std::endl;
		for (auto& node : nodes)
		{
			std::cout << "index : " << node.first << " node : " << node.second << std::endl;
		}

		std::cout << "LANDMARKS" << std::endl;
		for (auto& landmark : landmarks)
		{
			std::cout << "index : " << landmark.first << " node : " << landmark.second << std::endl;
		}
	}

	void optimize(int iteration) // pose graph optimization
	{
		std::cout << "Start optimization.." << std::endl;
		for (int i = 0; i < iteration; i++) // iteration
		{
			int size = nodes.size() + landmarks.size();
			H.resize(size, size);
			H.setIdentity();
			b.resize(size);
			b.setZero();

			linearize();
			solve(iteration);
		}
	}


	void linearize() // linearize error functions and formulate a linear system
	{
		for (auto& edge : edges)
		{
			size_t i_index = edge.get_from();
			size_t j_index = edge.get_to();

			Mat3 T_z = v2t(edge.get_odometry());
			Mat3 omega = edge.get_information_matrix();

			Vec3 v_i = nodes[i_index];
			Vec3 v_j = nodes[j_index];

			Mat3 T_i = v2t(v_i);
			Mat3 T_j = v2t(v_j);
			Mat2 R_i = T_i.block(0, 0, 2, 2);
			Mat2 R_j = T_j.block(0, 0, 2, 2);

			double si = sin(v_i(2));
			double ci = cos(v_i(2));
			Mat2 dR_i;
			dR_i << -si, -ci,
				ci, -si; // transposed.
			Vec2 dt_ij = v_j.head(2) - v_i.head(2); // transpose..?

			// calulate jacobian
			Mat3 A;
			Mat3 B;

			// calculate error vector
			Vec3 e;

			// Formulate blocks


			// Update H and b matrix

		}
	}

	void solve(int iteration) // solve linear system, update poses
	{
		H.block(0, 0, 3, 3) += Mat3::Identity(); // initial post 0, 0, 0

		
	}
};