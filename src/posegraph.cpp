#include "common.hpp"
#include <unordered_map>
#include "utils.hpp"
#include <Eigen/Sparse>

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
			H.setZero();
			b.resize(size);
			b.setZero();

			linearize();
			solve();
		}
	}


	void linearize() // linearize error functions and formulate a linear system
	{
		for (auto& edge : edges) // adding pose constraints
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
			Mat2 R_z = T_z.block(0, 0, 2, 2);

			double si = sin(v_i(2));
			double ci = cos(v_i(2));
			Mat2 dR_i;
			dR_i << -si, -ci,
				ci, -si; // transposed.
			Vec2 dt_ij = v_j.head(2) - v_i.head(2); // transpose..?

			// calulate jacobian
			Mat3 A;
			A.block(0, 0, 2, 2) = -R_z.transpose() * R_i.transpose();
			A.block(0, 2, 2, 1) = R_z.transpose() * dR_i.transpose();
			A.row(2) << 0, 0, -1;
			Mat3 B;
			B.block(0, 0, 2, 2) = R_z.transpose() * R_i.transpose();
			B.col(2) << 0, 0, 1;
			B.row(2) << 0, 0, 1;

			// calculate error vector
			Vec3 e;
			e = t2v((T_z.inverse() * T_i.inverse()) * T_j);

			// Formulate blocks
			Mat3 H_ii = (A.transpose() * omega) * A;
			Mat3 H_ij = (A.transpose() * omega) * B;
			Mat3 H_jj = (B.transpose() * omega) * B;
			Vec3 b_i = (-A.transpose() * omega) * e;
			Vec3 b_j = (-B.transpose() * omega) * e;

			// Update H and b matrix
			H.block(3 * i_index, 3 * i_index, 3, 3) += H_ii;
			H.block(3 * i_index, 3 * j_index, 3, 3) += H_ij;
			H.block(3 * j_index, 3 * i_index, 3, 3) += H_ii.transpose();
			H.block(3 * j_index, 3 * j_index, 3, 3) += H_jj;
			b.segment(3 * i_index, 3) += b_i;
			b.segment(3 * j_index, 3) += b_j;
		}

		// adding landmark constraints
	}

	void update_node(Eigen::VectorXd dx)
	{
		for (size_t i = 0; i < nodes.size(); i++)
		{
			nodes[i] += dx.segment(3 * i, 3);
		}
	}

	void solve() // solve linear system, update poses
	{
		H.block(0, 0, 3, 3) += Mat3::Identity(); // initial post 0, 0, 0

		Eigen::SparseMatrix<double> H_sparse;
		H_sparse = H.sparseView();
		Eigen::VectorXd dx;
		Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
		solver.compute(H_sparse);
		dx = solver.solve(b);

		dx.segment(0, 3) << 0, 0, 0;

		update_node(dx);
	}
};