#include "posegraph.hpp"


// LandmarkEdge functions


void LandmarkEdge::set_edge(int i, int j, Vec2 meas, Mat2 infm)
{
	pose_index = i;
	landmark_index = j;
	measurement = meas;
	information_matrix = infm;
}

int LandmarkEdge::get_pose_index()
{
	return pose_index;
}

int LandmarkEdge::get_landmark_index()
{
	return landmark_index;
}

Vec2 LandmarkEdge::get_measurement()
{
	return measurement;
}

Mat2 LandmarkEdge::get_information_matrix()
{
	return information_matrix;
}


// PoseEdge functions

void PoseEdge::set_edge(int i, int j, Vec3 odom, Mat3 infm)
{
	from = i;
	to = j;
	odometry = odom;
	information_matrix = infm;
}

int PoseEdge::get_from()
{
	return from;
}

int PoseEdge::get_to()
{
	return to;
}

Vec3 PoseEdge::get_odometry()
{
	return odometry;
}

Mat3 PoseEdge::get_information_matrix()
{
	return information_matrix;
}

// PoseGraph functions

void PoseGraph::load_data(std::string path)
{
	std::string line;
	std::ifstream file(path);


	if (file.is_open())
	{
		while (std::getline(file, line))
		{
			std::vector<std::string> result = split(line, ' ');
			std::string type = result[0];

			if (type == "VERTEX")
			{
				Vec3 node;
				node << std::stod(result[2]), std::stod(result[3]), std::stod(result[4]) / 180.0 * PI;
				add_node(node);
			}

			else if (type == "EDGE")
			{
				PoseEdge edge;
				Vec3 odom;
				odom << std::stod(result[3]), std::stod(result[4]), std::stod(result[5]) / 180.0 * PI;
				Mat3 infm;
				infm << std::stod(result[6]), std::stod(result[7]), std::stod(result[10]),
					std::stod(result[7]), std::stod(result[8]), std::stod(result[11]),
					std::stod(result[10]), std::stod(result[11]), std::stod(result[9]);
				edge.set_edge(std::stoi(result[2]), std::stoi(result[1]), odom, infm);
				add_edge(edge);
			}

			else if (type == "LANDMARK")
			{
				Vec2 pos;
				pos << std::stod(result[2]), std::stod(result[3]);
				add_landmark(pos);
			}

			else if (type == "LEDGE")
			{
				LandmarkEdge l_edge;
				Vec2 measurement;
				measurement << std::stod(result[3]), std::stod(result[4]) / 180.0 * PI;
				Mat2 infm;
				infm << std::stod(result[5]), std::stod(result[7]),
					std::stod(result[7]), std::stod(result[6]);
				l_edge.set_edge(std::stoi(result[1]), std::stoi(result[2]), measurement, infm);
				add_landmark_edge(l_edge);
			}
		}
		file.close();
	}
}

void PoseGraph::add_node(Vec3 node)
{
	size_t index = nodes.size();
	nodes.insert({ index, node });
}

void PoseGraph::add_landmark(Vec2 pose)
{
	size_t index = landmarks.size();
	landmarks.insert({ index, pose });
}

void PoseGraph::add_edge(PoseEdge edge)
{
	edges.push_back(edge);
}

void PoseGraph::add_landmark_edge(LandmarkEdge edge)
{
	landmark_edges.push_back(edge);
}

void PoseGraph::print_graph()
{
	std::cout << "NODES" << std::endl;
	for (auto& node : nodes)
	{
		std::cout << "index : " << node.first << "\n" <<
			"x     : " << node.second(0) << "\n" <<
			"y     : " << node.second(1) << "\n" <<
			"theta : " << node.second(2) * 180 / PI << " ( (rad) " << node.second(2) << " )" << "\n" << std::endl;
	}

	if (!landmarks.empty())
	{
		std::cout << "\nLANDMARKS" << std::endl;
		for (auto& landmark : landmarks)
		{
			std::cout << "index : " << landmark.first << "\n" <<
				"x : " << landmark.second(0) << "\n" <<
				"y : " << landmark.second(1) << "\n" << std::endl;
		}
	}
}

void PoseGraph::optimize(int iteration) // pose graph optimization
{
	std::cout << "Start optimization.." << std::endl;
	for (int i = 0; i < iteration; i++) // iteration
	{
		int size = 3 * nodes.size() + 2 * landmarks.size();
		H.resize(size, size);
		b.resize(size);
		H.setZero();
		b.setZero();

		linearize();
		solve();

		std::cout << "Iteration " << i << std::endl;
		print_graph();
	}
}


void PoseGraph::linearize() // linearize error functions and formulate a linear system
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
		A.setZero();
		A.block(0, 0, 2, 2) = -R_z.transpose() * R_i.transpose();
		A.block(0, 2, 2, 1) = (R_z.transpose() * dR_i.transpose()) * dt_ij;
		A(2, 2) = -1;
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
		H.block(3 * j_index, 3 * i_index, 3, 3) += H_ij.transpose();
		H.block(3 * j_index, 3 * j_index, 3, 3) += H_jj;
		b.segment(3 * i_index, 3) += b_i;
		b.segment(3 * j_index, 3) += b_j;
	}

	// adding landmark constraints

	if (!landmarks.empty())
	{
		for (auto& edge : landmark_edges) // adding pose constraints
		{
			// TODO

			size_t i_index = edge.get_pose_index();
			size_t j_index = edge.get_landmark_index();

			Vec2 meas_d_t = edge.get_measurement();
			Vec2 meas;

			meas << meas_d_t(0) * cos(meas_d_t(1)), meas_d_t(0)* sin(meas_d_t(1));

			Vec3 meas_3;
			meas_3.segment(0, 2) = meas;
			meas_3(2) = 0;

			Mat3 T_z = v2t(meas_3);
			Mat2 omega = edge.get_information_matrix();
			Mat3 omega_3;
			omega_3.setIdentity();

			Vec3 v_i = nodes[i_index];
			Vec2 v_j = landmarks[j_index];

			Vec3 v_j_3;
			v_j_3.segment(0, 2) = v_j;
			v_j_3(2) = 0;

			Mat3 T_i = v2t(v_i);
			Mat3 T_j = v2t(v_j_3);
			Mat2 R_i = T_i.block(0, 0, 2, 2);
			Mat2 R_j = T_j.block(0, 0, 2, 2);
			Mat2 R_z = T_z.block(0, 0, 2, 2);

			double si = sin(v_i(2));
			double ci = cos(v_i(2));
			Mat2 dR_i;
			dR_i << -si, -ci,
				ci, -si; // transposed.
			Vec2 dt_ij = v_j - v_i.head(2); // transpose..?

			// calulate jacobian
			MatX A(2, 3);
			A.setZero();
			A.block(0, 0, 2, 2) = -R_i.transpose();
			A.block(0, 2, 2, 1) = (dR_i.transpose()) * dt_ij;
			// A(2, 2) = -1;
			Mat2 B;
			// B.setZero();
			B = R_i.transpose();
			// B.col(2) << 0, 0, 1;
			// B.row(2) << 0, 0, 1;

			// calculate error vector
			//Vec3 e3;
			//e3 = t2v((T_z.inverse() * T_i.inverse()) * T_j);

			Vec2 e;
			e = R_i.transpose() * dt_ij - meas;

			// Formulate blocks
			Mat3 H_ii = (A.transpose() * omega) * A; // 3*3
			MatX H_ij(3, 2);
			H_ij = (A.transpose() * omega) * B; // 3*2
			Mat2 H_jj = (B.transpose() * omega) * B; // 2*2
			Vec3 b_i = (-A.transpose() * omega) * e; // 3*1
			Vec2 b_j = (-B.transpose() * omega) * e; // 2*1

			// Update H and b matrix // add vertex size to j index
			int vertex_size = nodes.size();

			H.block(3 * i_index, 3 * i_index, 3, 3) += H_ii;
			H.block(3 * i_index, 3 * vertex_size + 2 * j_index, 3, 2) += H_ij;
			H.block(3 * vertex_size + 2 * j_index, 3 * i_index, 2, 3) += H_ij.transpose();
			H.block(3 * vertex_size + 2 * j_index, 3 * vertex_size + 2 * j_index, 2, 2) += H_jj;
			b.segment(3 * i_index, 3) += b_i;
			b.segment(3 * vertex_size + 2 * j_index, 2) += b_j;
		}
	}


	// std::cout << H << std::endl;
	// std::cout << b << std::endl;
}

void PoseGraph::update_node(Eigen::VectorXd dx)
{
	for (size_t i = 0; i < nodes.size(); i++)
	{
		nodes[i] += dx.segment(3 * i, 3);
	}

	int node_size = nodes.size();

	for (size_t i = 0; i < landmarks.size(); i++)
	{
		landmarks[i] += dx.segment(3 * node_size + 2 * i, 2);
	}
}

void PoseGraph::solve() // solve linear system, update poses
{
	H.block(0, 0, 3, 3) += Mat3::Identity(); // initial post 0, 0, 0

	Eigen::SparseMatrix<double> H_sparse;
	H_sparse = H.sparseView();

	std::cout << H_sparse << std::endl;
	std::cout << b << std::endl;

	int size = 3 * nodes.size() + 2 * landmarks.size();
	Eigen::VectorXd dx;
	dx.resize(size);

	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
	solver.compute(H_sparse);
	dx = solver.solve(b);

	std::cout << dx << std::endl;

	dx.segment(0, 3) << 0, 0, 0;

	std::cout << dx << std::endl;

	update_node(dx);
}
