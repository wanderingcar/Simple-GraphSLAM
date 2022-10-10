#pragma once

#include "common.hpp"
#include <unordered_map>
#include "utils.hpp"
#include <Eigen/Sparse>
#include <fstream>
#include <sstream>
#include <vector>

class LandmarkEdge
{
private:
	int pose_index = -1;
	int landmark_index = -1;
	Vec2 measurement; // (distance, theta)
	Mat2 information_matrix;
	
public:
	void set_edge(int i, int j, Vec2 meas, Mat2 infm);
	int get_pose_index();
	int get_landmark_index();
	Vec2 get_measurement();
	Mat2 get_information_matrix();
};

class PoseEdge
{
private:
	int from = -1;
	int to = -1;
	Vec3 odometry;
	Mat3 information_matrix;

public:
	void set_edge(int i, int j, Vec3 odom, Mat3 infm);
	int get_from();
	int get_to();
	Vec3 get_odometry();
	Mat3 get_information_matrix();
};

class PoseGraph
{
private:
	std::unordered_map<size_t, Vec3> nodes; // <index, (x, y, theta)>
	std::unordered_map<size_t, Vec2> landmarks; // <index, (x, y)>
	std::vector<PoseEdge> edges;
	std::vector<LandmarkEdge> landmark_edges;
	Eigen::MatrixXd H;
	Eigen::VectorXd b;


public:
	void load_data(std::string path);
	void add_node(Vec3 node);
	void add_landmark(Vec2 pose);
	void add_edge(PoseEdge edge);
	void add_landmark_edge(LandmarkEdge edge);
	void print_graph();
	void optimize(int iteration); // pose graph optimization
	void linearize(); // linearize error functions and formulate a linear system
	void update_node(Eigen::VectorXd dx);
	void solve(); // solve linear system, update poses
};