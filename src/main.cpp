#include <ros/ros.h>
#include <cstdint>
#include <limits.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <vector>
#include <a3planner/plan.h>
#include "../../a3env/src/common.hpp"

// functions
bool gen_defs_file();
// function to write the definitions file when the service is launched

bool gen_world_file(int world[a3env::MAP_WIDTH][a3env::MAP_WIDTH], int &xpos, int &ypos, int &maxSteps);
// function to save the received information to world.csp

std::string getExecutablePath();
// function to get the directory of the executable

bool get_moves(std::vector<unsigned char> &moves);
// function to call PAT and read moves from out.txt

// global variables
std::string exeDir = getExecutablePath();
std::string rootDir = exeDir.substr(0, exeDir.find("catkin_ws") - 1); // catkin_ws directory is located here
std::string worldDir = rootDir + "/catkin_ws/src/a3planner/pat/world.csp";
std::string defsDir = rootDir + "/catkin_ws/src/a3planner/pat/definitions.csp";
std::string PATDir = rootDir + "/MONO-PAT-v3.6.0/PAT3.Console.exe";
std::string outDir = rootDir + "/catkin_ws/src/a3planner/pat/out.txt";
std::string searchDir = rootDir + "/catkin_ws/src/a3planner/pat/search.csp";
// temp global
int maxsteps = 10;

bool plan_callback(a3planner::plan::Request &req, a3planner::plan::Response &res)
{
	// check size matches
	if (req.world.size() != a3env::MAP_WIDTH * a3env::MAP_WIDTH)
	{
		ROS_ERROR("Received data array of incorrect size");
		return false;
	}
	// convert world to 2d array
	int world[a3env::MAP_WIDTH][a3env::MAP_WIDTH];
	for (int i = 0; i < a3env::MAP_WIDTH; i++)
	{
		for (int j = 0; j < a3env::MAP_WIDTH; j++)
		{
			world[i][j] = req.world[i * a3env::MAP_WIDTH + j];
		}
	}
	// write world.csp
	if (!gen_world_file(world, req.row, req.col, maxsteps))
	{
		ROS_ERROR("Write to world.csp failed.");
		return false;
	}
	// get moves
	std::vector<unsigned char> moves;
	get_moves(moves);
	res.plan = moves;
	res.moves = moves.size();

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "a3planner");
	ros::NodeHandle n;
	if (!gen_defs_file())
	{
		ROS_ERROR("Write to definitions.csp failed.");
	}
	ros::ServiceServer service = n.advertiseService("a3planner/plan", plan_callback);
	ros::spin();

	return 0;
}

bool gen_defs_file()
{
	// open world.csp file to write to
	std::ofstream file(defsDir);
	if (!file.is_open())
	{
		std::cerr << "Open definitions.csp failed." << std::endl;
		return false;
	}
	ROS_INFO("Writing to definitions.csp");

	file << "#define Seen " << a3env::BLOCK_AIR << ";\n";
	file << "#define Unseen " << a3env::BLOCK_UNKNOWN << ";\n";
	file << "#define Hostile " << a3env::BLOCK_WALL << ";\n";
	file << "#define Survivor " << a3env::BLOCK_SURVIVOR << ";\n";
	file << "#define Home_x " << 1 << ";\n";
	file << "#define Home_y " << 1 << ";\n";
	file << "#define Rows " << a3env::MAP_WIDTH << ";\n";
	file << "#define Cols " << a3env::MAP_WIDTH << ";\n";
	file << "#define vision 1;\n";

	file.close();
	return true;
}

bool gen_world_file(int world[a3env::MAP_WIDTH][a3env::MAP_WIDTH], int &xpos, int &ypos, int &maxSteps)
{
	// open world.csp file to write to
	std::ofstream file(worldDir);
	if (!file.is_open())
	{
		ROS_INFO("Open world.csp failed.");
		return false;
	}
	ROS_INFO("Writing to world.csp");

	// define target location
	int xtarget = 0;
	int ytarget = 0;

	// write world matrix
	file << "var world[" << a3env::MAP_WIDTH << "][" << a3env::MAP_WIDTH << "]:{0.." << a3env::BLOCK_SURVIVOR << "} = [\n";
	for (int i = 0; i < a3env::MAP_WIDTH; i++)
	{
		for (int j = 0; j < a3env::MAP_WIDTH; j++)
		{
			if (world[i][j] <= a3env::BLOCK_SURVIVOR)
			{
				file << world[i][j];
			}
			else if (world[i][j] == a3env::BLOCK_HOSTILE)
			{
				file << a3env::BLOCK_WALL;
			}
			else
			{
				ROS_INFO("Invalid block type in request.");
				return false;
			}
			if (i != a3env::MAP_WIDTH - 1 || j != a3env::MAP_WIDTH - 1)
			{
				file << ", ";
			}
		}
		file << "\n";
	}
	file << "];\n\n";

	file << "var xpos:{0.." << a3env::MAP_WIDTH - 1 << "} = " << xpos << ";\n";
	file << "var ypos:{0.." << a3env::MAP_WIDTH - 1 << "} = " << ypos << ";\n";
	file << "var xtarget:{0.." << a3env::MAP_WIDTH - 1 << "} = " << xtarget << ";\n";
	file << "var ytarget:{0.." << a3env::MAP_WIDTH - 1 << "} = " << ytarget << ";\n";
	file << "#define maxSteps " << maxSteps << ";\n";

	file.close();
	return true;
}

std::string getExecutablePath()
{
	char result[PATH_MAX];
	ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
	if (count != -1)
	{
		std::string fullPath(result, count);
		return fullPath.substr(0, fullPath.find_last_of('/'));
	}
	return "";
}

bool get_moves(std::vector<unsigned char> &moves)
{
	// run PAT
	ROS_INFO("Calculating a path to optimize exploration");
	if (std::system(("mono " + PATDir + " -engine 1 " + searchDir + " " + outDir).c_str()) < 0)
	{
		ROS_INFO("There has been a fatal error!");
		exit(1);
	}
	// read out.txt
	std::ifstream file(outDir);
	if (file.is_open())
	{
		std::string line;
		std::string move;
		while (std::getline(file, line))
		{
			if (line[0] == '<')
			{
				// create a string stream from the line so that we may read into move
				std::istringstream ss(line);
				while (ss >> move)
				{
					if (move == "moveright")
					{
						moves.push_back('r');
					}
					else if (move == "moveup")
					{
						moves.push_back('u');
					}
					else if (move == "movedown")
					{
						moves.push_back('d');
					}
					else if (move == "moveleft")
					{
						moves.push_back('l');
					}
				}
				break;
			}
		}
		file.close();
	}
	else
	{
		ROS_INFO("Unable to open out.txt");
		return false;
	}
	return true;
}