#include <ros/ros.h>
#include <cstdint>
#include <limits.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <a3planner/plan.h>
#include "../../a3env/src/common.hpp"

// functions
bool gen_defs_file();
// function to write the definitions file when the service is launched

bool gen_world_file(a3env::BlockType world[a3env::MAP_WIDTH][a3env::MAP_WIDTH], int &xpos, int &ypos);
// function to save the received information to world.csp

std::string getExecutablePath();
// function to get the directory of the executable

bool get_moves(a3env::BlockType world[a3env::MAP_WIDTH][a3env::MAP_WIDTH], a3planner::plan::Request &req, std::vector<unsigned char> &moves);
// function to call PAT and read moves from out.txt

int manhattan_distance(int x1, int y1, int x2, int y2);
// function to calculate the manhattan distance between two coordinates (used to determine which agent should attempt a rescue)

bool init_rescue(a3planner::plan::Request &req, int surv_x, int surv_y);
// function to determine whether rescue should be made by this agent.

bool read_out(std::vector<unsigned char> &moves);
// function to read out.txt and place moves into provided vector.

// global variables
std::string exeDir = getExecutablePath();
std::string rootDir = exeDir.substr(0, exeDir.find("catkin_ws") - 1); // catkin_ws directory is located here
std::string worldDir = rootDir + "/catkin_ws/src/a3planner/pat/world.csp";
std::string defsDir = rootDir + "/catkin_ws/src/a3planner/pat/definitions.csp";
std::string PATDir = rootDir + "/MONO-PAT-v3.6.0/PAT3.Console.exe";
std::string outDir = rootDir + "/catkin_ws/src/a3planner/pat/out.txt";
std::string searchDir = rootDir + "/catkin_ws/src/a3planner/pat/search.csp";
std::string rescueDir = rootDir + "/catkin_ws/src/a3planner/pat/rescue.csp";
std::string targetDir = rootDir + "/catkin_ws/src/a3planner/pat/target.csp";
int maxsteps = 10;
bool attempt_rescue = false;
bool go_home = true;
int target_x = 0;
int target_y = 0;

bool plan_callback(a3planner::plan::Request &req, a3planner::plan::Response &res)
{
	ROS_INFO("Service call initiated");
	// check size matches
	if (req.world.size() != a3env::MAP_WIDTH * a3env::MAP_WIDTH)
	{
		ROS_ERROR("Received data array of incorrect size");
		return false;
	}
	// reset targeting
	attempt_rescue = false;
	go_home = true;
	target_x = req.home_row;
	target_y = req.home_col;
	// convert world to 2d array
	ROS_INFO("Constructing Matrix");
	a3env::BlockType world[a3env::MAP_WIDTH][a3env::MAP_WIDTH];
	for (int i = 0; i < a3env::MAP_WIDTH; i++)
	{
		for (int j = 0; j < a3env::MAP_WIDTH; j++)
		{
			a3env::BlockType block = static_cast<a3env::BlockType>(req.world[i * a3env::MAP_WIDTH + j]);
			switch (block)
			{
				case a3env::BLOCK_UNKNOWN:
					if (go_home)
					{
						target_x = i;
						target_y = j;
						go_home = false;
					}
					else if (manhattan_distance(req.row, req.col, i, j) < manhattan_distance(req.row, req.col, target_x, target_y))
					{
						target_x = i;
						target_y = j;
					}
					world[i][j] = block;
					break;
				case a3env::BLOCK_AIR:
					world[i][j] = block;
					break;
				case a3env::BLOCK_WALL:
					world[i][j] = block;
					break;
				case a3env::BLOCK_SURVIVOR:
					attempt_rescue = init_rescue(req, i, j);
					world[i][j] = block;
					break;
				default:
					ROS_ERROR("Unexpected data in world matrix: %d", block);
					return false;
			}
		}
	}
	ROS_INFO("Matrix Constructed Successfully.");
	ROS_INFO("Marking hostiles and agents in world view.");
	//insert hostiles and agents to world matrix
	for (int i=0; i < a3env::NUM_HOSTILES; i++)
	{
		if (req.hostile_cells[i] != 65535)
		{
			int hostile_x = req.hostile_cells[i] / a3env::MAP_WIDTH;
			int hostile_y = req.hostile_cells[i] % a3env::MAP_WIDTH;
			world[hostile_x][hostile_y] = a3env::BLOCK_WALL;
		}
	}
	for (int i=0; i < a3env::NUM_AGENTS; i++)
	{
		int agent_x = req.agent_cells[i] / a3env::MAP_WIDTH;
		int agent_y = req.agent_cells[i] % a3env::MAP_WIDTH;
		world[agent_x][agent_y] = a3env::BLOCK_WALL;
	}
	ROS_INFO("Marked hostiles and agents in world view successfully.");
	// get moves
	std::vector<unsigned char> moves;
	if (!get_moves(world, req, moves))
	{
		ROS_ERROR("Error fetching plan.");
		return false;
	}
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
	file << "#define Rows " << a3env::MAP_WIDTH << ";\n";
	file << "#define Cols " << a3env::MAP_WIDTH << ";\n";
	file << "#define vision 1;\n";

	file.close();
	return true;
}

bool gen_world_file(a3env::BlockType world[a3env::MAP_WIDTH][a3env::MAP_WIDTH], int &xpos, int &ypos)
{
	// open world.csp file to write to
	std::ofstream file(worldDir);
	if (!file.is_open())
	{
		ROS_INFO("Open world.csp failed.");
		return false;
	}
	ROS_INFO("Writing to world.csp");

	// write world matrix
	file << "var world[" << a3env::MAP_WIDTH << "][" << a3env::MAP_WIDTH << "]:{0.." << a3env::BLOCK_SURVIVOR << "} = [\n";
	for (int i = 0; i < a3env::MAP_WIDTH; i++)
	{
		for (int j = 0; j < a3env::MAP_WIDTH; j++)
		{
			file << world[i][j];
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
	file << "var xtarget:{0.." << a3env::MAP_WIDTH - 1 << "} = " << target_x << ";\n";
	file << "var ytarget:{0.." << a3env::MAP_WIDTH - 1 << "} = " << target_y << ";\n";
	file << "#define maxSteps " << maxsteps << ";\n";

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

bool get_moves(a3env::BlockType world[a3env::MAP_WIDTH][a3env::MAP_WIDTH], a3planner::plan::Request &req, std::vector<unsigned char> &moves)
{
	// write world.csp
	if (!gen_world_file(world, req.row, req.col))
	{
		ROS_ERROR("Write to world.csp failed.");
		return false;
	}

	// run PAT
	ROS_INFO("Calculating a path to optimize exploration");
	// select file to run
	std::string processDir = targetDir;
	if(attempt_rescue)
	{
		processDir = rescueDir;
	}
	else if(!go_home)
	{
		processDir = searchDir;
	}
	// PAT system call
	if (std::system(("mono " + PATDir + " -engine 1 " + processDir + " " + outDir).c_str()) < 0)
	{
		ROS_ERROR("Fatal error in system call.");
		return false;
	}
	// read out.txt
	if (!read_out(moves))
	{
		return false;
	}
	if (moves.size() == 0 && processDir != targetDir) // run target.csp if first model assertion was invalid
	{
		// PAT system call
		if (std::system(("mono " + PATDir + " -engine 1 " + targetDir + " " + outDir).c_str()) < 0)
		{
			ROS_ERROR("Fatal error in system call.");
			return false;
		}
		// read out.txt
		if (!read_out(moves))
		{
			return false;
		}
	}
	return true;
}

int manhattan_distance(int x1, int y1, int x2, int y2)
{
	return abs(x1 - x2) + abs(y1 - y2);
}

bool init_rescue(a3planner::plan::Request &req, int surv_x, int surv_y)
{
	std::vector<int> dists;
	for (int i=0; i < a3env::NUM_AGENTS; i++)
	{
		int agent_x = req.agent_cells[i] / a3env::MAP_WIDTH;
		int agent_y = req.agent_cells[i] % a3env::MAP_WIDTH;
		int dist = manhattan_distance(agent_x, agent_y, surv_x, surv_y);
		auto it = std::lower_bound(dists.begin(), dists.end(), dist);
		dists.insert(it, dist);
	}
	int agent_x = req.row;
	int agent_y = req.col;
	int dist = manhattan_distance(agent_x, agent_y, surv_x, surv_y);
	if (dist > dists[dists.size() / 2] || (attempt_rescue && dist > manhattan_distance(agent_x, agent_y, target_x, target_y)))
	{
		return attempt_rescue;
	}
	else
	{
		target_x = surv_x;
		target_y = surv_y;
		return true;
	}
}

bool read_out(std::vector<unsigned char> &moves)
{
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