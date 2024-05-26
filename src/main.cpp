#include <ros/ros.h>

#include "a3planner/plan.h"


/*
    This is just a mock planner which hosts the /a3planner/plan as a starting point.
*/



bool plan_callback( a3planner::plan::Request &req, a3planner::plan::Response &res )
{
    // std::cout << "Plan requested!\n";
    // std::cout << "Origin: " << req.row << ", " << req.col << "\n";

    // for (int i=0; i<144; i++)
    // {
    //     std::cout << int(req.world[i]) << " ";

    //     if ((i+1)%12 == 0)
    //     {
    //         std::cout << "\n";
    //     }
    // }
    // std::cout << "\n";

    return true;
}


int main( int argc, char **argv )
{
    ros::init(argc, argv, "a3planner");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("a3planner/plan", plan_callback);
    ros::spin();

    return 0;
}


