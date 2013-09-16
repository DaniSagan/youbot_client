#include "youbot_client/app.h"
#include <ros/ros.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "youbot_client");
    ros::NodeHandle node_handle;
    
    App app(node_handle);
    app.Initialize();
    
    std::string res_path = argv[0];
    res_path.erase(res_path.end() - 10, res_path.end());
    if (app.LoadAssets(res_path))
    {
        app.Run();
    }
    else
    {
        ROS_ERROR("Could not load assets. Quitting...");
    }
    
    return 0;    
}
