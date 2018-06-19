#include <pathFollower/manager.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "manager_node");

    Manager manager;

    while (ros::ok())
    {
        sleep(1);
    }

    return 0;
}