#include <ros/ros.h>
#include <dataproxy/EkfOdomProxy.hpp>

// test safe queue
// test dataproxy

using namespace dataproxy;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_dataproxy");
    ros::NodeHandle nh;

    int q_size = 10;
    dataproxy::EkfOdomProxy<true> eop(nh, q_size);
    
    return 0;
}
