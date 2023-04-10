#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <dataproxy/EkfOdomProxy.hpp>
#include <dataproxy/LidarDataProxy.hpp>

#include <utils/Logger.hpp>

using namespace dataproxy;
using namespace utils;
using namespace std;

using PCType = PCxyz;
static constexpr bool UseBag = true;

class Test
{
protected:
    atomic_bool running{true};
    std::thread thd;
public:

    Test(LidarDataProxy<PCType, UseBag>& ldp)
    {
        thd = std::move(std::thread(&Test::test_blocking, this, std::ref(ldp)));
    }

    void test_blocking(LidarDataProxy<PCType, UseBag>& ldp)
    {
        auto lg = logger::Logger::getInstance();
        lg->info("test blocking pc dataproxy!!");

        string bag_fn = "/home/rick/jtcx/hgy_gitee/minimal_ws/bag/huaqiaocheng-1.bag";
        rosbag::Bag bag;
        bag.open(bag_fn, rosbag::bagmode::Read);
        for(const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery("/lidar_points")))
        {
            sensor_msgs::PointCloud2ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
            if(pc){
                lg->info("pre deque size {}", ldp.get()->size());
                ldp.subscribe(pc);
                lg->info("push one");
                this_thread::sleep_for(chrono::milliseconds(200));
            }
            if(!running.load())    break;
        }
        
        bag.close();
    }

    ~Test(){
        running.store(false);
        if(thd.joinable())  thd.join();
    }
};

class Consumer
{
public:
    atomic_bool running{true};
    shared_ptr<logger::Logger> lg;
    unique_ptr<std::thread> thd;
    DataProxy<PCType, UseBag>::DataPtr data;

    Consumer(const DataProxy<PCType, UseBag>::DataPtr& d) : data(d){
        lg = logger::Logger::getInstance();
        thd = make_unique<std::thread>(&Consumer::run, this);
    }

    void run()
    {
        while(running.load()){
            lg->info("before consume front {}!!", data->size());
            data->consume_front();
            lg->info("after consume front {}!!", data->size());
            this_thread::sleep_for(chrono::seconds(1));
        }
    }

    ~Consumer()
    {
        running.store(false);
        if(thd->joinable()) thd->join();
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_dataproxy");
    ros::NodeHandle nh;

    if(nh.ok()){
        int q_size = 10;
        // EkfOdomProxy eop(nh, q_size);
        LidarDataProxy<PCType, UseBag> ldp(nh, q_size);
        Test t(ldp);
        Consumer c(ldp.get());
        ros::spin();
        ldp.get()->abort();
    }

    return 0;
}


