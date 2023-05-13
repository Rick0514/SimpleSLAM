#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dataproxy/EkfOdomProxy.hpp>
#include <dataproxy/LidarDataProxy.hpp>

#include <utils/Logger.hpp>

using namespace dataproxy;
using namespace utils;
using namespace std;

class Test
{
protected:
    atomic_bool running{true};
    std::thread thd;
public:

    Test(LidarDataProxy& ldp)
    {
        thd = std::move(std::thread(&Test::test_blocking, this, std::ref(ldp)));
    }

    void test_blocking(LidarDataProxy& ldp)
    {
        auto lg = logger::Logger::getInstance();
        lg->info("test blocking pc dataproxy!!");

        string bag_fn = "/home/hgy/sda2/rosbag/huaqiaocheng-1.bag";
        rosbag::Bag bag;
        bag.open(bag_fn, rosbag::bagmode::Read);
        for(const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery("/lidar_points")))
        {
            sensor_msgs::PointCloud2ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
            if(pc){
                auto cloud = pcl::make_shared<pc_t>();
                pcl::fromROSMsg(*pc, *cloud);
                pcl_conversions::toPCL(pc->header.stamp, cloud->header.stamp);
                lg->info("pre deque size {}", ldp.get()->size());

                #if PCL_VERSION_COMPARE(<=, 1, 10, 0)
                    auto stdcloud = utils::make_shared_ptr(cloud);
                    ldp->subscribe(stdcloud);
                #else
                    ldp.subscribe(cloud);
                #endif
                lg->info("push one");
                this_thread::sleep_for(chrono::milliseconds(200));
            }
            if(!running.load())    break;
        }
        
        bag.close();
    }

    ~Test(){
        cout << "prepare Test is destroy!!" << endl;
        running.store(false);
        if(thd.joinable())  thd.join();
        cout << "Test is destroy!!" << endl;
    }
};

class Consumer
{
public:
    atomic_bool running{true};
    shared_ptr<logger::Logger> lg;
    unique_ptr<std::thread> thd;
    LidarDataProxy::DataPtr data;

    Consumer(const LidarDataProxy::DataPtr& d) : data(d){
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
        cout << "Consumer is destroy!!" << endl;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_dataproxy");
    ros::NodeHandle nh;

    if(nh.ok()){
        int q_size = 10;
        LidarDataProxy ldp(nh, q_size);
        Test t(ldp);
        Consumer c(ldp.get());
        ros::spin();
        // object who carry dataproxy should abort it explicitly in destructor in usebag mode, 
        // or it wont exit because of cv
        ldp.get()->abort();
    }

    return 0;
}


