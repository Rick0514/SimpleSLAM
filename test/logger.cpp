#include <utils/Logger.hpp>
#include <thread>

using namespace utils;

void test_single_thread()
{
    auto lg = logger::Logger::getInstance();

    lg->trace("console trace");
    lg->info("console info");
    lg->warn("console warn");
    lg->error("console error");

    // print for 5s
    for(int i=0; i<5; i++){
        lg->info("counting: {}/5...", i);
        sleep(1);
    }
}

void test_multi_threads()
{
    auto lg = logger::Logger::getInstance();

    auto printInfo = [&](){
        for(int i=0; i<7; i++){
            lg->info("printing info...");
            sleep(1);
        }
    };

    auto printWarn = [&](){
        for(int i=0; i<5; i++){
            lg->warn("printing warn...");
            sleep(1);
        }
    };

    std::thread t1(printInfo);
    sleep(1);
    std::thread t2(printWarn);

    t1.join();
    t2.join();

}

void test_log_vector()
{
    auto lg = logger::Logger::getInstance();
    std::vector<int> a{1,2,3,4,5};
    lg->info("{}", a);    
}

int main(int argc, char const *argv[])
{
    // test_single_thread();
    // test_multi_threads();
    test_log_vector();
    return 0;
}
