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

void progressBar(float perc)
{
    #define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
    #define PBWIDTH 60
    int val = (int) (perc * 100);
    int lpad = (int) (perc * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

void test_progress_bar()
{
    auto lg = logger::Logger::getInstance();
    for(int i=0; i<=10; i++){
        progressBar(0.1f * i);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

int main(int argc, char const *argv[])
{
    // test_single_thread();
    // test_multi_threads();
    // test_log_vector();
    test_progress_bar();
    return 0;
}
