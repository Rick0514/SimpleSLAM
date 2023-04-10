#include <iostream>
#include <chrono>
#include <thread>
#include <utils/Thread.hpp>
#include <utils/Logger.hpp>
#include <memory>

using namespace std;
using namespace utils;
using namespace chrono_literals;

void coutEverySeconds()
{
    logger::Logger::getInstance()->info("hello");
    this_thread::sleep_for(1s);
}

void TEST_function()
{
    utils::trd::ResidentThread rt(coutEverySeconds);

    this_thread::sleep_for(2s);

    rt.Stop();

    this_thread::sleep_for(1s);
    
    rt.Resume();

    this_thread::sleep_for(2s);
}

class TestMemberFunc
{

protected:

    std::unique_ptr<utils::trd::ResidentThread> rt;

public:

    TestMemberFunc()
    {
        rt = std::make_unique<utils::trd::ResidentThread>(&TestMemberFunc::echo, this);
    }

    void echo()
    {
        logger::Logger::getInstance()->info("i am A");
        this_thread::sleep_for(0.5s);
    }
};

void test_lambda()
{
    utils::trd::ResidentThread rt([](){
        logger::Logger::getInstance()->info("test lambda");
        this_thread::sleep_for(1s);
    });
    this_thread::sleep_for(3s);
}

int main()
{

    TestMemberFunc tfm;
    TEST_function();
    test_lambda();
    
    return 0;
}