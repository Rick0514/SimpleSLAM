#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <mutex>
#include <utils/SafeDeque.hpp>

using namespace std;

int getRandomNum()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(1, 9);
    return distrib(gen);
}

class TestBlockingDeque
{
private:
    int qs;
    unique_ptr<utils::concurrency::SafeDeque<int, true>> sd_ptr; 

    std::mutex mtx;

public:
    TestBlockingDeque(int nums) : qs(nums){
        sd_ptr = make_unique<utils::concurrency::SafeDeque<int, true>>(qs);
    }

    void worker(){
        lock_guard<mutex> lk(mtx);
        auto rand_num = getRandomNum();
        cout << "now deque size: " << sd_ptr->size();
        cout << "produce nums: " << rand_num << endl;
        sd_ptr->push_back(rand_num);
    }

    void consumer()
    {
        lock_guard<mutex> lk(mtx);
        cout << "time to consume!!" << endl;
        int got;
        if(sd_ptr->front(got)){
            cout << "get value: " << got << " and pop it" << endl;
            sd_ptr->pop_front();
        }
    }

}




int main(int argc, char const *argv[])
{
    

    return 0;
}
