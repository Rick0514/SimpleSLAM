#include <stdio.h>
#include <memory>
#include <thread>
#include <random>
#include <mutex>
#include <atomic>
#include <utils/SafeDeque.hpp>
#include <utils/NonCopyable.hpp>

using namespace std;

template<typename T>
using SD = utils::concurrency::SafeDeque<T>;

int getRandomNum()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(1, 9);
    return distrib(gen);
}

template<bool blocking>
class Worker : private utils::noncopyable::NonCopyable
{
private:
    int tid;
    std::thread work;
    std::atomic_bool running{true};

    std::weak_ptr<SD<int>> sd_ptr; 

public:
    explicit Worker(int id, shared_ptr<SD<int>>& ptr) : tid(id), sd_ptr(ptr){
        printf("construct worker %d\n", tid);
    }

    Worker(Worker&& w){
        tid = w.tid;
        sd_ptr = w.sd_ptr;
    }
    
    void start(){
        work = std::move(thread(&Worker<blocking>::run, this));
    }

    void run(){
        while (running.load())
        {
            auto rand_num = std::make_shared<int>(getRandomNum());
            auto sd = sd_ptr.lock();
            if(sd){
                printf("thread [%d] produce num : %d\n", tid, *rand_num);
                sd->push_back<blocking>(rand_num);
                printf("thread [%d] queue size: %d!!\n", tid, sd->size());
            }else{
                printf("ptr is dead??\n");
            }
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
    
    ~Worker(){
        running.store(false);
        if(work.joinable()) work.join();
        printf("exit worker %d\n", tid);
    }
};

class Consumer : private utils::noncopyable::NonCopyable
{
private:
    int tid;
    std::thread cons;
    std::atomic_bool running{true};

    std::weak_ptr<SD<int>> sd_ptr; 
public:
    explicit Consumer(int id, shared_ptr<SD<int>>& ptr): tid(id), sd_ptr(ptr){
        printf("construct consumer %d\n", tid);
    }

    Consumer(Consumer&& c)
    {
        tid = c.tid;
        sd_ptr = c.sd_ptr;
    }

    void start(){
        cons = std::move(thread(&Consumer::run, this));
    }

    void run()
    {
        while (running.load())
        {
            auto sd = sd_ptr.lock();
            if(sd){
                printf("thread [%d] consume front!\n", tid);
                printf("thread [%d] queue size: %d!!\n", tid, sd->size());
                auto got = sd->consume_front();
                if(got)
                    printf("thread [%d] got %d\n", tid, *got);
                else
                    printf("deque is empty!!\n");
            }
            this_thread::sleep_for(chrono::seconds(2));
        }
    }

    ~Consumer(){
        running.store(false);
        if(cons.joinable()) cons.join();
        printf("exit consumer %d\n", tid);
    }
};

template <bool blocking>
class TestSafeDeque
{
private:
    int qs;
    vector<Worker<blocking>> wks;
    vector<Consumer> cons;

    shared_ptr<SD<int>> sd_ptr;

public:

    TestSafeDeque(int nums) : qs(nums){
        sd_ptr = make_shared<SD<int>>(qs);
    }

    void run(int work_nums, int consumer_nums){
        
        wks.reserve(work_nums);
        cons.reserve(consumer_nums);

        for(int i=0; i<work_nums; i++){
            wks.emplace_back(std::move(Worker<blocking>(i, sd_ptr)));
        }

        for(int i=work_nums; i<work_nums + consumer_nums; i++){
            cons.emplace_back(std::move(Consumer(i, sd_ptr)));
        }

        // start it
        for(auto&& w : wks)     w.start();
        for(auto&& c : cons)    c.start();
        
    }

    ~TestSafeDeque(){
        sd_ptr->abort();
        printf("deconstruct TestSafeDeque!!\n");
    }

};


int main(int argc, char const *argv[])
{
    constexpr bool blocking{true};    
    int q_size{5};
    int worker_nums{2};
    int consumer_nums{1};

    {
        TestSafeDeque<blocking> tbq(q_size);    
        tbq.run(worker_nums, consumer_nums);

        this_thread::sleep_for(chrono::seconds(5));
    }

    printf("exit test\n");

    return 0;
}
