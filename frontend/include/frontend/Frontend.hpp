#include <types/EigenTypes.hpp>

#include <thread>

using namespace EigenTypes;

namespace frontend
{
class Frontend
{
private:

    Pose6d mOdom2Map;


public:
    Frontend();



    void publish() const;


    ~Frontend();
};

    
} // namespace frontend


