#pragma once

#include <fstream>
#include <types/EigenTypes.hpp>
#include <spdlog/fmt/fmt.h>
#include <boost/filesystem.hpp>

#include <map>

namespace utils {

namespace file {

using namespace EigenTypes;
namespace bfs = boost::filesystem;

inline void removeTum(const std::string& dir)
{
    auto tum = fmt::format("{}/tum.txt", dir);
    if(bfs::exists(tum)){
        bfs::remove(tum);
    }
}

template <typename Scalar>
void writeAsTum(const std::string& dir, double stamp, const Pose6<Scalar>& p)
{
    auto tum = fmt::format("{}/tum.txt", dir);
    V3<Scalar> t = p.translation();
    Qt<Scalar> q(p.rotation());
    // stamp to ms, trans to mm, rot save 6 precision
    auto msg = fmt::format("{:.3f} {:.3f} {:.3f} {:.3f} {:.6f} {:.6f} {:.6f} {:.6f}\n", stamp, t(0), t(1), t(2), q.x(), q.y(), q.z(), q.w());
    // perhaps the first time or rebuild map
    std::ofstream out;

    if(!bfs::exists(tum)){
        out.open(tum, std::ios::out);
    }else{
        // append
        out.open(tum, std::ios::app);
    }
    
    out << msg;                
    out.close();
}

// container should be stl iterable container
// item must be struct with member pose and pc
template<typename Container, typename Item = typename Container::value_type>
void writeAsTum(const std::string &dir, const Container& con)
{
    auto tum = fmt::format("{}/tum.txt", dir);
    using Scalar = typename Item::Scalar;
    std::ofstream out(tum, std::ios::out);

    for(const auto& e : con)
    {
        double stamp = (double)e.pc->header.stamp / 1e6;
        V3<Scalar> t = e.pose.translation();
        Qt<Scalar> q(e.pose.rotation());
        auto msg = fmt::format("{:.3f} {:.3f} {:.3f} {:.3f} {:.6f} {:.6f} {:.6f} {:.6f}\n", stamp, t(0), t(1), t(2), q.x(), q.y(), q.z(), q.w());
        out << msg;        
    }

    out.close();
}

template<typename Scalar>
std::vector<std::pair<double, Pose6<Scalar>>> loadFromTum(const std::string& dir)
{
    auto tum = fmt::format("{}/tum.txt", dir);
    std::vector<std::pair<double, Pose6<Scalar>>> res;
    std::ifstream inf(tum);

    if(inf.is_open())
    {
        std::string line;
        while(std::getline(inf, line))
        {
            std::stringstream ss(line);
            double key;
            ss >> key;
            V3<Scalar> v;
            Qt<Scalar> q;
            for(int i=0; i<3; i++)  ss >> v(i);
            ss >> q.x() >> q.y() >> q.z() >> q.w();
            Pose6<Scalar> p;
            p.setIdentity();
            p.translate(v);
            p.rotate(q);
            res.emplace_back(std::make_pair(key, p));
        }
    }
    return res;
}

}

}