#pragma once

#include <fstream>
#include <types/EigenTypes.hpp>
#include <spdlog/fmt/fmt.h>
#include <boost/filesystem.hpp>

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

}

}