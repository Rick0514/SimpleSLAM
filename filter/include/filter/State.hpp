#pragma once

#include <kalman/Matrix.hpp>

namespace filter {

static constexpr int N = 3;

template<typename T>
class State : public Kalman::Vector<T, N>
{

public:

    KALMAN_VECTOR(State, T, N) 
    
    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! Orientation
    static constexpr size_t YAW = 2;
    
    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    T yaw()   const { return (*this)[ YAW ]; }
    
    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& yaw()  { return (*this)[ YAW ]; }

};

}