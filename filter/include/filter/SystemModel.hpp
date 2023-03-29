#pragma once

#include <cmath>
#include <kalman/LinearizedSystemModel.hpp>

namespace filter
{

template<typename T>
class State : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(State, T, 3)
    
    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! Orientation
    static constexpr size_t THETA = 2;
    
    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    T theta()   const { return (*this)[ THETA ]; }
    
    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& theta()  { return (*this)[ THETA ]; }
};

template<typename T>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>>
{
public:
    //! State type shortcut definition
	typedef State<T> S;
    
    S f(const S& x) const
    {
        //! Predicted state vector after transition
        S x_;
        
        // keep still for temp
        x_.x() = x.x();
        x_.y() = x.y();
        x_.theta() = x.theta();
        
        return x_;
    }
    
protected:
    
    void updateJacobians( const S& x )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setIdentity();

        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }
};

} // namespace filter
