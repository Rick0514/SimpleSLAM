#pragma once

#include <cmath>
#include <filter/State.hpp>
#include <kalman/LinearizedSystemModel.hpp>

namespace filter
{

template<class T, class ControlType = Kalman::Vector<T, 0>, template<class> class CovarianceBase = Kalman::StandardBase >
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, ControlType, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef State<T> S;
    typedef Kalman::LinearizedSystemModel<State<T>> Base;

    using typename Base::Control;

    virtual S f(const S& x, const Control& u) const override
    {
        return x;
    }
    
protected:
    
    virtual void updateJacobians(const S& x, const Control& u) override
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
