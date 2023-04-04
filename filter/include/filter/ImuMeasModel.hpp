#pragma once

#include <filter/State.hpp>
#include <kalman/LinearizedMeasurementModel.hpp>

namespace filter {

template<typename T>
class ImuMeas : public Kalman::Vector<T, 1>
{
public:
    // only observe yaw for now
    KALMAN_VECTOR(ImuMeas, T, 1)

    T yaw() const { return (*this)[0]; }
    T& yaw() { return (*this)[0]; }
};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class ImuMeasModel : public Kalman::LinearizedMeasurementModel<State<T>, ImuMeas<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  filter::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  filter::ImuMeas<T> M;

    ImuMeasModel(){
        this->V.setIdentity();
    }

    M h(const S& x) const
    {
        M m;
        m.yaw() = x.yaw();
        return m;
    }

protected:

    void updateJacobians( const S& x )
    {
        // H = dh/dx (Jacobian of measurement function w.r.t. the state)
        this->H.setZero();
        this->H(0, S::YAW) = 1.0;
    }

};

}