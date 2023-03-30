#pragma once

#include <filter/State.hpp>
#include <kalman/LinearizedMeasurementModel.hpp>

namespace filter {

template<typename T>
class WheelMeas : public Kalman::Vector<T, 2>
{
public:
    // only observe yaw for now
    KALMAN_VECTOR(WheelMeas, T, 2)

    T x() const { return (*this)[0]; }
    T& x() { return (*this)[0]; }

    T y() const { return (*this)[0]; }
    T& y() { return (*this)[0]; }
};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class WheelMeasModel : public Kalman::LinearizedMeasurementModel<State<T>, WheelMeas<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  filter::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  filter::WheelMeas<T> M;
    
    WheelMeasModel() = default;

    M h(const S& x) const
    {
        M m;
        m.x() = x.x();
        m.y() = x.y();
        return m;
    }

protected:

    void updateJacobians( const S& x )
    {
        this->H.setZero();
        this->H(0, S::X) = 1.0;
        this->H(1, S::Y) = 1.0;
    }

};

}