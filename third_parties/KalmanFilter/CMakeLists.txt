cmake_minimum_required(VERSION 3.0.0)
project(KalmanFilter CXX)

set(KalmanFilterBase
	# Utils
    include/kalman/Matrix.hpp
    include/kalman/Types.hpp
	
	# Base
	include/kalman/KalmanFilterBase.hpp
	include/kalman/StandardBase.hpp
	include/kalman/SquareRootBase.hpp
	include/kalman/StandardFilterBase.hpp
	include/kalman/SquareRootFilterBase.hpp
)

set(KalmanFilterModel
	# Models
	include/kalman/MeasurementModel.hpp
	include/kalman/SystemModel.hpp
	include/kalman/LinearizedMeasurementModel.hpp
	include/kalman/LinearizedSystemModel.hpp
)

set(KalmanFilterEKF
	# EKF
	include/kalman/ExtendedKalmanFilter.hpp
	include/kalman/SquareRootExtendedKalmanFilter.hpp
)

set(KalmanFilterUKF
    # UKF
    include/kalman/UnscentedKalmanFilterBase.hpp
    include/kalman/UnscentedKalmanFilter.hpp
    include/kalman/SquareRootUnscentedKalmanFilter.hpp    
)