/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <robot_ekf/odom_estimation_node.h>
#include <robot_ekf/param.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <spdlog/fmt/fmt.h>

using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;


static const double EPS = 1e-5;


//#define __EKF_DEBUG_FILE__

namespace estimation
{
  // constructor
  	OdomEstimationNode::OdomEstimationNode()
		: odom_active_(false),
		imu_active_(false),
		odom_initializing_(false),
		imu_initializing_(false),
		odom_covariance_(6),
		imu_covariance_(3),
		odom_callback_counter_(0),
		imu_callback_counter_(0),
		ekf_sent_counter_(0)
  	{
		ros::NodeHandle nh_private("~");
		ros::NodeHandle nh;

		auto p = Param::getInstance();

		// paramters
		nh_private.param("sensor_timeout", timeout_, 1.0);
		nh_private.param("debug",   debug_, false);
		nh_private.param("self_diagnose",  self_diagnose_, false);
		double freq;
		nh_private.param("freq", freq, 30.0);

		output_frame_ = p.output_frame;
		base_footprint_frame_ = p.base_frame;

		ROS_INFO_STREAM("output frame: " << output_frame_);
		ROS_INFO_STREAM("base frame: " << base_footprint_frame_);

		// timer_ = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &OdomEstimationNode::spin, this);

		// advertise our estimation
		// pose_pub_ = nh_private.advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_combined", 10);

		// initialize
		filter_stamp_ = ros::Time(0);

		// subscribe to odom messages
		// if (odom_used_){
		// 	ROS_DEBUG("Odom sensor can be used");
		// 	odom_sub_ = nh.subscribe("odom", 10, &OdomEstimationNode::odomCallback, this);
		// }
		// else ROS_DEBUG("Odom sensor will NOT be used");

		// subscribe to imu messages
		// if (imu_used_){
		// 	ROS_DEBUG("Imu sensor can be used");
		// 	imu_sub_ = nh.subscribe("imu_data", 10,  &OdomEstimationNode::imuCallback, this);
		// }
		// else ROS_DEBUG("Imu sensor will NOT be used");
	}

	// destructor
	OdomEstimationNode::~OdomEstimationNode(){
		if(tum.is_open())	tum.close();
	};

	// callback function for odom data
	void OdomEstimationNode::odomCallback(const OdomConstPtr& odom)
	{
		odom_callback_counter_++;

		// receive data 
		odom_stamp_ = odom->header.stamp;
		odom_time_ = odom_stamp_;
		Quaternion q;
		tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
		odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
		for (unsigned int i=0; i<6; i++)
			for (unsigned int j=0; j<6; j++)
				odom_covariance_(i+1, j+1) = odom->pose.covariance[6*i+j];

		my_filter_.addMeasurement(StampedTransform(odom_meas_.inverse(), odom_stamp_, base_footprint_frame_, Param::getInstance().wheel_frame), odom_covariance_);
    
		// activate odom
		if (!odom_active_) {
			if (!odom_initializing_){
				odom_initializing_ = true;
				odom_init_stamp_ = odom_stamp_;
				ROS_INFO("Initializing Odom sensor");      
			}
			if ( filter_stamp_ >= odom_init_stamp_){
				odom_active_ = true;
				odom_initializing_ = false;
				ROS_INFO("Odom sensor activated");      
			}
			else ROS_DEBUG("Waiting to activate Odom, because Odom measurements are still %f sec in the future.", 
					(odom_init_stamp_ - filter_stamp_).toSec());
		}
  	};

	// callback function for imu data
	void OdomEstimationNode::imuCallback(const ImuConstPtr& imu)
	{
		imu_callback_counter_++;

		// receive data 
		imu_stamp_ = imu->header.stamp;
		tf::Quaternion orientation;
		quaternionMsgToTF(imu->orientation, orientation);
		imu_meas_ = tf::Transform(orientation, tf::Vector3(0,0,0));
		for (unsigned int i=0; i<3; i++)
			for (unsigned int j=0; j<3; j++)
				imu_covariance_(i+1, j+1) = imu->orientation_covariance[3*i+j];

		imu_time_  = imu_stamp_;

		// manually set covariance untile imu sends covariance
		// if (imu_covariance_(1,1) == 0.0){
		// }
		SymmetricMatrix measNoiseImu_Cov(3);  
		measNoiseImu_Cov = 0;
		double ns = 0.5 / 180 * M_PI;
		measNoiseImu_Cov(1,1) = pow(ns, 2);  // = 0.01 degrees / sec
		measNoiseImu_Cov(2,2) = pow(ns, 2);  // = 0.01 degrees / sec
		measNoiseImu_Cov(3,3) = pow(ns, 2);  // = 0.01 degrees / sec
		imu_covariance_ = measNoiseImu_Cov;

		my_filter_.addMeasurement(StampedTransform(imu_meas_.inverse(), imu_stamp_, base_footprint_frame_, Param::getInstance().imu_frame), imu_covariance_);

		// activate imu
		if (!imu_active_) {
			if (!imu_initializing_){
				imu_initializing_ = true;
				imu_init_stamp_ = imu_stamp_;
				ROS_INFO("Initializing Imu sensor");      
			}
			if ( filter_stamp_ >= imu_init_stamp_){
				imu_active_ = true;
				imu_initializing_ = false;
				ROS_INFO("Imu sensor activated");      
			}
			else ROS_DEBUG("Waiting to activate IMU, because IMU measurements are still %f sec in the future.", 
				(imu_init_stamp_ - filter_stamp_).toSec());
		}

	};

	// filter loop
	void OdomEstimationNode::spin(const ros::TimerEvent& e)
	{
		ROS_DEBUG("Spin function at time %f", e.current_real.toSec());

		// check for timing problems
		if ( (odom_initializing_ || odom_active_) && (imu_initializing_ || imu_active_) ){
			double diff = fabs( Duration(odom_stamp_ - imu_stamp_).toSec() );
			if (diff > 1.0) ROS_ERROR("Timestamps of odometry and imu are %f seconds apart.", diff);
		}
		
		// initial value for filter stamp; keep this stamp when no sensors are active
		filter_stamp_ = e.current_real;
		
		// check which sensors are still active
		if ((odom_active_ || odom_initializing_) && 
			(filter_stamp_ - odom_time_).toSec() > timeout_){
			odom_active_ = false; odom_initializing_ = false;
			ROS_INFO("Odom sensor not active any more");
		}
		if ((imu_active_ || imu_initializing_) && 
			(filter_stamp_ - imu_time_).toSec() > timeout_){
			imu_active_ = false;  imu_initializing_ = false;
			ROS_INFO("Imu sensor not active any more");
		}

    
		// only update filter when one of the sensors is active
		if (odom_active_ || imu_active_){
			
			// update filter at time where all sensor measurements are available
			if (odom_active_)  filter_stamp_ = min(filter_stamp_, odom_stamp_);
			if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
      
			// update filter
			if ( my_filter_.isInitialized() )  {
				bool diagnostics = true;
				if (my_filter_.update(odom_active_, imu_active_, filter_stamp_, diagnostics)){
				
					// output most recent estimate and relative covariance
					my_filter_.getEstimate(output_);
					// pose_pub_.publish(output_);
					ekf_sent_counter_++;
					
					auto q = output_.pose.pose.orientation;
					auto p = output_.pose.pose.position;
					auto tum_str = fmt::format("{} {} {} {} {} {} {} {}", filter_stamp_.toSec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w);
					tum << tum_str << std::endl;
        		}
				if (self_diagnose_ && !diagnostics)
				ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
			}

			if (odom_active_ && !my_filter_.isInitialized()){
				my_filter_.initialize(odom_meas_, odom_stamp_);
				ROS_INFO("Kalman filter initialized with odom measurement");
			}
		}
	}

};

using namespace estimation;
int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "robot_ekf");

	// create filter class
	OdomEstimationNode my_filter_node;
	std::ofstream tum;
//   ros::spin();

// run rosbag
	rosbag::Bag bag;
	std::string bag_fn;
#ifdef DATA_FILE
	bag_fn = std::string(DATA_FILE "/arb.bag");
	my_filter_node.tum = std::ofstream(DATA_FILE "/rekf.txt");
#endif

	bag.open(bag_fn, rosbag::bagmode::Read);
	std::string imu_topic = "/imu/data";
	std::string odom_topic = "/odom/raw";

	std::vector<std::string> topics{imu_topic, odom_topic};
	double start_time = -1;
	for(const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topics)))
	{
		if(start_time < 0)  start_time = m.getTime().toSec();
		if(m.getTime().toSec() - start_time > 50){
			std::cout << "50s pass..." << std::endl;
			start_time = m.getTime().toSec();
		}

		if(m.getTopic() == odom_topic){
			nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
			// pred freq equal to wheel
			my_filter_node.odomCallback(odom);
			ros::TimerEvent e;
			e.current_real = odom->header.stamp;
			my_filter_node.spin(e);
		}else if(m.getTopic() == imu_topic){
			sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
			my_filter_node.imuCallback(imu);
		}
	}

	bag.close();
	
	return 0;
}
