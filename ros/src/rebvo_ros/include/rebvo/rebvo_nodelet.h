/**
 *  \author     Claus Smitt <claus.smitt@ib.edu.ar.com>
 *
 * ROS Wrapper for REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
 * Copyright (C) 2016  Juan José Tarrio

 * Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
 * for a Monocular Camera. In Proceedings of the IEEE International Conference
 * on Computer Vision (pp. 702-710).

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 */

#ifndef REBVO_INCLUDE_REBVO_REBVO_NODELET_H_
#define REBVO_INCLUDE_REBVO_REBVO_NODELET_H_

#include <string>

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>

#include "rebvo/rebvo.h"

namespace rebvo {
class RebvoNodelet: public nodelet::Nodelet {
public:
	RebvoNodelet();
	~RebvoNodelet();

private:

	virtual void onInit();

	/**
	 * Load rebvo parameters
	 */
	void construct();
	/**
	 * edgeMap subscription callback
	 */
	void connectCb();
	/**
	 * edgeMap un-subscription callback
	 */
	void disconnectCb();
	/**
	 * Intra-process Camera image callback
	 * @param image constant pointer to published image
	 */
	void imageCb(const sensor_msgs::ImageConstPtr &image);
	/**
	 * Intra-process Imu data callback
	 * @param imuData constant pointer to published imu data
	 */
	void imuCb(const sensor_msgs::ImuConstPtr &imuMsg);
	/**
	 * edgeMap puclish callback.
	 * Intended to be called by rebvoLib pipeline's final stage
	 */
	bool edgeMapPubCb(rebvo::PipeBuffer &edgeMapRebvo);

	ros::NodeHandle nh_, private_nh_; //Private for namespace rebvo
	ros::Subscriber camera_sub_, imu_sub_;
	ros::Publisher edgeMap_pub_;
    ros::Publisher pointCloud_pub_;
	//ros::Timer clean_timer_;

	std::string imuTopic_, imageTopic_;
    std::string frame_id_cam;
    std::string frame_id_robot;
    tf::TransformBroadcaster tf_broad;
    tf::Transform tf_cam2robot;


	std::unique_ptr<rebvo::REBVO> rebvo_;

};

bool imgMsg2Rebvo(const sensor_msgs::ImageConstPtr &imgMsg,
		std::shared_ptr<rebvo::Image<rebvo::RGB24Pixel> > &imgRebvo);

bool imuMsg2Rebvo(const sensor_msgs::ImuConstPtr &imuMsg,
		rebvo::ImuData &imuRebvo);

//edgeMap2msg(const revboLib::keyLine * keylines, rebvo::EdgeMapMsg &edgeMap);

}// namespace rebvo

#endif /* REBVO_INCLUDE_REBVO_REBVO_NODELET_H_ */
