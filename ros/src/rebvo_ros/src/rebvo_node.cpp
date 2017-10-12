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

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "rebvo_node");

	//ros::NodeHandle nh;

	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	std::string nodelet_name = ros::this_node::getName();
	ROS_DEBUG("Initializing REBVO standalone node");
	nodelet.load(nodelet_name, "rebvo/rebvo_nodelet", remap, nargv);


//	while (ros::ok())
//		ros::spinOnce();

	ros::spin();

	return 0;

}
