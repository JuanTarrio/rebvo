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

#include "../../include/rebvo/rebvo_nodelet.h"

#include "pluginlib/class_list_macros.h"
#include "sensor_msgs/image_encodings.h"
#include "rebvo/EdgeMap.h"
#include "TooN/so3.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/point_cloud_conversion.h"


namespace rebvo {

RebvoNodelet::RebvoNodelet() {
}

RebvoNodelet::~RebvoNodelet() {

	camera_sub_.shutdown();

	//TODO shutdown REBVO
	//rebvo_->CleanUp();
}

void RebvoNodelet::onInit() {

	nh_ = getNodeHandle();
	private_nh_ = getPrivateNodeHandle();

	NODELET_DEBUG("Initializing REBVO nodelet");

	//TODO Construct REBVO
	//rebvo_ = std::unique_ptr<REBVO>(new REBVO("/home/l_vis/GlobalConfig_EuRoC"));
	//rebvo_ = REBVO(const REBVOParameters &parameters);

    construct();


	rebvo_->setOutputCallback(&RebvoNodelet::edgeMapPubCb, this);

	//TODO initialize REBVO
	if (!rebvo_->Init()) {
		NODELET_ERROR("Error while initializing rebvoLib");
		return;
	}

	//TODO change to keyline message array
    edgeMap_pub_ = nh_.advertise<EdgeMap>("rebvo_edgemap", 10,
			boost::bind(&RebvoNodelet::connectCb, this),
			boost::bind(&RebvoNodelet::disconnectCb, this));

    pointCloud_pub_=nh_.advertise<sensor_msgs::PointCloud2>("rebvo_pcl",5);

    private_nh_.param<std::string>("rebvo/imu_topic", imuTopic_, "imu");
    private_nh_.param<std::string>("rebvo/image_topic", imageTopic_, "image");
    private_nh_.param<std::string>("rebvo/frame_id_cam", frame_id_cam, "rebvo_frame_cam");
    private_nh_.param<std::string>("rebvo/frame_id_robot", frame_id_robot, "rebvo_frame_robot");

	camera_sub_ = nh_.subscribe(imageTopic_, 10, &RebvoNodelet::imageCb, this);
	imu_sub_ = nh_.subscribe(imuTopic_, 100, &RebvoNodelet::imuCb, this);

    tf_cam2robot.setOrigin(tf::Vector3(0,0,0));
    tf_cam2robot.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI/2));


}

void RebvoNodelet::connectCb() {

	if (!camera_sub_ && edgeMap_pub_.getNumSubscribers() > 0) {
		NODELET_INFO("Connecting to camera topic.");

		//TODO initialize REBVO here instead of in onInit();
		//rebvo_.Init();

		//TODO subscribe to cam here instead of in onInit();
		//TODO Parametrize topic names
		//camera_sub_ = nh_.subscribe(imageTopic_, 10, &RebvoNodelet::imageCb, this);
		//imu_sub_ = nh_.subscribe(imuTopic_, 10, &RebvoNodelet::imuCb, this);
	}
}
;
void RebvoNodelet::disconnectCb() {

	if (edgeMap_pub_.getNumSubscribers() == 0) {
		NODELET_INFO("Unsubscribing from camera topic.");

		//TODO shutdown sub & REBVO here as well
		//rebvo_.CleanUp();
		//camera_sub_.shutdown();
	}
}

void RebvoNodelet::imageCb(const sensor_msgs::ImageConstPtr &image) {

	std::shared_ptr<Image<RGB24Pixel> > imgRebvoPtr;

    if (!rebvo_->requestCustomCamBuffer(imgRebvoPtr,
            image->header.stamp.toSec(), 0.001)) {
		NODELET_ERROR("Droping Frame");
		return;
	}

	if (!imgMsg2Rebvo(image, imgRebvoPtr))
        NODELET_ERROR_STREAM("Img msg doesn't match with revbo config, only MONO8 and rgb8 encodings are suported. Img Size:"<<image->width<<"x"<<image->height<<" Encoding:"<<image->encoding);

	rebvo_->releaseCustomCamBuffer();

}

void RebvoNodelet::imuCb(const sensor_msgs::ImuConstPtr &imuMsg) {

	ImuData imuRebvo;

	imuMsg2Rebvo(imuMsg, imuRebvo);

	if (!rebvo_->pushIMU(imuRebvo))
		NODELET_ERROR("IMU package dropped");


}

bool RebvoNodelet::edgeMapPubCb(PipeBuffer &edgeMapRebvo) {

	Keyline keylineMsg;
	EdgeMap edgeMapMsg;


    ros::Time msg_stamp;
    msg_stamp.fromSec(edgeMapRebvo.t);
    //Fill edgemap header
    edgeMapMsg.header.stamp=msg_stamp;

    edgeMapMsg.header.frame_id = frame_id_cam;

    //Fill pcl header

    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_cam;
    cloud.header.stamp=msg_stamp;
    cloud.width  = edgeMapRebvo.ef->KNum();
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(cloud.width );

    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

    for (KeyLine &kl:(*edgeMapRebvo.ef)) {

        //Copy keylines into edgemap msg
        keylineMsg.KlGrad[0] = kl.m_m.x;
        keylineMsg.KlGrad[1] = kl.m_m.y;

        keylineMsg.KlImgPos[0] = kl.c_p.x;
        keylineMsg.KlImgPos[1] = kl.c_p.y;

        keylineMsg.invDepth = kl.rho;
        keylineMsg.invDepthS = kl.s_rho;

        keylineMsg.KlFocPos[0] = kl.p_m.x;
        keylineMsg.KlFocPos[1] = kl.p_m.y;

        keylineMsg.KlMatchID = kl.m_id;

        keylineMsg.ConsMatch = kl.m_num;

        keylineMsg.KlPrevMatchID = kl.p_id;
        keylineMsg.KlNextMatchID = kl.n_id;

		edgeMapMsg.Keylines.push_back(keylineMsg);


        //fill point cloud msg

        rebvo::cam_model &rebvo_cam=edgeMapRebvo.ef->GetCam();
        TooN::Vector<3> p3d=rebvo_cam.unprojectHomCordVec(TooN::makeVector(kl.p_m.x,kl.p_m.y,kl.rho/edgeMapRebvo.K));

        *(out_x) = p3d[0];
        *(out_y) = p3d[1];
        *(out_z) = p3d[2];
        ++out_x;
        ++out_y;
        ++out_z;
	}

	edgeMap_pub_.publish(edgeMapMsg);


    pointCloud_pub_.publish(cloud);


    tf::Transform transform;

    tf::Vector3 rot(edgeMapRebvo.nav.PoseLie[0],edgeMapRebvo.nav.PoseLie[1],edgeMapRebvo.nav.PoseLie[2]);
    tfScalar angle=rot.length();

    if(angle>0){
        transform.setRotation(tf::Quaternion(rot/angle,angle));
    }else{
        transform.setIdentity();
    }

    transform.setOrigin(tf::Vector3(edgeMapRebvo.nav.Pos[0],edgeMapRebvo.nav.Pos[1],edgeMapRebvo.nav.Pos[2]));

    tf_broad.sendTransform(tf::StampedTransform(tf_cam2robot.inverse()*transform, msg_stamp, "map", frame_id_cam));

    //tf_broad.sendTransform(tf::StampedTransform(tf_cam2robot.inverse(),msg_stamp, "map", "map_cam"));
    tf_broad.sendTransform(tf::StampedTransform(tf_cam2robot, msg_stamp, frame_id_cam, frame_id_robot));


    //NODELET_ERROR("transmiting Frame");

	return true;
}

bool imgMsg2Rebvo(const sensor_msgs::ImageConstPtr& imgMsg,
		std::shared_ptr<Image<RGB24Pixel> >& imgRebvo) {

	if (imgMsg->width != imgRebvo->Size().w
			|| imgMsg->height != imgRebvo->Size().h)
		return false;

	if (imgMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0) {

		for (int y = 0; y < imgRebvo->Size().h; ++y) {
			for (int x = 0; x < imgRebvo->Size().w; ++x) {

				(*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x];
				(*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x];
				(*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x];
			}
		}

    } else if (imgMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) {

        for (int y = 0; y < imgRebvo->Size().h; ++y) {
            for (int x = 0; x < imgRebvo->Size().w; ++x) {

                (*imgRebvo)(x, y).pix.r = imgMsg->data[imgMsg->step * y + x*3+0];
                (*imgRebvo)(x, y).pix.g = imgMsg->data[imgMsg->step * y + x*3+1];
                (*imgRebvo)(x, y).pix.b = imgMsg->data[imgMsg->step * y + x*3+2];
            }
        }

    }else{
		return false;
	}

	return true;
}

bool imuMsg2Rebvo(const sensor_msgs::ImuConstPtr& imuMsg, ImuData& imuRebvo) {

	imuRebvo.comp = TooN::Zeros;
	imuRebvo.acel = TooN::makeVector(imuMsg->linear_acceleration.x,
			imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z);
	imuRebvo.giro = TooN::makeVector(imuMsg->angular_velocity.x,
			imuMsg->angular_velocity.y, imuMsg->angular_velocity.z);
	;

	imuRebvo.tstamp = imuMsg->header.stamp.toSec();

	return true;
}

void RebvoNodelet::construct() {

	REBVOParameters rebvoPars;

	std::vector<double> transCam2ImuPar;
	std::vector<double> rotCam2ImuPar;

	TooN::Vector<3> transCam2Imu;
	TooN::Matrix<3,3> rotCam2Imu;


	// Rebvo Detector parameters
    private_nh_.param<double>("rebvo/detector/Sigma0", rebvoPars.Sigma0, 3.56359);
    private_nh_.param<double>("rebvo/detector/KSigma", rebvoPars.KSigma, 1.2599);
	private_nh_.param<int>("rebvo/detector/ReferencePoints", rebvoPars.ReferencePoints, 12000);
	private_nh_.param<int>("rebvo/detector/MaxPoints", rebvoPars.MaxPoints, 16000);
    private_nh_.param<int>("rebvo/detector/TrackPoints", rebvoPars.TrackPoints, rebvoPars.MaxPoints);
	private_nh_.param<double>("rebvo/detector/DetectorThresh", rebvoPars.DetectorThresh, 0.01);
	private_nh_.param<double>("rebvo/detector/DetectorAutoGain", rebvoPars.DetectorAutoGain, 5e-7);
	private_nh_.param<double>("rebvo/detector/DetectorMaxThresh", rebvoPars.DetectorMaxThresh, 0.5);
	private_nh_.param<double>("rebvo/detector/DetectorMinThresh", rebvoPars.DetectorMinThresh, 0.005);
	private_nh_.param<int>("rebvo/detector/DetectorPlaneFitSize", rebvoPars.DetectorPlaneFitSize, 2);
	private_nh_.param<double>("rebvo/detector/DetectorPosNegThresh", rebvoPars.DetectorPosNegThresh, 0.4);
	private_nh_.param<double>("rebvo/detector/DetectorDoGThresh", rebvoPars.DetectorDoGThresh, 0.095259868922420);

	// Rebvo TrackMaper parameters
	private_nh_.param<double>("rebvo/TrackMaper/SearchRange", rebvoPars.SearchRange, 40);
	private_nh_.param<double>("rebvo/TrackMaper/QCutOffNumBins", rebvoPars.QCutOffNumBins, 100);
	private_nh_.param<double>("rebvo/TrackMaper/QCutOffQuantile", rebvoPars.QCutOffQuantile, 0.9);
	private_nh_.param<int>("rebvo/TrackMaper/TrackerIterNum", rebvoPars.TrackerIterNum, 5);
	private_nh_.param<int>("rebvo/TrackMaper/TrackerInitType", rebvoPars.TrackerInitType, 2);
	private_nh_.param<int>("rebvo/TrackMaper/TrackerInitIterNum", rebvoPars.TrackerInitIterNum, 2);
	private_nh_.param<double>("rebvo/TrackMaper/TrackerMatchThresh", rebvoPars.TrackerMatchThresh, 0.5);
	private_nh_.param<double>("rebvo/TrackMaper/MatchThreshModule", rebvoPars.MatchThreshModule, 1);
	private_nh_.param<double>("rebvo/TrackMaper/MatchThreshAngle", rebvoPars.MatchThreshAngle, 45);
	private_nh_.param<int>("rebvo/TrackMaper/MatchNumThresh", (int&)rebvoPars.MatchNumThresh, 0);
	private_nh_.param<double>("rebvo/TrackMaper/ReweigthDistance", rebvoPars.ReweigthDistance, 2);
	private_nh_.param<double>("rebvo/TrackMaper/RegularizeThresh", rebvoPars.RegularizeThresh, 0.5);
	private_nh_.param<double>("rebvo/TrackMaper/LocationUncertaintyMatch", rebvoPars.LocationUncertaintyMatch, 2);
	private_nh_.param<double>("rebvo/TrackMaper/ReshapeQAbsolute", rebvoPars.ReshapeQAbsolute, 1e-4);
	private_nh_.param<double>("rebvo/TrackMaper/ReshapeQRelative", rebvoPars.ReshapeQRelative, 1.6968e-04);
	private_nh_.param<double>("rebvo/TrackMaper/LocationUncertainty", rebvoPars.LocationUncertainty, 1);
	private_nh_.param<double>("rebvo/TrackMaper/DoReScaling", rebvoPars.DoReScaling, 0);
	private_nh_.param<int>("rebvo/TrackMaper/GlobalMatchThreshold", rebvoPars.MatchThreshold, 500);

	// Rebvo camera parameters

	private_nh_.param<float>("rebvo/Camera/ZfX", rebvoPars.z_f_x, 458.654);
	private_nh_.param<float>("rebvo/Camera/ZfY", rebvoPars.z_f_y, 457.296);
	private_nh_.param<float>("rebvo/Camera/PPx", rebvoPars.pp_x, 367.215);
	private_nh_.param<float>("rebvo/Camera/PPy", rebvoPars.pp_y, 248.375);

	private_nh_.param<double>("rebvo/Camera/KcR2", rebvoPars.kc.Kc2, -0.28340811);
	private_nh_.param<double>("rebvo/Camera/KcR4", rebvoPars.kc.Kc4, 0.07395907);
	private_nh_.param<double>("rebvo/Camera/KcR6", rebvoPars.kc.Kc6, 0);
	private_nh_.param<double>("rebvo/Camera/KcP1", rebvoPars.kc.P1, 0.00019359);
	private_nh_.param<double>("rebvo/Camera/KcP2", rebvoPars.kc.P2, 1.76187114e-05);


	private_nh_.param<int>("rebvo/Camera/ImageWidth", (int&)rebvoPars.ImageSize.w, 752);
	private_nh_.param<int>("rebvo/Camera/ImageHeight", (int&)rebvoPars.ImageSize.h, 480);
	private_nh_.param<double>("rebvo/Camera/FPS", rebvoPars.config_fps, 20);
    private_nh_.param<double>("rebvo/Camera/SoftFPS", rebvoPars.soft_fps, rebvoPars.config_fps);
    private_nh_.param<bool>("rebvo/Camera/UseUndistort", rebvoPars.useUndistort, false);
	private_nh_.param<bool>("rebvo/Camera/Rotate180", rebvoPars.rotatedCam, 0);


	// Rebvo global parameters

	rebvoPars.CameraType = 3;

	private_nh_.param<std::string>("rebvo/global/VideoNetHost", rebvoPars.VideoNetHost, "127.0.0.1");
	private_nh_.param<int>("rebvo/global/VideoNetPort", rebvoPars.VideoNetPort, 2708);
    private_nh_.param<bool>("rebvo/global/BlockingUDP", rebvoPars.BlockingUDP, 0);
    private_nh_.param<bool>("rebvo/global/VideoNetEnabled", rebvoPars.VideoNetEnabled, 0);

	rebvoPars.VideoSave=0;
	rebvoPars.encoder_type=1;
	rebvoPars.EdgeMapDelay=0;

    private_nh_.param<bool>("rebvo/global/SaveLog", rebvoPars.SaveLog, 0);
	private_nh_.param<std::string>("rebvo/global/LogFile", rebvoPars.LogFile, "rebvo_log.m");
	private_nh_.param<std::string>("rebvo/global/TrayFile", rebvoPars.TrayFile, "rebvo_tray.txt");


    private_nh_.param<bool>("rebvo/global/TrackKeyFrames", rebvoPars.TrackKeyFrames, 0);
    private_nh_.param<double>("rebvo/global/KFSavePercent", rebvoPars.KFSavePercent, 0.7);
    rebvoPars.StereoAvaiable=0;


	// Rebvo DataSetCamera parameters
	rebvoPars.CamTimeScale =1;

    // Rebvo Imu parameters
	rebvoPars.UseCamIMUSE3File = 0;

    private_nh_.param<int>("rebvo/imu/ImuMode", rebvoPars.ImuMode,1);
    if(rebvoPars.ImuMode>0)
        rebvoPars.ImuMode=1;

	private_nh_.param<std::vector<double>>("rebvo/imu/transCam2Imu", transCam2ImuPar, std::vector<double>{0,0,0});
	private_nh_.param<std::vector<double>>("rebvo/imu/rotCam2Imu", rotCam2ImuPar, std::vector<double>{1,0,0, 0,1,0, 0,0,1});

	// Copy and set rebvo cam2imu transformation
	if(transCam2ImuPar.size() == 3){

		for (int i = 0; i < 3; ++i)
			transCam2Imu[i] = transCam2ImuPar[i];
	} else {

        transCam2Imu=TooN::Zeros;
	}


	if(rotCam2ImuPar.size() == 9){

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {

				rotCam2Imu[i][j] = rotCam2ImuPar[i * 3 + j];
			}
		}
	} else {

        rotCam2Imu=TooN::Identity;
	}




	rebvoPars.ImuTimeScale = 1;
	private_nh_.param<double>("rebvo/imu/TimeDesinc", rebvoPars.TimeDesinc, 0);
	private_nh_.param<bool>("rebvo/imu/InitBias", rebvoPars.InitBias, 0);
	private_nh_.param<int>("rebvo/imu/InitBiasFrameNum", rebvoPars.InitBiasFrameNum, 10);
	private_nh_.param<double>("rebvo/imu/BiasHintX", rebvoPars.BiasInitGuess[0], 0.0);
	private_nh_.param<double>("rebvo/imu/BiasHintY", rebvoPars.BiasInitGuess[1], 0.0);
	private_nh_.param<double>("rebvo/imu/BiasHintZ", rebvoPars.BiasInitGuess[2], 0.0);
	private_nh_.param<double>("rebvo/imu/GiroMeasStdDev", rebvoPars.GiroMeasStdDev, 1.6968e-04);
	private_nh_.param<double>("rebvo/imu/GiroBiasStdDev", rebvoPars.GiroBiasStdDev, 1.9393e-05);
	private_nh_.param<double>("rebvo/imu/AcelMeasStdDev", rebvoPars.AcelMeasStdDev, 2.0000e-3);
	private_nh_.param<double>("rebvo/imu/g_module", rebvoPars.g_module, 9.8);
	private_nh_.param<double>("rebvo/imu/g_module_uncer", rebvoPars.g_module_uncer, 0.2e3);
	private_nh_.param<double>("rebvo/imu/g_uncert", rebvoPars.g_uncert, 2e-3);
	private_nh_.param<double>("rebvo/imu/VBiasStdDev", rebvoPars.VBiasStdDev, 1e-7);
	private_nh_.param<double>("rebvo/imu/ScaleStdDevMult", rebvoPars.ScaleStdDevMult, 1e-2);
	private_nh_.param<double>("rebvo/imu/ScaleStdDevMax", rebvoPars.ScaleStdDevMax, 1e-4);
	private_nh_.param<double>("rebvo/imu/ScaleStdDevInit", rebvoPars.ScaleStdDevInit, 1.2e-3);
	private_nh_.param<int>("rebvo/imu/CircBufferSize", rebvoPars.CircBufferSize, 1000);
	private_nh_.param<double>("rebvo/imu/SampleTime", rebvoPars.SampleTime, 0.001250);


	// rebvo ProcesorConfig parameters

	private_nh_.param<int>("rebvo/ProcesorConfig/SetAffinity", rebvoPars.cpuSetAffinity, 1);
	private_nh_.param<int>("rebvo/ProcesorConfig/Thread1", rebvoPars.cpu0, 1);
	private_nh_.param<int>("rebvo/ProcesorConfig/Thread2", rebvoPars.cpu1, 2);
	private_nh_.param<int>("rebvo/ProcesorConfig/Thread3", rebvoPars.cpu2, 3);



	rebvo_ = std::unique_ptr<REBVO>(new REBVO(rebvoPars));
	rebvo_->setCamImuSE3(rotCam2Imu,transCam2Imu);
}

PLUGINLIB_DECLARE_CLASS(rebvo, RebvoNodelet, rebvo::RebvoNodelet,nodelet::Nodelet);

} //namespace rebvo
