/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */


/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
- @b "~/kernelSize" @b [int] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [int] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/



#include "slam_gmapping.h"

#include <iostream>
#include <chrono>

#include <time.h>
#include <assert.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2/utils.h"
#include "tf2_ros/create_timer_ros.h"


#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"


// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
  Node("slam_gmapping"), map_update_interval_(5, 0), map_to_odom_(tf2::Transform(tf2::Quaternion(0, 0, 0, 1))),
  laser_count_(0), transform_thread_(NULL)
{
  seed_ = time(NULL);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                                                get_node_base_interface(),
                                                get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf_  = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
  init();
}

void SlamGMapping::init()
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // The library is pretty chatty
  //gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  gsp_ = new GMapping::GridSlamProcessor();
  assert(gsp_);

  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  assert(tfB_);

  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;
  

  
  // Parameters used by our GMapping wrapper
  throttle_scans_ = this->declare_parameter("throttle_scans", 1);
  base_frame_ = this->declare_parameter("base_frame", "base_link");
  map_frame_ = this->declare_parameter("map_frame", "map");
  odom_frame_ = this->declare_parameter("odom_frame", "odom");
  transform_publish_period_ = this->declare_parameter("transform_publish_period", 0.05);

  double tmp;
  tmp = this->declare_parameter("map_update_interval", 5.0);
  map_update_interval_ = tf2::durationFromSec(tmp);
  
  // Parameters used by GMapping itself
  // can be overwritten in initMapper based on LaserScan message
  maxUrange_ = this->declare_parameter("maxUrange", 0.0);
  maxRange_ = this->declare_parameter("maxRange", 0.0);

  minimum_score_ = this->declare_parameter("minimumScore", 0);
  sigma_ = this->declare_parameter("sigma", 0.05);
  kernelSize_ = this->declare_parameter("kernelSize", 1);
  lstep_ = this->declare_parameter("lstep", 0.05);
  astep_ = this->declare_parameter("astep", 0.05);
  iterations_ = this->declare_parameter("iterations", 5);
  lsigma_ = this->declare_parameter("lsigma", 0.075);
  ogain_ = this->declare_parameter("ogain", 3.0);
  lskip_ = this->declare_parameter("lskip", 0);
  srr_ = this->declare_parameter("srr", 0.1);
  srt_ = this->declare_parameter("srt", 0.2);
  str_ = this->declare_parameter("str", 0.1);
  stt_ = this->declare_parameter("stt", 0.2);
  linearUpdate_ = this->declare_parameter("linearUpdate", 1.0);
  angularUpdate_ = this->declare_parameter("angularUpdate", 0.5);
  temporalUpdate_ = this->declare_parameter("temporalUpdate", -1.0);
  resampleThreshold_ = this->declare_parameter("resampleThreshold", 0.5);
  particles_ = this->declare_parameter("particles", 30);
  xmin_ = this->declare_parameter("xmin", -100.0);
  ymin_ = this->declare_parameter("ymin", -100.0);
  xmax_ = this->declare_parameter("xmax", 100.0);
  ymax_ = this->declare_parameter("ymax", 100.0);
  delta_ = this->declare_parameter("delta", 0.05);
  occ_thresh_ = this->declare_parameter("occ_thresh", 0.25);
  llsamplerange_ = this->declare_parameter("llsamplerange", 0.01);
  llsamplestep_ = this->declare_parameter("llsamplestep", 0.01);
  lasamplerange_ = this->declare_parameter("lasamplerange", 0.005);
  lasamplestep_ = this->declare_parameter("lasamplestep", 0.005);
  tf_delay_ = this->declare_parameter("tf_delay", transform_publish_period_);

}


void SlamGMapping::startLiveSlam()
{
  entropy_publisher_ = this->create_publisher<std_msgs::msg::Float64>("entropy", 1);
  sst_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  sstm_ = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  ss_ = this->create_service<nav_msgs::srv::GetMap>("dynamic_map", std::bind(&SlamGMapping::mapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  scan_filter_sub_.subscribe(this, "scan");
  std::chrono::duration<int> buffer_timeout(1);
  scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(scan_filter_sub_, *tf2_buffer_, odom_frame_, 5, this->get_node_logging_interface(), this->get_node_clock_interface(), buffer_timeout);
  scan_filter_->registerCallback(&SlamGMapping::laserCallback, this);

  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}

void SlamGMapping::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  rclcpp::Rate r(1.0 / transform_publish_period);
  while(rclcpp::ok()){
    publishTransform();
    r.sleep();
  }
}

SlamGMapping::~SlamGMapping()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
}

bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t)
{
  // Get the pose of the centered laser at the right time
  centered_laser_pose_.stamp_ = tf2_ros::fromRclcpp(t);
  // Get the laser's pose that is centered
  tf2::Stamped<tf2::Transform> odom_pose;
  try
  {
    geometry_msgs::msg::TransformStamped odom_pose_msg;
    tf2_buffer_->transform(tf2::toMsg(centered_laser_pose_), odom_pose_msg, odom_frame_);
    tf2::fromMsg(odom_pose_msg, odom_pose);
  }
  catch(tf2::TransformException e)
  {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf2::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  laser_frame_ = scan->header.frame_id;
  // Get the laser's pose, relative to base.
  tf2::Stamped<tf2::Transform> ident;
  tf2::Stamped<tf2::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = tf2_ros::fromMsg(scan->header.stamp);
  try
  {
    geometry_msgs::msg::TransformStamped laser_pose_msg;
    tf2_buffer_->transform(tf2::toMsg(ident), laser_pose_msg, base_frame_);
    tf2::fromMsg(laser_pose_msg, laser_pose);
  }
  catch(tf2::TransformException e)
  {
    RCLCPP_WARN(get_logger(), "Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf2::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  geometry_msgs::msg::PointStamped up;
  up.header.stamp = scan->header.stamp;
  up.header.frame_id = base_frame_;
  up.point.x = 0.0;
  up.point.y = 0.0;
  up.point.z = 1 + laser_pose.getOrigin().z();
  try
  {
    tf2_buffer_->transform(up, up, laser_frame_);
    RCLCPP_DEBUG(get_logger(), "Z-Axis in sensor frame: %.3f", up.point.z);
  }
  catch(tf2::TransformException& e)
  {
    RCLCPP_WARN(get_logger(), "Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.point.z) - 1) > 0.001)
  {
    RCLCPP_WARN(get_logger(), "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.point.z);
    return false;
  }

  gsp_laser_beam_count_ = scan->ranges.size();

  double angle_center = (scan->angle_min + scan->angle_max)/2;

  if (up.point.z > 0)
  {
    do_reverse_range_ = scan->angle_min > scan->angle_max;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, angle_center);
    centered_laser_pose_ = tf2::Stamped<tf2::Transform>(tf2::Transform(quat, tf2::Vector3(0,0,0)), tf2_ros::fromRclcpp(this->now()), laser_frame_);
    RCLCPP_INFO(get_logger(), "Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan->angle_min < scan->angle_max;
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, -angle_center);
    centered_laser_pose_ = tf2::Stamped<tf2::Transform>(tf2::Transform(quat, tf2::Vector3(0,0,0)), tf2_ros::fromRclcpp(this->now()), laser_frame_);
    RCLCPP_INFO(get_logger(), "Laser is mounted upside down.");
  }

  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan->ranges.size());
  // Make sure angles are started so that they are centered
  double theta = - std::fabs(scan->angle_min - scan->angle_max)/2;
  for(unsigned int i=0; i<scan->ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan->angle_increment);
  }

  RCLCPP_DEBUG(get_logger(), "Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan->angle_min, scan->angle_max,
            scan->angle_increment);
  RCLCPP_DEBUG(get_logger(), "Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan->angle_increment));

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  if(maxRange_ == 0.0)
    maxRange_ = scan->range_max - 0.01;
  if(maxUrange_ == 0.0)
    maxUrange_ = maxRange_;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,
                                         fabs(scan->angle_increment),
                                         gmap_pose,
                                         0.0,
                                         maxRange_);
  assert(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  assert(gsp_odom_);


  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, rclcpp::Time(scan->header.stamp)))
  {
    RCLCPP_WARN(get_logger(), "Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }

  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1,seed_);

  RCLCPP_INFO(get_logger(), "Initialization complete");

  return true;
}

bool
SlamGMapping::addScan(const sensor_msgs::msg::LaserScan::SharedPtr scan, GMapping::OrientedPoint& gmap_pose)
{
  if(!getOdomPose(gmap_pose, rclcpp::Time(scan->header.stamp)))
     return false;
  
  if(scan->ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan->ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (do_reverse_range_)
  {
    RCLCPP_DEBUG(get_logger(), "Inverting scan");
    int num_ranges = scan->ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan->ranges[num_ranges - i - 1] < scan->range_min)
        ranges_double[i] = (double)scan->range_max;
      else
        ranges_double[i] = (double)scan->ranges[num_ranges - i - 1];
    }
  } else 
  {
    for(unsigned int i=0; i < scan->ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan->ranges[i] < scan->range_min)
        ranges_double[i] = (double)scan->range_max;
      else
        ranges_double[i] = (double)scan->ranges[i];
    }
  }

  GMapping::RangeReading reading(scan->ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 tf2::timeToSec(tf2_ros::fromMsg(scan->header.stamp)));

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);

  /*
  RCLCPP_DEBUG(get_logger(), "scanpose (%.3f): %.3f %.3f %.3f\n",
            scan->header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  RCLCPP_DEBUG(get_logger(), "processing scan");

  return gsp_->processScan(reading);
}

void
SlamGMapping::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;


  static rclcpp::Time last_map_update(0,0);

  // We can't initialize the mapper until we've got the first scan
  if(!got_first_scan_)
  {
    if(!initMapper(scan))
      return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;

  if(addScan(scan, odom_pose))
  {
    RCLCPP_DEBUG(get_logger(), "scan processed");

    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    RCLCPP_DEBUG(get_logger(), "new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    RCLCPP_DEBUG(get_logger(), "odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    RCLCPP_DEBUG(get_logger(), "correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf2::Quaternion quat_mpose;
    quat_mpose.setRPY(0, 0, mpose.theta);
    tf2::Transform laser_to_map = tf2::Transform(quat_mpose, tf2::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf2::Quaternion quat_opose;
    quat_opose.setRPY(0, 0, odom_pose.theta);
    tf2::Transform odom_to_laser = tf2::Transform(quat_opose, tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (rclcpp::Time(scan->header.stamp) - last_map_update) > map_update_interval_)
    {
      updateMap(scan);
      last_map_update = rclcpp::Time(scan->header.stamp);
      RCLCPP_DEBUG(get_logger(), "Updated the map");
    }
  } else
    RCLCPP_DEBUG(get_logger(), "cannot process scan");
}

double
SlamGMapping::computePoseEntropy()
{
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}

void
SlamGMapping::updateMap(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  RCLCPP_DEBUG(get_logger(), "Update map");
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  matcher.setLaserParameters(scan->ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::msg::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_->publish(entropy);

  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  GMapping::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, 
                                delta_);

  RCLCPP_DEBUG(get_logger(), "Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    RCLCPP_DEBUG(get_logger(), "  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      RCLCPP_DEBUG(get_logger(), "Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    RCLCPP_DEBUG(get_logger(), "map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    RCLCPP_DEBUG(get_logger(), "map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = this->now();
  map_.map.header.frame_id = map_frame_;

  sst_->publish(map_.map);
  sstm_->publish(map_.map.info);
}

bool
SlamGMapping::mapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                          nav_msgs::srv::GetMap::Request::SharedPtr  req,
                          nav_msgs::srv::GetMap::Response::SharedPtr res)
{
  boost::mutex::scoped_lock map_lock (map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    *res = map_;
    return true;
  }
  else
    return false;
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  rclcpp::Time tf_expiration = this->now() + rclcpp::Duration::from_seconds(tf_delay_);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = tf_expiration;
  t.header.frame_id = map_frame_;
  t.child_frame_id = odom_frame_;
  tf2::toMsg(map_to_odom_, t.transform);
  tfB_->sendTransform(t);
  map_to_odom_mutex_.unlock();
}
