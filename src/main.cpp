#include <mutex>
#include <math.h>
#include <iostream>
#include <functional>
#include <thread>
#include <opencv2/opencv.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"
#include "modules/core.h"

static std::mutex mutex;
static std::mutex fuzzy_cont;

typedef struct closest_obstacle 
{
  float dir_delta;
  float range;
  closest_obstacle(float dir_delta_, float range_) : dir_delta(dir_delta_) , range(range_){};
} closest_obstacle;

static closest_obstacle cloest_obs_front(0, 10);
static closest_obstacle cloest_obs_left(0, 10);
static closest_obstacle cloest_obs_right(0, 10);

void min_range(closest_obstacle & cloest_obs_right, float range_min, float range_max, float px_per_m, float angle, cv::Mat & im, ConstLaserScanStampedPtr &msg, int i, float compare1, float compare2);

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;
    }
  }
}

void cameraCallback(ConstImageStampedPtr &msg) {
  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, cv::COLOR_RGB2BGR);

  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}

void lidarCallback(ConstLaserScanStampedPtr &msg) {

  //  std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = 400;
  int height = 400;
  float px_per_m = 200 / range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);

  cloest_obs_front.range = range_max;
  cloest_obs_right.range = range_max;
  cloest_obs_left.range = range_max;  

  for (int i = 0; i < nranges; i++) 
  {

    float angle = angle_min + i * angle_increment;
      //Not using full RoM for the LIDAR.
    min_range(cloest_obs_front, range_min, range_max, px_per_m, angle, im, msg, i, 0.19, -0.19);
    min_range(cloest_obs_left, range_min, range_max, px_per_m, angle, im, msg, i, -1.37, -1.76);
    min_range(cloest_obs_right, range_min, range_max, px_per_m, angle, im, msg, i, 1.76, 1.37);

  }

  // Works like a signal semaphore
  fuzzy_cont.unlock();

  //Draw Current Direction 
  cv::Point2f start_dir(200.5f + range_min * px_per_m * std::cos(0),
                            200.5f - range_min * px_per_m * std::sin(0));
  cv::Point2f end_dir(200.5f + 1 * px_per_m * std::cos(0),
                          200.5f - 1 * px_per_m * std::sin(0));
  cv::arrowedLine(im, start_dir * 16, end_dir * 16, cv::Scalar(0, 0, 255, 0), 1,
        cv::LINE_AA, 4, 0.2);

  //
  cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
}

int main(int _argc, char **_argv) 
{
  
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // not sure if correct path
  fl::Engine* engine = fl::FllImporter().fromFile("../assets/ObstacleAvoidance.fll");

  std::string status;
  if (not engine->isReady(&status))
      throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);

  fl::InputVariable* obs_dist_s     = engine->getInputVariable("forward");  // 
  fl::InputVariable* obs_dist_r     = engine->getInputVariable("right");  // 
  fl::InputVariable* obs_dist_l     = engine->getInputVariable("left");  // 
  fl::OutputVariable* robot_dir     = engine->getOutputVariable("omega");       // 
  fl::OutputVariable* robot_speed   = engine->getOutputVariable("velocity");     //

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
   gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

  /*
  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", poseCallback);
  

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);
  */
  

  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

  gazebo::msgs::WorldControl controlMessage;

  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);

  const int key_esc = 27;

  fuzzy_cont.lock();

  // Loop
  while (true) 
  {

    gazebo::common::Time::MSleep(10);

    fuzzy_cont.lock();

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();

    if (key == key_esc)
      break;

    std::cout << "cloest_obs_front.range: " << cloest_obs_front.range << std::endl;
    std::cout << "cloest_obs_left.range: " << cloest_obs_left.range << std::endl;
    std::cout << "cloest_obs_right.range: " << cloest_obs_right.range << std::endl;

    //Sends data to fuzzy controller.
    obs_dist_s->setValue(cloest_obs_front.range);
    obs_dist_l->setValue(cloest_obs_left.range);
    obs_dist_r->setValue(cloest_obs_right.range);
    engine->process();
    double set_speed = static_cast<double>(robot_speed->getValue());
    double set_dir   = static_cast<double>(robot_dir->getValue());

    std::cout << "Set Speed: " << set_speed << std::endl;
    std::cout << "Set Dir: " <<  set_dir << std::endl;
    
    // Generate a pose
    ignition::math::Pose3d pose(set_speed, 0, 0, 0, 0, set_dir);

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }


  // Make sure to shut everything down.
  gazebo::client::shutdown();
}

void min_range(closest_obstacle & cloest_obs, float range_min, float range_max, float px_per_m, float angle, cv::Mat & im, ConstLaserScanStampedPtr &msg, int i, float compare1, float compare2)
{
  //Not using full RoM for the LIDAR.
  if ( angle < compare1 && angle > compare2 )
  {
      float range = std::min(float(msg->scan().ranges(i)), range_max);

      cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                          200.5f - range_min * px_per_m * std::sin(angle));
      cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                        200.5f - range * px_per_m * std::sin(angle));
      cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
              cv::LINE_AA, 4);

      //Find Shortest Obstacle
      if (  cloest_obs.range > range )
      {
            cloest_obs.range = range;
            cloest_obs.dir_delta = angle;
        }
    }
}

  
