#ifndef VIDEO_STREAM__VIDEO_STREAM_HPP_
#define VIDEO_STREAM__VIDEO_STREAM_HPP_

#include "video_stream/visibility_control.h"

#include "opencv2/highgui/highgui.hpp"

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>


namespace video_stream
{

class VideoStream : public rclcpp::Node
{

public:
  VideoStream();

  virtual ~VideoStream();

private:
  void timerCallback();
  std::string encodingCVtoROS(int mat_type);

  cv::VideoCapture cap_;

  std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cameraInfo_msg_;
  image_transport::CameraPublisher image_it_pub_;
  rclcpp::TimerBase::SharedPtr update_tm_;

  // ROS parameters
  unsigned int loopFreq_;
  unsigned int width_;
  unsigned int height_;
  std::string frameId_;
  std::string videoSource_;

  unsigned int imageMsgSize_;
  bool streamStarted_;

};

}  // namespace video_stream

#endif  // VIDEO_STREAM__VIDEO_STREAM_HPP_
