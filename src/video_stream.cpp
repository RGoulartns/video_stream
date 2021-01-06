#include "video_stream/video_stream.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace video_stream
{

VideoStream::VideoStream() : 
    Node("video_stream_node"),
    loopFreq_(20),
    streamStarted_(false)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    loopFreq_ = this->declare_parameter("frequency", 20);
    width_ = this->declare_parameter("width", 1280);
    height_ = this->declare_parameter("height", 640);
    frameId_ = this->declare_parameter("frame_id", "base_link");
    videoSource_ = this->declare_parameter("video_source", "videoplayback.mp4");

    cameraInfo_msg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    cameraInfo_msg_->header.frame_id = frameId_;
    cameraInfo_msg_->height = height_;
    cameraInfo_msg_->width = width_; 

    image_msg_ = std::make_shared<sensor_msgs::msg::Image>();
    image_msg_->header.frame_id = frameId_;
    image_msg_->is_bigendian = false;
    image_msg_->width = width_;
    image_msg_->height = height_;


    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        5
    ), rmw_qos_profile_sensor_data);
    //TODO: REMOVE THIS (next line)
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    //qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Initialize OpenCV video capture stream.
    cap_.open(videoSource_);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
    if (!cap_.isOpened())
      throw std::runtime_error("Could not open video stream");

    image_it_pub_ = image_transport::create_camera_publisher(this, "image_raw");
    update_tm_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / loopFreq_)),
      std::bind(&VideoStream::timerCallback, this));

}

VideoStream::~VideoStream(){}

void VideoStream::timerCallback()
{
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty())
      return;

    if(!streamStarted_)
    {
      streamStarted_ = true;
      image_msg_->encoding = encodingCVtoROS(frame.type());
      image_msg_->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      imageMsgSize_ = frame.step * frame.rows;
      image_msg_->data.resize(imageMsgSize_);
    }
    memcpy(&image_msg_->data[0], frame.data, imageMsgSize_);
    image_it_pub_.publish(*image_msg_, *cameraInfo_msg_);

}

// source: https://github.com/ros2/demos/tree/master/image_tools
std::string VideoStream::encodingCVtoROS(int mat_type)
{
  switch (mat_type)
  {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

}  // namespace video_stream
