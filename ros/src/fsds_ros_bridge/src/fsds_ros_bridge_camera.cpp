#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //Ignore errors inside the rpc package
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
    STRICT_MODE_ON

#include "ros/ros.h"
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include "common/AirSimSettings.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"


typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;


msr::airlib::CarRpcLibClient* airsim_api;
image_transport::ImageTransport* image_transporter;
image_transport::Publisher* image_pub;

std::string camera_name = "front_right_custom";
std::string airsim_ip = "localhost";

ros::Time first_imu_ros_ts;
int64_t first_imu_unreal_ts = -1;

ros::Time make_ts(uint64_t unreal_ts)
{
    if (first_imu_unreal_ts < 0)
    {
        first_imu_unreal_ts = unreal_ts;
        first_imu_ros_ts = ros::Time::now();
    }
    return first_imu_ros_ts + ros::Duration((unreal_ts - first_imu_unreal_ts) / 1e9);
}

void doImageUpdate(const ros::TimerEvent&)
{
    std::vector<ImageRequest> req;
    req.push_back(ImageRequest(camera_name, ImageType::Scene, false, false));
    std::vector<ImageResponse*> img_response = airsim_api->simGetImages(req, "FSCar");

    // if a render request failed for whatever reason, this img will be empty.
    // Attempting to use a make_ts(0) results in ros::Duration runtime error.
    if (img_response.size() == 0 || img_response[0]->time_stamp == 0)
        return;

    ImageResponse* curr_img_response = img_response[0];


    // todo: publish tf

    sensor_msgs::ImagePtr img_msg = boost::make_shared<sensor_msgs::Image>();

    std::vector<unsigned char> v(curr_img_response->image_data_uint8->size());
    for (int i = 0; i < curr_img_response->image_data_uint8->size(); i++) {
        v[i] = (*curr_img_response->image_data_uint8)[i];
    }
    img_msg->data = v;
    img_msg->step = curr_img_response->width * 8; // image_width * num_bytes
    img_msg->header.stamp = make_ts(curr_img_response->time_stamp);
    img_msg->header.frame_id = camera_name+"_optical";
    img_msg->height = curr_img_response->height;
    img_msg->width = curr_img_response->width;
    img_msg->encoding = "bgar8";
    img_msg->is_bigendian = 0;

    image_pub->publish(img_msg);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "fsds_ros_bridge_camera");
    ros::NodeHandle nh("~");
    image_transporter = new image_transport::ImageTransport(nh);

    nh.getParam("camera_name", camera_name);
    nh.getParam("airsim_ip", airsim_ip);
    airsim_api = new msr::airlib::CarRpcLibClient(airsim_ip);

    auto p = image_transporter->advertise("/fsds/camera/" + camera_name, 1);
    image_pub = &p;


    try {
        airsim_api->confirmConnection();
    } catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    ros::Timer timer = nh.createTimer(ros::Duration(0.03), doImageUpdate);
    ros::spin();
    return 0;
} 

