#include "ros/ros.h"
#include "airsim_ros_wrapper.h"
#include <ros/spinner.h>
#include <image_transport/image_transport.h>


msr::airlib::CarRpcLibClient* airsimapi;
image_transport::ImageTransport* imagetransport;
image_transport::Publisher* imagepub;

std::string cameraname;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "fsds_ros_bridge_camera");
    ros::NodeHandle nh("~");
    imagetransport = new image_transport::ImageTransport(nh);

    cameraname = nh.getParam("cameraname", "front_right_custom")
    airsimapi = new msr::airlib::CarRpcLibClient(nh.getParam("host_ip", "localhost"));

    imagepub = it.advertise("/fsds/camera/" + cameraname, 1);


    try {
        airsimapi.confirmConnection();
    } catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    ros::Timer timer = n.createTimer(ros::Duration(0.03), doImageUpdate);
    ros::spin();
    return 0;
} 

void doImageUpdate(const ros::TimerEvent&)
{
   std::vector<ImageResponse*> img_response = airsimapi->simGetImages(cameraname, VehicleCameraBase::ImageType::Scene);

    // if a render request failed for whatever reason, this img will be empty.
    // Attempting to use a make_ts(0) results in ros::Duration runtime error.
    if (curr_img_response.size() == null || img_response[0]->time_stamp == 0)
        continue;

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
    img_msg->header.frame_id = frame_id;
    img_msg->height = curr_img_response->height;
    img_msg->width = curr_img_response->width;
    img_msg->encoding = "bgar8";
    img_msg->is_bigendian = 0;

    imagepub->publish(img_msg);
}

ros::Time first_imu_ros_ts;
int64_t first_imu_unreal_ts = -1;

ros::Time AirsimROSWrapper::make_ts(uint64_t unreal_ts)
{
    if (first_imu_unreal_ts < 0)
    {
        first_imu_unreal_ts = unreal_ts;
        first_imu_ros_ts = ros::Time::now();
    }
    return first_imu_ros_ts + ros::Duration((unreal_ts - first_imu_unreal_ts) / 1e9);
}