#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "cfg/cpp/image_transport_tutorial/MyStuffConfig.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#define USING_CUDA 0
#if USING_CUDA
#include <opencv2/cuda.hpp>
#endif
class VideoBuffer
{
public:
    VideoBuffer(const std::string &name = "camera");
    ~VideoBuffer();
    void acquisitionLoop();
    void messageCallback(image_transport_tutorial::MyStuffConfig& config, uint32_t level);
    bool open(int device = 0);
    double heartBeatFrequency; // In milliseconds
    cv::VideoCapture* cam;
    bool display;
private:
    bool _quit;
    bool _active;
#if USING_CUDA
    boost::circular_buffer<cv::cuda::GpuMat> _frameBuffer;
#else
    boost::circular_buffer<cv::Mat> _frameBuffer;
#endif
 boost::thread _acqThread;
 std::string _name;
 ros::NodeHandle _nh;
 image_transport::ImageTransport _it;
 dynamic_reconfigure::Server<image_transport_tutorial::MyStuffConfig> _server;
 image_transport::Publisher _pub;
 image_transport::Publisher _rawPub;
};

VideoBuffer::VideoBuffer(const std::string& name):_name(name), _it(_nh)
{
    cam = NULL;
    heartBeatFrequency = 1;
    _quit = false;
    _active = false;
    dynamic_reconfigure::Server<image_transport_tutorial::MyStuffConfig>::CallbackType f;
    f = boost::bind(&VideoBuffer::messageCallback, this, _1, _2);
    _server.setCallback(f);
    _pub = _it.advertise(_name, 1);
   _rawPub = _it.advertise(_name + "_raw", 1);

}
VideoBuffer::~VideoBuffer()
{
    if(cam)
    {
        if(cam->isOpened())
            cam->release();
        delete cam;
    }
}
void VideoBuffer::acquisitionLoop()
{
    boost::posix_time::ptime lastTime = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime curTime;
    boost::posix_time::time_duration delta;
    cv::Mat prevImg;
    while(!_quit && _nh.ok())
    {
        curTime = boost::posix_time::microsec_clock::universal_time();
        delta = curTime - lastTime;
        cv::Mat img;
        if(!cam)
            continue;
        if(cam->read(img))
        {
            if(img.data != prevImg.data && !img.empty())
            {
                if(display)
                    cv::imshow(_pub.getTopic(), img);
	        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                _rawPub.publish(msg);
                if(_active)
                {
//                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                    _pub.publish(msg);
                }else
                {
                    if(delta.total_milliseconds() > heartBeatFrequency)
                    {
  //                      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                        _pub.publish(msg);
                        lastTime = curTime;
                    }else
                    {
                        _frameBuffer.push_back(img);
                    }
                }
            }

            int key = 0;
	    if(display)
		key =  cv::waitKey(5);
            ros::spinOnce();
            if(key == 113)
                break;
        }
    }
}
void VideoBuffer::messageCallback(image_transport_tutorial::MyStuffConfig& config, uint32_t level)
{
    ROS_INFO("Reconfigure request %d %f ", config.BufferSize, config.HeartbeatFrameRate*1000, config.Active? "True":"False");

    heartBeatFrequency = config.HeartbeatFrameRate * 1000; // 1000 ms per second, input in seconds
    if(cam)
    {
        //cam->set(cv::CAP_PROP_FOURCC,  cv::VideoWriter::fourcc('M','J','P','G'));
        //cam->set(cv::CAP_PROP_FRAME_WIDTH, config.Width);
       // cam->set(cv::CAP_PROP_FRAME_HEIGHT, config.Height);
        //cam->set(cv::CAP_PROP_FPS, 30);
    }
    if(config.Active && !_active)
    {
        // Dump video buffer
        for(boost::circular_buffer<cv::Mat>::iterator it = _frameBuffer.begin(); it != _frameBuffer.end(); ++it)
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *it).toImageMsg();
            _pub.publish(msg);

            //ros::spinOnce();
        }
        _frameBuffer.clear();
    }
    if(config.BufferSize != _frameBuffer.capacity())
        _frameBuffer.set_capacity(config.BufferSize);
    _active = config.Active;
}


bool VideoBuffer::open(int device)
{
    if(!cam)
        cam = new cv::VideoCapture();
    try
    {
        if(!cam->open(device))
            return false;

        //ret = cam->set(cv::CAP_PROP_FOURCC,  cv::VideoWriter::fourcc('M','J','P','G'));
        cv::Mat img;
        cam->read(img);
        int ret = cam->get(cv::CAP_PROP_FRAME_WIDTH);
        ret = cam->get(cv::CAP_PROP_FRAME_HEIGHT);
        ret = cam->get(cv::CAP_PROP_FPS);
        ret = cam->get(cv::CAP_PROP_FOURCC);
        /*
        ret = cam->set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        ret = cam->set(cv::CAP_PROP_FRAME_HEIGHT, 1280);
        ret = cam->set(cv::CAP_PROP_FPS, 30);*/
        acquisitionLoop();
    }catch(cv::Exception &e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }

    return true;
}



int main(int argc, char** argv)
{
    std::string nodeName = "camera" + std::string(argv[1]);
    ros::init(argc, argv, nodeName);
    std::string publishedName = std::string("camera") + std::string(argv[1]) + "/image";
    VideoBuffer buffer(publishedName);
    int device = 0;
    if(argc > 1)
        device = atoi(argv[1]);
    if(device < 0)
        device = 0;
    if(argc > 2)
        buffer.display = false;
    else
        buffer.display = true;
    buffer.display = false;
    buffer.open(device);

    return 1;
}


