#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "cfg/cpp/image_transport_tutorial/MyStuffConfig.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/program_options.hpp>
#include <gst/gst.h>
#include <gst/gstelement.h>
#include <gst/gstelementfactory.h>
#include <gst/gstutils.h>
#include <gst/gstpipeline.h>
#include <gst/app/gstappsrc.h>

//#include <boost/log/core.hpp>
//#include <boost/log/trivial.hpp>



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
    void set_rtsp_command(const std::string& command);
    bool open(int device, const std::string& command);
    bool open(const std::string& name, const std::string& command);
    void push_image(cv::Mat img);
    void gst_loop();

    guint need_data_id;
    guint enough_data_id;
    double heartBeatFrequency; // In milliseconds
    cv::VideoCapture* cam;
    bool display;
    bool feed_enabled;
    GstElement* source_OpenCV;
    GstElement* pipeline;
    GMainLoop* glib_MainLoop;
    cv::Size imgSize;
    boost::thread glibThread;
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
typedef class VideoBuffer App;
static gboolean
bus_message(GstBus * bus, GstMessage * message, App * app)
{
    //BOOST_LOG_TRIVIAL(debug) << "Received message type: " << gst_message_type_get_name(GST_MESSAGE_TYPE(message));
    GError *err = NULL;
    gchar *dbg_info = NULL;
    switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR:
    {
        gst_message_parse_error(message, &err, &dbg_info);
        std::cout << "Error from element " << GST_OBJECT_NAME(message->src) << ": " << err->message << std::endl;
        g_error_free(err);
        g_free(dbg_info);
        g_main_loop_quit(app->glib_MainLoop);
        break;
    }
    case GST_MESSAGE_WARNING:
    {
        gst_message_parse_warning(message, &err, &dbg_info);
        std::cout << "Warning from element " << GST_OBJECT_NAME(message->src) << ": " << err->message << std::endl;
        g_error_free(err);
        g_free(dbg_info);
        break;
    }
    case GST_MESSAGE_INFO:
    {
        gst_message_parse_info(message, &err, &dbg_info);
        std::cout << "Info from element " << GST_OBJECT_NAME(message->src) << ": " << err->message << std::endl;
        g_error_free(err);
        g_free(dbg_info);
        break;
    }

    case GST_MESSAGE_EOS:
    {

        g_main_loop_quit(app->glib_MainLoop);
        break;
    }
    default:
        break;
    }
    return TRUE;
}


void VideoBuffer::push_image(cv::Mat img)
{
    while(!feed_enabled){}
    static GstClockTime timestamp = 0;

    GstBuffer* buffer;
    GstMapInfo map;
    int bufferlength = img.cols * img.rows * img.channels();
    buffer = gst_buffer_new_and_alloc(bufferlength);
    gst_buffer_map(buffer, &map, (GstMapFlags)GST_MAP_WRITE);
    memcpy(map.data, img.data, map.size);
    gst_buffer_unmap(buffer, &map);

    GST_BUFFER_PTS(buffer) = timestamp;

    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30);
    timestamp += GST_BUFFER_DURATION(buffer);

    GstFlowReturn rw;
    g_signal_emit_by_name(source_OpenCV, "push-buffer", buffer, &rw);

    if (rw != GST_FLOW_OK)
    {
        return;
    }
    gst_buffer_unref(buffer);
}

VideoBuffer::VideoBuffer(const std::string& name):_name(name), _it(_nh)
{
    glib_MainLoop = NULL;
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
                push_image(img);
                //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                //_rawPub.publish(msg);
                if(_active)
                {
//                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                    //_pub.publish(msg);
                //    push_image(img);
                }else
                {
                    if(delta.total_milliseconds() > heartBeatFrequency)
                    {
  //                      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                        //_pub.publish(msg);
                  //      push_image(img);
                        lastTime = curTime;
                    }else
                    {
                    //    _frameBuffer.push_back(img);
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
            //_pub.publish(msg);
            push_image(*it);

            //ros::spinOnce();
        }
        _frameBuffer.clear();
    }
    if(config.BufferSize != _frameBuffer.capacity())
        _frameBuffer.set_capacity(config.BufferSize);
    _active = config.Active;
}

bool VideoBuffer::open(const std::string& name, const std::string& command)
{
    std::cout << "Opening: " << name << std::endl;
    if(!cam)
        cam = new cv::VideoCapture();
    try
    {
        if(!cam->open(name))
            return false;
        //ret = cam->set(cv::CAP_PROP_FOURCC,  cv::VideoWriter::fourcc('M','J','P','G'));
        cv::Mat img;
        cam->read(img);
        imgSize = img.size();
        set_rtsp_command(command);
        int ret = cam->get(cv::CAP_PROP_FRAME_WIDTH);
        ret = cam->get(cv::CAP_PROP_FRAME_HEIGHT);
        ret = cam->get(cv::CAP_PROP_FPS);
        ret = cam->get(cv::CAP_PROP_FOURCC);
        acquisitionLoop();
    }catch(cv::Exception &e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
}

bool VideoBuffer::open(int device, const std::string &command)
{

    if(!cam)
        cam = new cv::VideoCapture();
    try
    {
        if(!cam->open(device))
            return false;

        cv::Mat img;
        cam->read(img);
        imgSize = img.size();
        set_rtsp_command(command);
        int ret = cam->get(cv::CAP_PROP_FRAME_WIDTH);
        ret = cam->get(cv::CAP_PROP_FRAME_HEIGHT);
        ret = cam->get(cv::CAP_PROP_FPS);
        ret = cam->get(cv::CAP_PROP_FOURCC);
        acquisitionLoop();
    }catch(cv::Exception &e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }

    return true;
}
void VideoBuffer::gst_loop()
{
    if (!g_main_loop_is_running(glib_MainLoop))
    {
        g_main_loop_run(glib_MainLoop);
    }
    std::cout << "glib loop shutting down\n";
}
static void start_feed(GstElement * pipeline, guint size, App * app)
{
    app->feed_enabled = true;
}
static void stop_feed(GstElement * pipeline, App *app)
{
    app->feed_enabled = false;
}

void VideoBuffer::set_rtsp_command(const std::string& command)
{
    std::cout <<"RTSP server command: " << command << std::endl;

    glib_MainLoop = g_main_loop_new(NULL, 0);

    glibThread = boost::thread(boost::bind(&VideoBuffer::gst_loop, this));
    GError* error = NULL;
    pipeline = gst_parse_launch(command.c_str(), &error);

    if (error != NULL)
    {
        //BOOST_LOG_TRIVIAL(error) << "Error parsing pipeline " << error->message;
        std::cout << "Error parsing pipeline " << error->message << std::endl;
        return;
    }
    source_OpenCV = gst_bin_get_by_name(GST_BIN(pipeline), "mysource");

    GstCaps* caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, imgSize.width,
        "height", G_TYPE_INT, imgSize.height,
        "framerate", GST_TYPE_FRACTION, 1, 15,
        "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1,
        NULL);
    if (caps == NULL)
    {
        //BOOST_LOG_TRIVIAL(error) << "Error creating caps for appsrc";
        std::cout << "Error creating caps for appsrc" << std::endl;
        return;
    }

    gst_app_src_set_caps(GST_APP_SRC(source_OpenCV), caps);

    g_object_set(G_OBJECT(source_OpenCV),
        "stream-type", GST_APP_STREAM_TYPE_STREAM,
        "format", GST_FORMAT_TIME,
        NULL);

    need_data_id = g_signal_connect(source_OpenCV, "need-data", G_CALLBACK(start_feed), this);
    enough_data_id = g_signal_connect(source_OpenCV, "enough-data", G_CALLBACK(stop_feed), this);

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    CV_Assert(bus);
    gst_bus_add_watch(bus, (GstBusFunc)bus_message, this);
    gst_object_unref(bus);

    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    CV_Assert(ret != GST_STATE_CHANGE_FAILURE);
}



int main(int argc, char** argv)
{
    char** _argv = new char*{"-vvv"};
    int _argc = 1;
    gst_init(&_argc, &_argv);
    gst_debug_set_active(1);

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("CameraName", boost::program_options::value<std::string>(), "Optional opencv camera command")
            ("CameraNum", boost::program_options::value<int>(), "Optional camera number")
            ("Display", boost::program_options::value<bool>(false), "Set to true to popup a display window")
            ("Server_command", boost::program_options::value<std::string>(), "Optional gstreamer command for server, defaults to \"appsrc name=mysource ! videoconvert ! omxh264enc ! rtph264pay config-interval=1 pt=96 ! gdppay ! tcpserversink host=127.0.0.1\"")
            ("Help", boost::program_options::value<std::string>(), "Prints this menu");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    int device = 0;
    std::string name;
    std::string server_command = "appsrc name=mysource ! videoconvert ! omxh264enc ! rtph264pay config-interval=1 pt=96 ! gdppay ! tcpserversink host=127.0.0.1 port=8004";
    if(vm.count("Help"))
    {
        std::cout << desc << std::endl;
    }
    if(vm.count("Server_command"))
    {
        server_command = vm["Server_command"].as<std::string>();
    }
    if(vm.count("CameraName"))
    {
        name =  vm["CameraName"].as<std::string>();
        device = -1;
    }else
    {
        if(vm.count("CameraNum"))
        {
            device = vm["CameraNum"].as<int>();
        }
    }
    ros::init(argc, argv, "camera");
    std::string publishedName = "camera/image";
    VideoBuffer buffer(publishedName);
    if(vm.count("Display"))
    {
        buffer.display = vm["Display"].as<bool>();
    }else
    {
        buffer.display = false;
    }

    //buffer.set_rtsp_command(server_command);

    if(device != -1)
        buffer.open(device, server_command);
    else
        buffer.open(name, server_command);

    return 0;
}


