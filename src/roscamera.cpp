#include "roscamera.h"

namespace visionsystem
{

ROSCamera::ROSCamera()
: img(0), img_d(0), _img_size(640, 480), _active(false), _fps(30), _name("ros-unconfigured"),
  _topic("/camera/rgb/image_color"), _frame(0), _buffersize(100)
{
}

ROSCamera::~ROSCamera()
{
}

void ROSCamera::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
    if(!img && msg->encoding == "bgr8")
    {
        _img_size = vision::ImageRef(msg->width, msg->height);
        img = new vision::Image<uint32_t, vision::RGB>(_img_size);
        _prev_t = msg->header.stamp;
        _buffer.clear();
        for( unsigned int i = 0; i < _buffersize; ++i )
        {
            _buffer.enqueue( new Frame( get_coding(), get_size() ) );
        }
    }
    else if(!img_d)
    {
        _img_size = vision::ImageRef(msg->width, msg->height);
        img_d = new vision::Image<uint16_t, vision::DEPTH>(_img_size);
        _prev_t = msg->header.stamp;
        _buffer.clear();
        for( unsigned int i = 0; i < _buffersize; ++i )
        {
            _buffer.enqueue( new Frame( get_coding(), get_size() ) );
        }
    }
    else
    {
        ros::Duration dt = msg->header.stamp - _prev_t;
        _fps = 1e9/dt.nsec;
        _prev_t = msg->header.stamp;
    }
    if(img)
    {
        for(size_t i = 0; i < img->pixels; ++i)
        {
            img->raw_data[i] = msg->data[3*i+2] + (msg->data[3*i+1] << 8) + (msg->data[3*i] << 16);
        }
    }
    else
    {
        for(size_t i = 0; i < img_d->pixels; ++i)
        {
            img_d->raw_data[i] = msg->data[2*i] + (msg->data[2*i+1] << 8);
        }
    }
    _frame = msg->header.seq;
}

void ROSCamera::parse_config_line( std::vector<std::string> & line )
{
    if( fill_member(line, "Name", _name) )
        return;

    if( fill_member(line, "Active", _active) )
        return;

    if( fill_member(line, "Topic", _topic) )
        return;
}

} //namespace visionsystem
