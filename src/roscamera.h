#ifndef _H_ROSCAMERA_H_
#define _H_ROSCAMERA_H_

#include <configparser/configparser.h>
#include <visionsystem/genericcamera.h>
#include <vision/image/image.h>
#include <ctime>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace visionsystem
{

class ROSCamera : public visionsystem::GenericCamera, public configparser::WithConfigFile
{
public:
    ROSCamera();

    ~ROSCamera();

    void imageCallback(const sensor_msgs::ImageConstPtr & msg);

    unsigned char * get_data()
    {
        if(img)
            return (unsigned char *)(img->raw_data);
        else
            return (unsigned char *)(img_d->raw_data);
    }

    std::string get_topic() { return _topic; }
    
    /* Camera methods to implement */
    vision::ImageRef get_size() { return _img_size; }
    bool is_active() { return _active; }
    visionsystem::FrameCoding get_coding()
    {
        if(img)
            return VS_RGB32;
        else
            return VS_DEPTH16;
    }
    float get_fps() { return _fps; }
    std::string get_name() { return _name; }
    unsigned int get_frame() { return _frame; }

private:
    void parse_config_line( std::vector<std::string> & line );

    vision::Image<uint32_t, vision::RGB> * img;
    vision::Image<uint16_t, vision::DEPTH> * img_d;
    vision::ImageRef _img_size;
    bool _active;
    float _fps;
    std::string _name;
    std::string _topic;
    unsigned int _frame;
    ros::Time _prev_t;
    unsigned int _buffersize;
};

} // namespace visionsytem

#endif
