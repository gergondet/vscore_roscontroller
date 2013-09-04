#include "roscontroller.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include <map>
#include <vector>

using namespace visionsystem;
namespace bfs = boost::filesystem;

namespace visionsystem
{

struct ROSControllerImpl
{
public:
    ROSControllerImpl() : nh(), it(nh)
    {
    }

    ~ROSControllerImpl()
    {
        for(size_t i = 0; i < cams.size(); ++i)
        {
            delete cams[i];
        }
    }

    void AddCamera(ROSCamera * cam)
    {
        cams.push_back(cam);
        frames[cam] = 0;
        subs[cam] = it.subscribe(cam->get_topic(), 1, &ROSCamera::imageCallback, cam);
    }

    void PublishFrames()
    {
        for(size_t i = 0; i < cams.size(); ++i)
        {
            if(frames[cams[i]] != cams[i]->get_frame())
            {
                frames[cams[i]] = cams[i]->get_frame();
                Frame * vsframe = cams[i]->_buffer.pull();
                std::memcpy( vsframe->_data, cams[i]->get_data(), vsframe->_data_size );
                cams[i]->_buffer.push(vsframe);
            }
        }
    }
public:
    std::vector<ROSCamera *> cams;
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    std::map<ROSCamera *, unsigned int> frames;
    std::map<ROSCamera *, image_transport::Subscriber> subs;
};

ROSController::ROSController(visionsystem::VisionSystem * vs, std::string sandbox)
: Controller(vs, "roscontroller", sandbox)
{
    int argc = 0;
    char * argv[] = {};
    ros::init(argc, argv, "vscore_roscontroller");

    impl = new ROSControllerImpl();
}

ROSController::~ROSController()
{
    delete impl;
}

bool ROSController::pre_fct( std::vector< GenericCamera *> & cams)
{
    bfs::path sandbox( get_sandbox() );
    if( bfs::is_directory(sandbox) )
    {
        std::vector<bfs::path> ls_path;
        copy(bfs::directory_iterator(sandbox), bfs::directory_iterator(), back_inserter(ls_path));
        for(std::vector< bfs::path >::const_iterator it = ls_path.begin(); it != ls_path.end(); ++it)
        {
            if( bfs::is_regular_file(*it) && bfs::extension(*it) == ".conf" )
            {
                ROSCamera * cam = new ROSCamera();
                cam->read_config_file( (*it).string().c_str() );
                if(cam->is_active())
                {
                    impl->AddCamera(cam);
                    cams.push_back(cam);
                }
                else
                {
                    delete cam;
                }
            }
        }
    }
    return true;
}

void ROSController::preloop_fct()
{
}

void ROSController::loop_fct()
{
    ros::spinOnce();
    impl->PublishFrames();
}

bool ROSController::post_fct()
{
    return true;
}

void ROSController::get_cameras(std::vector<GenericCamera *> & cams)
{
    for(size_t i = 0; i < impl->cams.size(); ++i)
    {
        cams.push_back(impl->cams[i]);
    }
}

} // namespace visionsystem
