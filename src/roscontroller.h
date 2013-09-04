#ifndef _H_ROSCONTROLLER_H_
#define _H_ROSCONTROLLER_H_

#include <visionsystem/controller.h>

#include "roscamera.h"

namespace visionsystem
{

struct ROSControllerImpl;

class ROSController : public visionsystem::Controller
{
public:
    ROSController( visionsystem::VisionSystem * vs, std::string sandbox );
    ~ROSController();

    bool pre_fct( std::vector< GenericCamera *> & cams);
    void preloop_fct();
    void loop_fct();
    bool post_fct();

    void get_cameras(std::vector<GenericCamera *> & cams);
private:
    ROSControllerImpl * impl;
};

} // namespace visionsystem

CONTROLLER(visionsystem::ROSController)

#endif
