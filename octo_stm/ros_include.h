//std library
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_srvs/Trigger.h>
#include "ros/time.h"
#include "ros/duration.h"

// custom lib
#include <stm_interface/RelayControl.h>

using std_srvs::Trigger;
using stm_interface::RelayControl;

bool servo_flag = true;


//  -- END OF FILE --
