#include <array>
#include <cmath>
#include <cfloat>
#include <queue>
#include <string>
#include <list>
#include <chrono>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <iostream>
#include <sstream>
#include <ctime>
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>


#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"

#include "util/globalFuncs.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"
