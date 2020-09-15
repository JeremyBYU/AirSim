#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>

int main()
{
    using namespace msr::airlib;

    msr::airlib::MultirotorRpcLibClient client;

    try {
        client.confirmConnection();

        std::cout << "Press Enter to arm the drone" << std::endl; std::cin.get();
        client.enableApiControl(true);
        client.armDisarm(true);

        std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
        float takeoffTimeout = 5;
        client.takeoffAsync(takeoffTimeout)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when 
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hoverAsync()->waitOnLastTask();

        std::cout << "Counting Lidar Return Size. Press Enter." << std::endl; std::cin.get();
        // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        TTimePoint prev_timestamp = 0;
        for (int i = 0; i < 20;)
        {
            auto response = client.getLidarData("0", "");
            if (response.point_cloud.size() > 2 && response.time_stamp != prev_timestamp) {
                prev_timestamp = response.time_stamp;
                std::cout << "Time Stamp: " << prev_timestamp << "; PC size: " << response.point_cloud.size() / 3 << std::endl;
                i++; //increment
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        client.hoverAsync()->waitOnLastTask();

        std::cout << "Press Enter to land" << std::endl; std::cin.get();
        client.landAsync()->waitOnLastTask();

        std::cout << "Press Enter to disarm" << std::endl; std::cin.get();
        client.armDisarm(false);

    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}