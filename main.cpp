#include "openvr.h"
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

std::vector<float> matrix2YawPitchRoll(const float matrix[3][4]) {
    std::vector<float> angles(3, 0.0);

    double r02 = matrix[0][2];
    if (r02 < 1.0) {
        if (r02 > -1.0) {
            angles[1] = asin(r02); // Y
            angles[0] = atan2(-matrix[1][2], matrix[2][2]); // X
            angles[2] = atan2(-matrix[0][1], matrix[0][0]); // Z
        } else {
            angles[1] = -M_PI / 2.0; 
            angles[0] = -atan2(matrix[1][0], matrix[1][1]);
            angles[2] = 0.0;
        }
    } else {
        angles[1] = M_PI / 2.0;
        angles[0] = atan2(matrix[1][0], matrix[1][1]);
        angles[2] = 0.0;
    }

    return angles; // Order: X, Y, Z
}


int main ( int argc, char *argv[] )
{
    const uint32_t max_devices = 4;       // 2 lighthouses, 2 vive trackers
    const uint32_t number_of_devices = 2; //vr::k_unMaxTrackedDeviceCount; - depends on the number base stations (total number of devices)
    vr::EVRInitError *peError = 0;
    vr::IVRSystem *systemInstance = vr::VR_Init( peError, vr::EVRApplicationType::VRApplication_Background );

    //if ( *peError != 0)
    //{
    //    std::cerr << "Unable to init VR rundtime: " << vr:: VR_GetVRInitErrorAsEnglishDescription(*peError) << std::endl;
    //    return EXIT_FAILURE;
    //}
    

    std::cout << "VR Initialzied" << std::endl;
    std::vector<float> angles;
    const float predicitionHorizon = 0.0; 
    vr::TrackedDevicePose_t trackedDevicesPoses[vr::k_unMaxTrackedDeviceCount] = {0};


    // INIT Robot
    int rt_receive_priority = 90;
    int rt_control_priority = 85;
    uint16_t flags = ur_rtde::RTDEControlInterface::FLAG_VERBOSE | ur_rtde::RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
    int ur_cap_port = 50002;
    double vel = 0.5;
    double acc = 0.5;
    double frec = 500.0; // Hz
    double dt = 1.0/frec; // 2ms
    double lookahead_time = 0.1;
    double gain = 300;
    std::vector<double> joint_q = {0.0, -1.57, -2.28, 0.57, 1.60, 0.023};

    ur_rtde::RTDEControlInterface rtde_control("192.168.1.127", frec, flags, ur_cap_port, rt_control_priority);
    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.1.127", frec, {}, true, false, rt_receive_priority);
    //rtde_control.setWatchdog( frec/2.0f );
    rtde_control.moveJ( joint_q );
    std::vector<double> initPose = rtde_receive.getActualTCPPose();
    std::vector<double> servo_target;
    std::vector<double> frameOffset = initPose;

    // Make the transform difference
    systemInstance->GetDeviceToAbsoluteTrackingPose( vr::TrackingUniverseSeated, predicitionHorizon, trackedDevicesPoses, number_of_devices );
    angles = matrix2YawPitchRoll( trackedDevicesPoses[1].mDeviceToAbsoluteTracking.m );
    frameOffset[0] += trackedDevicesPoses[1].mDeviceToAbsoluteTracking.m[0][3];
    frameOffset[1] -= trackedDevicesPoses[1].mDeviceToAbsoluteTracking.m[2][3];
    frameOffset[2] -= trackedDevicesPoses[1].mDeviceToAbsoluteTracking.m[1][3];
    frameOffset[3] -= angles[0];
    frameOffset[4] -= angles[1];
    frameOffset[5] -= angles[2]; 

    std::ofstream logFile("log.txt", std::ios_base::app); // Open the log file in append mode


    while(true)
    {
        //rtde_control.kickWatchdog()
        systemInstance->GetDeviceToAbsoluteTrackingPose( vr::TrackingUniverseSeated, predicitionHorizon, trackedDevicesPoses, number_of_devices );
        

        for ( vr::TrackedDeviceIndex_t i = 1; i < max_devices; ++i )    
        {            
            vr::TrackedDeviceClass deviceClass = systemInstance->GetTrackedDeviceClass( i );
            if ( deviceClass == vr::TrackedDeviceClass_GenericTracker )
            {
                angles = matrix2YawPitchRoll( trackedDevicesPoses[i].mDeviceToAbsoluteTracking.m );
                servo_target = frameOffset;
                servo_target[0] -= trackedDevicesPoses[i].mDeviceToAbsoluteTracking.m[0][3];
                servo_target[1] += trackedDevicesPoses[i].mDeviceToAbsoluteTracking.m[2][3];
                servo_target[2] += trackedDevicesPoses[i].mDeviceToAbsoluteTracking.m[1][3];
                servo_target[3] += angles[0];
                servo_target[4] += angles[1];
                servo_target[5] += angles[2];

                logFile << servo_target[0] << " " << servo_target[1] << " " << servo_target[2] << " " << servo_target[3] << " " << servo_target[4] << " " << servo_target[5] << std::endl;
                
                if ( rtde_control.isJointsWithinSafetyLimits(servo_target) )
                {
                    std::chrono::steady_clock::time_point t_start = rtde_control.initPeriod();
                    rtde_control.servoL(servo_target, vel, acc, dt, lookahead_time, gain);
                    rtde_control.waitPeriod(t_start);
                }
            }
        }
    }
    /*
    */
    //vr::VR_Shutdown();
    return EXIT_SUCCESS;
}