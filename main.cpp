#include <iostream>
#include <stdio.h>
#include <rplidar.h>
#include <sl_lidar_driver.h>

using namespace rp::standalone::rplidar;
using namespace sl;
/*
int main() {

    RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        std::cout << "Failed to create driver" << std::endl;
        return 1;   
    }

    drv->connect("/dev/ttyUSB0", 115200);





    std::vector<LidarScanMode> scanModes;
    drv->getAllSupportedScanModes(scanModes);
    LidarScanMode scanMode;
    
    drv->startMotor();

    drv->startScan(false, true, 0, &scanMode);


    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);


    drv->grabScanDataHq(nodes, nodeCount);

    //drv->ascendScanData(nodes, nodeCount);

    for (int i = 0; i < nodeCount; i++) {
        //printf("%f %f\n", nodes[i].distance_q2 / 4.0f, nodes[i].angle_q6_checkbit / 64.0f);
        printf("%d \n", nodeCount);
    }

    drv->stop();
    drv->stopMotor();
    drv->disconnect();
    RPlidarDriver::DisposeDriver(drv);
    
    return 0;
}
*/
/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    }
    if (ms != 0)
        usleep(ms * 1000);
}
#endif

using namespace sl;

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) {
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, SLAMTEC Lidar internal error detected. Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }
    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main() {

    int i= 0;
    const char *port = "/dev/ttyUSB0";
    sl_u32 baudrate = 115200;

    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\nVersion: %s\n", SL_LIDAR_SDK_VERSION);

    // Create the driver instance
    ILidarDriver *drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "Insufficient memory or driver creation failed. Exiting.\n");
        exit(-2);
    }

    // Create and connect to the serial port channel
    IChannel *channel = (*createSerialPortChannel(port, baudrate));
    if (!channel) {
        fprintf(stderr, "Failed to create serial port channel. Exiting.\n");
        delete drv;
        exit(-3);
    }

    if (!SL_IS_OK(drv->connect(channel))) {
        fprintf(stderr, "Failed to connect to the device on %s. Exiting.\n", port);
        delete drv;
        exit(-4);
    }

    // Get device information
    sl_lidar_response_device_info_t devinfo;
    sl_result op_result = drv->getDeviceInfo(devinfo);
    if (!SL_IS_OK(op_result))    {
        fprintf(stderr, "Failed to retrieve device information. Exiting.\n");
        delete drv;
        exit(-5);
    }

    // Print device info
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\nFirmware Ver: %d.%02d\nHardware Rev: %d\n",
           devinfo.firmware_version >> 8,
           devinfo.firmware_version & 0xFF,
           (int)devinfo.hardware_version);

    // Check health
    if (!checkSLAMTECLIDARHealth(drv)) {
        delete drv;
        exit(-6);
    }

    signal(SIGINT, ctrlc);

    drv->setMotorSpeed();
    drv->startScan(0, 1);

    while (!ctrl_c_pressed) {
        i++;
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);
        if (!SL_IS_OK(op_result)) {
            fprintf(stderr, "Failed to grab scan data: %x. Stopping.\n", op_result);
            break;
        }

        drv->ascendScanData(nodes, count);
        for (size_t pos = 0; pos < count; ++pos) {
            printf("%s theta: %03.2f Dist: %08.2f Q: %d i: %d \n",
                   (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ",
                   (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                   nodes[pos].dist_mm_q2 / 4.0f,
                   nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT,
                   i);
        }
    }


    if(i>= 100) {
        ctrl_c_pressed = false;
    }
    
        drv->stop();
        delay(200);
        drv->setMotorSpeed(0);



    delete drv;
    return 0;
}
