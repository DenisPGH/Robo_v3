

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <fstream>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace sl;



bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
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
	const char * opt_is_channel = "--channel"; 
	const char * opt_channel = "--serial";
    const char * opt_channel_param_first = "/dev/ttyUSB0";
	sl_u32         opt_channel_param_second = 115200;
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

	bool useArgcBaudrate = true;

    IChannel* _channel;

 
    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

   // create a channel
    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second ));
    if (SL_IS_OK((drv)->connect(_channel))) {
        op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_OK(op_result)) 
        {
	        connectSuccess = true;
        }
        else{
            delete drv;
			drv = NULL;
        }
    }
    
    


    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
			(fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
				, opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
				, opt_channel_param_first));
		
        goto on_finished;
    }


    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);
    // open file connection
    
    // fetech result and print it out...
    while (1) {
        
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);
        int16_t count_point=0;
        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            std::ofstream out("current_scan.csv"); //open file
            for (int pos = 0; pos < (int)count ; ++pos) {
                if(nodes[pos].dist_mm_q2/4.0f>0){
                    count_point++;
                    // printf("%s theta: %03.2f Dist: %08.2f Q: %d points: %d \n", 
                    // (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    // (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                    // nodes[pos].dist_mm_q2/4.0f,
                    // nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT,
                    // count_point);

                    //write to outputfile
                    
                    out <<(nodes[pos].angle_z_q14 * 90.f) / 16384.f << "|" << nodes[pos].dist_mm_q2/4.0f<<std::endl; //deg,mm
                    


                }

                
            }
            out.close(); //close file
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }

    drv->stop();
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    // done!
on_finished:
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}

