// To compile:
// g++ printIMU.cpp MPU9250.cpp -lwiringPi -lm

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>

// MPU9250 device
#include "MPU9250.h"
MPU9250 device;

int main(int argc, char** argv) {
    printf("Starting program\n");
  
    int me1 = device.getMPU9250ID(); // Should return 0x71
    int me2 = device.getAK8963CID(); // Should return 0x48
  
    printf("MPU9250 found: %xh\n", me1);
    printf("AK8963 found: %xh\n", me2);
  
    device.initMPU9250();
    float calibration[3];
    device.initAK8963(calibration);
  
    printf("\tAccel\t\t\tGyro\t\t\tMag\n");
  
    // Start time
    struct timespec t_start;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t_start);
    uint64_t uptime_start = t_start.tv_nsec/1000;
  
    int index = 0;
    while (1)
    {
        int16_t accel[3] = {0}, gyro[3] = {0}, mag[3] = {0};
        device.readAccelData(accel);
        device.readGyroData(gyro);
        device.readMagData(mag);
      
        // Local time
        time_t rawtime;
        time(&rawtime);
        tm* timeinfo = localtime(&rawtime);
        char time_str[32];
        strftime(time_str, 32, "%Y-%m-%d %H:%M:%S", timeinfo);
        // Time since start
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
        uint64_t uptime = ts.tv_nsec/1000;
        double time_ms = ts.tv_sec - t_start.tv_sec + uptime/1000000.0 - uptime_start/1000000.0;
        printf("%d,%f,%s,\t%d,%d,%d,\t%d,%d,%d,\t%d,%d,%d\n", index, time_ms, time_str, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
        index++;
    }
}
