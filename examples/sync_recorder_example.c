#include <ines_argus.h>
#include <stdio.h>
#include <unistd.h>

int main(){
    int numDevices = 0;
    struct SensorCapabilties sensorCaps;
    struct CameraOptions cameraOptions;

    //initialize argus
    ines_argus_init();

    numDevices = ines_argus_get_num_device();
    printf("number of devices: %d\n", numDevices);

    ines_argus_print_sensor_caps(0);
    sensorCaps = ines_argus_get_sensor_caps(0, 0);

    // select used cameras
    uint32_t cameraIndices[2];
    cameraIndices[0] = 0;
    // cameraIndices[1] = 1; // to add camera 1 cameraOptions.cameraCount has to be incremented
    cameraOptions.cameraIndices = cameraIndices;
    cameraOptions.cameraCount = 1;

    // select camera settings
    cameraOptions.sensorMode = 1;
    cameraOptions.frameDuration = 33333334;

    // set the name of the output file
    char name[10] = "output";
    cameraOptions.fileName = name;

    // more optional options
    // cameraOptions.opticalBlackEnable = 1;
    // cameraOptions.opticalBlack[0] = 0.06;
    // cameraOptions.opticalBlack[1] = 0.0585;
    // cameraOptions.opticalBlack[2] = 0.0585;
    // cameraOptions.opticalBlack[3] = 0.06;

    // cameraOptions.exposureTime = 15000000; //ns
    // cameraOptions.analogGain = xxx;
    // cameraOptions.bitrate = width*height*4;

    // add synchronized camera for recording a h264 file
    ines_argus_add_sync_recorder(cameraOptions);

    // start all cameras
    ines_argus_start();

    // wait for 10 secons
    sleep(10);

    // stop all cameras
    ines_argus_stop();

    // clean up
    ines_argus_destroy();
}
