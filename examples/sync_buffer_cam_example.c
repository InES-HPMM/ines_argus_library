#include <ines_argus.h>
#include <stdio.h>
#include <unistd.h>

void testfunction_sync(void **buf, uint32_t numBuf, uint32_t lineWidth, uint32_t height, uint32_t pitch)
{
    uint8_t * p;
    FILE *pFile;

    printf("callback_sync ----->   numBuf = %d  lineWidth = %d  height = %d pitch = %d\n", numBuf, lineWidth, height, pitch);

    pFile = fopen("uyvymerge.raw", "a");
    if (!pFile){
        printf("could not open file\n");
        return;
    }

    for(uint32_t i=0; i<numBuf; i++){
        for(uint32_t r = 0; r < height; r++){
            p = (uint8_t*)buf[i] + r*pitch;
            fwrite(p, 1, lineWidth, pFile);
        }
    }
    fclose(pFile);
}

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
    //cameraIndices[1] = 1; // to add camera 1 cameraOptions.cameraCount has to be incremented
    cameraOptions.cameraCount = 1;
  	cameraOptions.cameraIndices = cameraIndices;

    // select camera settings
    cameraOptions.sensorMode = 0;
    cameraOptions.frameDuration = 33333334;

    // more optional options
    // cameraOptions.opticalBlackEnable = 1;
    // cameraOptions.opticalBlack[0] = 0.06;
    // cameraOptions.opticalBlack[1] = 0.0585;
    // cameraOptions.opticalBlack[2] = 0.0585;
    // cameraOptions.opticalBlack[3] = 0.06;

    // cameraOptions.exposureTime = xxx;
    // cameraOptions.analogGain = xxx;

    // add synchronized camera to manipulate buffer
    ines_argus_add_sync_cam(cameraOptions, testfunction_sync);

    // start all cameras
    ines_argus_start();

    // wait for 5 secons
    sleep(5);

    // stop all cameras
    ines_argus_stop();

    // clean up
    ines_argus_destroy();
}
