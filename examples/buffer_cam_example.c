#include <ines_argus.h>
#include <stdio.h>
#include <unistd.h>

void callback_1(void *buf, uint32_t lineWidth, uint32_t height, uint32_t pitch)
{
    printf("callback_1 ----->   lineWidth = %d  height = %d pitch = %d\n", lineWidth, height, pitch);
    uint8_t * p;
    FILE *pFile;

    pFile = fopen("uyvy422output_1.raw", "a");
    if (!pFile){
        return;
    }

    //write data to file
    for(uint32_t r = 0; r < height; r++){
        p = (uint8_t*)buf + r*pitch;
        fwrite(p, 1, lineWidth, pFile);
    }

    fclose(pFile);
}

void callback_2(void *buf, uint32_t lineWidth, uint32_t height, uint32_t pitch)
{
    printf("callback_2 ----->   lineWidth = %d  height = %d pitch = %d\n", lineWidth, height, pitch);
    uint8_t * p;
    FILE *pFile;

    pFile = fopen("uyvy422output_2.raw", "a");
    if (!pFile){
        return;
    }

    //write data to file
    for(uint32_t r = 0; r < height; r++){
        p = (uint8_t*)buf + r*pitch;
        fwrite(p, 1, lineWidth, pFile);
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

    // cameraOptions.exposureTime = xxx; //ns
    // cameraOptions.analogGain = xxx;

    // add camera to manipulate buffer
    ines_argus_add_cam(cameraOptions, callback_1);

    //In case you want to add camera 1 with its own callback (same general camera settings)
    //cameraIndices[0] = 1;
    //cameraOptions.cameraIndices = cameraIndices;
    //ines_argus_add_cam(cameraOptions, callback_2);


    // start all cameras
    ines_argus_start();

    // wait for 5 secons
    sleep(5);

    // stop all cameras
    ines_argus_stop();

    // clean up
    ines_argus_destroy();

}
