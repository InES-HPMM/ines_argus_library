/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
 * Copyright (c) 2018, ZHAW Institute of Embedded Systems (Richard Weiss)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This software contains code excerpts from the 10_camera_recording
 * sample of the Argus library.
 */

#include "ines_argus.h"
#include <Argus/Argus.h>
#include "BufferConsumer.h"
#include "SyncBufferConsumer.h"
#include "H264Consumer.h"
#include <stdlib.h>

#define EXIT_IF_NULL(val,msg)   \
        {if (!val) {printf("%s\n",msg); return EXIT_FAILURE;}}
#define EXIT_IF_NOT_OK(val,msg) \
        {if (val!=Argus::STATUS_OK) {printf("%s\n",msg); return 1;}}


/*****************define static variables******************/
//camera provider
static UniqueObj<CameraProvider> *cameraProvider;
static ICameraProvider *iCameraProvider = 0;

//camera devices
static std::vector<CameraDevice*> cameraDevices;
static std::vector<ICameraProperties*> iCameraDevice_v; //atm solved with vector not critical (unused)

//capture capture sessions
static UniqueObj<CaptureSession> *captureSession[MAX_DEVICES];
static ICaptureSession *iCaptureSession[MAX_DEVICES];
static uint32_t numberOfSessions = 0;
static uint32_t numberOfSets=0;

//stream
static UniqueObj<OutputStreamSettings> *streamSettings[MAX_DEVICES];
static IOutputStreamSettings *iStreamSettings[MAX_DEVICES];
static UniqueObj<OutputStream> *stream[MAX_DEVICES];

//consumers
static BufferConsumerThread *bufferConsumer[MAX_DEVICES];

//options
static struct CameraOptions opt[MAX_DEVICES];


/******************synch variables*********************/
//capture sessions
static std::vector<UniqueObj<CaptureSession>*> captureSession_v;
static std::vector<ICaptureSession*> iCaptureSession_v;

//stream
static std::vector<UniqueObj<OutputStreamSettings>*> streamSettings_v;
static std::vector<IOutputStreamSettings*> iStreamSettings_v;
static std::vector<UniqueObj<OutputStream>*> stream_v;

//consumer
static SyncBufferConsumerThread *syncBufferConsumer;

//options
static struct CameraOptions syncOpt;
static std::vector<int> camera_v;


/***************synch recorder variables***************/
//capture sessions
static std::vector<UniqueObj<CaptureSession>*> captureSession_rec;
static std::vector<ICaptureSession*> iCaptureSession_rec;

//stream
static std::vector<UniqueObj<OutputStreamSettings>*> streamSettings_rec;
static std::vector<IOutputStreamSettings*> iStreamSettings_rec;
static std::vector<UniqueObj<OutputStream>*> stream_rec;

//consumer
static H264Consumer *h264Consumer;

//options
static struct CameraOptions Opt_rec;
static std::vector<int> camera_rec;

// initalizes the Argus camera provider
// returns 0 if successful
int ines_argus_init(){
    //initialize the camera provider
    cameraProvider = new UniqueObj<CameraProvider>(CameraProvider::create());
    iCameraProvider = interface_cast<ICameraProvider>(*cameraProvider);
    if (!iCameraProvider){
        printf("Failed to get ICameraProvider interface\n");
        return 1;
    }

    // get the camera devices
    iCameraProvider->getCameraDevices(&cameraDevices);
    if (cameraDevices.size() == 0){
        printf("No cameras available\n");
        return 1;
    }

    // get camera ICameraProperties
    for (uint32_t i = 0; i<cameraDevices.size(); i++){
        if (i > 5)
              break;
        iCameraDevice_v.push_back(interface_cast<ICameraProperties>(cameraDevices[i]));
        if (!iCameraDevice_v[i]) {
            printf("Failed to get camera device properties\n");
        }
    }
    return 0;
}

// returns the number of available camera devices
int ines_argus_get_num_device(){
    // Get the camera devices.
    return cameraDevices.size();
}

// returns the capabilities of the chosen video device in the specified mode
struct SensorCapabilties ines_argus_get_sensor_caps(int device, int mode){
    struct SensorCapabilties sensorCaps;

    // check if device is available
    if (!iCameraDevice_v[device]) {
        printf("Failed to get camera device\n");
    }

    // get all sensor modes of specified device
    std::vector<Argus::SensorMode *> sensorModes;
    iCameraDevice_v[device]->getBasicSensorModes(&sensorModes);
    if (!sensorModes.size()) {
        printf("Failed to get valid sensor mode list.\n");
    }

    // check if mode is in range
    if (sensorModes.size() <= (uint32_t) mode){
        printf("Sensor mode is out of range\n");
        return sensorCaps;
    }

    // get demanded sensorMode
    ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensorModes[mode]);
    if (!iSensorMode){
        printf("Sensor mode could not be loaded\n");
        return sensorCaps;
    }

    // get capabilties
    sensorCaps.height = iSensorMode->getResolution().height();
    sensorCaps.width = iSensorMode->getResolution().width();
    sensorCaps.minExposureTime = iSensorMode->getExposureTimeRange().min();
    sensorCaps.maxExposureTime = iSensorMode->getExposureTimeRange().max();
    sensorCaps.minFrameDuration = iSensorMode->getFrameDurationRange().min();
    sensorCaps.maxFrameDuration = iSensorMode->getFrameDurationRange().max();
    sensorCaps.minAnalogGain = iSensorMode->getAnalogGainRange().min();
    sensorCaps.maxAnalogGain = iSensorMode->getAnalogGainRange().max();

    return sensorCaps;
}

// prints the capabilities of the chosen video device
int ines_argus_print_sensor_caps(int device){
    // Use the specified device.
    if (!iCameraDevice_v[device]) {
        printf("Failed to get camera device properties\n");
        return 1;
    }
    std::vector<Argus::SensorMode *> sensorModes;
    iCameraDevice_v[device]->getBasicSensorModes(&sensorModes);
    if (!sensorModes.size()) {
        printf("Failed to get valid sensor mode list.\n");
        return 1;
    }

    printf("\nAvailable sensor modes:\n");
    for (uint32_t i = 0; i < sensorModes.size(); i++) {
        ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensorModes[i]);
        printf("Mode %d\n", i);
        printf("resolution: %dx%d\n", iSensorMode->getResolution().width(),
               iSensorMode->getResolution().height());
        printf("ExposureTimeRange: [%ld, %ld]ns\n",
               iSensorMode->getExposureTimeRange().min(),
               iSensorMode->getExposureTimeRange().max());
        printf("FrameDurationRange: [%ld, %ld]ns\n",
               iSensorMode->getFrameDurationRange().min(),
               iSensorMode->getFrameDurationRange().max());
        printf("AnalogGainRange: [%f, %f]\n",
               iSensorMode->getAnalogGainRange().min(),
               iSensorMode->getAnalogGainRange().max());
        printf("InputBitDepth: %d\n", iSensorMode->getInputBitDepth());
        printf("OutputBitDepts: %d\n", iSensorMode->getOutputBitDepth());
    }
    printf("\n");
    return 0;
}

// creates a new camera stream and returns the frame buffer in the callback function
int ines_argus_add_cam(struct CameraOptions options, void (*callback)(void *, uint32_t, uint32_t, uint32_t)){
    if(options.cameraCount > 1){
        printf("!!!WARNING!!! multiple independent cameras added with the same callback");
    }
    for(uint32_t i=0; i<options.cameraCount; i++){
        //save options for later
        opt[numberOfSessions] = options;

        // check if device is available
        if (!iCameraDevice_v[options.cameraIndices[i]]) {
            printf("Failed to get camera device\n");
        }

        // set demanded sensor mode and define StreamSize
        Size2D<uint32_t> StreamSize;
        std::vector<Argus::SensorMode *> sensorModes;
        iCameraDevice_v[options.cameraIndices[i]]->getBasicSensorModes(&sensorModes);
        if (!sensorModes.size()) {
            printf("Failed to get valid sensor mode list.\n");
        }
        ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensorModes[options.sensorMode]);

        if (options.sensorMode >= sensorModes.size()){
            printf("Invalid sensor mode\n");
            return 1;
        }
        if (!iSensorMode){
            printf("Sensor mode could not be loaded\n");
            return 1;
        }
        StreamSize = Size2D<uint32_t>(iSensorMode->getResolution().width(), iSensorMode->getResolution().height());

        // Create the capture session.
        if (numberOfSessions >= MAX_DEVICES){
            printf("no more sessions allowed\n");
            return 1;
        }
        captureSession[numberOfSessions] = new UniqueObj<CaptureSession>(
                iCameraProvider->createCaptureSession(cameraDevices[options.cameraIndices[i]]));
        iCaptureSession[numberOfSessions] = interface_cast<ICaptureSession>(*captureSession[numberOfSessions]);
        if (!iCaptureSession[numberOfSessions])
            printf("Failed to create CaptureSession\n");

        // Create the stream settings and set the common properties.
        streamSettings[numberOfSessions] = new UniqueObj<OutputStreamSettings>(iCaptureSession[numberOfSessions]->createOutputStreamSettings());
        iStreamSettings[numberOfSessions] = interface_cast<IOutputStreamSettings>(*streamSettings[numberOfSessions]);
        if (!iStreamSettings[numberOfSessions])
            printf("Failed to create OutputStreamSettings\n");
        iStreamSettings[numberOfSessions]->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
        iStreamSettings[numberOfSessions]->setResolution(StreamSize);

        // create output stream
        stream[numberOfSessions] = new UniqueObj<OutputStream>(
            iCaptureSession[numberOfSessions]->createOutputStream(streamSettings[numberOfSessions]->get()));

        // initialize the buffer consumer
        bufferConsumer[numberOfSessions] = new BufferConsumerThread(stream[numberOfSessions]->get(), StreamSize);
        bufferConsumer[numberOfSessions]->setCallback(callback);

        // increment numberOfSessions and return
        numberOfSessions += 1;
    }
    numberOfSets += 1;

    return 0;
}

// create synchronized camera streames that are processed in one consumer
int ines_argus_add_sync_cam(struct CameraOptions options, void (*callback)(void **, uint32_t, uint32_t, uint32_t, uint32_t)){
    Size2D<uint32_t> StreamSize;
    //save options for later
    syncOpt = options;

    // set demanded sensor mode and define StreamSize
    std::vector<Argus::SensorMode *> sensorModes;
    iCameraDevice_v[options.cameraIndices[0]]->getBasicSensorModes(&sensorModes);
    if (!sensorModes.size()) {
        printf("Failed to get valid sensor mode list.\n");
    }
    ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensorModes[options.sensorMode]);

    if (options.sensorMode >= sensorModes.size()){
        printf("Invalid sensor mode\n");
        return 1;
    }
    if (!iSensorMode){
        printf("Sensor mode could not be loaded\n");
        return 1;
    }
    StreamSize = Size2D<uint32_t>(iSensorMode->getResolution().width(), iSensorMode->getResolution().height());

    // add all cameras
    for(uint32_t i = 0; i<options.cameraCount; i++){
        printf("Add sync camera Nr. %d\n", i);

        //save used cameras for later
        camera_v.push_back(options.cameraIndices[i]);

        // check if device is available
        if (!iCameraDevice_v[options.cameraIndices[i]]) {
            printf("Failed to get camera device\n");
            return 1;
        }

        // Create the capture session.
        captureSession_v.push_back(new UniqueObj<CaptureSession>(
                iCameraProvider->createCaptureSession(cameraDevices[options.cameraIndices[i]])));
        iCaptureSession_v.push_back(interface_cast<ICaptureSession>(*captureSession_v[i]));
        if (!iCaptureSession_v[iCaptureSession_v.size()-1])
            printf("Failed to create CaptureSession\n");

        // Create the stream settings and set the common properties.
        streamSettings_v.push_back(new UniqueObj<OutputStreamSettings>(iCaptureSession_v[iCaptureSession_v.size()-1]->createOutputStreamSettings()));
        iStreamSettings_v.push_back(interface_cast<IOutputStreamSettings>(*streamSettings_v[streamSettings_v.size()-1]));
        if (!iStreamSettings_v[iStreamSettings_v.size()-1])
            printf("Failed to create OutputStreamSettings\n");
        iStreamSettings_v[iStreamSettings_v.size()-1]->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
        iStreamSettings_v[iStreamSettings_v.size()-1]->setResolution(StreamSize);

        // create output stream
        stream_v.push_back(new UniqueObj<OutputStream>(
            iCaptureSession_v[i]->createOutputStream(streamSettings_v[i]->get())));

    }
    // create new BufferConsumer and set callback
    syncBufferConsumer = new SyncBufferConsumerThread(stream_v, StreamSize);
    syncBufferConsumer->setCallback(callback);
    return 0;
}
int ines_argus_add_sync_recorder(struct CameraOptions options){
  if (options.cameraIndices == 0){
      printf("!!WARNING!! cameraIndices not set -> no cameras added\n");
      return 1;
  }
  Size2D<uint32_t> StreamSize;

  //save options for later
  Opt_rec = options;

  // set demanded sensor mode and define StreamSize
  std::vector<Argus::SensorMode *> sensorModes;
  iCameraDevice_v[options.cameraIndices[0]]->getBasicSensorModes(&sensorModes);
  if (!sensorModes.size()) {
      printf("Failed to get valid sensor mode list.\n");
  }
  ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensorModes[options.sensorMode]);

  if (options.sensorMode >= sensorModes.size()){
      printf("Invalid sensor mode\n");
      return 1;
  }
  if (!iSensorMode){
      printf("Sensor mode could not be loaded\n");
      return 1;
  }
  StreamSize = Size2D<uint32_t>(iSensorMode->getResolution().width(), iSensorMode->getResolution().height());

  // add all cameras
  for(uint32_t i = 0; i<options.cameraCount; i++){
      printf("Add h264 recording camera Nr. %d\n", i);
      //save used cameras for later
      camera_rec.push_back(options.cameraIndices[i]);

      // check if device is available
      if (!iCameraDevice_v[options.cameraIndices[i]]) {
          printf("Failed to get camera device\n");
          return 1;
      }

      // Create the capture session.
      captureSession_rec.push_back(new UniqueObj<CaptureSession>(
              iCameraProvider->createCaptureSession(cameraDevices[options.cameraIndices[i]])));
      iCaptureSession_rec.push_back(interface_cast<ICaptureSession>(*captureSession_rec[i]));
      if (!iCaptureSession_rec[iCaptureSession_rec.size()-1])
          printf("Failed to create CaptureSession\n");

      // Create the stream settings and set the common properties.
      streamSettings_rec.push_back(new UniqueObj<OutputStreamSettings>(iCaptureSession_rec[iCaptureSession_rec.size()-1]->createOutputStreamSettings()));
      iStreamSettings_rec.push_back(interface_cast<IOutputStreamSettings>(*streamSettings_rec[streamSettings_rec.size()-1]));
      if (!iStreamSettings_rec[iStreamSettings_rec.size()-1])
          printf("Failed to create OutputStreamSettings\n");
      iStreamSettings_rec[iStreamSettings_rec.size()-1]->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
      iStreamSettings_rec[iStreamSettings_rec.size()-1]->setResolution(StreamSize);

      // create output stream
      stream_rec.push_back(new UniqueObj<OutputStream>(
          iCaptureSession_rec[i]->createOutputStream(streamSettings_rec[i]->get())));

  }
  // create new H264Consumer
  h264Consumer = new H264Consumer(stream_rec, StreamSize, Opt_rec.frameDuration, Opt_rec.bitrate, Opt_rec.fileName);
  return 0;
}

int start_independent_cams(){
  uint32_t count = 0;
  if(numberOfSessions != 0){
      printf("**********start unsynchronized cameras********\n");
  } else{
      //no cameras to start
      return 0;
  }
  for (uint32_t i = 0; i<numberOfSets; i++){
      for(uint32_t j = 0; j<opt[i].cameraCount; j++){
          printf("start unsynced session %d of %d\n", i+1, numberOfSessions);
          //set consumer to run
          bufferConsumer[count]->initialize();
          bufferConsumer[count]->waitRunning();

          UniqueObj<Request> request;
          std::vector<const Argus::Request*> requestVec;

          request = UniqueObj<Request>(iCaptureSession[i]->createRequest());
          IRequest *iRequest = interface_cast<IRequest>(request);
          if (!iRequest)
            ORIGINATE_ERROR("Failed to create Request");
          requestVec.push_back(request.get());

          // Get sensor mode settings and set exposure and gain
          ISourceSettings *iSourceSettings =
            interface_cast<Argus::ISourceSettings>(iRequest->getSourceSettings());
          EXIT_IF_NULL(iSourceSettings, "Failed to get source settings interface");

          // set sensor mode
          if (!iCameraDevice_v[opt[i].cameraIndices[j]]) {
              printf("Failed to get camera device properties\n");
          }
          std::vector<Argus::SensorMode *> sensorModes;
          iCameraDevice_v[opt[i].cameraIndices[j]]->getBasicSensorModes(&sensorModes);
          if (!sensorModes.size()) {
              printf("Failed to get valid sensor mode list.\n");
          }

          iSourceSettings->setSensorMode(sensorModes[opt[i].sensorMode]);
          ISensorMode *iSensorMode = interface_cast<ISensorMode>(iSourceSettings->getSensorMode());

          // set analog gain
          if (opt[i].analogGain != 0){
            if(opt[i].analogGain >= iSensorMode->getAnalogGainRange().min() && opt[i].analogGain <= iSensorMode->getAnalogGainRange().max()){
              printf("setting gain to %f\n",opt[i].analogGain);
              EXIT_IF_NOT_OK(iSourceSettings->setGainRange(Range<float>(opt[i].analogGain)),
                "Unable to set the Source Settings Gain Range");
            } else {
              printf("Error: Gain %f is not in range [%f,%f]\n", opt[i].analogGain, iSensorMode->getAnalogGainRange().min(), iSensorMode->getAnalogGainRange().min());
            }
          }

          // set exposure time
          if (opt[i].exposureTime != 0){
            if(opt[i].exposureTime >= iSensorMode->getExposureTimeRange().min() && opt[i].exposureTime <= iSensorMode->	getExposureTimeRange().max()){
              printf("setting exposure time to %ld\n", opt[i].exposureTime);
              EXIT_IF_NOT_OK(iSourceSettings->setExposureTimeRange(Range<uint64_t>(opt[i].exposureTime)),
                "Unable to set the Source Settings Exposure Time Range");
            } else {
              printf("Error: exposure time %ld is not in range [%ld,%ld]\n", opt[i].exposureTime, iSensorMode->	getExposureTimeRange().min(), iSensorMode->getExposureTimeRange().max());
            }
          }

          // set frame duration
          if (opt[i].frameDuration != 0){
            if(opt[i].frameDuration >= iSensorMode->getFrameDurationRange().min() && opt[i].frameDuration <= iSensorMode->	getFrameDurationRange().max()){
              EXIT_IF_NOT_OK(iSourceSettings->setFrameDurationRange(Range<uint64_t>(opt[i].frameDuration)),
                "Unable to set the Source Settings Frame Duration Time Range");
            } else {
              printf("Error: frame duration %ld is not in range [%ld,%ld]\n", opt[i].frameDuration, iSensorMode->	getFrameDurationRange().min(), iSensorMode->getFrameDurationRange().max());
            }
          }

          EXIT_IF_NULL(iSensorMode, "Failed to get sensor mode interface");
          printf("\nselected sensor settings:\n");
          printf("resolution: %dx%d\n",iSensorMode->getResolution().width(),iSensorMode->getResolution().height());
          printf("ExposureTimeRange: [%ld, %ld]ns\n", iSourceSettings->getExposureTimeRange().min(),iSourceSettings->getExposureTimeRange().max());
          printf("GainRangeRange: [%f, %f]\n", iSourceSettings->getGainRange().min(),iSourceSettings->getGainRange().max());
          printf("FrameDurationRange: [%ld, %ld]ns\n\n", iSourceSettings->getFrameDurationRange().min(),iSourceSettings->getFrameDurationRange().max());

          //set optical black
          if(opt[i].opticalBlackEnable != 0){
              Argus::BayerTuple<float> opticalBlack = Argus::BayerTuple<float>(
                      opt[i].opticalBlack[0], opt[i].opticalBlack[1],
                      opt[i].opticalBlack[2], opt[i].opticalBlack[3]);
              iSourceSettings->setOpticalBlack(opticalBlack);
              iSourceSettings->setOpticalBlackEnable(true);
              opticalBlack = iSourceSettings->getOpticalBlack();
              printf("OpticalBlack:\nr: %f\ngEven: %f\ngOdd: %f\nb: %f\n",
                     opticalBlack.r(), opticalBlack.gEven(), opticalBlack.gOdd(),
                     opticalBlack.b());
              if (iSourceSettings->getOpticalBlackEnable()) {
                printf("OpticalBlack setting enabled\n\n");
              } else {
                printf("OpticalBlack setting disabled\n\n");
              }
          }

          // set ISP settings
          IAutoControlSettings *iAutoControlSettings =
            interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
          // disable all automatic things in the ISP
          EXIT_IF_NOT_OK(iAutoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_OFF),
            "Could not set anti banding mode to off");
          EXIT_IF_NOT_OK(iAutoControlSettings->setAeLock(true),
            "Could not set auto exposure lock");
          EXIT_IF_NOT_OK(iAutoControlSettings->setAwbLock(true),
            "Could not set auto white balance to lock");
          EXIT_IF_NOT_OK(iAutoControlSettings->setAwbMode(AWB_MODE_DAYLIGHT),
            "Could not set white balance mode to daylight");
          EXIT_IF_NOT_OK(iAutoControlSettings->setIspDigitalGainRange(Range<float>(1.0)),
            "Could not set ISP gain to 1");


          // Enable frame capture streams
          if (iRequest->enableOutputStream(stream[count]->get()) != STATUS_OK)
              ORIGINATE_ERROR("Failed to enable streamFrameCap in Request");

          if (iCaptureSession[i]->repeatBurst(requestVec) != STATUS_OK)
              ORIGINATE_ERROR("Failed to start repeat burst capture request");

          count ++;
      }
  }
  return 0;
}

int start_synchronized_cams(){
  //set consumer to run
  if(iCaptureSession_v.size()){
      printf("**********start synchronized cameras********\n");
      syncBufferConsumer->initialize();
      syncBufferConsumer->waitRunning();
  }else{
      //no cameras to start
      return 0;
  }

  std::vector<std::vector<const Argus::Request*>> req_v;
  std::vector<UniqueObj<Request>> request_v;
  std::vector<IRequest*> iRequest_v;
  for (uint32_t i = 0; i<iCaptureSession_v.size(); i++){
      std::vector<const Argus::Request*> requestVec;
      request_v.push_back(UniqueObj<Request>(iCaptureSession_v[i]->createRequest()));
      iRequest_v.push_back(interface_cast<IRequest>(request_v[i]));
      if (!iRequest_v[i])
        ORIGINATE_ERROR("Failed to create Request");

      // Get sensor mode settings and set exposure and gain
      ISourceSettings *iSourceSettings =
        interface_cast<Argus::ISourceSettings>(iRequest_v[i]->getSourceSettings());
      EXIT_IF_NULL(iSourceSettings, "Failed to get source settings interface");

      // set sensor mode
      if (!iCameraDevice_v[camera_v[i]]) {
          printf("Failed to get camera device properties\n");
      }
      std::vector<Argus::SensorMode *> sensorModes;
      iCameraDevice_v[camera_v[i]]->getBasicSensorModes(&sensorModes);
      if (!sensorModes.size()) {
          printf("Failed to get valid sensor mode list.\n");
      }

      iSourceSettings->setSensorMode(sensorModes[syncOpt.sensorMode]);
      ISensorMode *iSensorMode = interface_cast<ISensorMode>(iSourceSettings->getSensorMode());

      // set analog gain
      if (syncOpt.analogGain != 0){
        if(syncOpt.analogGain >= iSensorMode->getAnalogGainRange().min() && syncOpt.analogGain <= iSensorMode->getAnalogGainRange().max()){
          printf("setting gain to %f\n",syncOpt.analogGain);
          EXIT_IF_NOT_OK(iSourceSettings->setGainRange(Range<float>(syncOpt.analogGain)),
            "Unable to set the Source Settings Gain Range");
        } else {
          printf("Error: Gain %f is not in range [%f,%f]\n", syncOpt.analogGain, iSensorMode->getAnalogGainRange().min(), iSensorMode->getAnalogGainRange().min());
        }
      }

      // set exposure time
      if (syncOpt.exposureTime != 0){
        if(syncOpt.exposureTime >= iSensorMode->getExposureTimeRange().min() && syncOpt.exposureTime <= iSensorMode->getExposureTimeRange().max()){
          printf("setting exposure time to %ld\n", syncOpt.exposureTime);
          EXIT_IF_NOT_OK(iSourceSettings->setExposureTimeRange(Range<uint64_t>(syncOpt.exposureTime)),
            "Unable to set the Source Settings Exposure Time Range");
        } else {
          printf("Error: exposure time %ld is not in range [%ld,%ld]\n", syncOpt.exposureTime, iSensorMode->getExposureTimeRange().min(), iSensorMode->getExposureTimeRange().max());
        }
      }

      // set frame duration
      if (syncOpt.frameDuration != 0){
        if(syncOpt.frameDuration >= iSensorMode->getFrameDurationRange().min() && syncOpt.frameDuration <= iSensorMode->getFrameDurationRange().max()){
          EXIT_IF_NOT_OK(iSourceSettings->setFrameDurationRange(Range<uint64_t>(syncOpt.frameDuration)),
            "Unable to set the Source Settings Frame Duration Time Range");
        } else {
          printf("Error: frame duration %ld is not in range [%ld,%ld]\n", syncOpt.frameDuration, iSensorMode->getFrameDurationRange().min(), iSensorMode->getFrameDurationRange().max());
        }
      }

      EXIT_IF_NULL(iSensorMode, "Failed to get sensor mode interface");
      printf("\nselected sensor settings for synchronized streams:\n");
      printf("resolution: %dx%d\n",iSensorMode->getResolution().width(),iSensorMode->getResolution().height());
      printf("ExposureTimeRange: [%ld, %ld]ns\n", iSourceSettings->getExposureTimeRange().min(),iSourceSettings->getExposureTimeRange().max());
      printf("GainRangeRange: [%f, %f]\n", iSourceSettings->getGainRange().min(),iSourceSettings->getGainRange().max());
      printf("FrameDurationRange: [%ld, %ld]ns\n\n", iSourceSettings->getFrameDurationRange().min(),iSourceSettings->getFrameDurationRange().max());

      //set optical black
      if(syncOpt.opticalBlackEnable != 0){
          Argus::BayerTuple<float> opticalBlack = Argus::BayerTuple<float>(
                  syncOpt.opticalBlack[0], syncOpt.opticalBlack[1],
                  syncOpt.opticalBlack[2], syncOpt.opticalBlack[3]);
          iSourceSettings->setOpticalBlack(opticalBlack);
          iSourceSettings->setOpticalBlackEnable(true);
          opticalBlack = iSourceSettings->getOpticalBlack();
          printf("OpticalBlack:\nr: %f\ngEven: %f\ngOdd: %f\nb: %f\n",
                 opticalBlack.r(), opticalBlack.gEven(), opticalBlack.gOdd(),
                 opticalBlack.b());
          if (iSourceSettings->getOpticalBlackEnable()) {
            printf("OpticalBlack setting enabled\n\n");
          } else {
            printf("OpticalBlack setting disabled\n\n");
          }
      }

      // set ISP settings
      IAutoControlSettings *iAutoControlSettings =
        interface_cast<IAutoControlSettings>(iRequest_v[i]->getAutoControlSettings());
      // disable all automatic things in the ISP
      EXIT_IF_NOT_OK(iAutoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_OFF),
        "Could not set anti banding mode to off");
      EXIT_IF_NOT_OK(iAutoControlSettings->setAeLock(true),
        "Could not set auto exposure lock");
      EXIT_IF_NOT_OK(iAutoControlSettings->setAwbLock(true),
        "Could not set auto white balance to lock");
      EXIT_IF_NOT_OK(iAutoControlSettings->setAwbMode(AWB_MODE_DAYLIGHT),
        "Could not set white balance mode to daylight");
      EXIT_IF_NOT_OK(iAutoControlSettings->setIspDigitalGainRange(Range<float>(1.0)),
        "Could not set ISP gain to 1");

      requestVec.push_back(request_v[i].get());
      req_v.push_back(requestVec);

  }

  // Enable frame capture streams
  for(uint32_t i=0; i<iCaptureSession_v.size(); i++){
      printf("enable stream Nr. %d\n", i);
      if (iRequest_v[i]->enableOutputStream(stream_v[i]->get()) != STATUS_OK)
          ORIGINATE_ERROR("Failed to enable streamFrameCap in Request");
  }

  for(uint32_t i=0; i<iCaptureSession_v.size(); i++){
      printf("start repeat burst Nr. %d\n\n", i);
      if (iCaptureSession_v[i]->repeatBurst(req_v[i]) != STATUS_OK)
          ORIGINATE_ERROR("Failed to start repeat burst capture request");
  }
  return 0;
}

int start_recording_cams(){
  /******start H264Consumer******/
  //set consumer to run
  if(iCaptureSession_rec.size() != 0){
      printf("**********start recording cameras********\n");
      h264Consumer->initialize();
      h264Consumer->waitRunning();
  }else{
      //no cameras to start
      return 0;
  }

  std::vector<std::vector<const Argus::Request*>> req_rec;
  std::vector<UniqueObj<Request>> request_rec;
  std::vector<IRequest*> iRequest_rec;
  for (uint32_t i = 0; i<iCaptureSession_rec.size(); i++){
      std::vector<const Argus::Request*> requestVec;
      printf("create request\n");
      request_rec.push_back(UniqueObj<Request>(iCaptureSession_rec[i]->createRequest()));
      iRequest_rec.push_back(interface_cast<IRequest>(request_rec[i]));
      if (!iRequest_rec[i])
        ORIGINATE_ERROR("Failed to create Request");

      // Get sensor mode settings and set exposure and gain
      ISourceSettings *iSourceSettings =
        interface_cast<Argus::ISourceSettings>(iRequest_rec[i]->getSourceSettings());
      EXIT_IF_NULL(iSourceSettings, "Failed to get source settings interface");

      // set sensor mode
      if (!iCameraDevice_v[camera_rec[i]]) {
          printf("Failed to get camera device properties\n");
      }
      std::vector<Argus::SensorMode *> sensorModes;
      iCameraDevice_v[camera_rec[i]]->getBasicSensorModes(&sensorModes);
      if (!sensorModes.size()) {
          printf("Failed to get valid sensor mode list.\n");
      }

      iSourceSettings->setSensorMode(sensorModes[Opt_rec.sensorMode]);
      ISensorMode *iSensorMode = interface_cast<ISensorMode>(iSourceSettings->getSensorMode());

      // set analog gain
      if (Opt_rec.analogGain != 0){
        if(Opt_rec.analogGain >= iSensorMode->getAnalogGainRange().min() && Opt_rec.analogGain <= iSensorMode->getAnalogGainRange().max()){
          printf("setting gain to %f\n",Opt_rec.analogGain);
          EXIT_IF_NOT_OK(iSourceSettings->setGainRange(Range<float>(Opt_rec.analogGain)),
            "Unable to set the Source Settings Gain Range");
        } else {
          printf("Error: Gain %f is not in range [%f,%f]\n", Opt_rec.analogGain, iSensorMode->getAnalogGainRange().min(), iSensorMode->getAnalogGainRange().min());
        }
      }

      // set exposure time
      if (Opt_rec.exposureTime != 0){
        if(Opt_rec.exposureTime >= iSensorMode->getExposureTimeRange().min() && Opt_rec.exposureTime <= iSensorMode->	getExposureTimeRange().max()){
          printf("setting exposure time to %ld\n", Opt_rec.exposureTime);
          EXIT_IF_NOT_OK(iSourceSettings->setExposureTimeRange(Range<uint64_t>(Opt_rec.exposureTime)),
            "Unable to set the Source Settings Exposure Time Range");
        } else {
          printf("Error: exposure time %ld is not in range [%ld,%ld]\n", Opt_rec.exposureTime, iSensorMode->	getExposureTimeRange().min(), iSensorMode->getExposureTimeRange().max());
        }
      }

      // set frame duration
      if (Opt_rec.frameDuration != 0){
        if(Opt_rec.frameDuration >= iSensorMode->getFrameDurationRange().min() && Opt_rec.frameDuration <= iSensorMode->	getFrameDurationRange().max()){
          EXIT_IF_NOT_OK(iSourceSettings->setFrameDurationRange(Range<uint64_t>(Opt_rec.frameDuration)),
            "Unable to set the Source Settings Frame Duration Time Range");
        } else {
          printf("Error: frame duration %ld is not in range [%ld,%ld]\n", Opt_rec.frameDuration, iSensorMode->	getFrameDurationRange().min(), iSensorMode->getFrameDurationRange().max());
        }
      }else{
          printf("Error: frame duration for h264Consumer not set.\n");
          return 0;
      }


      EXIT_IF_NULL(iSensorMode, "Failed to get sensor mode interface");
      printf("\nselected sensor mode for h264 streams:\n");
      printf("resolution: %dx%d\n",iSensorMode->getResolution().width(),iSensorMode->getResolution().height());
      printf("ExposureTimeRange: [%ld, %ld]ns\n", iSourceSettings->getExposureTimeRange().min(),iSourceSettings->getExposureTimeRange().max());
      printf("GainRangeRange: [%f, %f]\n", iSourceSettings->getGainRange().min(),iSourceSettings->getGainRange().max());
      printf("FrameDurationRange: [%ld, %ld]ns\n\n", iSourceSettings->getFrameDurationRange().min(),iSourceSettings->getFrameDurationRange().max());

      //set optical black
      if(Opt_rec.opticalBlackEnable != 0){
          Argus::BayerTuple<float> opticalBlack = Argus::BayerTuple<float>(
                  Opt_rec.opticalBlack[0], Opt_rec.opticalBlack[1],
                  Opt_rec.opticalBlack[2], Opt_rec.opticalBlack[3]);
          iSourceSettings->setOpticalBlack(opticalBlack);
          iSourceSettings->setOpticalBlackEnable(true);
          opticalBlack = iSourceSettings->getOpticalBlack();
          printf("OpticalBlack:\nr: %f\ngEven: %f\ngOdd: %f\nb: %f\n",
                 opticalBlack.r(), opticalBlack.gEven(), opticalBlack.gOdd(),
                 opticalBlack.b());
          if (iSourceSettings->getOpticalBlackEnable()) {
            printf("OpticalBlack setting enabled\n\n");
          } else {
            printf("OpticalBlack setting disabled\n\n");
          }
      }

      // set ISP settings
      IAutoControlSettings *iAutoControlSettings =
        interface_cast<IAutoControlSettings>(iRequest_rec[i]->getAutoControlSettings());
      // disable all automatic things in the ISP
      EXIT_IF_NOT_OK(iAutoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_OFF),
        "Could not set anti banding mode to off");
      EXIT_IF_NOT_OK(iAutoControlSettings->setAeLock(true),
        "Could not set auto exposure lock");
      EXIT_IF_NOT_OK(iAutoControlSettings->setAwbLock(true),
        "Could not set auto white balance to lock");
      EXIT_IF_NOT_OK(iAutoControlSettings->setAwbMode(AWB_MODE_DAYLIGHT),
        "Could not set white balance mode to daylight");
      EXIT_IF_NOT_OK(iAutoControlSettings->setIspDigitalGainRange(Range<float>(1.0)),
        "Could not set ISP gain to 1");

      requestVec.push_back(request_rec[i].get());
      req_rec.push_back(requestVec);

  }

  // Enable frame capture streams
  for(uint32_t i=0; i<iCaptureSession_rec.size(); i++){
      printf("enable (record) stream Nr. %d\n", i);
      if (iRequest_rec[i]->enableOutputStream(stream_rec[i]->get()) != STATUS_OK)
          ORIGINATE_ERROR("Failed to enable streamFrameCap in Request");
  }

  for(uint32_t i=0; i<iCaptureSession_rec.size(); i++){
      printf("start (record) repeat burst Nr. %d\n\n", i);
      if (iCaptureSession_rec[i]->repeatBurst(req_rec[i]) != STATUS_OK)
          ORIGINATE_ERROR("Failed to start repeat burst capture request");
  }
  return 0;
}

// starts all streams
int ines_argus_start(){
    int ret = 0;
    ret = start_independent_cams();
    ret = start_synchronized_cams();
    ret = start_recording_cams();

    return ret;
}

// stop streams and consumers
int ines_argus_stop(){
  printf("\nstop cameras\n");
    // end unsynced consumers
    for(uint32_t i = 0; i<numberOfSessions; i++){

        // Stop the repeating request and wait for idle.
        printf("stop repeating request and wait for idle\n");
        iCaptureSession[i]->stopRepeat();
        iCaptureSession[i]->waitForIdle();
        stream[i]->reset();

        // Wait for the consumer threads to complete.
        printf("shutdown buffer consumer\n\n");
        PROPAGATE_ERROR(bufferConsumer[i]->shutdown());
    }

    //end synced consumer
    for(uint32_t i = 0; i<iCaptureSession_v.size(); i++){
        printf("end synced consumer Nr. %d\n", i);
        syncBufferConsumer->stopConsume();
        printf("stop, wait and reset of capture session Nr.: %d\n", i);
        // Stop the repeating request and wait for idle.
        iCaptureSession_v[i]->stopRepeat();
        iCaptureSession_v[i]->waitForIdle();
        stream_v[i]->reset();
        printf("--------------> done\n");
    }
    // Wait for the synced consumer thread to complete.
    if(captureSession_v.size() != 0){
        printf("shutdown synced buffer Consumer\n\n");
        PROPAGATE_ERROR(syncBufferConsumer->shutdown());
    }

    //end h264 consumer
    for(uint32_t i = 0; i<iCaptureSession_rec.size(); i++){
        printf("end h264 consumer Nr. %d\n", i);
        h264Consumer->stopConsume();
        printf("stop, wait and reset of (record) capture session Nr.: %d\n", i);
        // Stop the repeating request and wait for idle.
        iCaptureSession_rec[i]->stopRepeat();
        iCaptureSession_rec[i]->waitForIdle();
        stream_rec[i]->reset();
        printf("--------------> done\n");
    }

    // Wait for the h264Consumer to complete.
    if(captureSession_rec.size() != 0){
        printf("shutdown h264 consumer\n\n");
        PROPAGATE_ERROR(h264Consumer->shutdown());
    }

    // Shut down Argus.
    cameraProvider->reset();

    return 1;
}

// clean up
int ines_argus_destroy(){
    printf("clean up\n");

    iCameraDevice_v.clear();
    //delete independent cameras
    delete cameraProvider;
    for(uint32_t i = 0; i<numberOfSessions; i++){
        delete captureSession[i];
        delete streamSettings[i];
        delete stream[i];
        delete bufferConsumer[i];
    }

    //delete synced cameras
    for(uint32_t i = 0; i<captureSession_v.size(); i++){
        delete captureSession_v[i];
        delete streamSettings_v[i];
        delete stream_v[i];
    }
    captureSession_v.clear();
    iCaptureSession_v.clear();
    streamSettings_v.clear();
    iStreamSettings_v.clear();
    stream_v.clear();
    camera_v.clear();
    delete syncBufferConsumer;

    //delete record cameras
    for(uint32_t i = 0; i<captureSession_rec.size(); i++){
        delete captureSession_rec[i];
        delete streamSettings_rec[i];
        delete stream_rec[i];
    }
    captureSession_rec.clear();
    iCaptureSession_rec.clear();
    streamSettings_rec.clear();
    iStreamSettings_rec.clear();
    stream_rec.clear();
    camera_rec.clear();
    delete h264Consumer;

    numberOfSessions =  0;
    numberOfSets = 0;
    return 1;
}
