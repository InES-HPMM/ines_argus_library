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

#ifndef INES_ARGUS_H
#define INES_ARGUS_H
#include <stdint.h>
#define MAX_DEVICES 6


struct CameraOptions {
    uint32_t*   cameraIndices = 0;
    uint32_t    cameraCount = 1;
    uint32_t    sensorMode = 0;
    uint64_t    exposureTime = 0;
    float       analogGain = 0;
    uint64_t    frameDuration = 0; //ns
    uint32_t    verbose = 0;
    float       opticalBlack[4] = {0,0,0,0};
    uint32_t    opticalBlackEnable = 0;
    uint32_t    bitrate = 0;
    char*       fileName = 0;
};

struct SensorCapabilties{
    uint32_t    height;
    uint32_t    width;
    uint64_t    minExposureTime;
    uint64_t    maxExposureTime;
    uint64_t    minFrameDuration;
    uint64_t    maxFrameDuration;
    float       minAnalogGain;
    float       maxAnalogGain;
};



// initalizes the Argus camera provider
int ines_argus_init();

// returns the number of available camera devices
int ines_argus_get_num_device();

// returns the capabilities of the chosen video device
struct SensorCapabilties ines_argus_get_sensor_caps(int device, int mode);

// prints the capabilities of the chosen video device
int ines_argus_print_sensor_caps(int device);

// creates a new camera stream and returns the frame buffer in the callback function
// example for callback:
//            void processFrame(void* buffer, int lineWidth, int height);
int ines_argus_add_cam(struct CameraOptions options,
                  void (*callback)(void *, uint32_t, uint32_t, uint32_t));

// create synchronized camera streames that are processed in one consumer
// example for callback:
//            void processFrame(void* buffer, int lineWidth, int height);
int ines_argus_add_sync_cam(struct CameraOptions options, void (*callback)(void **, uint32_t, uint32_t, uint32_t, uint32_t));

// create synchronized camera streames that are stored in a h264 encodd file
int ines_argus_add_sync_recorder(struct CameraOptions options);

// starts all streams
int ines_argus_start();

// stop streaming
int ines_argus_stop();

// clean up
int ines_argus_destroy();

#endif
