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

#include "H264Encoder.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include "Error.h"
#include <NvApplicationProfiler.h>
#include <NvVideoEncoder.h>
#include "EGLGlobal.h"
#include "GLContext.h"


#define ENCODER_PRINT(...) printf("H264-Encoder: " __VA_ARGS__)

#define CHECK_ERROR(expr)                                                      \
  do {                                                                         \
    if ((expr) < 0) {                                                          \
      abort();                                                                 \
      ORIGINATE_ERROR(#expr " failed");                                        \
    }                                                                          \
  } while (0);
#define EXIT_IF_NULL(val, msg)                                                 \
  {                                                                            \
    if (!val) {                                                                \
      printf("%s\n", msg);                                                     \
      return EXIT_FAILURE;                                                     \
    }                                                                          \
  }
#define EXIT_IF_NOT_OK(val, msg)                                               \
  {                                                                            \
    if (val != Argus::STATUS_OK) {                                             \
      printf("%s\n", msg);                                                     \
      return EXIT_FAILURE;                                                     \
    }                                                                          \
  }


H264Encoder::H264Encoder(struct EncoderOptions options)
: m_gotError(false), m_options(options) {
  ENCODER_PRINT("start h264 encoder\n");
}
H264Encoder::~H264Encoder(){}

uint32_t H264Encoder::getNumQueuedBuffers(){
  return m_VideoEncoder->output_plane.getNumQueuedBuffers();
}

uint32_t H264Encoder::getNumBuffers(){
  return m_VideoEncoder->output_plane.getNumBuffers();
}

int H264Encoder::dqBuffer(struct v4l2_buffer& v4l2_buf, NvBuffer **buffer, NvBuffer **shared_buffer, uint32_t num_retries){
  return m_VideoEncoder->output_plane.dqBuffer(v4l2_buf, buffer, shared_buffer, num_retries);
}


int H264Encoder::qBuffer(struct v4l2_buffer& v4l2_buf, NvBuffer *shared_buffer){
  return m_VideoEncoder->output_plane.qBuffer(v4l2_buf, shared_buffer);
}

int H264Encoder::waitForDQThread(uint32_t max_wait_ms){
  return m_VideoEncoder->output_plane.waitForDQThread(max_wait_ms);
}

bool H264Encoder::initVideoEncoder() {
  ENCODER_PRINT("initialize the h264 encoder\n");

  int ret = 0;

  m_VideoEncoder = NvVideoEncoder::createVideoEncoder(m_options.fileName.c_str());
  if (!m_VideoEncoder) {
    ORIGINATE_ERROR("Could not create m_VideoEncoderoder");
  }

  // if (DO_STAT){
  //     m_VideoEncoder->enableProfiling();}

  ret = m_VideoEncoder->setCapturePlaneFormat(
      m_options.captureFormat, m_options.streamSize.width(), m_options.streamSize.height(),
      2 * m_options.streamSize.width() * m_options.streamSize.height());
  if (ret < 0) {
    ORIGINATE_ERROR("Could not set capture plane format");
  }

  ret = m_VideoEncoder->setOutputPlaneFormat(
      m_options.outputFormat, m_options.streamSize.width(), m_options.streamSize.height());
  if (ret < 0) {
    ORIGINATE_ERROR("Could not set output plane format");
  }

  if(m_options.bitrate == 0)
      m_options.bitrate = 4 * m_options.streamSize.width() * m_options.streamSize.height();

  ret = m_VideoEncoder->setBitrate(m_options.bitrate);
  if (ret < 0) {
    ORIGINATE_ERROR("Could not set bitrate");
  }

  if (m_options.captureFormat == V4L2_PIX_FMT_H264) {
    ret = m_VideoEncoder->setProfile(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH);
  } else {
    ret = m_VideoEncoder->setProfile(V4L2_MPEG_VIDEO_H265_PROFILE_MAIN);
  }
  if (ret < 0) {
    ORIGINATE_ERROR("Could not set m_VideoEncoderoder profile");
  }

  if (m_options.captureFormat == V4L2_PIX_FMT_H264) {
    ret = m_VideoEncoder->setLevel(V4L2_MPEG_VIDEO_H264_LEVEL_5_0);
    if (ret < 0) {
      ORIGINATE_ERROR("Could not set m_VideoEncoderoder level");
    }
  }

  ret = m_VideoEncoder->setRateControlMode(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
  if (ret < 0) {
    ORIGINATE_ERROR("Could not set rate control mode");
  }

  ret = m_VideoEncoder->setIFrameInterval((uint32_t) (1000000000/m_options.frameDuration));
  //ret = m_VideoEncoder->setIFrameInterval(30);
  if (ret < 0) {
    ORIGINATE_ERROR("Could not set I-frame interval");
  }

  //ret = m_VideoEncoder->setFrameRate(1000000000, m_options.frameDuration);
  ret = m_VideoEncoder->setFrameRate((uint32_t) (1000000000/m_options.frameDuration), 1);
  if (ret < 0) {
    ORIGINATE_ERROR("Could not set m_VideoEncoderoder framerate");
  }

  ret = m_VideoEncoder->setHWPresetType(V4L2_ENC_HW_PRESET_ULTRAFAST);
  if (ret < 0) {
    ORIGINATE_ERROR("Could not set m_VideoEncoderoder HW Preset");
  }

  // Query, Export and Map the output plane buffers so that we can read
  // raw data into the buffers
  ret = m_VideoEncoder->output_plane.setupPlane(V4L2_MEMORY_DMABUF, 10, true,
                                                false);
  if (ret < 0) {
    ORIGINATE_ERROR("Could not setup output plane");
  }

  // Query, Export and Map the output plane buffers so that we can write
  // m_VideoEncoderoded data from the buffers
  ret = m_VideoEncoder->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true,
                                                 false);

  // Create output file
  m_outputFile = new std::ofstream(m_options.fileName.c_str());
  if (!m_outputFile) {
      ORIGINATE_ERROR("Failed to open output file.");
  }

  // Stream on
  int e = m_VideoEncoder->output_plane.setStreamStatus(true);
  if (e < 0) {
   ORIGINATE_ERROR("Failed to stream on output plane");
  }
  e = m_VideoEncoder->capture_plane.setStreamStatus(true);
  if (e < 0) {
   ORIGINATE_ERROR("Failed to stream on capture plane");
  }

  // Set video encoder callback
  m_VideoEncoder->capture_plane.setDQThreadCallback(
     encoderCapturePlaneDqCallback);

  // startDQThread starts a thread internally which calls the
  // encoderCapturePlaneDqCallback whenever a buffer is dequeued
  // on the plane
  m_VideoEncoder->capture_plane.startDQThread(this);

  // Enqueue all the empty capture plane buffers
  for (uint32_t j = 0; j < m_VideoEncoder->capture_plane.getNumBuffers(); j++) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

    v4l2_buf.index = j;
    v4l2_buf.m.planes = planes;

    CHECK_ERROR(m_VideoEncoder->capture_plane.qBuffer(v4l2_buf, NULL));
  }

  if (ret < 0) {
    ORIGINATE_ERROR("Could not setup capture plane");
  }

  ENCODER_PRINT("create video encoder return true\n");
  return true;
}

void H264Encoder::abort() {
  m_VideoEncoder->abort();
  m_gotError = true;
}

bool H264Encoder::encoderCapturePlaneDqCallback(
    struct v4l2_buffer *v4l2_buf, NvBuffer *buffer, NvBuffer *shared_buffer,
    void *arg) {
  H264Encoder *thiz = (H264Encoder *)arg;

  if (!v4l2_buf) {
    thiz->abort();
    ORIGINATE_ERROR("Failed to dequeue buffer from encoder capture plane");
  }

  thiz->m_outputFile->write((char *)buffer->planes[0].data,
                            buffer->planes[0].bytesused);

  if (thiz->m_VideoEncoder->capture_plane.qBuffer(*v4l2_buf, NULL) < 0) {
    thiz->abort();
    ORIGINATE_ERROR("Failed to enqueue buffer to encoder capture plane");
    return false;
  }

  // GOT EOS from m_VideoEncoderoder. Stop dqthread.
  if (buffer->planes[0].bytesused == 0) {
    ENCODER_PRINT("Got EOS, exiting...\n");
    return false;
  }

  return true;
}
