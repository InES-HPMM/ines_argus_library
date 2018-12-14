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

#ifndef H264ENCODER_H
#define H264ENCODER_H

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include "Error.h"
#include <NvApplicationProfiler.h>
#include <NvVideoEncoder.h>

using namespace Argus;
using namespace EGLStream;

struct EncoderOptions {
    std::string fileName;
    uint32_t bitrate = 0;
    uint32_t frameDuration;
    uint32_t captureFormat;
    uint32_t outputFormat;
    Size2D<uint32_t> streamSize;
};

class H264Encoder {
public:
  explicit H264Encoder(struct EncoderOptions options);
  ~H264Encoder();

  void setVerbose(bool verbose);
  bool initVideoEncoder();
  uint32_t getNumBuffers();
  uint32_t getNumQueuedBuffers();
  int dqBuffer(struct v4l2_buffer& v4l2_buf, NvBuffer **buffer, NvBuffer **shared_buffer, uint32_t num_retries);
  int qBuffer(struct v4l2_buffer& v4l2_buf, NvBuffer *shared_buffer);
  int waitForDQThread(uint32_t max_wait_ms);
private:

  void abort();
  static bool encoderCapturePlaneDqCallback(struct v4l2_buffer *v4l2_buf,
                                            NvBuffer *buffer,
                                            NvBuffer *shared_buffer, void *arg);

  //member variables
  bool m_verbose;
  NvVideoEncoder* m_VideoEncoder;
  std::ofstream *m_outputFile;
  bool m_gotError;
  struct EncoderOptions m_options;
};
#endif // H264ENCODER_H
