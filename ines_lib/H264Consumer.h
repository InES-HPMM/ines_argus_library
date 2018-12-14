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

#ifndef H264CONSUMER_H
#define H264CONSUMER_H

#include <Argus/Argus.h>
#include "Thread.h"
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include "Error.h"
#include <NvApplicationProfiler.h>
#include "H264Encoder.h"

using namespace Argus;
using namespace EGLStream;
using namespace ArgusSamples;

/*******************************************************************************
 * FrameConsumer thread:
 *   Creates an EGLStream::FrameConsumer object to read frames from the stream
 *   and create NvBuffers (dmabufs) from acquired frames before providing the
 *   buffers to V4L2 for video encoding. The encoder will save the encoded
 *   stream to disk.
 ******************************************************************************/
class H264Consumer : public Thread {
public:
  explicit H264Consumer(std::vector<UniqueObj<OutputStream>*> stream, Size2D<uint32_t> streamSize, uint64_t frameDuration, uint32_t bitrate, char* fileName);
  ~H264Consumer();

  bool isInError() { return m_gotError; }
  void setVerbose(bool verbose);
  int stopConsume();

private:
  /** @name Thread methods */
  /**@{*/
  virtual bool threadInitialize();
  virtual bool threadExecute();
  virtual bool threadShutdown();
  /**@}*/

  bool createVideoEncoder(int index);

  // member variables
  bool m_verbose;
  std::vector<UniqueObj<OutputStream>*> m_stream;
  std::vector<IStream*> m_iStream;
  std::vector<UniqueObj<FrameConsumer>> m_consumer;
  std::vector<IFrameConsumer*> m_iFrameConsumer;
  std::vector<H264Encoder*> m_encoder;
  std::vector<struct EncoderOptions> m_options;
  Size2D<uint32_t> m_streamSize;
  bool m_stop;
  bool m_gotError;
  uint64_t m_frameDuration;
  uint32_t m_bitrate;
  char* m_fileName;
};
#endif // H264CONSUMER_H
