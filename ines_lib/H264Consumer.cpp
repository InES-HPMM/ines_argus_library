#include "H264Consumer.h"
#include "H264Encoder.h"
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

#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("H264-CONSUMER: " __VA_ARGS__)
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


using namespace EGLStream;
static const uint32_t ENCODER_PIXFMT = V4L2_PIX_FMT_H264;
static const int MAX_ENCODER_FRAMES = 5;


H264Consumer::H264Consumer(std::vector<UniqueObj<OutputStream>*> stream, Size2D<uint32_t> streamSize, uint64_t frameDuration, uint32_t bitrate, char* fileName)
    : m_verbose(false), m_stream(stream), m_streamSize(streamSize), m_stop(false), m_gotError(false), m_frameDuration(frameDuration), m_bitrate(bitrate), m_fileName(fileName)
{
    m_stream = stream;
}

H264Consumer::~H264Consumer() {
    for(uint32_t i = 0; i<m_encoder.size(); i++){
        if (m_encoder[i]) {
          delete m_encoder[i];
        }
    }
    m_encoder.clear();
    m_options.clear();
}

int H264Consumer::stopConsume(){
    CONSUMER_PRINT("stop h264 consumer\n");
    m_stop = true;
    return 1;
}

void H264Consumer::setVerbose(bool verbose) { m_verbose = verbose; }

bool H264Consumer::threadInitialize() {
  CONSUMER_PRINT("thread initialize\n");

  for(uint32_t i = 0; i<m_stream.size(); i++){
      // Create the FrameConsumer.
      m_consumer.push_back(UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream[i]->get())));
      if (!m_consumer[i]) {
        ORIGINATE_ERROR("Failed to create FrameConsumer");
      }

      // set output file name
      std::string name("camera" + std::to_string((int)i) + ".h264");
      if(m_fileName != 0){
          name = (m_fileName + std::to_string((int)i) + ".h264");
      }

      // set encoder options
      struct EncoderOptions opt;
      opt.fileName = name;
      opt.streamSize = m_streamSize;
      opt.frameDuration = (uint32_t) m_frameDuration;
      opt.captureFormat = V4L2_PIX_FMT_H264;
      opt.outputFormat = V4L2_PIX_FMT_YUV420M;
      opt.bitrate = m_bitrate;

      m_options.push_back(opt);


      // Create Video Encoder
      m_encoder.push_back(new H264Encoder(m_options[i]));
      if (!(m_encoder[i]->initVideoEncoder())) {
        ORIGINATE_ERROR("Failed to create video m_VideoEncoderoder");
      }
  }

  return true;
}

bool H264Consumer::threadExecute() {
    int bufferIndex[m_stream.size()];
    NvBuffer* buffer[m_stream.size()];
    uint32_t frameCounter[m_stream.size()];
    struct timeval time_before[m_stream.size()], time_after[m_stream.size()], time_diff[m_stream.size()];
    double time_diff_sec;
    struct v4l2_buffer v4l2_buf[m_stream.size()];
    struct v4l2_plane planes[m_stream.size()][MAX_PLANES];

    for(uint32_t i = 0; i<m_stream.size(); i++){
          m_iStream.push_back(interface_cast<IStream>(m_stream[i]->get()));
          m_iFrameConsumer.push_back(interface_cast<IFrameConsumer>(m_consumer[i]));

          // Wait until the producer has connected to the stream.
          CONSUMER_PRINT("Waiting until producer is connected...\n");
          if (m_iStream[i]->waitUntilConnected() != STATUS_OK)
              ORIGINATE_ERROR("Stream failed to connect.");
          CONSUMER_PRINT("Producer has connected; continuing.\n");

          bufferIndex[i] = 0;
          frameCounter[i] = 0;

          memset(&v4l2_buf[i], 0, sizeof(v4l2_buf[i]));
          memset(planes[i], 0, MAX_PLANES * sizeof(struct v4l2_plane));
          v4l2_buf[i].m.planes = planes[i];

          gettimeofday(&time_before[i], NULL);
    }

    int fd = -1;

    // Keep acquire frames and queue into encoder
    while (!(m_gotError || m_stop)) {
        for(uint32_t i = 0; i<m_stream.size(); i++){
            if(m_gotError || m_stop){
                break;
            }
            // CONSUMER_PRINT("Check if we need dqBuffer first\n");
            // Check if we need dqBuffer first
            if (bufferIndex[i] < MAX_ENCODER_FRAMES &&
                m_encoder[i]->getNumQueuedBuffers() < //returns uint32_t
                    m_encoder[i]->getNumBuffers()) {  //returns uint32_t
              // The queue is not full, no need to dqBuffer
              // Prepare buffer index for the following qBuffer
              v4l2_buf[i].index = bufferIndex[i]++;
            } else {
              // Output plane full or max outstanding number reached
              // CONSUMER_PRINT("output plane full\n");
              CHECK_ERROR(
                  m_encoder[i]->dqBuffer(v4l2_buf[i], &buffer[i], NULL, 10));
              // Release the frame.
              fd = v4l2_buf[i].m.planes[0].m.fd;
              NvBufferDestroy(fd);
            }

            // Acquire a frame.
            UniqueObj<Frame> frame(m_iFrameConsumer[i]->acquireFrame());
            IFrame *iFrame = interface_cast<IFrame>(frame);
            if (!iFrame) {
              // Send EOS
              v4l2_buf[i].m.planes[0].m.fd = fd;
              v4l2_buf[i].m.planes[0].bytesused = 0;
              CHECK_ERROR(m_encoder[i]->qBuffer(v4l2_buf[i], NULL));
              break;
            }

            // Get the IImageNativeBuffer extension interface and create the fd.
            NV::IImageNativeBuffer *iNativeBuffer =
                interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
            if (!iNativeBuffer) {
              ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");
            }
            fd = iNativeBuffer->createNvBuffer(m_streamSize, NvBufferColorFormat_YUV420,
                                               NvBufferLayout_Pitch);

            if (m_verbose) {
              CONSUMER_PRINT("Acquired Frame nr.: %d\n", frameCounter[i]);
            }
            if (frameCounter[i] % 100 == 0) {
              gettimeofday(&time_after[i], NULL);
              if (frameCounter[i] > 0) {
                timersub(&time_after[i], &time_before[i], &time_diff[i]);
                time_diff_sec =
                    (double)time_diff[i].tv_sec + ((double)time_diff[i].tv_usec) / 1e6;
                printf("Capturing video %d with %.2lf fps.\n", i, 100.0 / time_diff_sec);
              }
              time_before[i] = time_after[i];
            }
            frameCounter[i]++;

            // Push the frame into V4L2.
            v4l2_buf[i].m.planes[0].m.fd = fd;
            v4l2_buf[i].m.planes[0].bytesused = 1; // byteused must be non-zero
            CHECK_ERROR(m_encoder[i]->qBuffer(v4l2_buf[i], NULL));


            // Wait till capture plane DQ Thread finishes
            // i.e. all the capture plane buffers are dequeued
            m_encoder[i]->waitForDQThread(2000);
        }
    }
    CONSUMER_PRINT("Done.\n");

    requestShutdown();

    return true;
}

bool H264Consumer::threadShutdown() { return true; }
