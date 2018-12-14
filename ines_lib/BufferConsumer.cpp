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

#include "BufferConsumer.h"

using namespace EGLStream;

static bool DO_CPU_PROCESS = true;
static bool VERBOSE_ENABLE = false;

#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)



BufferConsumerThread::BufferConsumerThread(OutputStream* stream, Size2D<uint32_t> streamSize) :
        // m_stream(stream),
        // m_streamSize(streamSize),
        m_gotError(false)
{
    m_stream = stream;
    m_streamSize = streamSize;
}

BufferConsumerThread::~BufferConsumerThread(){}

bool BufferConsumerThread::threadInitialize()
{
    CONSUMER_PRINT("init buffer consumer\n");
    // Create the FrameConsumer.
    m_consumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream));
    if (!m_consumer)
        ORIGINATE_ERROR("Failed to create FrameConsumer");

    return true;
}

void BufferConsumerThread::setCallback(void (*call)(void *, uint32_t, uint32_t, uint32_t)){
    callback = call;
}

void *BufferConsumerThread::getCallback(void){
    return (void *) callback;
}

bool BufferConsumerThread::threadExecute()
{
    IStream *iStream = interface_cast<IStream>(m_stream);
    IFrameConsumer *iFrameConsumer = interface_cast<IFrameConsumer>(m_consumer);

    // Wait until the producer has connected to the stream.
    CONSUMER_PRINT("Waiting until producer is connected...\n");
    if (iStream->waitUntilConnected() != STATUS_OK)
        ORIGINATE_ERROR("Stream failed to connect.");
    CONSUMER_PRINT("Producer has connected; continuing.\n");

    // aquire frames and pass them to de callback
    while (!m_gotError)
    {
        int fd = -1;
        void *buf;
        int lineWidth;

        // Acquire a frame.
        if (VERBOSE_ENABLE)
            CONSUMER_PRINT("buffer consumer aquire frame\n");
        UniqueObj<Frame> frame(iFrameConsumer->acquireFrame());
        IFrame *iFrame = interface_cast<IFrame>(frame);
        if (VERBOSE_ENABLE)
              printf("time: %lu\n", iFrame->getTime());

        // check if frame is valid
        if (!iFrame)
        {
            // stop aquiring frames
            printf("could not grab frame\n");
            m_gotError = true;
            break;
        }

        // Get the IImageNativeBuffer extension interface and create the fd.
        NV::IImageNativeBuffer *iNativeBuffer =
            interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
        if (!iNativeBuffer)
            ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");
        // fd = iNativeBuffer->createNvBuffer(m_streamSize,
        //                                    NvBufferColorFormat_YUV420,
        //                                    (DO_CPU_PROCESS)?NvBufferLayout_Pitch:NvBufferLayout_BlockLinear);
        fd = iNativeBuffer->createNvBuffer(m_streamSize,
                                           NvBufferColorFormat_UYVY,
                                           (DO_CPU_PROCESS)?NvBufferLayout_Pitch:NvBufferLayout_BlockLinear);

        // get the parameters of the buffer
        NvBufferParams par;
        NvBufferGetParams (fd, &par);

        // if (VERBOSE_ENABLE){
        //     CONSUMER_PRINT("Y p %d w %d h %d offset %d\n", par.pitch[0], par.width[0], par.height[0], par.offset[0]);
        //     CONSUMER_PRINT("U p %d w %d h %d offset %d\n", par.pitch[1], par.width[1], par.height[1], par.offset[1]);
        //     CONSUMER_PRINT("V p %d w %d h %d offset %d\n", par.pitch[2], par.width[2], par.height[2], par.offset[2]);
        // }

        // provide all planes to the callback function
        for (uint32_t i = 0; i<par.num_planes; i++){
            // CONSUMER_PRINT("i = %d p %d w %d h %d offset %d layout %d memsize %d pixelformat %d planesize %d nv_buffer_size %d\n ", i, par.pitch[i], par.width[i],
            //                                                 par.height[i], par.offset[i], par.layout[i], par.memsize, par.pixel_format, par.psize[i], par.nv_buffer_size);

            NvBufferMemMap(fd, i, NvBufferMem_Read, &buf);
            NvBufferMemSyncForCpu(fd, i, &buf);

            if (callback != NULL){
                // get used bytes of one pitch
                switch(par.pixel_format){
                    case NvBufferColorFormat_YUV420:
                        lineWidth = par.width[i];
                        break;
                    case NvBufferColorFormat_UYVY:
                        lineWidth = par.width[i] * 2;
                        break;
                    default:
                        lineWidth = par.width[i];
                }
                callback(buf, lineWidth, par.height[i], par.pitch[i]);
            }else{
                CONSUMER_PRINT("callback not set\n");
                break;
            }

            NvBufferMemSyncForDevice (fd, i, &buf);
            NvBufferMemUnMap(fd, i, &buf);
        }

        if (VERBOSE_ENABLE)
            CONSUMER_PRINT("Acquired Frame. %d\n", fd);

    }

    CONSUMER_PRINT("Done.\n");

    requestShutdown();

    return true;
}

bool BufferConsumerThread::threadShutdown()
{
    if(VERBOSE_ENABLE)
        printf("shutdown thread\n");
    m_gotError = true;
    return true;

}
