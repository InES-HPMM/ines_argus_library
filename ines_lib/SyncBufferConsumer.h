#ifndef SYNCBUFFERCONSUMER_H
#define SYNCBUFFERCONSUMER_H

#include <Argus/Argus.h>
#include "Thread.h"
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include "Error.h"


using namespace Argus;
using namespace EGLStream;
using namespace ArgusSamples;



class SyncBufferConsumerThread : public Thread
{
public:
    explicit SyncBufferConsumerThread(std::vector<UniqueObj<OutputStream>*> stream, Size2D<uint32_t> streamSize);
    ~SyncBufferConsumerThread();

    void setCallback(void (*callback)(void **, uint32_t, uint32_t, uint32_t, uint32_t));
    void *getCallback();
    int stopConsume();
private:

    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();


    Size2D<uint32_t> m_streamSize;
    std::vector<UniqueObj<OutputStream>*> m_stream;
    std::vector<UniqueObj<FrameConsumer>> m_consumer;
    std::vector<IStream*> m_iStream;
    std::vector<IFrameConsumer*> m_iFrameConsumer;
    bool m_gotError;
    void (*callback)(void **, uint32_t, uint32_t, uint32_t, uint32_t);
};

#endif // SYNCBUFFERCONSUMER_H
