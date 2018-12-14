#include "SyncBufferConsumer.h"

using namespace EGLStream;

static bool DO_CPU_PROCESS = true;
static bool VERBOSE_ENABLE = false;
static bool stop = false;

#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)



SyncBufferConsumerThread::SyncBufferConsumerThread(std::vector<UniqueObj<OutputStream>*> stream, Size2D<uint32_t> streamSize) :
        // m_stream(stream),
        // m_streamSize(streamSize),
        m_gotError(false)
{
    m_stream = stream;
    m_streamSize = streamSize;
}

SyncBufferConsumerThread::~SyncBufferConsumerThread(){}

bool SyncBufferConsumerThread::threadInitialize()
{
    CONSUMER_PRINT("init synchronized buffer consumer\n");
    // Create the FrameConsumers
    for(uint32_t i = 0; i<m_stream.size(); i++){
        m_consumer.push_back(UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream[i]->get())));
      //  m_consumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream));
        if (!m_consumer[i])
            ORIGINATE_ERROR("Failed to create FrameConsumer");
    }
    stop = false;
    return true;
}

void SyncBufferConsumerThread::setCallback(void (*call)(void **, uint32_t, uint32_t, uint32_t, uint32_t)){
    callback = call;
}

void *SyncBufferConsumerThread::getCallback(void){
    return (void *) callback;
}

int SyncBufferConsumerThread::stopConsume(){
    stop = true;
    return 1;
}

bool SyncBufferConsumerThread::threadExecute()
{
    for(uint32_t i = 0; i<m_stream.size(); i++){
        m_iStream.push_back(interface_cast<IStream>(m_stream[i]->get()));
        m_iFrameConsumer.push_back(interface_cast<IFrameConsumer>(m_consumer[i]));

        // Wait until the producer has connected to the stream.
        CONSUMER_PRINT("Waiting until producer is connected...\n");
        if (m_iStream[i]->waitUntilConnected() != STATUS_OK)
            ORIGINATE_ERROR("Stream failed to connect.");
        CONSUMER_PRINT("Producer has connected; continuing.\n");
    }
    UniqueObj<Frame> *frames[m_stream.size()];
    IFrame *iFrames[m_stream.size()];
    int fds[m_stream.size()];
    NV::IImageNativeBuffer *iNativeBuffers[m_stream.size()];
    NvBufferParams pars[m_stream.size()];
    void *bufs[m_stream.size()];
    uint32_t numBuf = m_stream.size();

    // aquire frames and pass them to the callback
    while (!(m_gotError || stop))
    {
        int fd = -1;
        int lineWidth;
        if(m_gotError || stop){
            printf("break loop\n");
            break;
        }

        // Acquire a frame of every stream
        if (VERBOSE_ENABLE)
            CONSUMER_PRINT("buffer consumer aquire frame\n");
        for(uint32_t j = 0; j<m_stream.size(); j++){
            frames[j] = new UniqueObj<Frame>(m_iFrameConsumer[j]->acquireFrame());
            iFrames[j] = interface_cast<IFrame>(*frames[j]);

            // check if frame is valid
            if (!iFrames[j])
            {
                // stop aquiring frames
                printf("could not grab frame\n");
                m_gotError = true;
                break;
            }

            iNativeBuffers[j] = interface_cast<NV::IImageNativeBuffer>(iFrames[j]->getImage());
            if (!iNativeBuffers[j])
                            ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");

            fds[j] = iNativeBuffers[j]->createNvBuffer(m_streamSize,
                                            NvBufferColorFormat_UYVY, //NvBufferColorFormat_YUV420,
                                            (DO_CPU_PROCESS)?NvBufferLayout_Pitch:NvBufferLayout_BlockLinear);
            NvBufferGetParams(fds[j], &pars[j]);
        }

        // provide all planes to the callback function
        for (uint32_t i = 0; i<pars[0].num_planes; i++){
            for (uint32_t j = 0; j < m_stream.size(); j++){
                NvBufferMemMap(fds[j], i, NvBufferMem_Read, &bufs[j]);
                NvBufferMemSyncForCpu(fds[j], i, &bufs[j]);
            }

            if (callback != NULL){
                // get used bytes of one pitch
                switch(pars[0].pixel_format){
                    case NvBufferColorFormat_YUV420:
                        lineWidth = pars[0].width[i];
                        break;
                    case NvBufferColorFormat_UYVY:
                        lineWidth = pars[0].width[i] * 2;
                        break;
                    default:
                        lineWidth = pars[0].width[i];
                }
                callback(bufs, numBuf, lineWidth, pars[0].height[i], pars[0].pitch[i]);
            }else{
                CONSUMER_PRINT("callback not set\n");
                break;
            }
            for (uint32_t j = 0; j < m_stream.size(); j++){
                NvBufferMemSyncForDevice (fds[j], i, &bufs[j]);
                NvBufferMemUnMap(fds[j], i, &bufs[j]);
                delete  frames[j];
            }
        }

        if (VERBOSE_ENABLE)
            CONSUMER_PRINT("Acquired Frame. %d\n", fd);

    }

    CONSUMER_PRINT("Done.\n");

    requestShutdown();

    return true;
}

bool SyncBufferConsumerThread::threadShutdown()
{
    if(VERBOSE_ENABLE)
        CONSUMER_PRINT("shutdown thread\n");
    m_gotError = true;
    return true;
}
