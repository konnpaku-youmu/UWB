#ifndef UWB_UTILITIES_H
#define UWB_UTILITIES_H

#include <pthread.h>
#include <mutex>

namespace uwb
{
    typedef boost::crc_optimal<16, 0x8005, 0xFFFF, 0, true, true> crc_modbus;
    
    class MultiThread
    {
    private:
        pthread_t __thread__;

        static void *__internal_thread_entry__(void *This)
        {
            std::lock_guard<std::mutex> locker(*(((MultiThread *)This)->mut));
            ((MultiThread *)This)->entryPoint();

            return NULL;
        }

    protected:
        std::mutex *mut;

        virtual void entryPoint() = 0;

    public:
        MultiThread() {}

        bool startThread(pthread_attr_t &attr)
        {
            return (pthread_create(&(this->__thread__), &attr, __internal_thread_entry__, this) == 0);
        }

        void stopThread()
        {
            return (void)pthread_join(this->__thread__, NULL);
        }
    };
}

#endif
