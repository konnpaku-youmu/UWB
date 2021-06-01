#include "../include/base_station_comm.h"

#include <pthread.h>
#include <iostream>

int main(int argc, char const *argv[])
{
    pthread_attr_t attr;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    std::mutex *mut = new std::mutex();
    uwb::BaseStationComm base_station(mut);

    base_station.configure();

    // base_station.startThread(attr);

    return 0;
}
