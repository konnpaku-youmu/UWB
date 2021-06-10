#include "../include/base_station_comm.h"

#include <pthread.h>
#include <iostream>

int main(int argc, char const *argv[])
{
    std::string config_path = "../cfg/config.yaml";

    pthread_attr_t attr;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    std::mutex *mut = new std::mutex();
    uwb::BaseStationComm base_station(mut, config_path);

    base_station.configure();

    base_station.startThread(attr);

    while (1);

    return 0;
}
