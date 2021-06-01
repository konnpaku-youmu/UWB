#ifndef UWB_BASE_STATION_COMM_H
#define UWB_BASE_STATION_COMM_H

#include <mutex>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include "utilities.hpp"

namespace uwb
{
    struct __attribute__((__packed__)) MODBUS_RTU_single_reg_struct
    {
        uint8_t modbus_addr = 0x01;
        uint8_t func_code = 0x06;
        uint8_t reg_addr_h;
        uint8_t reg_addr_l;
        uint8_t reg_data_h;
        uint8_t reg_data_l;
        uint8_t crc_h;
        uint8_t crc_l;
    };

    class BaseStationComm : public MultiThread
    {
    private:
        boost::asio::io_service __io_srv__;

        std::shared_ptr<boost::asio::serial_port> __serial_port_ptr__;

        boost::system::error_code __write_serial__(char *, size_t);

    protected:
        void entryPoint();

    public:
        BaseStationComm(std::mutex *);
        
        ~BaseStationComm();

        void configure();
    };
}

#endif