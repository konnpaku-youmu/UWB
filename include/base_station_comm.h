#ifndef UWB_BASE_STATION_COMM_H
#define UWB_BASE_STATION_COMM_H

#include <mutex>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/crc.hpp>
#include <yaml-cpp/yaml.h>

#include "utilities.hpp"

namespace uwb
{
    enum RANGING_MODE
    {
        ONE_TO_ONE = 0x00,
        LOC_2D = 0x01,
        LOC_3D = 0x02
    };
    
    enum DEVICE_MODE
    {
        TAG = 0x00,
        SUB_STATION = 0x01,
        MAIN_STATION = 0x02
    };

    enum RANGING_EN_SEL
    {
        DISABLE = 0x00,
        SINGLE = 0x01,
        CONTINUOUS = 0x02,
        SINGLE_OUT = 0x03,
        CONTINUOUS_OUT = 0x04
    };

    struct __attribute__((__packed__))
    {
        uint8_t modbus_addr = 0x01;
        uint8_t func_code = 0x06;
        uint8_t reg_addr_h;
        uint8_t reg_addr_l;
        uint8_t reg_data_h;
        uint8_t reg_data_l;
        uint8_t crc_l;
        uint8_t crc_h;
    } typedef MODBUS_RTU_single_reg_struct;

    struct
    {
        std::string serial_port;
        uint8_t modbus_id;
        RANGING_MODE ranging_mode;
        DEVICE_MODE device_mode;
        uint8_t id;
        RANGING_EN_SEL enable_sel;
        uint8_t tag_num;
    } typedef BaseStation_Param_struct;

    class BaseStationComm : public MultiThread
    {
    private:
        boost::asio::io_service __io_srv__;

        std::shared_ptr<boost::asio::serial_port> __serial_port_ptr__;

        BaseStation_Param_struct station_params;

        void __write_single_register(MODBUS_RTU_single_reg_struct *);

        boost::system::error_code __write_serial__(void *, size_t);

        boost::system::error_code __read_serial__(std::string &, unsigned char);

    protected:
        void entryPoint();

    public:
        BaseStationComm(std::mutex *);

        ~BaseStationComm();

        void loadConfigFile(std::string &);

        void configure();
    };
}

#endif
