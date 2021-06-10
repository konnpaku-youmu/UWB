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

    struct __attribute__((__packed__))
    {
        uint8_t modbus_addr = 0x01;
        uint8_t func_code = 0x10;
        uint8_t reg_addr_h;
        uint8_t reg_addr_l;
        uint8_t reg_cnt_h;
        uint8_t reg_cnt_l;
        uint8_t reg_byte_size;
        uint16_t reg_data;
        uint8_t crc_l;
        uint8_t crc_h;
    } typedef MODBUS_RTU_multi_reg_struct;

    struct
    {
        std::string serial_port;
        uint8_t modbus_id;
        RANGING_MODE ranging_mode;
        DEVICE_MODE device_mode;
        uint8_t id;
        RANGING_EN_SEL enable_sel;
        uint8_t tag_num;
        Eigen::Vector3f base_position;
        std::vector<Eigen::Vector3f> sub_stations_pos;
    } typedef BaseStation_Param_struct;

    struct
    {
        uint8_t modbus_id = 0x01;
        uint8_t func_code = 0x03;
        uint8_t byte_size = 0x1A;
        uint16_t tag_id;
        uint16_t flag;
        int16_t tag_x;
        int16_t tag_y;
        int16_t tag_z;
        uint16_t dist_a;
        uint16_t dist_b;
        uint16_t dist_c;
        uint16_t dist_d;
        uint16_t dist_e;
        uint16_t dist_f;
        uint16_t dist_g;
        uint16_t dist_h;
        uint16_t crc;
    } typedef Ranging_Dataframe;

    class BaseStationComm : public MultiThread
    {
    private:
        boost::asio::io_service __io_srv__;

        std::shared_ptr<boost::asio::serial_port> __serial_port_ptr__;

        BaseStation_Param_struct station_params;

        boost::array<char, 31> __buf__;

        void __write_single_register(MODBUS_RTU_single_reg_struct *);

        boost::system::error_code __write_serial__(void *, size_t);

        boost::system::error_code __read_serial__(std::string &, unsigned char);

        void __start_receive__();

        void __handle_receive(const boost::system::error_code, size_t);
    
    protected:
        void entryPoint();

    public:
        BaseStationComm(std::mutex *, std::string &);

        ~BaseStationComm();

        void loadConfigFile(std::string &);

        void configure();
    };
}

#endif
