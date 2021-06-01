#include "../include/base_station_comm.h"

#include <iostream>

namespace uwb
{
    BaseStationComm::BaseStationComm(std::mutex *m)
    {
        this->mut = m;

        try
        {
            std::string port = "/dev/ttyUSB1";

            this->__serial_port_ptr__ = std::make_shared<boost::asio::serial_port>(this->__io_srv__);
            this->__serial_port_ptr__->open(port);

            boost::asio::serial_port::baud_rate baud_rate(115200);
            boost::asio::serial_port::character_size c_size(8);
            boost::asio::serial_port::stop_bits stop_bits(boost::asio::serial_port::stop_bits::one);
            boost::asio::serial_port::parity parity(boost::asio::serial_port::parity::none);
            boost::asio::serial_port::flow_control flow_ctl(boost::asio::serial_port::flow_control::none);

            this->__serial_port_ptr__->set_option(baud_rate);
            this->__serial_port_ptr__->set_option(c_size);
            this->__serial_port_ptr__->set_option(stop_bits);
            this->__serial_port_ptr__->set_option(parity);
            this->__serial_port_ptr__->set_option(flow_ctl);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    BaseStationComm::~BaseStationComm()
    {
    }

    void BaseStationComm::entryPoint()
    {

        return;
    }

    void BaseStationComm::configure()
    {
        // set localization mode
        MODBUS_RTU_single_reg_struct *set_loc_mode;
        set_loc_mode = (MODBUS_RTU_single_reg_struct *)malloc(sizeof(MODBUS_RTU_single_reg_struct));
        set_loc_mode->reg_addr_h = 0x00;
        set_loc_mode->reg_addr_l = 0x02;
        set_loc_mode->reg_data_h = 0x00;
        set_loc_mode->reg_data_l = 0x01;

        uint16_t crc = calcCRC16(set_loc_mode, 6);

        set_loc_mode->crc_h = (crc >> 8);
        set_loc_mode->crc_l = (crc % 0xFF00);

        boost::system::error_code ret = this->__write_serial__((char *)set_loc_mode, sizeof(MODBUS_RTU_single_reg_struct));
        
        if (ret.failed())
        {
            std::cout << ret.message() << std::endl;
        }
        
        // set operation mode
        return;
    }

    boost::system::error_code BaseStationComm::__write_serial__(char *begin, size_t size)
    {
        boost::system::error_code ec;
        boost::asio::write(*(this->__serial_port_ptr__), boost::asio::buffer(begin, size), ec);
        return ec;
    }
}