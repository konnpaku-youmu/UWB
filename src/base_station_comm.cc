#include "../include/base_station_comm.h"
#include "../include/regmap.h"

#include <iostream>

namespace uwb
{
    BaseStationComm::BaseStationComm(std::mutex *m)
    {
        this->mut = m;

        try
        {
            std::string port = "/dev/ttyUSB0";

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

    void BaseStationComm::loadConfigFile(std::string &config_path)
    {
        YAML::Node config_yaml = YAML::LoadFile(config_path);

        // load config file
        station_params.serial_port = config_yaml["dev-name"].as<std::string>();
        station_params.modbus_id = config_yaml["modbus-id"].as<int>();
        station_params.ranging_mode = (RANGING_MODE)config_yaml["ranging-mode"].as<int>();
        station_params.device_mode = (DEVICE_MODE)config_yaml["device-mode"].as<int>();
        station_params.id = config_yaml["station-id"].as<int>();
        station_params.enable_sel = (RANGING_EN_SEL)config_yaml["loc-en"].as<int>();
        station_params.tag_num = config_yaml["tag-num"].as<int>();

        return;
    }

    void BaseStationComm::entryPoint()
    {

        return;
    }

    void BaseStationComm::configure()
    {
        // set localization mode
        MODBUS_RTU_single_reg_struct set_loc_mode;
        set_loc_mode.reg_addr_h = (REG_RANGING_MODE >> 8);
        set_loc_mode.reg_addr_l = (REG_RANGING_MODE & 0xFF);
        set_loc_mode.reg_data_h = 0x00;
        set_loc_mode.reg_data_l = station_params.ranging_mode;

        crc_modbus crc;
        crc.process_bytes(&set_loc_mode, 6);
        uint16_t crc_res = crc.checksum();

        set_loc_mode.crc_l = (crc_res & 0xFF);
        set_loc_mode.crc_h = (crc_res >> 8);

        this->__write_single_register(&set_loc_mode);

        // set operation mode
        MODBUS_RTU_single_reg_struct set_dev_mode;
        set_dev_mode.reg_addr_h = (REG_DEVICE_MODE >> 8);
        set_dev_mode.reg_addr_l = (REG_DEVICE_MODE & 0xFF);
        set_dev_mode.reg_data_h = 0x00;
        set_dev_mode.reg_data_l = station_params.device_mode;

        crc.reset();
        crc.process_bytes(&set_dev_mode, 6);
        crc_res = crc.checksum();

        set_dev_mode.crc_l = (crc_res & 0xFF);
        set_dev_mode.crc_h = (crc_res >> 8);

        this->__write_single_register(&set_dev_mode);

        return;
    }

    void BaseStationComm::__write_single_register(MODBUS_RTU_single_reg_struct *reg_struct)
    {
        boost::system::error_code ret = this->__write_serial__(reg_struct, sizeof(MODBUS_RTU_single_reg_struct));

        // listen for echo
        std::string echo;

        if (ret.failed())
        {
            std::cout << ret.message() << std::endl;
        }
        else
        {
            this->__read_serial__(echo, reg_struct->crc_h);
            MODBUS_RTU_single_reg_struct *echo_struct = (MODBUS_RTU_single_reg_struct *)echo.c_str();

            assert(*(uint64_t *)echo_struct == *(uint64_t *)reg_struct);
        }

        return;
    }

    boost::system::error_code BaseStationComm::__write_serial__(void *begin, size_t size)
    {
        boost::system::error_code ec;
        boost::asio::write(*(this->__serial_port_ptr__), boost::asio::buffer(begin, size), ec);
        return ec;
    }

    boost::system::error_code BaseStationComm::__read_serial__(std::string &message, unsigned char eol = '\n')
    {
        message = "";
        size_t bytes = 0;
        boost::asio::streambuf buf;
        boost::system::error_code ec;

        /* Synchronous reading*/
        while (message.find(eol) == std::string::npos)
        {
            bytes += boost::asio::read(*this->__serial_port_ptr__, buf, boost::asio::transfer_exactly(1), ec);
            if (ec)
            {
                std::cout << "Reading UART response failed..." << std::endl;
                break;
            }

            boost::asio::streambuf::const_buffers_type cont_buf = buf.data();
            message = std::string(boost::asio::buffers_begin(cont_buf), boost::asio::buffers_begin(cont_buf) + bytes);
        }

        return ec;
    }
}
