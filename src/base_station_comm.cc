#include "../include/base_station_comm.h"
#include "../include/regmap.h"

#include <iostream>

namespace uwb
{
    BaseStationComm::BaseStationComm(std::mutex *m, std::string &config_path)
    {
        this->mut = m;

        this->loadConfigFile(config_path);

        try
        {
            std::cout << this->station_params.serial_port << std::endl;
            this->__serial_port_ptr__ = std::make_shared<boost::asio::serial_port>(this->__io_srv__);
            this->__serial_port_ptr__->open(this->station_params.serial_port);

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

        YAML::Node base_pos_node = config_yaml["base-station-pos"];
        station_params.base_position[0] = base_pos_node["x"].as<int>();
        station_params.base_position[1] = base_pos_node["y"].as<int>();
        station_params.base_position[2] = base_pos_node["z"].as<int>();

        char station_name;
        for (station_name = 'b'; station_name != 'h'; ++station_name)
        {
            std::stringstream station_ss;
            station_ss << "station-" << station_name << "-pos";
            YAML::Node sub_station_node = config_yaml[station_ss.str().c_str()];
            if (sub_station_node["enable"].as<int>() == 1)
            {
                Eigen::Vector3f sub_station_pos;
                sub_station_pos[0] = sub_station_node["x"].as<int>();
                sub_station_pos[1] = sub_station_node["y"].as<int>();
                sub_station_pos[2] = sub_station_node["z"].as<int>();

                station_params.sub_stations_pos.push_back(sub_station_pos);
            }
        }

        return;
    }

    void BaseStationComm::entryPoint()
    {
        // send ranging command
        usleep(100e3);
        MODBUS_RTU_multi_reg_struct ranging_cmd;
        ranging_cmd.reg_addr_h = (REG_RANGING_EN >> 8);
        ranging_cmd.reg_addr_l = (REG_RANGING_EN & 0xFF);
        ranging_cmd.reg_cnt_h = 0x00;
        ranging_cmd.reg_cnt_l = 0x01;
        ranging_cmd.reg_byte_size = 0x02;

        // ranging_cmd.reg_data = (uint16_t *)malloc(ranging_cmd.reg_byte_size);
        ranging_cmd.reg_data = 0x0400;

        crc_modbus crc;
        crc.process_bytes(&ranging_cmd, 9);
        uint16_t crc_res = crc.checksum();

        ranging_cmd.crc_l = (crc_res & 0xFF);
        ranging_cmd.crc_h = (crc_res >> 8);

        crc_modbus crc_echo;
        crc_echo.process_bytes(&ranging_cmd, 6);
        uint16_t crc_echo_res = crc_echo.checksum();

        std::string echo;
        this->__write_serial__(&ranging_cmd, 11);
        this->__read_serial__(echo, (crc_echo_res >> 8));

        usleep(100e3);

        std::string message;
        while (1)
        {
            boost::asio::streambuf buf;
            boost::system::error_code ec;
            boost::asio::read(*this->__serial_port_ptr__, buf, boost::asio::transfer_exactly(31), ec);

            boost::asio::streambuf::const_buffers_type cont_buf = buf.data();
            message = std::string(boost::asio::buffers_begin(cont_buf), boost::asio::buffers_begin(cont_buf) + 31);

            Ranging_Dataframe dfc = *(Ranging_Dataframe *)message.c_str();

            std::cout <<  std::dec << dfc.tag_x << " " << dfc.tag_y << ", " << dfc.tag_z << std::endl;
        }

        return;
    }

    void BaseStationComm::configure()
    {
        // set localization mode
        MODBUS_RTU_single_reg_struct set_register;
        set_register.reg_addr_h = (REG_RANGING_MODE >> 8);
        set_register.reg_addr_l = (REG_RANGING_MODE & 0xFF);
        set_register.reg_data_h = 0x00;
        set_register.reg_data_l = station_params.ranging_mode;

        crc_modbus crc;
        crc.process_bytes(&set_register, 6);
        uint16_t crc_res = crc.checksum();

        set_register.crc_l = (crc_res & 0xFF);
        set_register.crc_h = (crc_res >> 8);

        this->__write_single_register(&set_register);

        usleep(100e3);
        // set operation mode
        set_register.reg_addr_h = (REG_DEVICE_MODE >> 8);
        set_register.reg_addr_l = (REG_DEVICE_MODE & 0xFF);
        set_register.reg_data_h = 0x00;
        set_register.reg_data_l = station_params.device_mode;

        crc.reset();
        crc.process_bytes(&set_register, 6);
        crc_res = crc.checksum();

        set_register.crc_l = (crc_res & 0xFF);
        set_register.crc_h = (crc_res >> 8);

        this->__write_single_register(&set_register);

        usleep(100e3);
        // set base station position
        uint16_t base_pos_x = (uint16_t)station_params.base_position.x();
        uint16_t base_pos_y = (uint16_t)station_params.base_position.y();
        uint16_t base_pos_z = (uint16_t)station_params.base_position.z();

        set_register.reg_addr_h = (REG_STA_A_POS_X >> 8);
        set_register.reg_addr_l = (REG_STA_A_POS_X & 0xFF);
        set_register.reg_data_h = (base_pos_x >> 8);
        set_register.reg_data_l = (base_pos_x & 0xFF);

        crc.reset();
        crc.process_bytes(&set_register, 6);
        crc_res = crc.checksum();

        set_register.crc_l = (crc_res & 0xFF);
        set_register.crc_h = (crc_res >> 8);

        this->__write_single_register(&set_register);

        usleep(100e3);
        set_register.reg_addr_h = (REG_STA_A_POS_Y >> 8);
        set_register.reg_addr_l = (REG_STA_A_POS_Y & 0xFF);
        set_register.reg_data_h = (base_pos_y >> 8);
        set_register.reg_data_l = (base_pos_y & 0xFF);

        crc.reset();
        crc.process_bytes(&set_register, 6);
        crc_res = crc.checksum();

        set_register.crc_l = (crc_res & 0xFF);
        set_register.crc_h = (crc_res >> 8);

        this->__write_single_register(&set_register);

        usleep(100e3);
        set_register.reg_addr_h = (REG_STA_A_POS_Z >> 8);
        set_register.reg_addr_l = (REG_STA_A_POS_Z & 0xFF);
        set_register.reg_data_h = (base_pos_z >> 8);
        set_register.reg_data_l = (base_pos_z & 0xFF);

        crc.reset();
        crc.process_bytes(&set_register, 6);
        crc_res = crc.checksum();

        set_register.crc_l = (crc_res & 0xFF);
        set_register.crc_h = (crc_res >> 8);

        this->__write_single_register(&set_register);

        // enable&set sub stations
        uint8_t reg_offset = 0;
        for (auto sub_pos : station_params.sub_stations_pos)
        {
            usleep(100e3);
            set_register.reg_addr_h = ((REG_STA_B_EN + reg_offset) >> 8);
            set_register.reg_addr_l = ((REG_STA_B_EN + reg_offset) & 0xFF);
            set_register.reg_data_h = 0x00;
            set_register.reg_data_l = 0x01;

            crc.reset();
            crc.process_bytes(&set_register, 6);
            crc_res = crc.checksum();

            set_register.crc_l = (crc_res & 0xFF);
            set_register.crc_h = (crc_res >> 8);

            this->__write_single_register(&set_register);

            usleep(100e3);
            uint16_t sub_station_pos_x = (uint16_t)sub_pos.x();
            uint16_t sub_station_pos_y = (uint16_t)sub_pos.y();
            uint16_t sub_station_pos_z = (uint16_t)sub_pos.z();
            set_register.reg_addr_h = ((REG_STA_B_POS_X + reg_offset) >> 8);
            set_register.reg_addr_l = ((REG_STA_B_POS_X + reg_offset) & 0xFF);
            set_register.reg_data_h = (sub_station_pos_x >> 8);
            set_register.reg_data_l = (sub_station_pos_x & 0xFF);

            crc.reset();
            crc.process_bytes(&set_register, 6);
            crc_res = crc.checksum();

            set_register.crc_l = (crc_res & 0xFF);
            set_register.crc_h = (crc_res >> 8);

            this->__write_single_register(&set_register);

            usleep(100e3);
            set_register.reg_addr_h = ((REG_STA_B_POS_Y + reg_offset) >> 8);
            set_register.reg_addr_l = ((REG_STA_B_POS_Y + reg_offset) & 0xFF);
            set_register.reg_data_h = (sub_station_pos_y >> 8);
            set_register.reg_data_l = (sub_station_pos_y & 0xFF);

            crc.reset();
            crc.process_bytes(&set_register, 6);
            crc_res = crc.checksum();

            set_register.crc_l = (crc_res & 0xFF);
            set_register.crc_h = (crc_res >> 8);

            this->__write_single_register(&set_register);

            usleep(100e3);
            set_register.reg_addr_h = ((REG_STA_B_POS_Z + reg_offset) >> 8);
            set_register.reg_addr_l = ((REG_STA_B_POS_Z + reg_offset) & 0xFF);
            set_register.reg_data_h = (sub_station_pos_z >> 8);
            set_register.reg_data_l = (sub_station_pos_z & 0xFF);

            crc.reset();
            crc.process_bytes(&set_register, 6);
            crc_res = crc.checksum();

            set_register.crc_l = (crc_res & 0xFF);
            set_register.crc_h = (crc_res >> 8);

            this->__write_single_register(&set_register);

            reg_offset += 4;
        }

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

    void BaseStationComm::__start_receive__()
    {
        this->__serial_port_ptr__->async_read_some(boost::asio::buffer(this->__buf__),
                                                   boost::bind(&BaseStationComm::__handle_receive, this,
                                                               boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

    void BaseStationComm::__handle_receive(const boost::system::error_code ec, size_t bytes_transferred)
    {

        this->__start_receive__();
        return;
    }
}
