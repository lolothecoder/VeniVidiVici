#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
# define M_PI           3.14159265358979323846  /* pi */
// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }

  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "o " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }

  void set_servo_door_values(int val)
  {
    std::stringstream ss;
    // Sending command to the servo 
    if(val ==0)
      ss << "h 0 0" << "\r"; // Stop the servo
    else
      ss << "h 0 1" << "\r"; // Set the servo to a position
    //ss << "h " + std::to_string(val) + "\r";

    send_msg(ss.str());
  }

  void set_servo_ramp_values(int val)
  {
    std::stringstream ss;
    // Sending command to the servo 
    if(val ==0)
      ss << "h 1 0" << "\r"; // Stop the servo
    else
      ss << "h 1 1" << "\r"; // Set the servo to a position
    //ss << "h " + std::to_string(val) + "\r";

    send_msg(ss.str());
  }

  void set_servo_collector_values(int val)
  {
    std::stringstream ss;
    // Sending command to the servo 
    ss << "g " << std::to_string(val) << "\r"; // Set the servo to a position

    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

   void get_values(double &val_1, double &val_2)
  {
    std::string response = send_msg("i\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());

    del_pos = response.find(delimiter);
    std::string token_2 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());


    val_1 = 0.0;
    val_2 = 0.0;

    val_1 = std::stod(token_1.c_str())*(2*M_PI)/(60.0*60.0);
    val_2 = std::stod(token_2.c_str())*(2*M_PI)/(60.0*60.0);
  }

  void get_imu (double &a_x, double &a_y, double &a_z, 
                  double &g_x, double &g_y, double &g_z)
  {
    std::string response = send_msg("j\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());

    del_pos = response.find(delimiter);
    std::string token_2 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());

    del_pos = response.find(delimiter);
    std::string token_3 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());

    del_pos = response.find(delimiter);
    std::string token_4 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());

    del_pos = response.find(delimiter);
    std::string token_5 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());

    del_pos = response.find(delimiter);
    std::string token_6 = response.substr(0, del_pos);

    a_x = 0.0;
    a_y = 0.0;
    a_z = 0.0;
    g_x = 0.0;
    g_y = 0.0;
    g_z = 0.0;

    a_x = std::stod(token_1.c_str());
    a_y = std::stod(token_2.c_str());
    a_z = std::stod(token_3.c_str());
    
    g_x = std::stod(token_4.c_str());
    g_y = std::stod(token_5.c_str());
    g_z = std::stod(token_6.c_str());
  }
private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP