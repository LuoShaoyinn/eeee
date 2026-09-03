#pragma once

#include <chrono>
#include <string>

namespace robot {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    void open(const std::string& path, int baud_rate);
    void close();
    void write_all(const std::string& bytes);
    std::string read_line(std::chrono::milliseconds timeout);

private:
    int fd_ = -1;
    std::string buffered_input_;
};

}  // namespace robot
