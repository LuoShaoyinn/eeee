#include "robot/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace robot {
namespace {

speed_t baud_constant(int baud_rate) {
    if (baud_rate == 115200) return B115200;
    throw std::runtime_error("unsupported UART baud rate");
}

std::runtime_error system_error(const std::string& operation) {
    return std::runtime_error(operation + ": " + std::strerror(errno));
}

}  // namespace

SerialPort::~SerialPort() { close(); }

void SerialPort::open(const std::string& path, int baud_rate) {
    close();
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) throw system_error("open " + path);

    termios settings{};
    if (tcgetattr(fd_, &settings) != 0) throw system_error("tcgetattr");
    cfmakeraw(&settings);
    if (cfsetispeed(&settings, baud_constant(baud_rate)) != 0 ||
        cfsetospeed(&settings, baud_constant(baud_rate)) != 0) {
        throw system_error("cfset speed");
    }
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CRTSCTS;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &settings) != 0) throw system_error("tcsetattr");
    tcflush(fd_, TCIOFLUSH);
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    buffered_input_.clear();
}

void SerialPort::write_all(const std::string& bytes) {
    size_t offset = 0;
    while (offset < bytes.size()) {
        const ssize_t count = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
        if (count < 0) {
            if (errno == EINTR) continue;
            throw system_error("UART write");
        }
        offset += static_cast<size_t>(count);
    }
    if (tcdrain(fd_) != 0) throw system_error("UART drain");
}

std::string SerialPort::read_line(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
        const size_t newline = buffered_input_.find('\n');
        if (newline != std::string::npos) {
            std::string line = buffered_input_.substr(0, newline);
            buffered_input_.erase(0, newline + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return line;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) throw std::runtime_error("UART response timeout");
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        pollfd descriptor{.fd = fd_, .events = POLLIN, .revents = 0};
        const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
        if (ready < 0) {
            if (errno == EINTR) continue;
            throw system_error("UART poll");
        }
        if (ready == 0) continue;
        char bytes[256];
        const ssize_t count = ::read(fd_, bytes, sizeof(bytes));
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            throw system_error("UART read");
        }
        if (count > 0) buffered_input_.append(bytes, static_cast<size_t>(count));
    }
}

}  // namespace robot
