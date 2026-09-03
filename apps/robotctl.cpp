#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

std::string request(const std::string& socket_path, const std::string& command) {
    const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) throw std::runtime_error("cannot create Unix socket");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (socket_path.size() >= sizeof(address.sun_path)) throw std::runtime_error("socket path too long");
    std::strncpy(address.sun_path, socket_path.c_str(), sizeof(address.sun_path) - 1);
    if (connect(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        close(fd);
        throw std::runtime_error("cannot connect to robotd");
    }
    const std::string wire = command + "\n";
    if (write(fd, wire.data(), wire.size()) < 0) {
        close(fd);
        throw std::runtime_error("cannot write robot command");
    }
    char buffer[1024]{};
    const ssize_t count = read(fd, buffer, sizeof(buffer) - 1);
    close(fd);
    if (count <= 0) throw std::runtime_error("robotd closed the command socket");
    return std::string(buffer, static_cast<size_t>(count));
}

}  // namespace

int main(int argc, char** argv) {
    std::string socket_path = "/tmp/robotd.sock";
    int hold_ms = 0;
    int first_command = 1;
    while (first_command < argc) {
        const std::string argument = argv[first_command];
        if (argument == "--socket" && first_command + 1 < argc) socket_path = argv[++first_command];
        else if (argument == "--hold-ms" && first_command + 1 < argc) hold_ms = std::stoi(argv[++first_command]);
        else break;
        ++first_command;
    }
    if (first_command >= argc) {
        std::cerr << "usage: robotctl [--socket PATH] [--hold-ms N] COMMAND...\n";
        return 2;
    }
    std::string command;
    for (int index = first_command; index < argc; ++index) {
        if (!command.empty()) command += ' ';
        command += argv[index];
    }
    try {
        std::string reply;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(hold_ms);
        do {
            reply = request(socket_path, command);
            if (hold_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(80));
        } while (hold_ms > 0 && std::chrono::steady_clock::now() < deadline);
        std::cout << reply;
    } catch (const std::exception& error) {
        std::cerr << "robotctl: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
