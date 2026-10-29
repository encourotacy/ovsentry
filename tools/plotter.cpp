#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

namespace tools {
Plotter::Plotter(std::string host, uint16_t port) {
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

    destination_.sin_family = AF_INET;
    destination_.sin_port = ::htons(port);
    destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

Plotter::~Plotter() { ::close(socket_); }

void Plotter::plot(const nlohmann::json &json) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto data = json.dump();
    // ::sendto(socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr
    // *>(&destination_),
    //          sizeof(destination_));
    ssize_t sent_bytes =
        ::sendto(socket_, data.c_str(), data.length(), 0,
                 reinterpret_cast<sockaddr *>(&destination_), sizeof(destination_));

    if (sent_bytes < 0) {
        perror("sendto failed");
    } else if (static_cast<size_t>(sent_bytes) != data.length()) {
        // 对于UDP，通常要么全部发送，要么失败。如果发送不完整，说明数据报被截断。
        fprintf(stderr, "Warning: sendto sent only %zd of %zu bytes\n", sent_bytes, data.length());
    }
}

}  // namespace tools