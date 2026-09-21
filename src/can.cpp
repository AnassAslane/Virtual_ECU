#include "can.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>

#include <unistd.h>

namespace vecu {

CanBus::~CanBus()
{
    close();
}

bool CanBus::open(const std::string& interface_name)
{
    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (socket_fd_ < 0) {
        std::cerr << "[CAN] socket() failed: "
                  << std::strerror(errno) << '\n';
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name,
                 interface_name.c_str(),
                 IFNAMSIZ - 1);

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "[CAN] Cannot find interface "
                  << interface_name << ": "
                  << std::strerror(errno) << '\n';

        close();
        return false;
    }

    interface_index_ = ifr.ifr_ifindex;

    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = interface_index_;

    if (::bind(socket_fd_,
               reinterpret_cast<struct sockaddr*>(&address),
               sizeof(address)) < 0) {

        std::cerr << "[CAN] bind() failed: "
                  << std::strerror(errno) << '\n';

        close();
        return false;
    }

    std::cout << "[CAN] Connected to "
              << interface_name << '\n';

    return true;
}

void CanBus::close()
{
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }

    interface_index_ = -1;
}

bool CanBus::is_open() const
{
    return socket_fd_ >= 0;
}

bool CanBus::send_engine_data(const EngineData& engine)
{
    if (!is_open()) {
        return false;
    }

    struct can_frame frame {};

    frame.can_id = ENGINE_DATA_ID;
    frame.can_dlc = 5;

    /*
     * Byte 0-1: RPM
     * Byte 2-3: vehicle speed
     * Byte 4: throttle
     *
     * Big-endian encoding.
     */

    frame.data[0] =
        static_cast<uint8_t>((engine.rpm >> 8) & 0xFF);

    frame.data[1] =
        static_cast<uint8_t>(engine.rpm & 0xFF);

    frame.data[2] =
        static_cast<uint8_t>((engine.speed_kmh >> 8) & 0xFF);

    frame.data[3] =
        static_cast<uint8_t>(engine.speed_kmh & 0xFF);

    frame.data[4] = engine.throttle_percent;

    const ssize_t bytes_written =
        ::write(socket_fd_, &frame, sizeof(frame));

    if (bytes_written != sizeof(frame)) {
        std::cerr << "[CAN] Failed to transmit frame: "
                  << std::strerror(errno) << '\n';
        return false;
    }

    return true;
}

bool CanBus::receive(uint32_t& can_id,
                     uint8_t* data,
                     uint8_t& dlc,
                     int timeout_ms)
{
    if (!is_open() || data == nullptr) {
        return false;
    }

    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    struct timeval timeout {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    const int result =
        select(socket_fd_ + 1,
               &read_set,
               nullptr,
               nullptr,
               &timeout);

    if (result < 0) {
        if (errno == EINTR) {
            return false;
        }

        std::cerr << "[CAN] select() failed: "
                  << std::strerror(errno) << '\n';

        return false;
    }

    if (result == 0) {
        return false;
    }

    struct can_frame frame {};

    const ssize_t bytes_read =
        ::read(socket_fd_, &frame, sizeof(frame));

    if (bytes_read != sizeof(frame)) {
        return false;
    }

    can_id = frame.can_id & CAN_EFF_MASK;
    dlc = std::min<uint8_t>(frame.can_dlc, CAN_MAX_DLEN);

    std::memcpy(data, frame.data, dlc);

    return true;
}

} // namespace vecu