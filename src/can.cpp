#include "can.hpp"

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
    close();

    socket_fd_ = socket(
        PF_CAN,
        SOCK_RAW,
        CAN_RAW);

    if (socket_fd_ < 0)
    {
        std::cerr
            << "[CAN] Failed to create socket: "
            << std::strerror(errno)
            << '\n';

        return false;
    }

    struct ifreq ifr {};

    std::strncpy(
        ifr.ifr_name,
        interface_name.c_str(),
        IFNAMSIZ - 1);

    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(
            socket_fd_,
            SIOCGIFINDEX,
            &ifr) < 0)
    {
        std::cerr
            << "[CAN] Failed to get interface index for "
            << interface_name
            << ": "
            << std::strerror(errno)
            << '\n';

        close();
        return false;
    }

    interface_index_ = ifr.ifr_ifindex;

    struct sockaddr_can address {};

    address.can_family = AF_CAN;
    address.can_ifindex = interface_index_;

    if (bind(
            socket_fd_,
            reinterpret_cast<struct sockaddr*>(&address),
            sizeof(address)) < 0)
    {
        std::cerr
            << "[CAN] Failed to bind socket to "
            << interface_name
            << ": "
            << std::strerror(errno)
            << '\n';

        close();
        return false;
    }

    std::cout
        << "[CAN] Connected to "
        << interface_name
        << '\n';

    return true;
}

void CanBus::close()
{
    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }

    interface_index_ = -1;
}

bool CanBus::receive(
    CanFrame& frame,
    int timeout_ms)
{
    if (!is_open())
    {
        return false;
    }

    fd_set read_fds;

    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    struct timeval timeout {};

    timeout.tv_sec =
        timeout_ms / 1000;

    timeout.tv_usec =
        (timeout_ms % 1000) * 1000;

    const int result = select(
        socket_fd_ + 1,
        &read_fds,
        nullptr,
        nullptr,
        &timeout);

    if (result < 0)
    {
        if (errno == EINTR)
        {
            return false;
        }

        std::cerr
            << "[CAN] select() failed: "
            << std::strerror(errno)
            << '\n';

        return false;
    }

    if (result == 0)
    {
        return false;
    }

    struct can_frame can_frame_data {};

    const ssize_t bytes_read =
        read(
            socket_fd_,
            &can_frame_data,
            sizeof(can_frame_data));

    if (bytes_read < 0)
    {
        std::cerr
            << "[CAN] read() failed: "
            << std::strerror(errno)
            << '\n';

        return false;
    }

    if (bytes_read !=
        static_cast<ssize_t>(
            sizeof(can_frame_data)))
    {
        std::cerr
            << "[CAN] Invalid CAN frame size: "
            << bytes_read
            << '\n';

        return false;
    }

    frame.id =
        can_frame_data.can_id &
        CAN_SFF_MASK;

    frame.dlc =
        can_frame_data.can_dlc;

    if (frame.dlc > 8U)
    {
        std::cerr
            << "[CAN] Invalid DLC: "
            << static_cast<int>(frame.dlc)
            << '\n';

        return false;
    }

    std::memcpy(
        frame.data,
        can_frame_data.data,
        frame.dlc);

    return true;
}

bool CanBus::send(
    uint32_t can_id,
    const uint8_t* data,
    uint8_t dlc)
{
    if (!is_open())
    {
        return false;
    }

    if (data == nullptr || dlc > 8U)
    {
        return false;
    }

    struct can_frame frame {};

    frame.can_id = can_id;
    frame.can_dlc = dlc;

    std::memcpy(
        frame.data,
        data,
        dlc);

    const ssize_t bytes_written =
        write(
            socket_fd_,
            &frame,
            sizeof(frame));

    if (bytes_written !=
        static_cast<ssize_t>(
            sizeof(frame)))
    {
        std::cerr
            << "[CAN] write() failed: "
            << std::strerror(errno)
            << '\n';

        return false;
    }

    return true;
}

bool CanBus::is_open() const
{
    return socket_fd_ >= 0;
}

} // namespace vecu