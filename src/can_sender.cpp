
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
constexpr const char* CAN_INTERFACE = "vcan0";
constexpr canid_t ENGINE_DATA_ID = 0x100;
constexpr std::uint8_t CAN_DLC = 5;

// Vehicle data transmission period.
constexpr unsigned int TRANSMISSION_PERIOD_MS = 100;

bool encode_engine_data(
    std::uint16_t rpm,
    std::uint16_t speed_kmh,
    std::uint8_t throttle_percent,
    can_frame& frame)
{
    if (throttle_percent > 100)
    {
        return false;
    }

    frame.can_id = ENGINE_DATA_ID;
    frame.can_dlc = CAN_DLC;

    // Big-endian encoding:
    // Byte 0-1: RPM
    // Byte 2-3: vehicle speed
    // Byte 4:   throttle percentage

    frame.data[0] = static_cast<std::uint8_t>((rpm >> 8) & 0xFF);
    frame.data[1] = static_cast<std::uint8_t>(rpm & 0xFF);

    frame.data[2] = static_cast<std::uint8_t>((speed_kmh >> 8) & 0xFF);
    frame.data[3] = static_cast<std::uint8_t>(speed_kmh & 0xFF);

    frame.data[4] = throttle_percent;

    return true;
}

} // namespace

int main()
{
    /*
     * 1. Create a raw CAN socket.
     */
    const int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (socket_fd < 0)
    {
        std::cerr << "ERROR: Failed to create CAN socket: "
                  << std::strerror(errno) << '\n';

        return 1;
    }

    /*
     * 2. Find the CAN interface index.
     */
    ifreq interface_request{};

    std::strncpy(
        interface_request.ifr_name,
        CAN_INTERFACE,
        IFNAMSIZ - 1);

    interface_request.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd, SIOCGIFINDEX, &interface_request) < 0)
    {
        std::cerr << "ERROR: Cannot find CAN interface '"
                  << CAN_INTERFACE << "': "
                  << std::strerror(errno) << '\n';

        close(socket_fd);
        return 1;
    }

    /*
     * 3. Bind the socket to vcan0.
     */
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = interface_request.ifr_ifindex;

    if (bind(
            socket_fd,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address)) < 0)
    {
        std::cerr << "ERROR: Failed to bind CAN socket: "
                  << std::strerror(errno) << '\n';

        close(socket_fd);
        return 1;
    }

    std::cout << "VECU-X CAN sender started\n";
    std::cout << "Interface : " << CAN_INTERFACE << '\n';
    std::cout << "CAN ID    : 0x"
              << std::hex << ENGINE_DATA_ID << std::dec << '\n';
    std::cout << "Period    : "
              << TRANSMISSION_PERIOD_MS << " ms\n\n";

    /*
     * 4. Generate and transmit simulated engine data.
     */
    std::uint16_t rpm = 1000;
    std::uint16_t speed_kmh = 0;
    std::uint8_t throttle_percent = 10;

    while (true)
    {
        can_frame frame{};

        if (!encode_engine_data(
                rpm,
                speed_kmh,
                throttle_percent,
                frame))
        {
            std::cerr << "ERROR: Invalid vehicle data\n";
            break;
        }

        /*
         * 5. Send CAN frame.
         */
        const ssize_t bytes_written =
            write(socket_fd, &frame, sizeof(frame));

        if (bytes_written < 0)
        {
            std::cerr << "ERROR: Failed to send CAN frame: "
                      << std::strerror(errno) << '\n';

            break;
        }

        if (static_cast<std::size_t>(bytes_written) != sizeof(frame))
        {
            std::cerr << "ERROR: Incomplete CAN frame transmission\n";
            break;
        }

        /*
         * Display what was transmitted.
         */
        std::cout
            << "TX  ID=0x"
            << std::hex
            << frame.can_id
            << std::dec
            << "  RPM=" << rpm
            << "  Speed=" << speed_kmh << " km/h"
            << "  Throttle=" << static_cast<unsigned>(throttle_percent)
            << "%\n";

        /*
         * 6. Simple simulated vehicle dynamics.
         *
         * This will later be replaced by the proper
         * vehicle simulator.
         */
        if (speed_kmh < 120)
        {
            ++speed_kmh;
        }

        if (rpm < 3500)
        {
            rpm += 50;
        }

        /*
         * 7. Wait until the next transmission.
         */
        usleep(TRANSMISSION_PERIOD_MS * 1000);
    }

    close(socket_fd);

    return 0;
}