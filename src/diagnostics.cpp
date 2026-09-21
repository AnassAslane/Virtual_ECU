#include "diagnostics.hpp"

namespace vecu {

Diagnostics::Diagnostics()
{
    dtcs_.emplace(
        FaultCode::CAN_COMMUNICATION,
        DiagnosticTroubleCode{
            0xC001,
            FaultCode::CAN_COMMUNICATION,
            false
        });

    dtcs_.emplace(
        FaultCode::WATCHDOG_TIMEOUT,
        DiagnosticTroubleCode{
            0xC002,
            FaultCode::WATCHDOG_TIMEOUT,
            false
        });

    dtcs_.emplace(
        FaultCode::INVALID_ENGINE_DATA,
        DiagnosticTroubleCode{
            0xC003,
            FaultCode::INVALID_ENGINE_DATA,
            false
        });

    dtcs_.emplace(
        FaultCode::SENSOR_TIMEOUT,
        DiagnosticTroubleCode{
            0xC004,
            FaultCode::SENSOR_TIMEOUT,
            false
        });

    dtcs_.emplace(
        FaultCode::OVERSPEED,
        DiagnosticTroubleCode{
            0xC005,
            FaultCode::OVERSPEED,
            false
        });
}

void Diagnostics::record_fault(FaultCode fault)
{
    auto it = dtcs_.find(fault);

    if (it != dtcs_.end()) {
        it->second.active = true;
    }
}

void Diagnostics::clear_fault(FaultCode fault)
{
    auto it = dtcs_.find(fault);

    if (it != dtcs_.end()) {
        it->second.active = false;
    }
}

std::vector<DiagnosticTroubleCode>
Diagnostics::get_dtcs() const
{
    std::vector<DiagnosticTroubleCode> result;

    for (const auto& [fault, dtc] : dtcs_) {
        if (dtc.active) {
            result.push_back(dtc);
        }
    }

    return result;
}

std::vector<uint8_t>
Diagnostics::process_request(
    const std::vector<uint8_t>& request)
{
    if (request.empty()) {
        return {};
    }

    /*
     * Minimal UDS-inspired services:
     *
     * 0x10  Diagnostic Session Control
     * 0x19  Read DTC Information
     * 0x14  Clear Diagnostic Information
     */

    switch (request[0]) {

        case 0x10:
            if (request.size() >= 2) {
                return {
                    0x50,
                    request[1]
                };
            }
            break;

        case 0x19:
            return read_dtc_information();

        case 0x14:
            return clear_diagnostic_information();

        default:
            /*
             * UDS negative response:
             *
             * 0x7F = negative response
             * original SID
             * 0x11 = service not supported
             */
            return {
                0x7F,
                request[0],
                0x11
            };
    }

    return {
        0x7F,
        request[0],
        0x13
    };
}

std::vector<uint8_t>
Diagnostics::read_dtc_information()
{
    std::vector<uint8_t> response;

    /*
     * Positive response to 0x19.
     */

    response.push_back(0x59);

    for (const auto& [fault, dtc] : dtcs_) {

        if (!dtc.active) {
            continue;
        }

        response.push_back(
            static_cast<uint8_t>(
                (dtc.code >> 8) & 0xFF));

        response.push_back(
            static_cast<uint8_t>(
                dtc.code & 0xFF));

        /*
         * Status:
         * 0x01 = DTC currently active.
         */
        response.push_back(0x01);
    }

    return response;
}

std::vector<uint8_t>
Diagnostics::clear_diagnostic_information()
{
    for (auto& [fault, dtc] : dtcs_) {
        dtc.active = false;
    }

    return {
        0x54
    };
}

uint32_t Diagnostics::fault_to_dtc(FaultCode fault)
{
    auto value =
        static_cast<uint16_t>(fault);

    return 0xC000U + (value & 0xFFU);
}

} // namespace vecu