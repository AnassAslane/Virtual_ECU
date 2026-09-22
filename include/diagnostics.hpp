#pragma once

#include "safety.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace vecu {

struct DiagnosticTroubleCode {
    uint32_t code;
    FaultCode fault;
    bool active;
};

class Diagnostics {
public:
    Diagnostics();

    void record_fault(FaultCode fault);

    void clear_fault(FaultCode fault);

    std::vector<DiagnosticTroubleCode> get_dtcs() const;

    std::vector<uint8_t> process_request(
        const std::vector<uint8_t>& request);

private:
    std::map<FaultCode, DiagnosticTroubleCode> dtcs_;

    std::vector<uint8_t> read_dtc_information();

    std::vector<uint8_t> clear_diagnostic_information();

    static uint32_t fault_to_dtc(FaultCode fault);
};

} // namespace vecu