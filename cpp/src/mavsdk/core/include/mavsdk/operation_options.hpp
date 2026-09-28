#pragma once

#include "mavsdk_export.h"

#include <chrono>
#include <functional>

namespace mavsdk {

// The callback must invoke the supplied transmit function exactly once while
// admission is held, or return false without invoking it.
using TransmissionAdmission = std::function<bool(const std::function<void()>&)>;

/**
 * @brief Options shared by blocking MAVSDK operations.
 */
struct MAVSDK_PUBLIC OperationOptions {
    /**
     * @brief Maximum time for the complete operation, including queueing and retries.
     *
     * A timeout must be greater than zero. The operation returns its existing timeout result
     * when this budget expires.
     */
    std::chrono::milliseconds timeout{};

    /**
     * @brief Admit each command transmission, including retransmissions.
     *
     * The callback is copied into queued work and runs at final message
     * delivery. Captures must own any state needed until the operation ends.
     */
    TransmissionAdmission transmission_admission{};
};

} // namespace mavsdk
