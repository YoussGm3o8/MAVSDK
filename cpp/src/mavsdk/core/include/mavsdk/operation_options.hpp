#pragma once

#include "mavsdk_export.h"

#include <chrono>

namespace mavsdk {

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
};

} // namespace mavsdk
