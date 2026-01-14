#pragma once

#include <array>
#include <string>
#include <vector>

namespace ur_motion {

struct MotionResult {
    bool        success;
    std::string message;
};

// move_j_backend.hpp
class MoveJBackend {
   public:
    virtual ~MoveJBackend() = default;

    virtual MotionResult moveJ(
        const std::vector<double>& joints, double vel) = 0;

    virtual void moveCancel() = 0;
};

// move_l_backend.hpp
class MoveLBackend {
   public:
    virtual ~MoveLBackend() = default;

    virtual MotionResult moveL(
        const std::array<double, 16>& T, double vel) = 0;

    virtual void moveCancel() = 0;
};

}  // namespace ur_motion
