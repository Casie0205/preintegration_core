#pragma once

#include <cstddef>

#include "preintegration_core/types.hpp"

namespace preintegration_core {

class Preintegrator {
public:
    explicit Preintegrator(PreintegrationOptions options = {});

    void clear();
    void seedPreviousIncrement(const ImuIncrement& increment);
    void add(const ImuIncrement& increment);

    const PreintegratedDelta& delta() const { return delta_; }
    const PreintegrationOptions& options() const { return options_; }
    std::size_t sampleCount() const { return sample_count_; }

private:
    PreintegrationOptions options_;
    PreintegratedDelta delta_;
    ImuIncrement previous_increment_{};
    bool has_previous_increment_{false};
    std::size_t sample_count_{0};
};

// Minimal gravity composition without Earth rotation or Coriolis. This is a
// convenience for comparing pure integration; it is not a factor or optimizer.
NavState compose(
    const NavState& initial,
    const PreintegratedDelta& delta,
    const Eigen::Vector3d& gravity_ned_mps2);

}  // namespace preintegration_core
