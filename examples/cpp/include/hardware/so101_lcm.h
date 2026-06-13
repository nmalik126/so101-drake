#pragma once

#include "messages/so101/lcmt_so101_configuration.hpp"

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/basic_vector.h>

using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace motion_planning {
namespace hardware {

class SO101StatusReceiver final : public LeafSystem<double> {
public:
    SO101StatusReceiver();

private:
    void ParseObservation(
        const Context<double>& context,
        BasicVector<double>* output
    ) const;

    static constexpr std::string_view input_port_name { "lcmt_so101_configuration" };
    static constexpr std::string_view output_port_name { "position_measured" };
};

class SO101CommandSender final : public LeafSystem<double> {
public:
    SO101CommandSender();
    
private:
    void FormAction(
        const Context<double>& context,
        so101::lcmt_so101_configuration* output
    ) const;

    static constexpr std::string_view input_port_name { "position" };
    static constexpr std::string_view output_port_name { "lcmt_command" };
};

} // namespace hardware
} // namespace motion_planning
