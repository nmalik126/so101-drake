#include "so101_lcm.h"

#include "constants.h"

#include "lcmdefs/messages/so101/lcmt_so101_configuration.hpp"

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/common/value.h>

using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace motion_planning {
namespace hardware {

SO101StatusReceiver::SO101StatusReceiver() {
    DeclareAbstractInputPort(
        std::string { input_port_name },
        drake::Value<so101::lcmt_so101_configuration> {}
    );
    DeclareVectorOutputPort(
        std::string { output_port_name },
        constants::SO101_NUM_Q,
        &SO101StatusReceiver::ParseObservation
    );
}

void SO101StatusReceiver::ParseObservation(
    const Context<double>& context,
    BasicVector<double>* output
) const {
    const drake::AbstractValue* input { EvalAbstractInput(context, 0) };
    if (input == nullptr)
        return;
    const auto& observation { 
        input->get_value<so101::lcmt_so101_configuration>()
    };
    auto output_vec { output->get_mutable_value() };
    for (int i { 0 }; i < constants::SO101_NUM_Q; ++i) {
        output_vec(i) = observation.q[i];
    }
}

SO101CommandSender::SO101CommandSender() {
    DeclareVectorInputPort(
        std::string { input_port_name },
        constants::SO101_NUM_Q
    );
    DeclareAbstractOutputPort(
        std::string { output_port_name },
        so101::lcmt_so101_configuration {},
        &SO101CommandSender::FormAction
    );
}

void SO101CommandSender::FormAction(
    const Context<double>& context,
    so101::lcmt_so101_configuration* output
) const {
    const BasicVector<double>* command { EvalVectorInput(context, 0) };
    if (command == nullptr)
        return;
    for (int i { 0 }; i < constants::SO101_NUM_Q; ++i) {
        output->q[i] = command->GetAtIndex(i);
    }
}

} // namespace hardware
} // namespace motion_planning
