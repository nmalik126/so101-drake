#pragma once

#include "constants.h"

#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <Eigen/Dense>

namespace ob = ompl::base;

namespace helpers {

    inline Eigen::VectorXd state_to_vector(const ob::State *state) {
        const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd v { constants::SO101_NUM_Q };
        for (int i { 0 }; i < constants::SO101_NUM_Q; ++i) {
            v(i) = (*s)[i];
        }
        return v;
    }

}
