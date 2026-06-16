#pragma once

#include <string_view>
#include <numbers>

using std::numbers::pi;

namespace constants {

    namespace model_paths {

        inline constexpr std::string_view SO101 { "/home/noor/SO-ARM100/Simulation/SO101/so101_new_calib_urdf_drake_hydro.urdf" };
        inline constexpr std::string_view SO101_OBJ { "/home/noor/SO-ARM100/Simulation/SO101/so101_object.urdf" };
        inline constexpr std::string_view MAT { "/home/noor/SO-ARM100/Simulation/assets/mat.sdf" };
        inline constexpr std::string_view BOX { "/home/noor/SO-ARM100/Simulation/assets/box.sdf" };
        inline constexpr std::string_view BIN { "/home/noor/SO-ARM100/Simulation/assets/bin_small.sdf" };
        inline constexpr std::string_view BIN_CLR { "/home/noor/SO-ARM100/Simulation/assets/bin_small_clearance.sdf" };

    } // namespace model_paths

    namespace transforms {

        namespace X_MAT_SO101 {

            inline constexpr double T[3] { 0, -0.1775, 0.0074 };
            inline constexpr double R[3] { 0, 0, pi/2 };

        } // namespace X_MAT_SO101

        namespace X_MAT_BOX {

            inline constexpr double T[3] { -0.075, 0.075, 0.02 };
            inline constexpr double R[3] { 0, 0, pi/2 };

        } // namespace X_MAT_BOX

        namespace X_MAT_BOXGOAL {

            // inline constexpr double T[3] { 0.075, 0.025, 0.02 };
            inline constexpr double T[3] { 0.1, 0, 0.02 };
            inline constexpr double R[3] { 0, 0, -pi/2 };

        } // namespace X_MAT_BOXGOAL

        namespace X_MAT_BIN {

            inline constexpr double T[3] { 0.1, 0, 0.006 };
            inline constexpr double R[3] { 0, 0, 0 };

        } // namespace X_MAT_BIN

    } // namespace transforms

    inline constexpr int SO101_NUM_Q { 6 };
    inline constexpr double SO101_Q_INIT[SO101_NUM_Q] { 0, -1.822, 1.55, 0.906, 0, 0 };

    inline constexpr int TRAJOPT_N_WAYPOINTS { 7 };

} // namespace constants
