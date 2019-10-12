#include "glider.h"
#include "gflags/gflags.h"
#include <drake/common/text_logging_gflags.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/solve.h"
#include "limits"
#include "drake/manipulation/util/trajectory_utils.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/controllers/linear_model_predictive_controller.h"

// taken from : https://github.com/RobotLocomotion/drake/blob/last_sha_with_original_matlab/drake/examples/Glider/runDircolPerching.m


namespace drake {
    namespace examples {
        namespace glider {
            typedef trajectories::PiecewisePolynomial<double> PPoly;
            namespace {

                int run_dircol() {
                    systems::DiagramBuilder<double> builder;
                    auto glider = std::make_unique<Glider<double>>();
                    //Glider<double> glider;
                    glider->set_name("glider");
                    auto context = glider->CreateDefaultContext();

                    const int kNumTimeSamples = 41;
                    const double kMinimumTimeStep = 0.01, kMaximumTimeStep = 2;

                    const double kPhiLimitL = -M_PI / 3, kPhiLimitU = M_PI / 6, kPhiDotLimit = 13;
                    const double kInf = std::numeric_limits<double>::infinity();

                    VectorX<double> takeoff_state(7),
                            traj_state_l(7), traj_state_u(7),
                            land_state(7), land_state_l(7), land_state_u(7),
                            eps(7);

                    // set initial state
                    takeoff_state << -3.5, 0.1, 0, 0, 7, 0, 0;

                    traj_state_l << -4, -1, -M_PI/2, kPhiLimitL, -kInf, -kInf, -kInf;
                    traj_state_u << 1, 1, M_PI/2, kPhiLimitU, kInf, kInf, kInf;

                    land_state << 0, 0, M_PI / 4, 0, 0, -0.5, -0.5;
                    land_state_l << 0, 0, M_PI / 8, -kInf, -2, -2, -kInf;
                    land_state_u << 0, 0, M_PI / 2, kInf, 2, 2, kInf;

                    const double fly_time_init = 1;//abs(takeoff_state(0)) / takeoff_state(4);

                    const double R = 100;

                    systems::trajectory_optimization::DirectCollocation dircol(
                            glider.get(), *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);

                    dircol.AddEqualTimeIntervalsConstraints();

                    //input constraints
                    auto u = dircol.input();
                    dircol.AddConstraintToAllKnotPoints(u(0) >= -kPhiDotLimit);
                    dircol.AddConstraintToAllKnotPoints(u(0) <= kPhiDotLimit);

                    // state constraints
                    auto state = dircol.state();
                    auto i_state = dircol.initial_state();
                    auto f_state = dircol.final_state();
                    dircol.AddConstraintToAllKnotPoints(traj_state_l <=  state);
                    dircol.AddConstraintToAllKnotPoints(state <= traj_state_u);
                    //dircol.AddBoundingBoxConstraint(traj_state_l, traj_state_u, state);

                    dircol.AddLinearConstraint(i_state == takeoff_state);

                    // set final box target
                    dircol.AddBoundingBoxConstraint(land_state_l,land_state_u,f_state);

                    // bound running time
                    dircol.AddDurationBounds(0, 10);

                    dircol.AddRunningCost(u.transpose() * R * u);

                    VectorX<double> q(7);
                    q << 10,10,1,10,1,1,1;
                    MatrixX<double> Q(7,7);
                    Q = q.asDiagonal();
                    auto x_err = f_state - land_state;
                    dircol.AddFinalCost((Q * x_err).cwiseProduct(x_err).sum());

                    const double eps_d = std::numeric_limits<double>::epsilon();
                    eps << eps_d, eps_d, eps_d, 0, 0, 0, 0;

                    auto traj_init = PPoly::FirstOrderHold({0, fly_time_init},
                                                           {takeoff_state+eps, land_state});

                    dircol.SetInitialTrajectory(PPoly(), traj_init);
                    const auto result = solvers::Solve(dircol);

                    if (!result.is_success()) {
                        std::cerr << result.get_solver_id()
                                  << " Failed to solve optimization for the perching trajectory"
                                  << std::endl;
                        return 1;
                    }

                    const PPoly pp_xtraj = dircol.ReconstructStateTrajectory(result);
                    //drake::manipulation::PiecewiseCubicTrajectory<double> pc_traj(pp_xtraj);
                    for(auto time: pp_xtraj.get_segment_times()){
                        auto _state = pp_xtraj.value(time);
                        std::cout<<time<<"\t"<<_state(0)<<"\t"<<_state(1)<<std::endl;
                    }

                    return 0;
                }

                // taken from: https://github.com/RobotLocomotion/drake/blob/last_sha_with_original_matlab/drake/examples/Glider/runLQR.m
                // and https://github.com/RobotLocomotion/drake/blob/last_sha_with_original_matlab/drake/examples/Glider/GliderLQR.m
                int run_lqr(){
                    // u_traj and x_traj should be avail.
                    systems::DiagramBuilder<double> builder;
                    auto glider = std::make_unique<Glider<double>>();
                    //Glider<double> glider;
                    glider->set_name("glider");
                    auto context = glider->CreateDefaultContext();

                    VectorX<double> qf(7);
                    qf << 1/0.0025, 1/0.0025, 1/9, 1/9, 1, 1, 1/9;
                    Eigen::MatrixXd<double> Qf = qf.asDiagonal();

                    VectorX<double> q(7);
                    q << 10,10,10,1,1,1,1;
                    Eigen::MatrixXd Q(7,7);
                    Q = q.asDiagonal();
                    Eigen::MatrixXd R(1,1);
                    R << 0.1;
                    const double timePeriod = 0.01, timeHorizon = 1;
                    systems::controllers::LinearModelPredictiveController<double> lmpc(glider.get(),*context, Q, R, timePeriod, timeHorizon);
                    auto controller = builder.AddSystem(lmpc);
                }
            }

        }
    }
}

namespace drake{
    namespace systems{
        namespace controllers{
            /// The return type of ConnectController.
            template <typename T>
            struct ConnectResult {
                /// The feed forward control input.
                const InputPort<T>& control_input_port;
                /// The feedback state input.
                const InputPort<T>& state_input_port;
            };

            template <typename T>
            static ConnectResult<T> ConnectController(
                    const InputPort<T>& plant_input,
                    const OutputPort<T>& plant_output,
                    const MatrixX<double>& feedback_selector, const Eigen::VectorXd& Kp,
                    const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd,
                    DiagramBuilder<T>* builder) {
                auto controller = builder->template AddSystem<PidController<T>>(
                        feedback_selector,
                                Kp, Ki, Kd);

                auto plant_input_adder =
                builder->template AddSystem<Adder<T>>(2, plant_input.size());

                builder->Connect(plant_output, controller->get_input_port_estimated_state());
                builder->Connect(controller->get_output_port_control(),
                                 plant_input_adder->get_input_port(0));
                builder->Connect(plant_input_adder->get_output_port(), plant_input);

                return ConnectResult<T>{
                        plant_input_adder->get_input_port(1),
                        controller->get_input_port_desired_state()};
            }

            }

    }
}






int main(int argc, char *argv[]) {
    gflags::SetUsageMessage(
            "Trajectory optimization for perching glider.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();
    return drake::examples::glider::run_dircol();
}