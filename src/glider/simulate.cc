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
#include "tvlqr.h"
#include "iostream"
#include "fstream"

namespace drake {
    using systems::DiagramBuilder;
    using systems::Simulator;
    using systems::Context;
    using systems::ContinuousState;
    using systems::VectorBase;
    using systems::controllers::TimeVaryingLQR;

    namespace examples {
        namespace glider {
            typedef trajectories::PiecewisePolynomial<double> PPoly;
            typedef std::pair<PPoly, PPoly> TrajPair;
            namespace {

                /*
                 * ref:
                 * https://github.com/RobotLocomotion/drake/blob/last_sha_with_original_matlab/drake/examples/Glider/runDircolPerching.m
                 */
                TrajPair run_dircol(const Glider<double>& glider) {
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

                    auto context = glider.CreateDefaultContext();
                    systems::trajectory_optimization::DirectCollocation dircol(
                            &glider, *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);

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
                        throw result.get_solver_id().name()
                                  + " Failed to solve optimization for the perching trajectory";
                    }

                    PPoly x_des = dircol.ReconstructStateTrajectory(result);
                    PPoly u_des = dircol.ReconstructInputTrajectory(result);

                    return TrajPair(x_des, u_des);
                }

                /*
                 * ref:
                 * https://github.com/RobotLocomotion/drake/blob/last_sha_with_original_matlab/drake/examples/Glider/runLQR.m
                 * https://github.com/RobotLocomotion/drake/blob/last_sha_with_original_matlab/drake/examples/Glider/GliderLQR.m
                 * quadrotor_plant.cc
                 */
                std::unique_ptr<drake::systems::controllers::TimeVaryingLQR<double>> StabilizingLQRController(
                const Glider<double>* glider, const PPoly& x_des,const PPoly& u_des){
                    const MatrixX<double> Qf{(VectorX<double>(7) << 1/0.0025, 1/0.0025, 1/9.0, 1/9.0, 1, 1, 1/9.0).finished().asDiagonal()};
                    const MatrixX<double> Q{(VectorX<double>(7) << 10,10,10,1,1,1,1).finished().asDiagonal()};
                    const MatrixX<double> R{(MatrixX<double>(1,1) << 0.1).finished()};

                    // auto tvlqr_context = glider->CreateDefaultContext();
                    return std::make_unique<TimeVaryingLQR<double>>(*glider, x_des, u_des, Q, R, Qf);
                }

                void simulate(const PPoly& x_des,const PPoly& u_des){
                    DiagramBuilder<double> builder;

                    auto glider = builder.AddSystem(std::make_unique<Glider<double>>());
                    glider->set_name("glider");

                    auto controller = builder.AddSystem(StabilizingLQRController(
                            glider, x_des, u_des));
                    controller->set_name("controller");

                    builder.Connect(glider->get_output_port(0), controller->get_input_port());
                    builder.Connect(controller->get_output_port(), glider->get_input_port(0));

                    auto diagram = builder.Build();
                    Simulator<double> simulator(*diagram);

                    const VectorX<double> kXi0(x_des.value(0));
                    simulator.get_mutable_context()
                            .get_mutable_continuous_state_vector()
                            .SetFromVector(kXi0);

                    simulator.Initialize();

                    const double kTimeSpan(x_des.end_time());
                    simulator.AdvanceTo(kTimeSpan);

                    const Context<double>& sim_context = simulator.get_context();
                    const VectorX<double>& Xf(sim_context.get_continuous_state_vector());

                    const VectorX<double> kXf0(x_des.value(x_des.end_time()));

                    std::cout<<"Final state is off by: "<<(Xf - kXf0).norm()<<std::endl;
                }

                /*
                 * tutorial on unique_ptr: https://thispointer.com/c11-unique_ptr-tutorial-and-examples/
                 */
                void do_main(){
                    Glider<double> glider;;
                    glider.set_name("glider");

                    // get optimal state and input trajectories to reach goal
                    auto tr_des = run_dircol(glider);

                    simulate(tr_des.first, tr_des.second);
                }
            }

        }
    }
}



int main(int argc, char *argv[]) {
    gflags::SetUsageMessage(
            "Trajectory optimization for perching glider.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();
    drake::examples::glider::do_main();
    return 0;
}