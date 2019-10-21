//
// Created by Akshit Jain on 09/10/19.
//
#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include "drake/systems/primitives/linear_system.h"
#include "eigen3/Eigen/Core"
#include "drake/systems/analysis/initial_value_problem-inl.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/default_scalars.h"


namespace drake {
    namespace systems {
        namespace controllers {
            template<typename T>
            using PPoly = trajectories::PiecewisePolynomial<T>;
            /*
             * ref:
             * https://github.com/RobotLocomotion/drake/issues/9013
             * https://github.com/andybarry/flight/blob/master/controllers/matlab_tvlqr/TVLQRControl.m
             * https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab/drake/examples/Glider
             * http://underactuated.csail.mit.edu/underactuated.html?chapter=lqr
             */

            // util functions
            template<typename T>
            static std::unique_ptr<AffineSystem<double>> Linearize(const System<T> &system,
                                                                   const VectorX<T> &x0,
                                                                   const VectorX<T> &u0) {
                auto lin_context = system.CreateDefaultContext();
                // todo: set time too?
                lin_context->SetContinuousState(x0);
                lin_context->FixInputPort(0, u0);
                auto affine_system = FirstOrderTaylorApproximation(system, *lin_context,
                                                                   InputPortSelection::kUseFirstInputIfItExists,
                                                                   OutputPortSelection::kNoOutput);
                return affine_system;
            }

            template<typename T>
            static MatrixX<double> unravel(VectorX<T> &x, const int kDim) {
                MatrixX<T> X(kDim, kDim);
                // kDim can be derived from x.size() also as square
                X << Eigen::Map<MatrixX<T>>(x.data(), kDim, kDim);
                return X;
            }

            template<typename T>
            static VectorX<double> ravel(MatrixX<T> &X) {
                return Eigen::Map<VectorX<T>>(X.data(), X.size());
            }

            // todo: change return type to TimeVaryingAffineSystem as LinearQuadraticRegulator returns AffineSystem
            class TimeVaryingLQR final : public LeafSystem<double> {
            public:
                DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeVaryingLQR);

            private:
                const MatrixX<double> &Q, Qf, R;
                const System<double> &system;
                // todo: replace by input ports for state and input trajectory ref: PidController
                const PPoly<double> &x_des;
                const PPoly<double> &u_des;
                const int kNumStates, kNumInputs;
                const double kTimeSpan;

                int input_index_state_{-1};
                int output_index_control_{-1};

                std::unique_ptr<DenseOutput<double>> s_traj;
            public:
                TimeVaryingLQR(
                        const System<double> &_system,
                        const PPoly<double> &_x_des,
                        const PPoly<double> &_u_des,
                        const MatrixX<double> &_Q,
                        const MatrixX<double> &_R,
                        const MatrixX<double> &_Qf) :
                        system(_system), Q(_Q), Qf(_Qf), R(_R), x_des(_x_des),
                        u_des(_u_des),
                        kTimeSpan(_x_des.end_time()), kNumStates(_Q.rows()), kNumInputs(_R.rows()) {

                    Eigen::LLT<MatrixX<double>> R_cholesky(R);
                    if (R_cholesky.info() != Eigen::Success) {
                        throw std::runtime_error("R must be positive definite");
                    }
                    // todo: check needs for Q too?

                    //this->DeclareContinuousState(kNumInputs);

                    output_index_control_ = this->DeclareVectorOutputPort("corrected_control",
                                                                          BasicVector<double>(kNumInputs),
                                                                          &TimeVaryingLQR::CalcControl,
                                                                          {this->all_state_ticket()}
                    ).get_index();

                    input_index_state_ = this->DeclareInputPort(kVectorValued, kNumStates).get_index();

                    computeSTrajectory();
                }


/*                template <typename U>
                TimeVaryingLQR(const TimeVaryingLQR<U>& other)
                        : TimeVaryingLQR(other.system, other.x_des, other.u_des, other.Q, other.R, other.Qf) {}*/


                /**
                 * Returns the output port for computed control.
                 */
                const OutputPort<double> &get_output_port() const {
                    return System<double>::get_output_port(output_index_control_);
                }

                /**
                 * Returns the input port for the estimated state.
                 */
                const InputPort<double> &get_input_port() const {
                    return System<double>::get_input_port(input_index_state_);
                }

/*
                // todo: implement this
                void DoCalcTimeDerivatives(const systems::Context<double>& context,
                                           systems::ContinuousState<double> *derivatives) const override {}
*/


            private:
                double revTime(const double &t) const {
                    return revTime(t, kTimeSpan);
                }

                static double revTime(const double &t, const double &kTimeSpan) {
                    return std::max<double>(0.0, kTimeSpan - t);
                }


            public:
                void CalcControl(const Context<double> &context,
                                 BasicVector<double> *control) const {
                    const Eigen::VectorBlock<const VectorX<double>> x = get_input_port().Eval(context);
                    const double &t = context.get_time();
                    const VectorX<double> x0 = x_des.value(t);
                    // State error.
                    const VectorX<double> x_bar = x - x0;

                    const double rev_t = this->revTime(t);
                    VectorX<double> s = s_traj->Evaluate(rev_t);
                    MatrixX<double> S = unravel(s, kNumStates);

                    auto affine_system = Linearize<double>(system, x_des.value(t), u_des.value(t));

                    auto K = R.inverse() * affine_system->B().transpose() * S;
                    VectorX<double> u_delta = -K * x_bar;

                            SPDLOG_DEBUG(drake::log(), "time : {}\tu_des : {}\tu_delta : {}", t, u_des.value(t),
                                         u_delta);

                    control->SetFromVector(u_des.value(t) + u_delta);
                }

            private:

                void computeSTrajectory() {
                    // PPoly xdot_traj = x_des.derivative();
                    MatrixX<double> S_f = Qf;
                    VectorX<double> kDefaultInitialState = ravel(S_f);
                    const double kDefaultInitialTime = 0.0;
                    VectorX<double> kDefaultParams(0);

                    const InitialValueProblem<double>::SpecifiedValues kDefaultValues(
                            kDefaultInitialTime, kDefaultInitialState, kDefaultParams
                    );

                    InitialValueProblem<double> ivp(sDot<double>(system, Q, R, x_des, u_des), kDefaultValues);
                    //VectorX<double> s_cur = ivp.Solve(kTimeSpan);
                    s_traj = ivp.DenseSolve(kTimeSpan);
                    //debugRes();
                }

            private:
                    // todo: Should we use lambda instead? trade-off between readability and compactness
                template<typename T>
                class sDot {
                private:
                    const MatrixX<T> &Q, R;
                    const System<T> &system;
                    const PPoly<T> &x_des, u_des;
                    const int kNumStates;
                    const double kTimeSpan;

                public:
                    sDot(
                            const System<T> &system,
                            const MatrixX<T> &Q,
                            const MatrixX<T> &R,
                            const PPoly<T> &x_des,
                            const PPoly<T> &u_des) :
                            system(system), Q(Q), R(R), kNumStates(Q.cols()), x_des(x_des), u_des(u_des),
                            kTimeSpan(x_des.end_time()) {
                    }

                    VectorX<T> operator()(const T &t, const VectorX<T> &_s, const VectorX<T> &k) const {
                        unused(k);

                        VectorX<T> s(_s);
                        // we do not need to solve for S_0 as u* is independent of it
                        MatrixX<T> S(kNumStates, kNumStates), Sdot(kNumStates, kNumStates);
                        S = unravel(s, kNumStates);

                        // we have to reverse the desired trajectory to use ivp by converting backward integration to forward in time
                        // todo: ideally reverse the trajectory itself, as top level handling is more graceful
                        const double rev_t = TimeVaryingLQR::revTime(t, kTimeSpan);

                        auto affine_system = Linearize<T>(system, x_des.value(rev_t), u_des.value(rev_t));

                        const MatrixX<T> A = affine_system->A();
                        const MatrixX<T> B = affine_system->B();
                        const MatrixX<T> Ri = R.inverse();

                        // sign is changed as we are converting backward to forward integration
                        Sdot = S * A + A.transpose() * S - S * B * Ri * B.transpose() * S + Q;
                        VectorX<T> sdot = ravel(Sdot);
                        return sdot;
                    }
                };
            };
        }
/*        namespace scalar_conversion {
            template <>
            struct Traits<controllers::TimeVaryingLQR> : public NonSymbolicTraits {};
        }*/
    }
}

