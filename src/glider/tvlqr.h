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
            template <typename T>
            using PPoly = trajectories::PiecewisePolynomial<T>;
            /*
             * ref:
             * https://github.com/RobotLocomotion/drake/issues/9013
             * https://github.com/andybarry/flight/blob/master/controllers/matlab_tvlqr/TVLQRControl.m
             * https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab/drake/examples/Glider
             * http://underactuated.csail.mit.edu/underactuated.html?chapter=lqr
             */

            // util functions
            template <typename T>
            static std::unique_ptr<AffineSystem<T>> Linearize(const System<T> &system,
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

            template <typename T>
            static MatrixX<double> unravel(VectorX<T> &x, const int kDim) {
                MatrixX<T> X(kDim, kDim);
                // kDim can be derived from x.size() also as square
                X << Eigen::Map<MatrixX<T>>(x.data(), kDim, kDim);
                return X;
            }

            template <typename T>
            static VectorX<double> ravel(MatrixX<T> &X) {
                return Eigen::Map<VectorX<T>>(X.data(), X.size());
            }


            template<typename T>
            class TimeVaryingLQR final : public LeafSystem<T> {
            public:
                DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeVaryingLQR);

            private:
                const MatrixX<T> &Q, Qf, R;
                const System<T> &system;
                const PPoly<T>& x_des;
                const PPoly<T>& u_des;
                const int kNumStates, kNumInputs;
                const double kTimeSpan;

                int input_index_state_{-1};
                int output_index_control_{-1};

                std::unique_ptr<DenseOutput<double>> s_traj;
            public:
                TimeVaryingLQR(
                        const System<T> &_system,
                        const PPoly<T> &_x_des,
                        const PPoly<T> &_u_des,
                        const MatrixX<T> &_Q,
                        const MatrixX<T> &_R,
                        const MatrixX<T> &_Qf) :
                        LeafSystem<T>(SystemTypeTag<TimeVaryingLQR>{}),
                                      system(_system), Q(_Q), Qf(_Qf), R(_R), x_des(_x_des),
                                      u_des(_u_des),
                                      kTimeSpan(_x_des.end_time()), kNumStates(_Q.rows()), kNumInputs(_R.rows()) {

                    Eigen::LLT<MatrixX<T>> R_cholesky(R);
                    if (R_cholesky.info() != Eigen::Success) {
                        throw std::runtime_error("R must be positive definite");
                    }
                    // todo: check needs for Q too?

                    //this->DeclareContinuousState(kNumInputs);

                    output_index_control_ = this->DeclareVectorOutputPort(
                            BasicVector<T>(kNumInputs),
                            &TimeVaryingLQR<T>::CalcControl
                    ).get_index();

                    input_index_state_ = this->DeclareInputPort(kVectorValued, kNumStates).get_index();

                    computeSTrajectory();
                }

/*
                template <typename U>
                TimeVaryingLQR(const TimeVaryingLQR<U>& other)
                        : TimeVaryingLQR(other.system, other.x_des, other.u_des, other.Q, other.R, other.Qf) {}
*/


                /**
                 * Returns the output port for computed control.
                 */
                const OutputPort<T> &get_output_port() const {
                    return System<T>::get_output_port(output_index_control_);
                }

                /**
                 * Returns the input port for the estimated state.
                 */
                const InputPort<T>& get_input_port() const {
                    return System<T>::get_input_port(input_index_state_);
                }

                void DoCalcTimeDerivatives(const systems::Context<T>& context,
                                           systems::ContinuousState<T> *derivatives) const override {}


                    private:
                T revTime(const T& t) const{
                    return revTime(t, kTimeSpan);
                }

                static T revTime(const T& t, const T& kTimeSpan) {
                    return std::max<T>(0.0, kTimeSpan - t);
                }


            public:
                void CalcControl(const Context<T> &context,
                                                   BasicVector<T> *control) const {
                    const Eigen::VectorBlock<const VectorX<T>> x = get_input_port().Eval(context);
                    const T& t = context.get_time();
                    const VectorX<T> x0 = x_des.value(t);
                    // State error.
                    const VectorX<T> x_bar = x - x0;

                    const T rev_t = this->revTime(t);
                    VectorX<double> s = s_traj->Evaluate(rev_t);
                    MatrixX<double> S = unravel(s, kNumStates);

                    auto affine_system = Linearize<T>(system, x_des.value(t), u_des.value(t));

                    VectorX<double> u_delta = -R.inverse() * affine_system->B().transpose() * S * x_bar;

                    control->SetFromVector(u_des.value(t) + u_delta);
                }
                private:

                void computeSTrajectory() {
                    // PPoly xdot_traj = x_des.derivative();
                    MatrixX<T> S_f = Qf;
                    VectorX<T> kDefaultInitialState = ravel(S_f);
                    const T kDefaultInitialTime = 0.0;
                    VectorX<T> kDefaultParams(0);

                    // todo: why typename?
                    const typename InitialValueProblem<T>::SpecifiedValues kDefaultValues(
                            kDefaultInitialTime, kDefaultInitialState, kDefaultParams
                    );

                    InitialValueProblem<T> ivp(sDot(system, Q, R, x_des, u_des), kDefaultValues);
                    //VectorX<double> s_cur = ivp.Solve(kTimeSpan);
                    s_traj = ivp.DenseSolve(kTimeSpan);
                    //debugRes();
                }

            private:

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

                        // todo: handle sign
                        Sdot = S * A + A.transpose() * S - S * B * Ri * B.transpose() * S + Q;
                        VectorX<T> sdot = ravel(Sdot);
                        return sdot;
                    }
                };
            };

        }
        namespace scalar_conversion {
            template <>
            struct Traits<controllers::TimeVaryingLQR> : public NonSymbolicTraits {};
        }
    }
}

