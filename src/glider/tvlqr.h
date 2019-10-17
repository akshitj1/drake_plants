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

namespace drake {
    namespace systems {
        namespace controllers {
            /* helper functions */
            template <typename T>
            int sgn(T val) {
                return (T(0) < val) - (val < T(0));
            }

            typedef trajectories::PiecewisePolynomial<double> PPoly;
            /*
             * ref:
             * https://github.com/RobotLocomotion/drake/issues/9013
             * https://github.com/andybarry/flight/blob/master/controllers/matlab_tvlqr/TVLQRControl.m
             * https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab/drake/examples/Glider
             * http://underactuated.csail.mit.edu/underactuated.html?chapter=lqr
             */
            class TimeVaryingLQR {
            private:
                const Eigen::MatrixXd& Q, Qf, R;
                const System<double>& system;
                const Context<double>& context;
                const PPoly& x_des, u_des;
                const int kNumStates, kNumInputs;
                const double kTimeSpan;

                std::unique_ptr<DenseOutput<double>> s_traj;
            public:

                TimeVaryingLQR(
                        const System<double>& _system,
                        const drake::systems::Context<double>& context,
                        const PPoly& _x_des,
                        const PPoly& _u_des,
                        const Eigen::MatrixXd& _Q,
                        const Eigen::MatrixXd& _R,
                        const Eigen::MatrixXd& _Qf):
                        system(_system), context(context), Q(_Q), Qf(_Qf), R(_R), x_des(_x_des), u_des(_u_des), kTimeSpan(_x_des.end_time()), kNumStates(_Q.rows()), kNumInputs(_R.rows()){
                    // Descrption of the linearized plant model.
                    //std::unique_ptr<LinearSystem<double>> linear_model_ = Linearize(model, base_context);

                    Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
                    if (R_cholesky.info() != Eigen::Success) {
                        throw std::runtime_error("R must be positive definite");
                    }
                    // todo: check needs for Q too?
                }


                VectorX<double> GetControl(const double& t){
                    MatrixX<double> S2(kNumStates, kNumStates);
                    VectorX<double> s1(kNumStates);
                    const double rev_t = revTime(t);
                    VectorX<double> s = s_traj->Evaluate(rev_t);
                    sDynamicsOde<double>::unflattenXtoS(s, kNumStates, S2, s1);

                    auto u0 = u_des.value(t);
                    auto x0 = x_des.value(t);

                    auto affine_system = sDynamicsOde<double>::Linearize(system, context, x0, u0);

                    VectorX<double> u_opt = u0 - R.inverse() * affine_system->B().transpose() * (S2 * x0 + 0.5 * s1);
                    return u_opt;
                }

                void debugRes(){
                    const double kTimeStep = kTimeSpan/20;
                    for(double t=0; t <= kTimeSpan;t+=kTimeStep){
                        VectorX<double> u_des_t = u_des.value(t);
                        VectorX<double> u_corr_t = GetControl(t);
                        std::cout<<t<<"\t"<<u_des_t<<"\t"<<u_corr_t<<std::endl;
                    };
                }

            private:
                double revTime(const double& t){
                    return revTime(t, kTimeSpan);
                }

                static double revTime(const double& t, const double& kTimeSpan){
                    return std::max(0.0, kTimeSpan - t);
                }

            public:
                VectorX<double> CalcControl(){
                    // PPoly xdot_traj = x_des.derivative();
                    MatrixX<double> S2_f = Qf;
                    VectorX<double> s1_f = -2 * Qf * x_des.value(kTimeSpan);
                    VectorX<double> kDefaultInitialState(S2_f.size()+s1_f.size());
                    sDynamicsOde<double>::flattenStoX(S2_f, s1_f, kDefaultInitialState);

                    const double kDefaultInitialTime = 0;
                    VectorX<double> kDefaultParams(0);
                    InitialValueProblem<double>::SpecifiedValues kDefaultValues(
                            kDefaultInitialTime, kDefaultInitialState, kDefaultParams
                    );

                    InitialValueProblem<double> ivp(sDynamicsOde<double>(system, context, Q, R, x_des, u_des), kDefaultValues);
                    //VectorX<double> s_cur = ivp.Solve(kTimeSpan);
                    s_traj = ivp.DenseSolve(kTimeSpan);
                    debugRes();
                    VectorX<double> s_cur = s_traj->Evaluate(kTimeSpan);
                    return s_cur;
                }

            private:

                template<typename T>
                class sDynamicsOde {
                private:
                    const MatrixX<double> &Q, R;
                    const System<double>& system;
                    const Context<double>& context;
                    const PPoly &x_des, u_des;
                    const int kNumStates;
                    const double kTimeSpan;

                public:
                    sDynamicsOde(
                            const System<double>& system,
                            const Context<double>& context,
                            const MatrixX<double> &Q,
                            const MatrixX<double> &R,
                            const PPoly &x_des,
                            const PPoly &u_des) :
                            system(system), context(context), Q(Q), R(R), kNumStates(Q.cols()), x_des(x_des), u_des(u_des), kTimeSpan(x_des.end_time())  {
                    }

                    VectorX<T> operator()(const T &t,const VectorX<T> &_s, const VectorX<T> &k) const {
                        unused(k);

                        VectorX<T> s(_s);
                        // we do not need to solve for S_0 as u* is independent of it
                        MatrixX<double> S2(kNumStates, kNumStates), S2_dot(kNumStates, kNumStates);
                        VectorX<double> s1(kNumStates), s1_dot(kNumStates);
                        unflattenXtoS(s, kNumStates, S2, s1);

                        // we have to reverse the desired trajectory to use ivp by converting backward integration to forward in time
                        // todo: ideally reverse the trajectory itself, as top level handling is more gracefull
                        const double rev_t = TimeVaryingLQR::revTime(t, kTimeSpan);
                        const VectorX<double> x_des_t = x_des.value(rev_t);
                        const VectorX<double> u_des_t = u_des.value(rev_t);

                        auto linear_system = Linearize(static_cast<double>(rev_t));
                        // negate system derivatives
                        const MatrixX<double> A = linear_system->A();
                        const MatrixX<double> B = linear_system->B();
                        const MatrixX<double> Ri = R.inverse();

                        // todo: handle sign
                        S2_dot = 1.0 * (Q - S2 * B * Ri * B.transpose() * S2 + S2*A + A.transpose() * S2);
                        s1_dot = 1.0 * (-2 * Q * x_des_t +
                                (A.transpose() - S2 * B * Ri * B.transpose()) * s1 +
                                  2 * S2 * B * u_des_t);
                        VectorX<double> s_dot(s.size());
                        flattenStoX(S2_dot, s1_dot, s_dot);
                        //x_dot << Eigen::Map<VectorX<double>>(S2_dot.data(), S2_dot.size()), s1_dot;
                        // to enable backward integration we are reversing sign of time. hence reversing sign of derivative
                        return s_dot;
                    }
                public:
                    static void unflattenXtoS(VectorX<T> &x, const int kNumStates, MatrixX<double>& S2, VectorX<double>& s1){
                        // matrix is built column by column
                        S2 = Eigen::Map<Eigen::MatrixXd>(x.data(), kNumStates, kNumStates);
                        s1 = x.tail(kNumStates);
                    }

                    static void flattenStoX(MatrixX<double>& S2, const VectorX<double>& s1, VectorX<double>& x){
                        // todo: map not taking const!!! check if altering original matrix
                        // map flattens in column major format ie. column by column
                        x << Eigen::Map<Eigen::VectorXd>(S2.data(), S2.size()), s1;
                    }

                    static std::unique_ptr<AffineSystem<double>> Linearize(
                            const System<double>& system,
                            const Context<double>& context,
                            const Eigen::VectorXd& x0,
                            const Eigen::VectorXd& u0,
                            variant<InputPortSelection, InputPortIndex> input_port_index = InputPortSelection::kUseFirstInputIfItExists,
                            variant<OutputPortSelection, OutputPortIndex> output_port_index = OutputPortSelection::kUseFirstOutputIfItExists
                    ){
                        // Create an autodiff version of the system.
                        std::unique_ptr<System<AutoDiffXd>> autodiff_system =
                                drake::systems::System<double>::ToAutoDiffXd(system);

                        // Initialize autodiff.
                        std::unique_ptr<Context<AutoDiffXd>> autodiff_context =
                                autodiff_system->CreateDefaultContext();
                        autodiff_context->SetTimeStateAndParametersFrom(context);
                        autodiff_system->FixInputPortsFrom(system, context, autodiff_context.get());

                        const InputPort<AutoDiffXd>* input_port =
                                autodiff_system->get_input_port_selection(input_port_index);
                        const OutputPort<AutoDiffXd>* output_port =
                                autodiff_system->get_output_port_selection(output_port_index);

                        // Verify that the input port is not abstract valued.
                        if (input_port &&
                            input_port->get_data_type() == PortDataType::kAbstractValued) {
                            throw std::logic_error(
                                    "Port requested for differentiation is abstract, and differentiation "
                                    "of abstract ports is not supported.");
                        }

                        const int num_inputs = input_port->size();
                        const int num_outputs = output_port->size();

                        const int num_states = x0.size();

                        auto autodiff_args = math::initializeAutoDiffTuple(x0, u0);

                        auto input_vector = std::make_unique<BasicVector<AutoDiffXd>>(num_inputs);
                        input_vector->SetFromVector(std::get<1>(autodiff_args));
                        autodiff_context->FixInputPort(input_port->get_index(),
                                                       std::move(input_vector));

                        Eigen::MatrixXd A(num_states, num_states), B(num_states, num_inputs);
                        Eigen::VectorXd f0(num_states);

                        autodiff_context->get_mutable_continuous_state_vector().SetFromVector(
                                std::get<0>(autodiff_args));
                        std::unique_ptr<ContinuousState<AutoDiffXd>> autodiff_xdot =
                                autodiff_system->AllocateTimeDerivatives();
                        autodiff_system->CalcTimeDerivatives(*autodiff_context,
                                                             autodiff_xdot.get());
                        auto autodiff_xdot_vec = autodiff_xdot->CopyToVector();

                        const Eigen::MatrixXd AB =
                                math::autoDiffToGradientMatrix(autodiff_xdot_vec);
                        A = AB.leftCols(num_states);
                        B = AB.rightCols(num_inputs);

                        const Eigen::VectorXd xdot0 =
                                math::autoDiffToValueMatrix(autodiff_xdot_vec);

                        // todo: check equilibrium
                        /*if (equilibrium_check_tolerance &&
                            !xdot0.isZero(*equilibrium_check_tolerance)) {
                            throw std::runtime_error(
                                    "The nominal operating point (x0,u0) is not an equilibrium point "
                                    "of "
                                    "the system.  Without additional information, a time-invariant "
                                    "linearization of this system is not well defined.");
                        }*/

                        f0 = xdot0 - A * x0 - B * u0;

                        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_outputs, num_states);
                        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_outputs, num_inputs);
                        Eigen::VectorXd y0 = Eigen::VectorXd::Zero(num_outputs);

                        const auto& autodiff_y0 = output_port->Eval(*autodiff_context);
                        const Eigen::MatrixXd CD = math::autoDiffToGradientMatrix(autodiff_y0);
                        C = CD.leftCols(num_states);
                        D = CD.rightCols(num_inputs);

                        const Eigen::VectorXd y = math::autoDiffToValueMatrix(autodiff_y0);

                        // Note: No tolerance check needed here.  We have defined that the output
                        // for the system produced by Linearize is in the coordinates (y-y0).

                        y0 = y - C * x0 - D * u0;

                        return std::make_unique<AffineSystem<double>>(A, B, f0, C, D, y0);
                    }

                    std::unique_ptr<AffineSystem<double>> Linearize(const double& t,
                            variant<InputPortSelection, InputPortIndex> input_port_index = InputPortSelection::kUseFirstInputIfItExists,
                            variant<OutputPortSelection, OutputPortIndex> output_port_index = OutputPortSelection::kUseFirstOutputIfItExists
                    ) const{
                        const Eigen::VectorXd x0 =  x_des.value(t);
                        const Eigen::VectorXd u0 =  u_des.value(t);
                        return Linearize(system, context,x0,u0,input_port_index, output_port_index);
                    }


                };

            };
        }
    }
}