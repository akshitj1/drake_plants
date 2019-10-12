//
// Created by Akshit Jain on 09/10/19.
//
#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include "drake/systems/primitives/linear_system.h"
#include "eigen3/Eigen/Core"
#include "drake/systems/analysis/initial_value_problem-inl.h"

namespace drake {
    namespace systems {
        namespace controllers {
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
                const TimeVaryingLinearSystem<double>& ltv_model,
                const PPoly& x_des, u_des;
                const int kNumStates;

                std::unique_ptr<DenseOutput<double>> s_traj;
            public:

                TimeVaryingLQR(
                        const TimeVaryingLinearSystem<double> &ltv_model,
                        const PPoly &x_des,
                        const PPoly &u_des,
                        const Eigen::MatrixXd& Q,
                        const Eigen::MatrixXd& R,
                        const Eigen::MatrixXd& Qf):
                        ltv_model(ltv_model), Q(Q), Qf(Qf), R(R), x_des(x_des),u_des(u_des), kNumStates(Q.rows()){
                    // Descrption of the linearized plant model.
                    //std::unique_ptr<LinearSystem<double>> linear_model_ = Linearize(model, base_context);

                    Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
                    if (R_cholesky.info() != Eigen::Success) {
                        throw std::runtime_error("R must be positive definite");
                    }
                    // todo: check needs for Q too?
                }

                VectorX<double> GetControl(const double& t){
                    MatrixX<double> S2;
                    VectorX<double> s1;
                    VectorX<double> x = s_traj->Evaluate(-t);
                    sDynamicsOde<double>::unflattenXtoS(x, kNumStates, S2, s1);
                    VectorX<double> u_opt = u_des.value(-t) - R.inverse() * ltv_model.B(0).transpose() * (S2 * x + 0.5 * s1);
                }

                VectorX<double> CalcControl(){
                    PPoly xdot_traj = x_des.derivative();

                    const double kTSpan = x_des.end_time();

                    Eigen::VectorXd S_f[3];
                    MatrixX<double> S2_f = Qf;
                    VectorX<double> s1_f = -2 * Qf * x_des.value(kTSpan);
                    VectorX<double> kDefaultInitialState;
                    sDynamicsOde<double>::flattenStoX(S2_f, s1_f, kDefaultInitialState);

                    const double kDefaultInitialTime = -kTSpan;
                    VectorX<double> kDefaultParams(0);
                    InitialValueProblem<double>::SpecifiedValues kDefaultValues(
                            kDefaultInitialTime, kDefaultInitialState, kDefaultParams
                    );

                    InitialValueProblem<double> ivp(sDynamicsOde<double>(ltv_model, Q, R, x_des, u_des), kDefaultValues);
                    s_traj = ivp.DenseSolve(0);
                }

            private:

                template<typename T>
                class sDynamicsOde {
                private:
                    const MatrixX<double> &Q, R;
                    const TimeVaryingLinearSystem<double> &ltv_model;
                    const PPoly &x_des, u_des;
                    const int kNumStates;
                public:
                    sDynamicsOde(
                            const TimeVaryingLinearSystem<double> &ltv_model,
                            const MatrixX<double> &Q,
                            const MatrixX<double> &R,
                            const PPoly &x_des,
                            const PPoly &u_des) :
                            ltv_model(ltv_model), Q(Q), R(R), x_des(x_des), u_des(u_des), kNumStates(Q.cols()) {}

                    VectorX<T> operator()(const T &t, const VectorX<T> &s, const VectorX<T> &k) const {
                        unused(k);
                        // we do not need to solve for S_0 as u* is independent of it
                        MatrixX<double> S2, S2_dot;
                        VectorX<double> s1, s1_dot;
                        unflattenXtoS(s, kNumStates, S2, s1);

                        const MatrixX<double> A = ltv_model.A(t);
                        const MatrixX<double> B = ltv_model.B(t);
                        const MatrixX<double> Ri = R.inverse();
                        const VectorX<double> x_des_t = x_des.value(t);
                        const VectorX<double> u_des_t = u_des.value(t);

                        // todo: handle sign
                        -S2_dot = Q - S2 * B * Ri * B.transpose() * S2 + A.transpose() * S2;
                        -s1_dot = -2 * Q * x_des_t + (A.transpose() - S2 * B * Ri * B.transpose()) * s1 +
                                  2 * S2 * B * u_des_t;
                        VectorX<double> s_dot;
                        flattenStoX(S2_dot, s1_dot, s_dot);
                        //x_dot << Eigen::Map<VectorX<double>>(S2_dot.data(), S2_dot.size()), s1_dot;
                        return s_dot;
                    }
                public:
                    static void unflattenXtoS(const VectorX<T> &x, const int kNumStates, MatrixX<double>& S2, VectorX<double>& s1){
                        S2 = Eigen::Map<MatrixX<double>>(x, kNumStates, kNumStates);
                        s1 = x.tail(kNumStates);
                    }

                    static void flattenStoX(MatrixX<double>& S2, const VectorX<double>& s1, VectorX<double>& x){
                        // todo: map not taking const!!! check if altering original matrix
                        x << Eigen::Map<Eigen::VectorXd>(S2.data(), S2.rows()*S2.cols()), s1;
                    }

                };

            };
        }
    }
}