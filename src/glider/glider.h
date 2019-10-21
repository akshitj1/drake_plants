//
// Created by Akshit Jain on 09/09/19.
//
#pragma once
#include "drake/systems/framework/leaf_system.h"
#include <cmath>
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"
#include "drake/common/default_scalars.h"



namespace drake{ namespace examples{ namespace glider{
    template <typename T>
    class Glider final : public systems::LeafSystem<T>{
    private:
        // taken from https://github.com/RobotLocomotion/drake/blob/BeforeCCode/examples/Glider/GliderPlant.m
        const double kG = 9.81, kRho = 1.204; //atm density
        const double kMass = 0.082, kInertia = 0.0015;
        const double  kTailS = 0.0147, kWingS = 0.0885;
        const double kLe = 0.022, kL = 0.27;
        const double kLw = 0;

    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Glider);

        Glider(): systems::LeafSystem<T>(systems::SystemTypeTag<Glider>{}) {
            // only one controllable input that is tail deflection
            this->DeclareVectorInputPort("tail_defl_rate", drake::systems::BasicVector<T>(1));
            //this->DeclareInputPort("tail_defl_rate", drake::systems::kVectorValued, 1);
            // 3 pos -> x, y, theta ; 3 vel -> x_dot, y_dot, theta_dot ; phi
            this->DeclareContinuousState(7);
            this->DeclareVectorOutputPort("glider_state", drake::systems::BasicVector<T>(7), &Glider::CopyStateOut);
        }

        template <typename U>
        explicit Glider(const Glider<U>&) : Glider<T>(){}

        void CopyStateOut(const drake::systems::Context<T> &context,
                                     drake::systems::BasicVector<T> *output) const {
            // Write system output.2
            output->set_value(context.get_continuous_state_vector().CopyToVector());
        }

        void DoCalcTimeDerivatives(const systems::Context<T>& context,
                                              systems::ContinuousState<T> *derivatives) const override {
            const drake::systems::VectorBase<T>& q = context.get_continuous_state_vector();
            //const drake::systems::BasicVector<T> *input_vector = this->EvalVectorInput(context, 0);

            auto x = q[0], z = q[1], theta = q[2], phi = q[3], x_dot=q[4], z_dot=q[5];
            //Vector2<T> xdot(q[4],q[5]);
            auto theta_dot = q[6];
            // elevon defl. rate
            auto phi_dot= this->get_input_port(0).Eval(context)(0);

            auto xw_dot = x_dot - kLw * theta_dot * sin(theta);
            auto zw_dot = z_dot + kLw * theta_dot * cos(theta);
            auto alpha_w = theta - atan2(zw_dot,xw_dot);
            auto F_w = kRho*kWingS*sin(alpha_w)*(pow(zw_dot,2) + pow(xw_dot,2));

            auto xe_dot = x_dot + kL * theta_dot * sin(theta) + kLe * (theta_dot + phi_dot) * sin(theta + phi);
            auto ze_dot = z_dot - kL * theta_dot * cos(theta) - kLe * (theta_dot + phi_dot) * cos(theta + phi);
            auto alpha_e = theta + phi - atan2(ze_dot,xe_dot);
            auto F_e = kRho*kTailS*sin(alpha_e)*(pow(ze_dot,2)+pow(xe_dot,2));

            auto x_ddot = -(F_w*sin(theta) + F_e*sin(theta + phi))/kMass;
            auto z_ddot = (F_w*cos(theta) + F_e*cos(theta + phi))/kMass - kG;
            auto theta_ddot = (F_w*kLw - F_e*(kL*cos(phi)+kLe))/kInertia;

/*
            auto xdot_w = _calcSurfaceVel(xdot, kLw, theta, thetadot);
            auto F_w = _calcSurfaceForce(kWingS, xdot_w, theta);

            auto xdot_e = _calcSurfaceVel(xdot, kLe, theta+phi, thetadot + phidot)
                    + _calcSurfaceVel(Vector2<T>(0, 0), kL, theta, thetadot);
            auto F_e = _calcSurfaceForce(kTailS, xdot_e, theta + phi);

            const Vector2<T> kF_g(0, -kMass*kG);
            auto acc = (F_w + F_e + kF_g)/kMass;
            T theta_ddot = (Vector2<T>(kLw, 0).dot(F_w)
                    + Vector2<T>(-kL - kLe * cos(theta),-kL + kLe * sin(theta)).dot(F_e)) / kInertia;
*/
            VectorX<T> q_dot(7);
            //q_dot << xdot[0], xdot[1], thetadot, phidot, acc[0], acc[1], theta_ddot;
            q_dot << x_dot, z_dot, theta_dot, phi_dot, x_ddot , z_ddot, theta_ddot;
            derivatives->SetFromVector(q_dot);
        }

        T _liftCoeff(T alpha) const{
            return sin(2* alpha);
        }
        T _dragCoeff(T alpha) const{
            return 2 * pow(sin(alpha), 2);
        }

        Vector2<T> _calcSurfaceForce(double surface_area, Vector2<T> wind_vel, T inclination_intertial) const{
            Vector2<T> surface_dir(-sin(inclination_intertial), cos(inclination_intertial));

            T alpha = inclination_intertial - atan2(wind_vel[1], wind_vel[0]);
            Vector2<T> aero_force = (1/2.0 * kRho * wind_vel.squaredNorm() * surface_area *
                                         (_liftCoeff(alpha) + _dragCoeff(alpha))) * surface_dir;
            return aero_force;
        }

        Vector2<T> _calcSurfaceVel(Vector2<T> com_vel, double com_dist, T theta, T thetadot) const{
            return Vector2<T>(com_vel[0] + com_dist * thetadot * sin(theta), com_vel[1] - com_dist * thetadot * cos(theta));
        }
    };


        }}

    // The following code was added to prevent scalar conversion to symbolic scalar
    // types. The QuadrotorPlant makes use of classes that are not compatible with
    // the symbolic scalar. This NonSymbolicTraits is explained in
    // drake/systems/framework/system_scalar_converter.h.
    namespace systems {
        namespace scalar_conversion {
            template <>
            struct Traits<examples::glider::Glider> : public NonSymbolicTraits {};
        }  // namespace scalar_conversion
    }  // namespace systems
}
