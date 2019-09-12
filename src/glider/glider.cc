#include "glider.h"

/*
namespace drake {
    namespace examples {
        namespace glider {
            template<typename T>
            Glider<T>::Glider():
                    systems::LeafSystem<T>(systems::SystemTypeTag<examples::glider::Glider>{}) {
                // only one controllable input that is tail deflection
                this->DeclareVectorInputPort("tail", drake::systems::BasicVector<T>(1));
                // 3 pos -> x, y, theta ; 3 vel -> x_dot, y_dot, theta_dot
                this->DeclareContinuousState(6);
                this->DeclareVectorOutputPort("state", systems::BasicVector<T>(6), &Glider::CopyStateOut);

            }

            template<typename T>
            void Glider<T>::CopyStateOut(const drake::systems::Context<T> &context,
                                         drake::systems::BasicVector<T> *output) const {
                // Write system output.
                output->set_value(context.get_continuous_state_vector().CopyToVector());
            }

            template<typename T>
            void Glider<T>::DoCalcTimeDerivatives(const systems::Context<T> &context,
                                                  systems::ContinuousState<T> *derivatives) const {
                context.get_continuous_state();
                const drake::systems::VectorBase<T> &q = context.get_continuous_state();
                drake::systems::VectorBase<T> &q_dot = derivatives->get_mutable_vector();
                const drake::systems::BasicVector<T> *input_vector = this->EvalVectorInput(context, 0);

                double x = q[0], z = q[1], theta = q[2], xdot = q[3], zdot = q[4], thetadot= q[5];

                double xddot, zddot, thetaddot;

                const double kMass = 0.5, kG = 9.81;
                const Eigen::Vector2d F_g(1, -kMass*kG);
                const double kLen = 0.1, kBodyLen = 0.8*kLen, kTailLen = kLen - kBodyLen, kWingWide = 0.1 , kTailWide = 0.05;
                const double  kTailS = kTailLen * kWingWide, kWingAR = 10, kWingS = pow(kWingWide,2) * kWingAR;

                const double kLw = kBodyLen/3;
                auto xdot_w = _calcSurfaceVel(Eigen::Vector2d(xdot, zdot), kLw, theta, thetadot);
                auto F_w = _calcSurfaceForce(theta, kWingS, xdot_w, theta);


                Eigen::Vector2d acc = (F_w + F_e + F_g)/kMass;

                q_dot[0] = xdot;
                q_dot[1] = zdot;
                q_dot[2] = thetadot;
                q_dot[3] = xddot;
                q_dot[4] = zddot;
                q_dot[5] = thetaddot;


            }

            template<typename T>
            double Glider<T>::_liftCoeff(double alpha) {
                return sin(2* alpha);
            }
            template<typename T>
            double Glider<T>::_dragCoeff(double alpha) {
                return sin(2* alpha);
            }

            template<typename T>
            Eigen::Vector2d
            Glider<T>::_calcSurfaceForce(double theta, double surface_area, Eigen::Vector2d wind_vel, double inclination_intertial) {
                Eigen::Vector2d surface_dir(-sin(inclination_intertial), cos(inclination_intertial));

                double alpha = theta - atan2(wind_vel[1], wind_vel[0]);
                Eigen::Vector2d aero_force = (1/2.0) * kRho * wind_vel.squaredNorm() * surface_area *
                                      (_liftCoeff(alpha) + _dragCoeff(alpha)) * surface_dir;
                return aero_force;
            }

            template<typename T>
            Eigen::Vector2d
            Glider<T>::_calcSurfaceVel(Eigen::Vector2d com_vel, double com_dist, double body_surface_defl, double theta, double thetadot){
                return Eigen::Vector2d(com_vel[0] + com_dist * thetadot * sin(theta), com_vel[1] - com_dist * thetadot * cos(theta));
            }

            static Eigen::Vector2d wing_normal_dir(double theta){

            }

            static Eigen::Rotation2D<double> wing_normal_dir(double theta, double phi){
                return Eigen::Rotation2D<double>(theta+phi);
            }
        }
    }
}*/
