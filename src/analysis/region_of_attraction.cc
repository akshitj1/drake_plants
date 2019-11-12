// #include <cmath>
#include <vector>

#include "gflags/gflags.h"

#include "drake/common/symbolic.h"
#include "drake/common/unused.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/solvers/decision_variable.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_expression_cell.h"

namespace drake
{
namespace systems
{
namespace controllers
{
/**
 * This overload is not avail. in drake lib
 */

LinearQuadraticRegulatorResult LinearQuadraticRegulatorGains(
    const System<double> &system, const Context<double> &context,
    const Eigen::Ref<const Eigen::MatrixXd> &Q,
    const Eigen::Ref<const Eigen::MatrixXd> &R, const int input_port_index = 0)
{
    const int num_inputs = system.get_input_port(input_port_index).size();
    const int num_states = context.num_total_states();
    DRAKE_DEMAND(num_states > 0);
    auto linear_system = Linearize(
        system, context, InputPortIndex{input_port_index},
        OutputPortSelection::kNoOutput);

    LinearQuadraticRegulatorResult lqr_result = LinearQuadraticRegulator(
        linear_system->A(), linear_system->B(), Q, R);
    return lqr_result;
}
} // namespace controllers
} // namespace systems

namespace symbolic
{
using namespace drake::symbolic;
Expression TaylorExpand(const Expression &f, const Environment &a,
                        const int order)
{
    // The implementation uses the formulation:
    //      Taylor(f, a, order) = ∑_{|α| ≤ order} ∂fᵅ(a) / α! * (x - a)ᵅ.
    DRAKE_DEMAND(order >= 1);
    ExpressionAddFactory factory;
    factory.AddExpression(f.EvaluatePartial(a));
    const int num_vars = a.size();
    if (num_vars == 0)
    {
        return f;
    }
    std::vector<Expression> terms; // (x - a)
    for (const std::pair<const Variable, double> &p : a)
    {
        const Variable &var = p.first;
        const double v = p.second;
        terms.push_back(var - v);
    }
    for (int i = 1; i <= order; ++i)
    {
        DoTaylorExpand(f, a, terms, i, num_vars, &factory);
    }
    return factory.GetExpression();
}
} //namespace symbolic

namespace analysis
{
namespace pendulum
{

using std::cout;
using std::endl;

using symbolic::Environment;
using symbolic::Expression;
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;

using namespace drake::examples::pendulum;
using namespace drake::systems::controllers;

LinearQuadraticRegulatorResult getLqrGains(const PendulumState<double> &goal_state)
{
    Eigen::MatrixXd Q(2, 2);
    Q << 10, 0, 0, 1;
    Eigen::MatrixXd R(1, 1);
    R << 1;

    PendulumPlant<double> pendulum_d;
    auto lqr_context = pendulum_d.CreateDefaultContext();
    lqr_context->SetContinuousState(goal_state.CopyToVector());
    lqr_context->FixInputPort(0, PendulumInput<double>{}.with_tau(0.0));

    auto lqr_res = systems::controllers::LinearQuadraticRegulatorGains(pendulum_d, *lqr_context, Q, R);
    return lqr_res;
}

void LtiRegionOfAttraction()
{
    // Create the simple system.
    PendulumPlant<Expression> pendulum;
    auto context = pendulum.CreateDefaultContext();
    auto derivatives = pendulum.AllocateTimeDerivatives();

    PendulumState<double> goal_state;
    goal_state.set_theta(M_PI);
    goal_state.set_thetadot(0);

    auto lqr_res = getLqrGains(goal_state);

    // Setup the optimization problem.
    solvers::MathematicalProgram prog;
    const VectorX<Variable> xvar{prog.NewIndeterminates<2>(std::array<std::string, 2>{"theta", "thetadot"})};
    const VectorX<Expression> x = xvar.cast<Expression>();

    // Extract the polynomial dynamics.
    context->get_mutable_continuous_state_vector().SetFromVector(x);
    // tau_goal is zero, so additive term ignored
    context->FixInputPort(0, -lqr_res.K * (x - goal_state.CopyToVector()));
    pendulum.CalcTimeDerivatives(*context, derivatives.get());

    // Define the Lyapunov function.
    const Expression V = x.transpose() * lqr_res.S * x;
    const Environment poly_approx_env{{xvar(0), goal_state.theta()}, {xvar(1), goal_state.thetadot()}};
    const Expression theta_ddot_poly_approx = symbolic::TaylorExpand(derivatives->CopyToVector()[1], poly_approx_env, 2);
    VectorX<Expression> f_poly_approx(2);
    f_poly_approx << derivatives->CopyToVector()[0], theta_ddot_poly_approx;

    const Expression Vdot = 2 * x.transpose() * lqr_res.S * f_poly_approx;

    //const Variable rho{prog.NewContinuousVariables<1>("rho").coeff(0)};
    const Expression lambda{
        prog.NewSosPolynomial(Variables(xvar), 4).first.ToExpression()};

    const double kPrec = 0.1;
    double lb = 0.0, ub = 10.0, rho = (lb + ub) / 2;
    for (rho = (lb + ub) / 2; ub - lb >= kPrec; rho = (lb + ub) / 2)
    {
        std::cout << "rho: " << rho << endl;
        auto _prog = prog.Clone();
        _prog->AddSosConstraint(-(Vdot + lambda * (rho - V)));
        auto res = Solve(*_prog);
        const double is_feasible = res.is_success();
        if (is_feasible)
            lb = rho;
        else
            ub = rho;
    }

    //DRAKE_DEMAND(res.is_success());

    const double rho_max = rho; //result.GetSolution(rho);

    cout << "Verified that " << V << " < " << rho_max
         << " is in the region of attraction." << endl;

    // Check that ρ ≃ 1.0.
    DRAKE_DEMAND(std::abs(rho_max - 1.0) < 1e-6);
}
// // we want a copy, not refrence as we are going to modify constraint
// double rho_line_search(const solvers::MathematicalProgram &_prog, const double lb, const double ub){
//     auto prog = _prog.Clone();
//     prog->AddSosConstraint(-(Vdot + lambda * (rho - V)));
// }
} // namespace pendulum
} // namespace analysis
} // namespace drake

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::analysis::pendulum::LtiRegionOfAttraction();
    return 0;
}
