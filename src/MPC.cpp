#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
double ref_v = 100;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coefficients;
    MPC *mpc; // With start variables
    explicit FG_eval(Eigen::VectorXd _coefficients, MPC *_mpc) {
        this->coefficients = std::move(_coefficients);
        mpc = _mpc;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        // Implement MPC
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
        // Based on this link: https://youtu.be/bOQuhpz3YfU
        /// Lesson 19. Section 9.
        fg[0] = 0;
        for (int i = 0; i < N; i++) {
            fg[0] += 2000 * CppAD::pow(vars[mpc->starts[4] + i], 2);
            fg[0] += 2000 * CppAD::pow(vars[mpc->starts[5] + i], 2);
            fg[0] += CppAD::pow(vars[mpc->starts[3] + i] - ref_v, 2);
        }
        for (int i = 0; i < N - 1; i++) {
            fg[0] += 5 * CppAD::pow(vars[mpc->starts[6] + i], 2);
            fg[0] += 5 * CppAD::pow(vars[mpc->starts[7] + i], 2);
        }
        for (int i = 0; i < N - 2; i++) {
            fg[0] += 100 * CppAD::pow(vars[mpc->starts[6] + i + 1] - vars[mpc->starts[6] + i], 2);
            fg[0] += 5 * CppAD::pow(vars[mpc->starts[7] + i + 1] - vars[mpc->starts[7] + i], 2);
        }
        for (int i = 0; i < 6; i++) {
            fg[1 + mpc->starts[i]] = vars[mpc->starts[i]];
        }
        for (int i = 0; i < N - 1; i++) {
            // Time t
            AD<double> x0 = vars[mpc->starts[0] + i];
            AD<double> y0 = vars[mpc->starts[1] + i];
            AD<double> psi0 = vars[mpc->starts[2] + i];
            AD<double> v0 = vars[mpc->starts[3] + i];
            AD<double> cte0 = vars[mpc->starts[4] + i];
            AD<double> epsi0 = vars[mpc->starts[5] + i];
            // Time t + 1
            AD<double> x1 = vars[mpc->starts[0] + i + 1];
            AD<double> y1 = vars[mpc->starts[1] + i + 1];
            AD<double> psi1 = vars[mpc->starts[2] + i + 1];
            AD<double> v1 = vars[mpc->starts[3] + i + 1];
            AD<double> cte1 = vars[mpc->starts[4] + i + 1];
            AD<double> epsi1 = vars[mpc->starts[5] + i + 1];

            AD<double> delta0 = vars[mpc->starts[6] + i];
            AD<double> a0 = vars[mpc->starts[7] + i];
            // Equation of f0
            AD<double> f0 = coefficients[3] * CppAD::pow(x0, 3) + coefficients[2] * CppAD::pow(x0, 2)
                            + coefficients[1] * x0 + coefficients[0];
            // Equation of psides that is de derivative of f0
            AD<double> psides0 = CppAD::atan(3 * coefficients[3] * CppAD::pow(x0, 2)
                                             + 2 * coefficients[2] * x0 + coefficients[1]);

            fg[2 + mpc->starts[0] + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + mpc->starts[1] + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + mpc->starts[2] + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
            fg[2 + mpc->starts[3] + i] = v1 - (v0 + a0 * dt);
            fg[2 + mpc->starts[4] + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + mpc->starts[5] + i] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
        }
    }
};


// MPC class definition implementation.
MPC::MPC() {
    // Initialization of variables
    starts.push_back(0); //x -> 0
    // y -> 1, psi -> 2, v -> 3, cte -> 4, epsi -> 5, delta -> 6
    for (int i = 0; i < 6; i++) {
        starts.push_back(starts[i] + N);
    }
    starts.push_back(starts[6] + N - 1);//a -> 7
}

MPC::~MPC() = default;

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    typedef CPPAD_TESTVECTOR(double) Dvector;
    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    // Based on this link: https://youtu.be/bOQuhpz3YfU
    size_t n_vars = N * 6 + (N - 1) * 2;
    // Set the number of constraints
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    for (int i = 0; i < starts[6]; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    for (auto i = static_cast<int>(starts[6]); i < starts[7]; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }
    for (auto i = static_cast<int>(starts[7]); i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[starts[0]] = constraints_upperbound[starts[0]] = state[0]; //x
    constraints_lowerbound[starts[1]] = constraints_upperbound[starts[1]] = state[1]; //y
    constraints_lowerbound[starts[2]] = constraints_upperbound[starts[2]] = state[2]; //psi
    constraints_lowerbound[starts[3]] = constraints_upperbound[starts[3]] = state[3]; //v
    constraints_lowerbound[starts[4]] = constraints_upperbound[starts[4]] = state[4]; //cte
    constraints_lowerbound[starts[5]] = constraints_upperbound[starts[5]] = state[5]; //epsi
    // object that computes objective and constraints
    FG_eval fg_eval(std::move(coeffs), this);
    // NOTE: You don't have to worry about these options
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);
    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;
    // Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    vector<double> solved;
    solved.push_back(solution.x[starts[6]]);
    solved.push_back(solution.x[starts[7]]);
    for (int i = 0; i < N; ++i) {
        solved.push_back(solution.x[starts[0] + i]);
        solved.push_back(solution.x[starts[1] + i]);
    }
    return solved;
}
