#ifndef LINEAR_PROGRAM_H
#define LINEAR_PROGRAM_H

/* Linear Program Interface */

/* Usage example:
 * // Constructing the LP
 * // Maximize
 * //   10x1 - 2x2
 * // Subject To
 * //   x0 + 5x2 <= 10
 * //   3x0 - x1 >= -5
 * // Bounds
 * //   0 <= x0 <= 10
 * //   0 <= x1 <= 10
 * //   0 <= x2 <= 10
 * // End
 * 
 * // constructing lp variables and constraints
 * std::vector<lp::LPVariable> variables;
 * std::vector<lp::LPConstraint> constraints;
 * variables.push_back(lp::LPVariable(0, 0, 10));  // x0
 * variables.push_back(lp::LPVariable(10, 0, 10)); // x1
 * variables.push_back(lp::LPVariable(-2, 0, 10)); // x2
 * constraints.push_back(lp::LPConstraint(-lp::INFTY, 10)); // -infinity <= x0 + 5x2 <= 10
 * constraints[0].insert(0, 1); // x0
 * constraints[0].insert(2, 5); // 5x2
 * constraints.push_back(lp::LPConstraint(-5, lp::INFTY)); // -5 <= 3x0 - x1 <= infinity
 * constraints[1].insert(0, 3);  // 3x0
 * constraints[1].insert(1, -1); // -x1
 *
 * // creating linear program object
 * lp::LinearProgram linear_program(lp::LPObjectiveSense::MAXIMIZE,
 *                                  variables,
 *                                  constraints);
 *
 * // Writing the linear program to file 'linprog.lp'
 * linear_program.dump_to_file("linprog.lp");
 *
 * // solving the lp
 * linear_program.solve();
 *
 * // printing the objective value
 * std::cout << linear_program.get_objective_value() << std::endl;
 *
 * // getting and printing the solution vector
 * std::vector<double> solution = linear_program.extract_solution();
 * std::cout << "x0=" << solution[0] << "; x1=" << solution[1] << "; x2=" << solution[2] << std::endl;
 *
 */

#include <vector>

#if USE_LP
#include <soplex.h>
#endif

namespace lp {

extern const double INFTY;

enum class LPObjectiveSense {
    MAXIMIZE, MINIMIZE
};

class LPConstraint {
    std::vector<int> variables;
    std::vector<double> coefficients;
    double lower_bound;
    double upper_bound;
public:
    LPConstraint(double lower_bound, double upper_bound);
    LPConstraint(LPConstraint&& constraint);
    LPConstraint(const LPConstraint& constraint);
    LPConstraint& operator=(const LPConstraint& c);
    LPConstraint& operator=(LPConstraint&& c);

    const std::vector<int> &get_variables() const {return variables;}
    const std::vector<double> &get_coefficients() const {return coefficients;}

    double get_lower_bound() const {return lower_bound;}
    void set_lower_bound(double lb) {lower_bound = lb;}
    double get_upper_bound() const {return upper_bound;}
    void set_upper_bound(double ub) {upper_bound = ub;}

    void clear();
    bool empty() const;
    // Coefficients must be added without duplicate indices.
    void insert(int index, double coefficient);
};

struct LPVariable {
    double lower_bound;
    double upper_bound;
    double objective_coefficient;

    LPVariable(double objective_coefficient,
               double lower_bound,
               double upper_bound);
};

class LinearProgram {
#if USE_LP
    soplex::SoPlex soplex_solver;
    bool has_optimal_solution_;
    double lp_objective_;
#endif
public:
    explicit LinearProgram(LPObjectiveSense sense,
                           const std::vector<LPVariable> &variables,
                           const std::vector<LPConstraint> &constraints);
    ~LinearProgram() = default;

    /*
      Modifications to the linear program
    */
    void change_objective_coefficient(int var_index, double coefficient);
    void change_variable_lower_bound(int var_index, double bound);
    void change_variable_upper_bound(int var_index, double bound);
    void change_variable_bounds(int var_index, double lower, double upper);
    void change_constraint_lower_bound(int constraint_index, double bound);
    void change_constraint_upper_bound(int constraint_index, double bound);
    void change_constraint_bounds(int constraint_index, double lower, double upper);

    void solve();

    /*
      Return true if the solving the LP showed that it is bounded feasible and
      the discovered solution is guaranteed to be optimal. We test for
      optimality explicitly because solving the LP sometimes finds suboptimal
      solutions due to numerical difficulties.
      The LP has to be solved with a call to solve() before calling this method.
    */
    bool has_optimal_solution() const;

    /*
      Return the objective value found after solving an LP.
      The LP has to be solved with a call to solve() and has to have an optimal
      solution before calling this method.
    */
    double get_objective_value() const;

    /*
      Return the solution found after solving an LP as a vector with one entry
      per variable.
      The LP has to be solved with a call to solve() and has to have an optimal
      solution before calling this method.
    */
    std::vector<double> extract_solution();

    void dump_to_file(const char* filename);


    // void add_temporary_constraints(const std::vector<LPConstraint> &constraints);
    // void clear_temporary_constraints();
    // int get_num_variables() const;
    // int get_num_constraints() const;
    // int has_temporary_constraints() const;
    // void print_statistics() const;
};

}

#endif
 
