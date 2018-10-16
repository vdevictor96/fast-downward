#ifndef LINEAR_PROGRAM_H
#define LINEAR_PROGRAM_H

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
    LPConstraint(const LPConstraint& constraint) = default;
    LPConstraint& operator=(const LPConstraint& c) = default;
    LPConstraint& operator=(LPConstraint&& c) = default;

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
 
