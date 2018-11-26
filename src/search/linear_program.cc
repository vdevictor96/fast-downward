#include "linear_program.h"

#include <algorithm>
#include <cassert>

namespace lp {

LPConstraint::LPConstraint(double lower, double upper)
    : lower_bound(lower), upper_bound(upper)
{}

LPConstraint::LPConstraint(LPConstraint&& constraint)
    : variables(std::move(constraint.variables))
    , coefficients(std::move(constraint.coefficients))
    , lower_bound(std::move(constraint.lower_bound))
    , upper_bound(std::move(constraint.upper_bound))
{}

LPConstraint::LPConstraint(const LPConstraint& constraint)
    : variables(constraint.variables)
    , coefficients(constraint.coefficients)
    , lower_bound(constraint.lower_bound)
    , upper_bound(constraint.upper_bound)
{}

LPConstraint& LPConstraint::operator=(const LPConstraint& c)
{
    variables = c.variables;
    coefficients = c.coefficients;
    lower_bound = c.lower_bound;
    upper_bound = c.upper_bound;
    return *this;
}

LPConstraint& LPConstraint::operator=(LPConstraint&& c)
{
    variables = std::move(c.variables);
    coefficients = std::move(c.coefficients);
    lower_bound = std::move(c.lower_bound);
    upper_bound = std::move(c.upper_bound);
    return *this;
}

void LPConstraint::clear() {
    variables.clear();
    coefficients.clear();
}

bool LPConstraint::empty() const {
    return variables.empty();
}

void LPConstraint::insert(int index, double coefficient) {
    auto it = std::lower_bound(variables.begin(), variables.end(), index);
    if (it != variables.end()) {
        assert((*it) != index);
        int i = std::distance(variables.begin(), it);
        variables.insert(it, index);
        coefficients.insert(coefficients.begin() + i, coefficient);
    } else {
        variables.push_back(index);
        coefficients.push_back(coefficient);
    }
}

LPVariable::LPVariable(double objective_coefficient, double lower_bound, double upper_bound)
    : lower_bound(lower_bound),
      upper_bound(upper_bound),
      objective_coefficient(objective_coefficient) {
}

}

#if USE_LP

namespace lp {

const double INFTY = soplex::infinity;

LinearProgram::LinearProgram(LPObjectiveSense sense,
                             const std::vector<LPVariable>& vars,
                             const std::vector<LPConstraint>& constraints)
{
    auto soplex_sense = soplex::SoPlex::OBJSENSE_MAXIMIZE;
    if (sense == LPObjectiveSense::MINIMIZE) {
        soplex_sense = soplex::SoPlex::OBJSENSE_MINIMIZE;;
    }
    soplex_solver.setIntParam(soplex::SoPlex::OBJSENSE, soplex_sense);
//#ifndef NDEBUG
//    soplex_solver.setIntParam(soplex::SoPlex::SIMPLIFIER, soplex::SoPlex::SIMPLIFIER_OFF);
//#endif
    soplex_solver.setIntParam(soplex::SoPlex::VERBOSITY, 0);

    soplex::DSVector dummycol(0);
    for (unsigned i = 0; i < vars.size(); i++) {
        soplex_solver.addColReal(soplex::LPCol(vars[i].objective_coefficient,
                                               dummycol,
                                               vars[i].upper_bound,
                                               vars[i].lower_bound));
    }

    for (unsigned i = 0; i < constraints.size(); i++) {
        const LPConstraint& constraint = constraints[i];
        soplex::DSVector row(constraint.get_variables().size());
        for (unsigned j = 0; j < constraint.get_variables().size(); j++) {
            row.add(constraint.get_variables()[j],
                    constraint.get_coefficients()[j]);
        }
        soplex_solver.addRowReal(soplex::LPRow(constraint.get_lower_bound(),
                                               row,
                                               constraint.get_upper_bound()));
    }

    has_optimal_solution_ = false;
}

void
LinearProgram::change_objective_coefficient(int index, double coefficient)
{
    assert(index < soplex_solver.numColsReal());
    soplex_solver.changeObjReal(index, coefficient);
}

void
LinearProgram::change_variable_lower_bound(int index, double bound)
{
    assert(index < soplex_solver.numColsReal());
    soplex_solver.changeLowerReal(index, bound);
}

void
LinearProgram::change_variable_upper_bound(int index, double bound)
{
    assert(index < soplex_solver.numColsReal());
    soplex_solver.changeUpperReal(index, bound);
}

void LinearProgram::change_variable_bounds(int var_index, double lower, double upper)
{
    assert(var_index < soplex_solver.numColsReal());
    soplex_solver.changeBoundsReal(var_index, lower, upper);
}

void
LinearProgram::change_constraint_lower_bound(int index, double bound)
{
    assert(index < soplex_solver.numRowsReal());
    soplex_solver.changeLhsReal(index, bound);
}

void
LinearProgram::change_constraint_upper_bound(int index, double bound)
{
    assert(index < soplex_solver.numRowsReal());
    soplex_solver.changeRhsReal(index, bound);
}

void LinearProgram::change_constraint_bounds(int constraint_index, double lower, double upper)
{
    assert(constraint_index < soplex_solver.numRowsReal());
    soplex_solver.changeRangeReal(constraint_index, lower, upper);
}

void
LinearProgram::solve()
{
    has_optimal_solution_ = soplex_solver.optimize() == soplex::SPxSolver::OPTIMAL;
    if (has_optimal_solution_) {
        lp_objective_ = soplex_solver.objValueReal();
    }
}

bool
LinearProgram::has_optimal_solution() const
{
    return has_optimal_solution_;
}

double
LinearProgram::get_objective_value() const
{
    assert(has_optimal_solution_);
    return lp_objective_;
}

std::vector<double>
LinearProgram::extract_solution()
{
    assert(has_optimal_solution_);
    soplex::DVector sol_(soplex_solver.numColsReal());
    soplex_solver.getPrimalReal(sol_);
    std::vector<double> sol(sol_.dim());
    for (int i = sol.size() - 1; i >= 0; i--) {
        sol[i] = sol_[i];
    }
    return sol;
}

void
LinearProgram::dump_to_file(const char* filename)
{
    soplex_solver.writeFileReal(filename, NULL, NULL, NULL);
}

}


#else

namespace lp {

const double INFTY = 0;

LinearProgram::LinearProgram(LPObjectiveSense,
                             const std::vector<LPVariable>&,
                             const std::vector<LPConstraint>&)
{}

void
LinearProgram::change_objective_coefficient(int , double )
{
}

void
LinearProgram::change_constraint_lower_bound(int , double )
{
}

void
LinearProgram::change_constraint_upper_bound(int , double )
{
}

void
LinearProgram::change_variable_lower_bound(int , double )
{
}

void
LinearProgram::change_variable_upper_bound(int , double )
{
}

void
LinearProgram::change_variable_bounds(int , double , double )
{
}

void
LinearProgram::change_constraint_bounds(int , double , double )
{
}

void
LinearProgram::solve()
{
}

bool
LinearProgram::has_optimal_solution() const
{
    return false;
}

double
LinearProgram::get_objective_value() const
{
    return 0;
}

std::vector<double>
LinearProgram::extract_solution()
{
    return std::vector<double>();
}

void
LinearProgram::dump_to_file(const char* )
{
}

}

#endif
