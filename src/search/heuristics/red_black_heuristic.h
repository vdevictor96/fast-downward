#ifndef RED_BLACK_HEURISTIC_H
#define RED_BLACK_HEURISTIC_H

#include "ff_heuristic.h"

// Usage example: the command line option for using h^{RB} in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(red-black())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.

// In this class, you should implement the RB heuristic. You can either
// implement the Relaxed Plan Repair algorithm or the Relaxed-Facts Following
// algorithm. For both implementations, a working h^{FF} implementation is
// required. Additionally to the computation of an RB plan from a relaxed plan,
// you have to implement a painting strategy, i.e. you need to decide which
// variables are black, and which ones red.
//
// Note that we did not discuss painting strategies in the lecture. A simple
// painting strategy is enough to get full points. Note however that the
// paintining strategy must match the prerequisite of the algorithm that you use
// to compute an RB plan (e.g. Relaxed Plan Repair requires an disconnected black
// causal graph). Moreover, your painting should paint at least one variable
// black. More sophisticated painting strategies will be rewarded with
// additional bonus points.
//
// Helpful actions: in get_helpful_actions you should only add those
// operators of the Red-Black plan to result that are applicable in the state on which
// compute_heuristic is called.
//
// NOTE: evaluate(state) (which in turn calls compute_heuristic(state)) has to
// be called before accessing the helpful actions through
// get_helpful_actions(result)!
//
// Hint: you can use is_red_black_plan(state, painting, rb_plan) to verify
// whether your implementation is correct (see comment below).

#include <vector>

class RedBlackHeuristic : public FFHeuristic
{
protected:
    // is_red_black_plan checks whether the given plan is indeed a valid red
    // black plan for the given state and given painting. It assumes the
    // following arguments:
    //  state: it is verified whether rb_plan is a red black plan for state
    //  painting: painting.size() must be equal to the number of variables
    //  (that is painting.size() == g_variable_domain.size()); if painting[var] is true,
    //  then var is assumed to be a black variable, and vice versa, if painting[var] is
    //  false, then var is assumed to be a red variable
    //  rb_plan: the plan which should be verified. It is assumed that the
    //  actions affecting black variables are in correct order! (The order of
    //  red actions is not important)
    static bool is_red_black_plan(
        const State &state,
        const std::vector<bool> &painting,
        const std::vector<const Operator *> &rb_plan);

    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    RedBlackHeuristic(const Options &options);
    ~RedBlackHeuristic() = default;
    virtual void get_helpful_actions(std::vector<const Operator *> &result);
};

#endif
