#ifndef LANDMARK_HEURISTIC_H
#define LANDMARK_HEURISTIC_H

#include "../heuristic.h"

// Usage example: the command line option for using the LM count heuristic in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(lm())" Obviously,
// [path-to-PDDL-problem-file] has to be replaced by the actual path to the PDDL problem
// file.

// In this class you are supposed to implement a landmark count heuristic based on
// disjunctive action landmarks. This requires you to implement the following things: 1)
// Computing a set of action landmarks for the initial state (e.g. through necessary
// sub-goals). (You can access the initial state through the g_initial_state() function
// from ../globals.h) 2) Given a state and an action landmark, computing the landmark
// count heuristic. (e.g. an inadmissible heursitic if you just sum up all available
// landmarks, or an admissible heuristic if you compute the canonical heuristic) 3) For
// each state, you have to maintain the (sub-)set of still available landmarks. This is,
// you are not supposed to compute a set of landmarks in each heuristic call from scratch,
// but you should reuse the landmarks computed for the initial state. The determine
// whether a landmark is still available for a state, the search provides you information
// about the transitions in the state space through the reach_state
// function. reach_state(p, op, s) means that the state s is reached from p by applying
// the operator op.
//
// Hint: You can store per state information through the template class
// PerStateInformation<T> (defined in ../per_state_information.h). For example, you can
// store for each state a vector of bools through an object of type
// PerStateInformation<std::vector<bool> >. To access a PerStateInformation<T> object
// (let's call it info) given a state s, you can use the operator[] function of
// PerStateInformation<T>: info[s].
//
// Note that we did not discuss in detail how a set of landmarks can be computed. For
// example computing the landmarks for the initial state based on necessary subgoals is
// enough to get full points. But we will give bonus points for more sophisticated
// strategies.

class LandmarkHeuristic : public Heuristic
{
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    LandmarkHeuristic(const Options &options);
    ~LandmarkHeuristic() = default;
    virtual void reach_state(const State &parent_state, const Operator &op,
                             const State &state);
    virtual void get_helpful_actions(std::vector<const Operator *> &result);
};

#endif
