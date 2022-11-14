#ifndef ADDITIVE_HEURISTIC_H
#define ADDITIVE_HEURISTIC_H

#include "../heuristic.h"

// Usage example: the command line option for using h^{add} in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(hadd())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.

// In this class, you are supposed to implement the h^{add} heuristic.
// You can do this by either the table based computation as given in the
// lecture, or alternatively, you can use the (much more efficient) counter
// based computation as presented in http://fai.cs.uni-saarland.de/hoffmann/papers/jair01.pdf
// (Section 4.3, third text paragraph).
//
// NOTE: To get an efficient implementation you should try to precompute as many
// things as possible in the initialize() function (such as, for example,
// a relation of facts to those actions that add this fact).

class AdditiveHeuristic : public Heuristic
{
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    AdditiveHeuristic(const Options &options);
    ~AdditiveHeuristic() = default;
};

#endif
