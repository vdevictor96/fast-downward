#ifndef MAX_HEURISTIC_H
#define MAX_HEURISTIC_H

#include "../heuristic.h"

// Usage example: the command line option for using the h^{max} heuristic in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(hmax())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.

// In this class, you are supposed to implement the h^{max} heuristic.
// You can do this by either the table based computation as given in the
// lecture, or alternatively, you can use the (much more efficient) counter
// based computation as presented in http://fai.cs.uni-saarland.de/hoffmann/papers/jair01.pdf
// (Section 4.3, third text paragraph).
//
// NOTE: To get an efficient implementation you should try to precompute as many
// things as possible in the initialize() function (such as, for example,
// a relation of facts to those actions that add this fact).

class MaxHeuristic : public Heuristic
{
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    MaxHeuristic(const Options &options);
    ~MaxHeuristic() = default;
};

#endif
