#ifndef BLIND_HEURISTIC_H
#define BLIND_HEURISTIC_H

#include "../heuristic.h"

// Usage example: the command line option for using the blind heuristic in astar
// (and thus to run a blind search) is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(blind())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.

class BlindHeuristic : public Heuristic
{
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    BlindHeuristic(const Options &options);
    ~BlindHeuristic() = default;
};

#endif
