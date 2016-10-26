#ifndef STRONG_STUBBORN_SETS_H
#define STRONG_STUBBORN_SETS_H

#include "../pruning_method.h"

// Usage example: the command line option for using strong stubborn sets
// pruning in wastar using heuristic with name h is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(h(), pruning=strong_stubborn_sets)"
// So, for the blind heuristic h = blind it is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(blind(), pruning=strong_stubborn_sets)"
// For h^{max}
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(hmax(), pruning=strong_stubborn_sets)"
// And so on.
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.


// In this class you should implement strong stubborn set based successor
// pruning. In remove_pruned_operators you should remove all operators from ops
// that are not contained in the strong stubborn set of the given state. Thus, in
// each call to remove_pruned_operators you first have to compute a strong stubborn
// set for the given state.
//
// Note: To get an efficient implementation you should precompute static
// information (i.e. state independent information required to compute a strong stubborn
// sets) in the initialze() function (as for example action dependencies).

class StrongStubbornSets : public PruningMethod
{

protected:
    virtual void remove_pruned_operators(const State &state,
                                         std::vector<const Operator *> &ops) override;
public:
    StrongStubbornSets();

    virtual ~StrongStubbornSets() = default;

    virtual void initialize() override;

};

#endif
