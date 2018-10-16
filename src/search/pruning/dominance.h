#ifndef DOMINANCE_H
#define DOMINANCE_H

#include "../pruning_method.h"

// Usage example: the command line option for using dominance pruning in wastar
// using heuristic with name h is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(h(), pruning=dominance)"
// So, for the blind heuristic h = blind it is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(blind(), pruning=dominance)"
// For h^{max}
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(hmax(), pruning=dominance)"
// And so on.
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.

/*
  In initialize() you should precompute the dominance relation, i.e.,
  a relation on each variable that determines for each pair of values
  of the variable <v1, v2> whether v2 dominates v1.

  For that, you'll need to implement the label-dominance simulation
  algorithm from the lecture. To simplify the code, it is sufficient
  if you use noop-dominance, which means that for every operator o and
  variable v, you'll need to determine whether o is dominated by noop
  in v.

  In remove_prune_operators, you can remove an operator if the result of that
  operator is dominated by the parent state. As the only difference
  between the parent and the successor is in the "effects" of the
  operators, that is all you need to compare. If the values of the
  parent dominate the values of the effect for every effect then the
  operator can be removed.
 */

class Dominance : public PruningMethod
{

protected:
    virtual void remove_pruned_operators(const State &state,
                                         std::vector<const Operator *> &ops) override;
public:
    Dominance();

    virtual ~Dominance() = default;

    virtual void initialize() override;
};

#endif
