#ifndef POTENTIAL_HEURISTIC_H
#define POTENTIAL_HEURISTIC_H

#include "../heuristic.h"

#include <vector>

class PotentialHeuristic : public Heuristic {
    virtual void initialize() override;
    virtual int compute_heuristic(const State& state) override;
public:
    PotentialHeuristic(const Options& opts);
    ~PotentialHeuristic() = default;
};

#endif
