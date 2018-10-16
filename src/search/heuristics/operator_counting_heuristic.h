#ifndef OPERATOR_COUNTING_HEURISTIC_H
#define OPERATOR_COUNTING_HEURISTIC_H

#include "../heuristic.h"
#include "../linear_program.h"

#include <vector>
#include <memory>

class OperatorCountingHeuristic : public Heuristic {
    std::unique_ptr<lp::LinearProgram> linear_program;
protected:
    virtual void initialize() override;
    virtual int compute_heuristic(const State& state) override;
public:
    OperatorCountingHeuristic(const Options& opts);
};

#endif
