#ifndef MAX_HEURISTIC_H
#define MAX_HEURISTIC_H

#include "../heuristic.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include "../operator.h"

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
    virtual int compute_heuristic(const State& state);

public:
    MaxHeuristic(const Options& options);
    ~MaxHeuristic() = default;
private:
    std::vector<int> hash_arr;
    int compute_hash(std::pair<int, int>& x);
    void init_layers_and_counter(const State& state);
    void achieve_facts(int& pos);
    void small_iteration();
    bool doStep();
    void queue_clear(std::queue<std::pair<int, int>>& q);
    std::vector<int> counter;
    std::queue <std::pair<int, int>> fact_schedule;
    int timestep;
    int req_goal;
    int count;
    std::vector <std::pair<int, std::unordered_set<int>>> fact_set;


    std::vector <int> goal_set;

};

#endif

