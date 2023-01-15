#ifndef PDB_HEURISTIC_H
#define PDB_HEURISTIC_H

#include "../heuristic.h"
#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>


// Usage example: the command line option for using h^{PDB} in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(pdb())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.


/* The PDB heuristic is based on particular type of abstractions that is obtained by
 * considering the problem obtained by using only a subset of the state variables
 * (the pattern).
 *
 * The implementation of a single PDB heuristic requires you to

 * 1) Generate one (or multiple, see below) pattern(s). If test_pattern is provided you
 * MUST use the provided pattern (this is important for testing), else you should use your
 * own strategy as described below.
 * 2) Construct the state space of the syntactic projection of the pattern.
 * 3) Compute the optimal goal distances for all states of the state space of
 * the syntactic projection.
 * 4) Store a table for heuristic lookup during search.
 * 5) Heuristic lookup: given a state of the original problem, lookup the value
 * of its syntactic projection on the pattern.
 *
 * 1-4 should be done in initialize() to allow for fast heuristic lookup during
 * search.
 *
 * 5 should be used in compute_heuristic
 *
 * Pattern generation: we have not discussed pattern generation strategies in the
 * lecture. In principle any choice of the pattern (e.g. randomly selecting a subset of
 * variables) is accepted. However, note that the heuristic quality entirely depends on
 * how the pattern is selected (e.g. the PDB heuristic whose pattern does not contain a
 * goal variable, will return 0 for all states). Thus, we'll give extra points depending
 * on the generation algorithm you have implemented.
 *
 * Additive patterns: if you want to use multiple pattern database heuristics
 * (e.g. canonical PDB heuristic using multiple patterns), we suggest to move the code of
 * single PDB heuristic (using one pattern) to a seperate class, having the pattern as
 * constructor parameter.
 *
 * NOTE: Summing up multiple PDB heuristics might result in an inadmissible heuristic!
 * (of course depending on how you combine them, and how you choose the different
 * patterns). You should avoid this by using the canonical heuristic as described in the
 * lecture.
 *
 * IMPORTANT: We will use the test pattern parameter to test your implementation with
 * manually generated patterns. If a pattern is provided by the command line, this will be
 * stored in the m_test_pattern variable. So if m_test_pattern is not empty, you must
 * compute the PDB heuristic of the pattern in m_test_pattern. The pattern is provided as
 * a list of variable indexes specifying the variables in the pattern. In the command
 * line, you can pass it by e.g. pdb(test_pattern=[3,2]).  In this case, only the
 * variables with index 2 and 3 would be in the pattern.
 * We also recommend you to use this feature to test the PDB construction and lookup.
 *
 */

class PDBHeuristic : public Heuristic
{
protected:
    std::vector<int> m_test_pattern;

    virtual void initialize();
    virtual int compute_heuristic(const State& state);
public:
    PDBHeuristic(const Options& options);
    ~PDBHeuristic() = default;

private:
    struct hashFunction
    {
        size_t operator()(const int& x) const
        {
            return x;
        }
    };
    struct compare
    {
        size_t operator()(const int& x1, const int& x2) const
        {
            if (x1 == x2) {
                return true;
            }
            return false;
        }
    };
    //data structures for regular pattern collections
    std::vector<std::vector<int>> pattern_collection;
    std::vector <std::vector<int>>N_ind_collection;
    std::vector<std::vector<int> > PDB_collection;
    std::vector<std::vector<std::pair<Operator, int>>> applicable_ops_collection;
    std::vector <std::unordered_set <int, hashFunction, compare>> closed_list_collection;
    std::vector <std::unordered_map <int, std::unordered_set <int, hashFunction, compare>>> adjList_collection;
    std::vector<std::queue <int>> list_collection;
    std::vector <int>N_ind;

    //functions for normal PDB
    void Dijkstra(int ind);
    int unrank(int r, int var, int ind);
    int rank(std::vector <int>& s, int ind);
    int rankState(const State& state, int ind);
    bool goal_test(std::vector <int>& s, std::vector <int>& pat);
    bool op_applicable(Operator& op, std::vector<int>& s, std::vector <int>& pat);
    void computePDB();
    void apply_operation(Operator& op, std::vector <int>& s, std::vector<int>& pat);


    //For canonical heuristic and orthogonality
    void create_orthogonality_graph();
    std::vector <std::pair < Operator, int >> check_applicable_ops(std::vector <int>& pat);
    std::vector<std::vector<int>> find_max_cliques();
    bool is_clique(std::unordered_set<int>& set);
    std::unordered_set<int> adjoinable_vertices(std::unordered_set<int> Q);
    std::vector <int> find_adjoinable_vertex(std::unordered_set<int> Q);
    //std::unordered_set<int> clique(std::unordered_set<int> set);
    bool compare_cliques(std::unordered_set<int>& A, std::unordered_set <int>& B);
    std::vector<std::vector<bool>> orthogonality_graph;



};

#endif