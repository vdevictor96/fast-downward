#include "pattern_database_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"

#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>



using namespace std;


PDBHeuristic::PDBHeuristic(const Options& opts)
    : Heuristic(opts),
    m_test_pattern(opts.contains("test_pattern") ?
        opts.get_list<int>("test_pattern") : vector<int>()) {

}
void PDBHeuristic::initialize()
{
    cout << "Initializing PDB heuristic..." << endl;
    if (!m_test_pattern.empty()) {
        // Use m_test_pattern
        pattern_collection.push_back(m_test_pattern);
        computePDB();

        // TODO implementation
    }
    else {
        // Use automatic method
        pattern_collection.push_back({ 1,4 });
        pattern_collection.push_back({ 2 });
        pattern_collection.push_back({ 3 });
        pattern_collection.push_back({ 4,5 });
        computePDB();
        if (pattern_collection.size() > 1) {
            create_orthogonality_graph();
            max_cliques = find_cliques();
        }


    }

}
int PDBHeuristic::unrank(int r, int var, int ind) {
    int temp = r / N_ind_collection[ind][var];
    return temp % g_variable_domain[var];
}
int PDBHeuristic::rankState(const State& state, int ind) {
    int sum = 0;
    for (auto& i : pattern_collection[ind]) {
        sum = N_ind_collection[ind][i] * state[i];
    }
    return sum;
}

int PDBHeuristic::rank(vector <int>& s, int ind) {
    int sum = 0;
    for (auto& i : pattern_collection[ind]) {
        sum += N_ind_collection[ind][i] * s[i];
    }
    return sum;
}
bool PDBHeuristic::goal_test(vector <int>& s, vector <int>& pat) {
    for (auto& i : g_goal) {
        if (find(pat.begin(), pat.end(), i.first) != pat.end()) {
            if (s[i.first] != i.second) {
                return false;
            }
        }
    }
    return true;
}
vector <pair<Operator, int>> PDBHeuristic::check_applicable_ops(vector <int>& pat) {
    vector <pair<Operator, int>> pattern_ops;
    int count = 0;
    for (auto& op : g_operators) {
        for (auto& eff : op.get_effects()) {
            if (find(pat.begin(), pat.end(), eff.var) != pat.end()) {
                pattern_ops.push_back(make_pair(op, count));
                break;
            }
        }
        count++;

    }
    return pattern_ops;
}

bool PDBHeuristic::op_applicable(Operator& op, vector<int>& s, vector <int>& pat) {
    for (auto& i : op.get_preconditions()) {
        if (find(pat.begin(), pat.end(), i.var) != pat.end()) {
            if (s[i.var] != i.val) {
                return false;
            }
        }

    }
    return true;
}
void PDBHeuristic::apply_operation(Operator& op, vector <int>& s, vector<int>& pat) {
    for (auto& eff : op.get_effects()) {
        if (find(pat.begin(), pat.end(), eff.var) != pat.end()) {
            s[eff.var] = eff.val;
        }
    }
}

void PDBHeuristic::computePDB() {

    N_ind.assign(pattern_collection.size(), 0);
    adjList_collection.resize(pattern_collection.size());

    closed_list.clear();
    while (!open_list.empty()) {
        open_list.pop();
    }

    //Init vectors for ranking/unranking/perfect hach for each pattern
    vector <int> n_arr(g_variable_domain.size());
    fill(n_arr.begin(), n_arr.end(), 0);
    int ind_max = 0;
    for (auto& pat : pattern_collection) {
        int N = 1;
        N_ind_collection.push_back(n_arr);
        for (auto& i : pat) {
            N_ind_collection[ind_max][i] = N;
            N *= g_variable_domain[i]; //Compute 

        }
        N_ind[ind_max] = N;
        ind_max++;
    }


    //Init PDB with -1 for all available patterns
    for (size_t i = 0; i < pattern_collection.size(); i++) {
        vector<int> s(N_ind[i]);
        fill(s.begin(), s.end(), -1);
        PDB_collection.push_back(s);
    }

    //Find applicable operations for each pattern
    for (auto& pat : pattern_collection) {
        applicable_ops_collection.push_back(check_applicable_ops(pat));
    }
    int ind = 0;
    //for each pattern compute PDB
    for (auto& pat : pattern_collection) {
        vector <int> s(g_variable_domain.size());
        for (int r = 0; r < N_ind[ind]; ++r) {
            for (auto& j : pat) {
                s[j] = unrank(r, j, ind);
            }
            if (goal_test(s, pat)) {
                PDB_collection[ind][r] = 0;
                closed_list.insert(r);
                open_list.push(r);
            }
            for (auto& op : applicable_ops_collection[ind]) {
                if (op_applicable(op.first, s, pat)) {
                    vector <int> s2 = s;
                    apply_operation(op.first, s2, pat);
                    int r2 = rank(s2, ind);
                    if (adjList_collection[ind].find(r2) == adjList_collection[ind].end()) {
                        adjList_collection[ind].insert(make_pair(r2, r));
                    }
                    else {
                        adjList_collection[ind].at(r2).insert(r); //The graph is backwards
                    }
                }
            }
        }
        Dijkstra(ind);
        ind++;

    }



}




void PDBHeuristic::Dijkstra(int ind) { //This is actual breadth-first search with unit cost
    int h = 0;


    while (!open_list.empty()) {
        h++;
        int size = open_list.size();
        while (size > 0) {
            int front = open_list.front();
            closed_list.insert(front);
            if (adjList_collection[ind].find(front) != adjList_collection[ind].end()) {
                for (auto& neighbour : adjList_collection[ind].at(front)) {
                    if (closed_list.find(neighbour) == closed_list.end()) {
                        open_list.push(neighbour);
                        PDB_collection[ind][neighbour] = h;
                    }
                }
            }


            open_list.pop();
            size--;
        }


    }
}


int PDBHeuristic::compute_heuristic(const State& state)
{
    vector<int> pattern_rank(pattern_collection.size());
    for (size_t i = 0; i < pattern_collection.size(); i++) {
        pattern_rank.push_back(rankState(state, i));
    }

    // Canocial pattern database heuristic


    if (!max_cliques.empty()) {
        return compute_canonical_h(pattern_rank);
    }
    else {
        int h = PDB_collection[0][pattern_rank.at(0)];
        if (h == -1) {
            return DEAD_END;
        }
        else {
            return h;
        }
    }


}
int PDBHeuristic::compute_canonical_h(vector<int> pattern_rank) {
    int max = 0;
    for (auto& clique : max_cliques) {
        int h_sum = 0;
        for (auto& pat_ind : clique) {
            int h = PDB_collection[pat_ind][pattern_rank.at(pat_ind)];
            if (h == -1) {
                return DEAD_END;
            }
            h_sum += PDB_collection[pat_ind][pattern_rank.at(pat_ind)];
        }
        if (h_sum > max) {
            max = h_sum;
        }
    }
    return max;
}

void PDBHeuristic::create_orthogonality_graph() {

    //orthogonality_graph.resize(pattern_collection.size());
    //Create a matrix with the  patterns as nodes and arcs means orthogonality between patterns
    vector<bool> row(pattern_collection.size(), true);
    for (int i = 0; i < pattern_collection.size(); i++) {
        orthogonality_graph.push_back(row);
    }

    for (int i = 0; i < applicable_ops_collection.size(); ++i) {
        for (int j = i + 1; j < applicable_ops_collection.size(); ++j) {

            //Check whether two applicable operators are the same for two patterns. If there are, they are by definition not orthogonal
            for (auto& op1 : applicable_ops_collection[i]) {

                for (auto& op2 : applicable_ops_collection[j]) {
                    if (op1.second == op2.second) {
                        orthogonality_graph[i][j] = false;
                        orthogonality_graph[j][i] = false; //For symmetry
                        break;
                    }
                }
                if (orthogonality_graph[i][j] == false) {
                    break;
                }
            }
        }
    }
    for (int i = 0; i < orthogonality_graph.size(); i++) {
        orthogonality_graph[i][i] = true;
    }

}
vector<vector<int>> PDBHeuristic::find_cliques() {
    vector<vector<int>> cliques;
    //Iterate for each node list. All edges that are found for a maximal clique of a given node are deleted
    for (int i = 0; i < orthogonality_graph.size(); i++) {
        vector <int> list;
        for (int j = 0; j < orthogonality_graph[i].size(); j++) {
            if (orthogonality_graph[i][j] == true) {
                //Insert all edges for the given node into a list
                list.push_back(j);
            }
        }
        cliques.push_back(clique(list));
    }
    return cliques;
}
vector<int> PDBHeuristic::clique(vector<int> set) {
    if (is_clique(set)) {
        return set;
    }
    vector <int> temp = set;
    vector <int> temp2;
    vector <int> max_clique;
    int max = 0;
    for (int i = 0; i < set.size();i++) {
        temp.erase(temp.begin()+i);
        temp2 = clique(temp);
        //It should be irrelevant that we do not distinguish between equal sized cliques
        if (temp2.size() > max) {
            max = temp2.size();
            max_clique = temp2;
        }
    }
    //All edges that are contained in the max_clique can be deleted in the graph
    for (auto& node1 : max_clique) {
        for (auto& node2 : max_clique) {
            orthogonality_graph[node1][node2] = false;
        }

    }

    return max_clique;

}
bool PDBHeuristic::is_clique(vector<int> set) {
    for (auto& node1 : set) {
        for (auto& node2 : set) {
            if (!orthogonality_graph[node1][node2]) {
                return false;
            }
        }
    }
    return true;
}



static Heuristic* _parse(OptionParser& parser)
{
    Heuristic::add_options_to_parser(parser);
    parser.add_list_option<int>("test_pattern", "", "[]", OptionFlags(false));
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    }
    else {
        return new PDBHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("pdb", _parse);
