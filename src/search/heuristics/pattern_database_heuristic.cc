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

    closed_list_collection.resize(pattern_collection.size());
    list_collection.resize(pattern_collection.size());
    adjList_collection.resize(pattern_collection.size());
    N_ind.assign(pattern_collection.size(), 0);
    //N_ind_collection.resize(pattern_collection.size());


    vector <int> n_arr(g_variable_domain.size());
    fill(n_arr.begin(), n_arr.end(), 0);
    for (auto& pat : pattern_collection) {
        int N = 1;
        int count = 0;
        N_ind_collection.push_back(n_arr);
        for (auto& i : pat) {
            N_ind_collection[count][i] = N;
            N *= g_variable_domain[i]; //Compute 

        }
        N_ind[count] = N;
        count++;
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
    for (auto& pat : pattern_collection) {
        vector <int> s(g_variable_domain.size());
        for (int r = 0; r < N_ind[ind]; ++r) {
            for (auto& j : pat) {
                s[j] = unrank(r, j, ind);
            }
            if (goal_test(s, pat)) {
                PDB_collection[ind][r] = 0;
                closed_list_collection[ind].insert(r);
                list_collection[ind].push(r);
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
        computePDB();

        // TODO implementation
    }

}



void PDBHeuristic::Dijkstra(int ind) { //This is actual breadth-first search with unit cost
    int h = 0;


    while (!list_collection[ind].empty()) {
        h++;
        int size = list_collection[ind].size();
        while (size > 0) {
            int front = list_collection[ind].front();
            closed_list_collection[ind].insert(front);
            if (adjList_collection[ind].find(front) != adjList_collection[ind].end()) {
                for (auto& neighbour : adjList_collection[ind].at(front)) {
                    if (closed_list_collection[ind].find(neighbour) == closed_list_collection[ind].end()) {
                        list_collection[ind].push(neighbour);
                        PDB_collection[ind][neighbour] = h;
                    }
                }
            }


            list_collection[ind].pop();
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
    int max = 0;
    for (size_t i = 0; i < pattern_collection.size(); i++) {
        int h = PDB_collection[i][pattern_rank.at(i)];
        if (h == -1) {
            return DEAD_END;
        }
        else if (h > max) {
            max = h;
        }
    }
    return max;
}

void PDBHeuristic::create_orthogonality_graph() {
    orthogonality_graph.resize(pattern_collection.size());
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
/*
vector<vector<int>> PDBHeuristic::find_cliques() {
    vector<vector<int>> cliques;
    for (int i = 0;i<orthogonality_graph.size();i++) {
        unordered_set <int> list;
        list.insert(i);
        for (int j = 0;j<orthogonality_graph[i].size();j++) {
            if (orthogonality_graph[i][j] == true) {
                list.insert(j);
            }
        }
    }
    return cliques;
}*/

vector <int> PDBHeuristic::find_adjoinable_vertex(unordered_set<int> Q) {
    unordered_set <int> temp;
    vector <int> adjoinable_nodes;
    for (int i = 0; i < orthogonality_graph.size(); i++) {
        if (find(Q.begin(), Q.end(), i) == Q.end())
            temp = Q;
        temp.insert(i);
        if (is_clique(temp)) {
            adjoinable_nodes.push_back(i);
        }
    }
    return adjoinable_nodes;
}


unordered_set<int> PDBHeuristic::adjoinable_vertices(unordered_set<int> Q) { //Expand clique until its maximal
    vector <int> adjoinable_nodes = find_adjoinable_vertex(Q);
    unordered_set <int> temp;

    int max = 0;
    int node = -1;
    for (auto ad_node : adjoinable_nodes) {
        temp = Q;
        temp.insert(ad_node);
        vector <int> ad_vector = find_adjoinable_vertex(temp);
        if (max < ad_vector.size()) {
            node = ad_node;
            max = ad_vector.size();
        }
    }
    if (node == -1) {
        return Q;
    }
    else {
        Q.insert(node);
        return Q;
    }


}
bool PDBHeuristic::compare_cliques(unordered_set<int>& A, unordered_set <int>& B) {
    int count = 0;
    for (auto& node : A) {
        if (find(B.begin(), B.end(), node) != B.end()) {
            count++;
        }
    }
    return count == B.size(); //If all nodes from A are found in B, then they are equal. A cannot be a subset of B, because of the previous expansion
}

vector <vector<int>> PDBHeuristic::find_max_cliques() {
    //The algorithm is far away from potential worst-case performance Guarantees like O(3^{n/3}) as stated by
    // Etsuji Tomita, Akira Tanaka and Haruhisa Takahashi   in 
    // The Worst-Case Time Complexity for Generating All Maximal Cliques
    //Proceedings of the 10th Annual International Conference on Computing and Combinatorics(COCOON 2004), pp. 161 - 170, 2004.

    //However the algorithm is quite complex and the orthogonality graphs are quite small
    vector <unordered_set<int>> max_cliques;
    vector<vector<int>> clique_pattern;
    for (int i = 0; i < orthogonality_graph.size(); i++) {
        unordered_set <int> temp;
        temp.insert(i);
        max_cliques.push_back(adjoinable_vertices(temp));
    }

    //Delete duplicates
    for (size_t i = 0; i < max_cliques.size(); i++) {
        for (size_t j = i + 1; j < max_cliques.size(); j++) {
            if (!max_cliques[i].empty() && !max_cliques[j].empty()) {
                if (compare_cliques(max_cliques[i], max_cliques[j])) {
                    max_cliques[j].clear();
                }
            }

        }
    }

    for (auto& max_clique : max_cliques) {
        if (!max_clique.empty()) {
            vector <int> clique;
            for (auto& node : max_clique) {
                clique.push_back(node);
            }
            clique_pattern.push_back(clique);
        }
    }
    return clique_pattern;


}

/*
unordered_set<int> PDBHeuristic::clique(unordered_set<int> set) {


    if (is_clique(set)) {
        return set;
    }
    unordered_set <int> temp = set;
    //int max = 0;
    for (auto& node : set) {
        temp.erase(node);
        temp = clique(temp);
    }

    return set;

}*/
bool PDBHeuristic::is_clique(unordered_set<int>& set) {
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
