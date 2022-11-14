#include "red_black_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"

#include <algorithm>

using namespace std;

RedBlackHeuristic::RedBlackHeuristic(const Options &opts)
    : FFHeuristic(opts)
{
}

void RedBlackHeuristic::initialize()
{
    cout << "Initializing red-black heuristic..." << endl;
    //TODO implementation: decide painting 
}

int RedBlackHeuristic::compute_heuristic(const State &/*state*/)
{
    // TODO implementation

    // 1) Call FFHeuristic to obtain a relaxed plan (you can add a protected method in
    // FFHeuristic to do this)
    
    // 2) Extend the relaxed plan with additional actions that achieve the value of black
    // preconditions
    return -1;
}

void RedBlackHeuristic::get_helpful_actions(std::vector<const Operator *>
        &/*result*/)
{
    // TODO implementation
}

bool RedBlackHeuristic::is_red_black_plan(
    const State &state,
    const std::vector<bool> &painting,
    const std::vector<const Operator *> &rb_plan)
{
    struct t_rb_value {
        virtual ~t_rb_value() {}
        virtual bool set(int val) = 0;
        virtual bool is_set(int val) const = 0;
    };
    struct t_r_value : public t_rb_value {
        std::vector<bool> m_values;
        t_r_value(int var, int val)
            : t_rb_value(), m_values(g_variable_domain[var], false)
        {
            m_values[val] = true;
        }
        virtual bool set(int val)
        {
            bool isset = m_values[val];
            m_values[val] = true;
            return !isset;
        }
        virtual bool is_set(int val) const
        {
            return m_values[val];
        }
    };
    struct t_b_value : public t_rb_value {
        int val;
        t_b_value(int val) : t_rb_value(), val(val) {}
        virtual bool set(int val)
        {
            int oldval = this->val;
            this->val = val;
            return oldval != val;
        }
        virtual bool is_set(int val) const
        {
            return this->val == val;
        }
    };
    struct t_rb_action {
        const Operator *m_op;
        t_rb_action(const Operator *op) : m_op(op) {}
        bool is_applicable(const std::vector<t_rb_value *> &reached) const
        {
            for (const Condition &pre : m_op->get_preconditions()) {
                if (!reached[pre.var]->is_set(pre.val)) {
                    return false;
                }
            }
            return true;
        }
        bool apply(std::vector<t_rb_value *> &reached) const
        {
            bool res = false;
            for (const Effect &eff : m_op->get_effects()) {
                res = reached[eff.var]->set(eff.val) || res;
            }
            return res;
        }
    };
    std::vector<t_rb_value *> reached;
    reached.resize(g_variable_domain.size());
    for (unsigned var = 0; var < g_variable_domain.size(); var++) {
        if (painting[var]) {
            reached[var] = new t_b_value(state[var]);
        } else {
            reached[var] = new t_r_value(var, state[var]);
        }
    }
    std::vector<t_rb_action> black_part;
    std::vector<t_rb_action> red_part;
    for (unsigned i = 0; i < rb_plan.size(); i++) {
        bool is_black = false;
        for (const Effect &eff : rb_plan[i]->get_effects()) {
            if (painting[eff.var]) {
                is_black = true;
                break;
            }
        }
        if (is_black) {
            black_part.emplace_back(rb_plan[i]);
        } else {
            red_part.emplace_back(rb_plan[i]);
        }
    }
    std::reverse(black_part.begin(), black_part.end());
    bool state_changed = true;
    while (state_changed) {
        while (state_changed) {
            state_changed = false;
            std::vector<t_rb_action>::iterator it = red_part.begin();
            while (it != red_part.end()) {
                if (it->is_applicable(reached)) {
                    state_changed = it->apply(reached) || state_changed;
                    it = red_part.erase(it);
                } else {
                    it++;
                }
            }
        }
        if (!black_part.empty() && black_part.back().is_applicable(reached)) {
            state_changed = black_part.back().apply(reached);
            black_part.pop_back();
        }
    }
    bool is_valid = true;
    for (const std::pair<int, int> &g : g_goal) {
        if (!reached[g.first]->is_set(g.second)) {
            is_valid = false;
            break;
        }
    }
    for (unsigned var = 0; var < reached.size(); var++) {
        delete(reached[var]);
    }
    return red_part.empty() && black_part.empty() && is_valid;
}

static Heuristic *_parse(OptionParser &parser)
{
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    } else {
        return new RedBlackHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("red-black", _parse);
