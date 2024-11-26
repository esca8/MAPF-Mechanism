#include "IMM.h"
// #include "PathTable.h"
// #include "SpaceTimeAStar.h"

// TODO: add runtime statistics, print-graph functions

constexpr bool DEBUG = false;

IMM::IMM(Instance& instance, int screen, int seed)
    : instance(instance), screen(screen), seed(seed)
{
    agents.reserve(instance.num_of_agents);
    search_engines.resize(instance.num_of_agents);
    for (int i = 0; i < instance.num_of_agents; i++)
    {
        agents.emplace_back(i, instance.start_locations[i],
                            instance.goal_locations[i]);
        search_engines[i] = new SpaceTimeAStar(instance, i);
    }

    this->gen = mt19937(this->seed);

    this->min_sum_of_cost_wo_i.resize(this->agents.size(), MAX_COST);
    this->max_welfare_wo_i.resize(this->agents.size(), INT_MIN);
    this->best_paths.resize(this->agents.size(), nullptr);
}

void print_memo(const vector<pair<vector<set<int>>, double>>& data) {
    for (const auto& [fst, snd] : data) {
        const auto& vec = fst;  // Vector of sets

        std::cout << "welfare: " << snd << "\n";

        std::cout << "partial order: [\n";
        for (const auto& s : vec) {
            std::cout << "  { ";
            for (const auto& elem : s) {
                std::cout << elem << " ";
            }
            std::cout << "}\n";
        }
        std::cout << "]\n";
    }
}

void IMM::print_partial_order(vector<set<int>>& partial_order) {
    for (size_t i = 0; i < partial_order.size(); ++i) {
        if(partial_order[i].empty()) continue;
        std::cout << "Set " << i + 1 << ": { ";
        for (const int& element : partial_order[i]) {
            std::cout << element << " ";
        }
        std::cout << "}\n";
    }
}

// check if total_order is consistent with any of the cached partial_orders
bool IMM::is_in_memo(const vector<int>& total_order) {
    size_t n = this->agents.size();
    int indices[n];
    for(int i = 0; i < n; i++) {
        indices[total_order[i]] = i;
    }

    for (const auto& cached_result : memo) {
        vector<set<int>> partial_order = cached_result.first;
        bool is_broken = false;
        for(int i = 0; i < partial_order.size(); i++) {
            for (const int j : partial_order[i]) {
                // partial_order[i][j]: i has lower priority than j
                if(indices[i] < indices[j])
                {
                    is_broken = true;
                    break;
                }
            }
        }
        if(!is_broken) return true;
    }
    return false;
}

void IMM::run(int n, double time_out_sec) {
    vector<int> total_ordering(agents.size());
    iota(total_ordering.begin(), total_ordering.end(), 0);
    int i = 0;
    while (true) {
        i++;
        if(DEBUG)
        {
            cout << "== imm total ord: " << endl;
            for (const int id : total_ordering) {
                std::cout << id << " ";
            }
            std::cout << std::endl;
        }
        if(!is_in_memo(total_ordering)) {
            // cout << i << ": out" << endl;
            PP* pp = new PP(instance, screen, seed);
            pp->setLowLevelSolver(true);
            pp->reset();
            pp->ordering = total_ordering;

            int failed_agent_id = -1;
            vector<set<int>> partial_order;
            double sum_of_cost, curr_welfare;
            std::tie(partial_order, sum_of_cost, curr_welfare) = pp->run_once(failed_agent_id, 0, time_out_sec);
            cout << "welfare: " << curr_welfare << endl;
            cout << "cost: " << sum_of_cost << endl;
            memo.emplace_back(partial_order, curr_welfare);
            pp->clearSearchEngines();
        } else
        {
            // cout << i << ": in" << endl;
        }
        if(!std::next_permutation(total_ordering.begin(), total_ordering.end())) break;
    }

    if(DEBUG)
    {
        cout << "memo size: " << memo.size() << endl;
        print_memo(memo); cout << endl;
        for(auto & [fst, snd] : memo)
        {
            cout << "== dep graph: ==" << endl;
            IMM::print_partial_order(fst);
            cout << "== welfare: ==" << endl << snd << endl;
        }
    }
}
