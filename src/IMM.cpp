#include "IMM.h"
#include <boost/math/special_functions/factorials.hpp>  // For factorial
#include <chrono>     // std::chrono::system_clock

// #include "PathTable.h"
// #include "SpaceTimeAStar.h"

// TODO: add runtime statistics, print-graph functions

IMM::IMM(Instance& instance, int screen, int seed, boost::filesystem::path logdir)
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
    this->best_paths.resize(this->agents.size(), nullptr);
    // TODO: delete later
    this->logdir = logdir;
}

void print_memo_entry(tuple<vector<set<int>>, double, double>& memo_entry)
{
    cout << "== partial order: ==" << endl;
    IMM::print_partial_order(get<0>(memo_entry));
    cout << " -- welfare: " << get<1>(memo_entry) << endl;
    cout << " -- cost: " << get<2>(memo_entry) << endl;
    cout << "====" << endl;
}

// void print_memo(const vector<pair<vector<set<int>>, double>>& data) {
//     for (const auto& [fst, snd] : data) {
//         const auto& vec = fst;  // Vector of sets
//
//         std::cout << "welfare: " << snd << "\n";
//
//         std::cout << "partial order: [\n";
//         for (const auto& s : vec) {
//             std::cout << "  { ";
//             for (const auto& elem : s) {
//                 std::cout << elem << " ";
//             }
//             std::cout << "}\n";
//         }
//         std::cout << "]\n";
//     }
// }

void IMM::print_partial_order(vector<set<int>>& partial_order) {
    for (size_t i = 0; i < partial_order.size(); ++i) {
        if(partial_order[i].empty()) continue;
        std::cout << "Agent " << i << " depends on : { ";
        for (const int& element : partial_order[i]) {
            std::cout << element << " ";
        }
        std::cout << "}\n";
    }
}

// Check if total_order_permutation is consistent with any of the cached partial_orders.
bool IMM::is_in_memo(const vector<int>& total_order_permutation) {
    for (const auto& cached_result : memo) {
        vector<set<int>> partial_order = std::get<0>(cached_result);
        bool is_broken = false;
        for(int i = 0; i < partial_order.size(); i++) {
            for (const int j : partial_order[i]) {
                // partial_order[i][j]: i has lower priority than j
                if(total_order_permutation[i] < total_order_permutation[j])
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

// If there are stronger partial orders in the memo, remove them.
int IMM::removeStrongerMemoEntry(vector<set<int>> partial_order)
{
    int num_removed = 0;
    int ind = 0;
    while(ind < memo.size())
    {
        tuple<vector<set<int>>, double, double>& cached_result = memo[ind];
        vector<set<int>> memo_partial_order = get<0>(cached_result);
        bool is_consistent = true;
        for(int i = 0; i < partial_order.size(); i++)
        {
            for(int j : partial_order[i])
            {
                // j has lower priority than i
                if(memo_partial_order[i].find(j) == memo_partial_order[i].end())
                {
                    is_consistent = false;
                    break;
                }
            }
        }
        if(is_consistent)
        {
            // if consistent, remove the stronger memo entry
            if(screen > 1)
            {
                cout << " ++ Removing stronger entry: " << endl;
                print_memo_entry(cached_result);
            }
            memo.erase(memo.begin() + ind);
            num_removed++;
        } else
        {
            ind++;
        }
    }
    return num_removed;
}

void IMM::run(int n, double time_out_sec) {
    bool saveResult = true;
    memo.clear();
    vector<int> total_order_permutation(agents.size());
    iota(total_order_permutation.begin(), total_order_permutation.end(), 0);
    int i = 0;
    while (true) {
        i++;
        if(screen > 1)
        {
            cout << "total ord: ";
            for (const int id : total_order_permutation) {
                std::cout << id << " ";
            }
            std::cout << std::endl;
        }
        clock_t t1 = clock();
        if(!is_in_memo(total_order_permutation)) {
            vector total_order = vector<int>(agents.size(), 0);
            for (int i = 0; i < total_order_permutation.size(); i++)
            {
                total_order[total_order_permutation[i]] = i;
            }

            if(screen >= 1)
            {
                for (const int id : total_order) {
                    cout << id << " ";
                }
                cout << endl;
            }
            cout << "   uncached " << endl;
            // TODO: move out of loop
            PP* pp = new PP(instance, screen, seed);
            pp->setLowLevelSolver(true);
            pp->reset();
            pp->ordering = total_order;

            int failed_agent_id = -1;
            vector<set<int>> partial_order;
            double sum_of_cost, curr_welfare;
            std::tie(partial_order, sum_of_cost, curr_welfare) = pp->run_once(failed_agent_id, 0, time_out_sec);

            // Remove stronger partial orders
            removeStrongerMemoEntry(partial_order);

            memo.emplace_back(partial_order, curr_welfare, sum_of_cost);
            if(screen >= 1)
            {
                cout << "partial order: " << endl;
                print_partial_order(partial_order);
            }

            runtime_uncached_pp1 += (double)(clock() - t1) / CLOCKS_PER_SEC;
            if(screen > 0) cout << "    welfare: " << curr_welfare << endl;
            if(screen > 0) cout << "    cost: " << sum_of_cost << endl;
            if (solution_cost == -2 || curr_welfare > max_social_welfare)
            {
                solution_cost = sum_of_cost;
                max_social_welfare = curr_welfare;
            } else if(curr_welfare == max_social_welfare)
            {
                solution_cost = min(solution_cost, sum_of_cost);
            }

            if(saveResult)
            {
                std::stringstream result;
                std::copy(total_order.begin(), total_order.end(), std::ostream_iterator<int>(result, "_"));
                pp->storeBestPath();
                pp->savePaths((logdir / result.str().c_str()).string());
            }

        } else
        {
            runtime_cached_lookup += (double)(clock() - t1) / CLOCKS_PER_SEC;
            // if(screen > 0)
            // {
            //     cout << " --runtime cached: " << runtime_cached_lookup << endl;
            //     cout << " --runtime uncached: " << runtime_uncached_pp1 << endl;
            // }
            if(screen >= 2 && i % 1000 == 0)
            {
                auto total_iters = boost::math::factorial<double>(agents.size());
                cout << "progress: " << (((float)i)/total_iters) << endl;
            }
        }
        clock_t t2 = clock();
        if(!std::next_permutation(total_order_permutation.begin(), total_order_permutation.end())) break;
        runtime_update_permutation += (double)(clock() - t2) / CLOCKS_PER_SEC;
    }

    if(screen > 0)
    {
        cout << "results: " << "[cost]" << solution_cost << " [welfare]" << max_social_welfare << endl;
        cout << "runtimes: " << "[uncached]" << runtime_uncached_pp1 << " [cached]" << runtime_cached_lookup << " [shuffle]" << runtime_update_permutation << endl;
        cout << "memo size: " << memo.size() << endl;
        cout << "memo: " << endl;
        for(auto & [fst, snd, third] : memo)
        {
            cout << "== dependency graph: ==" << endl;
            print_partial_order(fst);
            cout << " -- welfare: " << snd << endl;
            cout << " -- cost: " << third << endl;
        }
    }
}
