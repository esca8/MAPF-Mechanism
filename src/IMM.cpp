#include "IMM.h"
#include <boost/math/special_functions/factorials.hpp>  // For factorial
#include <chrono>     // std::chrono::system_clock

// #include "PathTable.h"
// #include "SpaceTimeAStar.h"

// TODO: add runtime statistics, print-graph functions

// get all combinations of reversing any number of edges. If acyclic, add to queue.
void IMM::reverse_all_edge_combos_helper(graph_t graph_orig, graph_t graph_upd, int ind1, set<int>::iterator ind2)
{
    if(ind2 == graph_orig[ind1].end())
    {
        int next_ind = min(graph_orig.size()-1, ind1+1);
        return reverse_all_edge_combos_helper(graph_orig, graph_upd, ind1 + 1,
                                              graph_orig[next_ind].begin());
    }
    if(ind1 == graph_orig.size())
    {
        if(!containsCycle(graph_upd))
        {
            // TODO: store hash instead of graph (then decode)?
            PartialOrdersQueue.push(std::make_pair(graph_upd, hashDAG(graph_upd)));
        }
    }
    int src = ind1;
    int dst = *ind2; ++ind2;
    reverse_all_edge_combos_helper(graph_orig, graph_upd, ind1, ind2);
    reverseEdge(graph_upd, src, dst);
    reverse_all_edge_combos_helper(graph_orig, graph_upd, ind1, ind2);
    reverseEdge(graph_upd, src, dst);
}

void IMM::reverse_all_edge_combos(graph_t graph)
{
    // vector<vector<int>> graph_vec;
    vector<set<int>> graph_upd;
    for (int i = 0; i < graph.size(); i++)
    {
        // graph_vec.push_back(vector<int>(graph[i].begin(), graph[i].end()));
        graph_upd.push_back(graph[i]);
    }

    reverse_all_edge_combos_helper(graph, graph_upd, 0, graph[0].begin());
}

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

void IMM::print_partial_order(graph_t& partial_order) {
    for (size_t i = 0; i < partial_order.size(); ++i) {
        if(partial_order[i].empty()) continue;
        std::cout << "Agent " << i << " depends on : { ";
        for (const int& element : partial_order[i]) {
            std::cout << element << " ";
        }
        std::cout << "}\n";
    }
}

void IMM::run(int n, double time_out_sec) {
    bool saveResult = true;
    memo.clear();
    vector<int> total_order_permutation(agents.size());
    iota(total_order_permutation.begin(), total_order_permutation.end(), 0);
    int i = 0;

    PP* pp = new PP(instance, screen, seed);
    pp->setLowLevelSolver(true);
    while (!PartialOrdersQueue.empty())
    {
        graph_t partial_order;
        string po_hash;
        std::tie(partial_order, po_hash) = PartialOrdersQueue.back();
        PartialOrdersQueue.pop();

        if(screen > 1)
        {
            cout << "partial ord: ";
            print_partial_order(partial_order);
            std::cout << std::endl;
        }

        // consistent total order
        vector<int> total_order = topoSort(partial_order);

        if(screen >= 1)
        {
            cout << "total ord (uncached): ";
            for (const int id : total_order) {
                cout << id << " ";
            }
            cout << endl;
        }
        pp->reset();
        pp->ordering = total_order;

        int failed_agent_id = -1;
        vector<set<int>> partial_order_2;
        double sum_of_cost, curr_welfare;
        std::tie(partial_order_2, sum_of_cost, curr_welfare) = pp->run_once(failed_agent_id, 0, time_out_sec);
        memo2.insert( std::make_pair(po_hash, std::make_pair(sum_of_cost, curr_welfare)));
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

        string po2_hash = hashDAG(partial_order_2);
        clock_t t1 = clock();
        if(!memo2.contains(po2_hash)) {
            reverse_all_edge_combos(partial_order_2);

            runtime_uncached_pp1 += (double)(clock() - t1) / CLOCKS_PER_SEC;
        }

        if(saveResult)
        {
            std::stringstream result;
            std::copy(total_order.begin(), total_order.end(), std::ostream_iterator<int>(result, "_"));
            pp->storeBestPath();
            pp->savePaths((logdir / result.str().c_str()).string());
        }
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
