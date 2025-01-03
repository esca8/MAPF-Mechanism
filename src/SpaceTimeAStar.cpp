#include "SpaceTimeAStar.h"

void SpaceTimeAStar::updatePath(const LLNode* goal, vector<PathEntry>& path, vector<set<int>>* dependency_graph)
{
    const LLNode* curr = goal;

    path.reserve(curr->g_val + 1);
    while (curr != nullptr)
    {
        path.emplace_back(curr->location);
        for(int prev_agent_id : curr->agents_higher_priority)
        {
            (*dependency_graph)[agent_id].insert(prev_agent_id);
        }
        curr = curr->parent;
    }
    std::reverse(path.begin(), path.end());
}

Path SpaceTimeAStar::findOptimalPath(const ConstraintTable& constraint_table,
                                     int lowerbound, bool dummy_start_node,
                                     vector<set<int>>* dependency_graph)
{
    bool debug = false;
    optimal = true;  // using A* search
    Path path;
    num_expanded = 0;
    num_generated = 0;

    // build constraint table
    auto t = clock();

    if (constraint_table.constrained(start_location, 0, agent_id))
    {
        // cout << "called" << endl;
        return path;
    }

    // the earliest timestep that the agent can hold its goal location. The
    // length_min is considered here.
    // everything is static after this timestep
    auto static_timestep = constraint_table.getMaxTimestep() + 1;

    // generate start and add it to the OPEN list
    AStarNode* start;
    if (dummy_start_node)
    {
        // Create dummy start node if specified
        start = new AStarNode(GLOBAL_VAR::dummy_start_loc, 0,
                              1 + max(lowerbound, my_heuristic[start_location]),
                              nullptr, 0, 0);
    }
    else
    {
        start = new AStarNode(start_location, 0,
                              max(lowerbound, my_heuristic[start_location]),
                              nullptr, 0, 0);
    }

    num_generated++;
    start->open_handle = open_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(start);

    while (!open_list.empty())
    {
        auto* curr = popNode();
        assert(curr->location >= 0);
        // cout << "Expanding " << curr->location << ", Goal: " << goal_location
        //      << endl;
        // check if the popped node is a goal
        if (curr->location == goal_location)  // arrive at the goal location
        {
            updatePath(curr, path, dependency_graph);
            break;
        }

        if (curr->timestep >= constraint_table.length_max)
            continue;

        // Generate next location. For dummy start node, next location is the
        // start location.
        list<int> next_locations;
        if (curr->location == GLOBAL_VAR::dummy_start_loc)
        {
            next_locations.emplace_back(start_location);
            next_locations.emplace_back(curr->location);
        }
        else
        {
            next_locations = instance.getNeighbors(curr->location);
            next_locations.emplace_back(curr->location);
        }
        for (int next_location : next_locations)
        {
            // cout << "Generating " << next_location << endl;
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep)
            {
                // now everything is static, so switch to space A* where we
                // always use the same timestep
                if (next_location == curr->location)
                {
                    continue;
                }
                next_timestep--;
            }

            int constrained_ind = -1;
            if (constraint_table.constrained(next_location, next_timestep, &constrained_ind, agent_id) ||
                constraint_table.constrained(curr->location, next_location,
                                             next_timestep, &constrained_ind, agent_id))
            {
                // TODO: double check
                if(dependency_graph && agent_id != -1
                    && constrained_ind != -1 && agent_id != constrained_ind)
                {
                    if(debug) cout << "add llnode dependency: " << agent_id << "," << constrained_ind << endl;
                    curr->agents_higher_priority.emplace_back(constrained_ind);
                    // (*dependency_graph)[agent_id].insert(constrained_ind);
                }
                if(debug)
                {
                    if (constraint_table.constrained(next_location, next_timestep))
                        cout << "vertex constrained" << endl;
                    if (constraint_table.constrained(curr->location, next_location,
                                                 next_timestep))
                        cout << "edge constrained" << endl;
                }
                continue;
            }

            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            // int next_h_val =
            //     max(lowerbound - next_g_val, my_heuristic[next_location]);
            int next_h_val = my_heuristic[next_location];
            if (next_g_val + next_h_val > constraint_table.length_max)
            {
                // cout << "n_g + n_h = " << next_g_val << " + " << next_h_val << " vs " << "len_max = " << constraint_table.length_max << endl;
                continue;
            }

            // int next_internal_conflicts =
            //     curr->num_of_conflicts +
            //     constraint_table.getNumOfConflictsForStep(
            //         curr->location, next_location, next_timestep);

            // generate (maybe temporary) node
            auto next =
                new AStarNode(next_location, next_g_val, next_h_val, curr,
                              next_timestep, 0);

            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                pushNode(next);
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            auto existing_next = *it;
            // if (existing_next->parent->location ==
            // GLOBAL_VAR::dummy_start_loc)
            // assert(existing_next->parent->location !=
            // next->parent->location); if f-val decreased through this new path
            if (existing_next->getFVal() > next->getFVal() ||
                // or it remains the same but location is "better" (our tie
                // breaking)
                (existing_next->getFVal() == next->getFVal() &&
                 existing_next->parent->location > next->parent->location))
            {
                existing_next->copy(*next);  // update existing node
                // if its in the closed list (reopen)
                if (!existing_next->in_openlist)
                {
                    pushNode(existing_next);
                }
                else
                {
                    // increase because #conflicts improved
                    open_list.increase(existing_next->open_handle);
                }
            }
            // not needed anymore -- we already generated it before
            delete (next);
        }  // end for loop that generates successors
        // cout << endl;
    }  // end while loop

    releaseNodes();
    return path;
}

Path SpaceTimeAStar::findOptimalPath(const HLNode& node,
                                     const ConstraintTable& initial_constraints,
                                     const vector<Path*>& paths, int agent,
                                     int lowerbound, bool dummy_start_node)
{
    optimal = true;  // using A* search
    Path path;
    num_expanded = 0;
    num_generated = 0;

    // build constraint table
    auto t = clock();
    ConstraintTable constraint_table(initial_constraints);
    // initial constraints for PBS already contains all the required constraints
    if (node.getName() != "PBS Node")
    {
        constraint_table.insert2CT(node, agent);
    }
    runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;
    if (constraint_table.constrained(start_location, 0))
    {
        return path;
    }

    t = clock();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;


    return findOptimalPath(constraint_table, lowerbound, dummy_start_node);
}

// find path by time-space A* search
// Returns a bounded-suboptimal path that satisfies the constraints of the give
// node  while minimizing the number of internal conflicts (that is conflicts
// with known_paths for other agents found so far). lowerbound is an
// underestimation of the length of the path in order to speed up the search.
pair<Path, int> SpaceTimeAStar::findSuboptimalPath(
    const HLNode& node, const ConstraintTable& initial_constraints,
    const vector<Path*>& paths, int agent, int lowerbound, double w,
    double agent_w, bool dummy_start_node)
{
    optimal = false;  // using focal search
    this->w = w;
    Path path;
    num_expanded = 0;
    num_generated = 0;

    // build constraint table
    auto t = clock();
    ConstraintTable constraint_table(initial_constraints);
    // initial constraints for PBS already contains all the required constraints
    if (node.getName() != "PBS Node")
    {
        constraint_table.insert2CT(node, agent);
    }
    runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;
    if (constraint_table.constrained(start_location, 0))
    {
        return {path, 0};
    }

    t = clock();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // the earliest timestep that the agent can hold its goal location. The
    // length_min is considered here.
    // everything is static after this timestep
    auto static_timestep = constraint_table.getMaxTimestep() + 1;

    // generate start and add it to the OPEN & FOCAL list
    AStarNode* start;
    if (dummy_start_node)
    {
        // Create dummy start node if specified
        start = new AStarNode(GLOBAL_VAR::dummy_start_loc, 0,
                              1 + max(lowerbound, my_heuristic[start_location]),
                              nullptr, 0, 0);
    }
    else
    {
        start = new AStarNode(start_location, 0,
                              max(lowerbound, my_heuristic[start_location]),
                              nullptr, 0, 0);
    }

    num_generated++;
    start->open_handle = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(start);
    min_f_val = (int)start->getFVal();
    // lower_bound = int(w * min_f_val));

    while (!open_list.empty())
    {
        updateFocalList();  // update FOCAL if min f-val increased
        auto* curr = popNode();
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->location == goal_location)  // arrive at the goal location
        {
            updatePath(curr, path);
            break;
        }

        if (curr->timestep >= constraint_table.length_max)
            continue;

        // Generate next location. For dummy start node, next location is the
        // start location.
        list<int> next_locations;
        if (curr->location == GLOBAL_VAR::dummy_start_loc)
        {
            next_locations.emplace_back(start_location);
            next_locations.emplace_back(curr->location);
        }
        else
        {
            next_locations = instance.getNeighbors(curr->location);
            next_locations.emplace_back(curr->location);
        }
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep)
            {
                // now everything is static, so switch to space A* where we
                // always use the same timestep
                if (next_location == curr->location)
                {
                    continue;
                }
                next_timestep--;
            }

            if (constraint_table.constrained(next_location, next_timestep) ||
                constraint_table.constrained(curr->location, next_location,
                                             next_timestep))
                continue;

            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            int next_h_val = my_heuristic[next_location];
            if (next_g_val + next_h_val > constraint_table.length_max)
                continue;

            // Skip if current path does not satisfy per agent suboptimal bound
            if (agent_w != -1 &&
                next_g_val + next_h_val >
                    agent_w * (1 + my_heuristic[start_location]))
            {
                continue;
            }
            int next_internal_conflicts =
                curr->num_of_conflicts +
                constraint_table.getNumOfConflictsForStep(
                    curr->location, next_location, next_timestep);

            // generate (maybe temporary) node
            auto next =
                new AStarNode(next_location, next_g_val, next_h_val, curr,
                              next_timestep, next_internal_conflicts);

            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                pushNode(next);
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            auto existing_next = *it;
            // if f-val decreased through this new path
            if (existing_next->getFVal() > next->getFVal() ||
                // or it remains the same but there's fewer conflicts
                (existing_next->getFVal() == next->getFVal() &&
                 existing_next->num_of_conflicts > next->num_of_conflicts))
            {
                // if it is in the closed list (reopen)
                if (!existing_next->in_openlist)
                {
                    existing_next->copy(*next);
                    pushNode(existing_next);
                }
                else
                {
                    // check if it was above the focal bound before and now
                    // below (thus need to be inserted)
                    bool add_to_focal = false;

                    // check if it was inside the focal and needs to be updated
                    // (because f-val changed)
                    bool update_in_focal = false;

                    bool update_open = false;

                    // if the new f-val qualify to be in FOCAL
                    if ((next_g_val + next_h_val) <= w * min_f_val)
                    {
                        // and the previous f-val did not qualify to be in
                        // FOCAL then add
                        if (existing_next->getFVal() > w * min_f_val)
                            add_to_focal = true;
                        // and the previous f-val did qualify to be in FOCAL
                        // then update
                        else
                            update_in_focal = true;
                    }
                    if (existing_next->getFVal() > next_g_val + next_h_val)
                        update_open = true;

                    existing_next->copy(*next);  // update existing node

                    // increase because f-val improved
                    if (update_open)
                        open_list.increase(existing_next->open_handle);
                    if (add_to_focal)
                        existing_next->focal_handle =
                            focal_list.push(existing_next);
                    // should we do update? yes, because number of conflicts
                    // may go up or down
                    if (update_in_focal)
                        focal_list.update(existing_next->focal_handle);
                }
            }

            // not needed anymore -- we already generated it before
            delete (next);
        }  // end for loop that generates successors
    }      // end while loop

    releaseNodes();
    return {path, min_f_val};
}

int SpaceTimeAStar::getTravelTime(int start, int end,
                                  const ConstraintTable& constraint_table,
                                  int upper_bound)
{
    int length = MAX_TIMESTEP;

    // everything is static after this timestep
    auto static_timestep = constraint_table.getMaxTimestep() + 1;
    auto root =
        new AStarNode(start, 0, compute_heuristic(start, end), nullptr, 0, 0);
    root->open_handle = open_list.push(root);  // add root to heap
    allNodes_table.insert(root);               // add root to hash_table (nodes)
    AStarNode* curr = nullptr;
    while (!open_list.empty())
    {
        curr = open_list.top();
        open_list.pop();
        if (curr->location == end)
        {
            length = curr->g_val;
            break;
        }
        list<int> next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            int next_g_val = curr->g_val + 1;
            if (static_timestep < next_timestep)
            {
                if (curr->location == next_location)
                {
                    continue;
                }
                next_timestep--;
            }
            if (!constraint_table.constrained(next_location, next_timestep) &&
                !constraint_table.constrained(curr->location, next_location,
                                              next_timestep))
            {  // if that grid is not blocked
                int next_h_val = compute_heuristic(next_location, end);
                // the cost of the path is larger than the upper bound
                if (next_g_val + next_h_val >= upper_bound)
                    continue;
                auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                          nullptr, next_timestep, 0);
                auto it = allNodes_table.find(next);
                if (it == allNodes_table.end())
                {  // add the newly generated node to heap and hash table
                    next->open_handle = open_list.push(next);
                    allNodes_table.insert(next);
                }
                else
                {  // update existing node's g_val if needed (only in the heap)
                    // not needed anymore -- we already generated it before
                    delete (next);
                    auto existing_next = *it;
                    if (existing_next->g_val > next_g_val)
                    {
                        existing_next->g_val = next_g_val;
                        existing_next->timestep = next_timestep;
                        open_list.increase(existing_next->open_handle);
                    }
                }
            }
        }
    }
    releaseNodes();
    return length;
}

inline AStarNode* SpaceTimeAStar::popNode()
{
    AStarNode* node;
    if (optimal)
    {
        node = open_list.top();
        open_list.pop();
    }
    else
    {
        node = focal_list.top();
        focal_list.pop();
        assert(node->in_openlist);
        open_list.erase(node->open_handle);
    }
    node->in_openlist = false;
    num_expanded++;
    return node;
}

inline void SpaceTimeAStar::pushNode(AStarNode* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
    if (!optimal && node->getFVal() <= w * min_f_val)
        node->focal_handle = focal_list.push(node);
}

void SpaceTimeAStar::updateFocalList()
{
    auto open_head = open_list.top();
    if (open_head->getFVal() > min_f_val)
    {
        int new_min_f_val = (int)open_head->getFVal();
        for (auto n : open_list)
        {
            if (n->getFVal() > w * min_f_val &&
                n->getFVal() <= w * new_min_f_val)
                n->focal_handle = focal_list.push(n);
        }
        min_f_val = new_min_f_val;
    }
}

void SpaceTimeAStar::releaseNodes()
{
    open_list.clear();
    focal_list.clear();
    for (auto node : allNodes_table) delete node;
    allNodes_table.clear();
}
