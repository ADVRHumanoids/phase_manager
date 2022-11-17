#include <phase_manager/phase.h>

Phase::Phase(int n_nodes, std::string name):
    _n_nodes(n_nodes),
    _name(name)
{
}

std::string Phase::getName()
{
    return _name;
}

int Phase::getNNodes()
{
    return _n_nodes;
}

std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>> Phase::getConstraints()
{
    return _constraints;
}

std::unordered_map<ItemBase::Ptr, std::vector<int>> Phase::getCosts()
{
    return _costs;
}

std::unordered_map<ItemWithBoundsBase::Ptr, Phase::BoundsContainer> Phase::getVariables()
{
    return _variables;
}

std::unordered_map<ItemWithValuesBase::Ptr, Phase::ValuesContainer> Phase::getParameters()
{
    return _parameters;
}

PhaseToken::PhaseToken(Phase::Ptr phase):
    _abstract_phase(phase)

{
//    _active_nodes = std::make_shared<std::vector<int>>();
    //    _n_nodes = _abstract_phase.getNNodes();
}

int PhaseToken::_get_n_nodes()
{
    return _abstract_phase->getNNodes();
}

std::vector<int>& PhaseToken::_get_active_nodes()
{
    return _active_nodes;
}

Phase::Ptr PhaseToken::get_phase()
{
    return _abstract_phase;
}

bool PhaseToken::_update_constraints(int initial_node)
{
    for (auto cnstr_map : _abstract_phase->getConstraints())
    {
        auto pair_nodes = _compute_horizon_nodes(cnstr_map.second, initial_node);

        cnstr_map.first->addNodes(pair_nodes.second);

    }

    return true;
}

bool PhaseToken::_update_variables(int initial_node)
{
    for (auto var_map : _abstract_phase->getVariables())
    {
        auto pair_nodes = _compute_horizon_nodes(var_map.second.nodes, initial_node);


        std::cout << var_map.second.lower_bounds(Eigen::indexing::all, pair_nodes.first) << std::endl;

        var_map.first->addBounds(pair_nodes.second,
                                 var_map.second.lower_bounds(Eigen::indexing::all, pair_nodes.first),
                                 var_map.second.upper_bounds(Eigen::indexing::all, pair_nodes.first));

    }

    return true;
}

bool PhaseToken::_update_costs(int initial_node)
{
    for (auto cost_map : _abstract_phase->getCosts())
    {
        auto pair_nodes = _compute_horizon_nodes(cost_map.second, initial_node);

        cost_map.first->addNodes(pair_nodes.second);

//        std::cout << " adding horizon nodes: ";
//        for (auto elem : pair_nodes.second)
//        {
//            std::cout << elem << " ";
//        }
//        std::cout << std::endl;
    }

    return true;
}

bool PhaseToken::_update_parameters(int initial_node)
{
    for (auto par_map : _abstract_phase->getParameters())
    {
        auto pair_nodes = _compute_horizon_nodes(par_map.second.nodes, initial_node);

        par_map.first->addValues(pair_nodes.second, par_map.second.values(Eigen::indexing::all, pair_nodes.first));

    }

    return true;
}

std::pair<std::vector<int>, std::vector<int>> PhaseToken::_compute_horizon_nodes(std::vector<int> nodes, int initial_node)
{
    std::vector<int> active_nodes;

    std::set_intersection(nodes.begin(), nodes.end(),
                          _active_nodes.begin(), _active_nodes.end(),
                          std::back_inserter(active_nodes));

    std::vector<int> horizon_nodes(active_nodes.size());
    // active phase nodes   : [1 2 3 4]
    // active fun nodes     : [2 3]
    // phase pos in horizon : 7
    // constr pos in horizon: 7 + 2 - 1 = 8
    std::iota(std::begin(horizon_nodes), std::end(horizon_nodes), initial_node + active_nodes[0] - _active_nodes[0]);


    return std::make_pair(active_nodes, horizon_nodes);

}


bool PhaseToken::_update(int initial_node)
{
    _update_constraints(initial_node);
    _update_variables(initial_node);
    _update_costs(initial_node);
    _update_parameters(initial_node);
    return true;
}
