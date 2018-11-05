#include "ompl/geometric/planners/prm/URPRM.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

std::vector<int> path_boost_UR;
std::vector<int> path_index_UR;

ompl::geometric::URPRM::URPRM(const base::SpaceInformationPtr &si) : base::Planner(si, "URPRM")
{
//    specs_.approximateSolutions = true;
//    specs_.directed = true;

    std::cout<<"URPRM"<<std::endl<<std::endl;

    std::ifstream ist("/home/smj/ros/jrc3/moveit_ws/graph_up_right.xml");
    boost::dynamic_properties dp;
    dp.property("weight", get(boost::edge_weight_t(), g));
    dp.property("string", get(vertex_state_str(), g));
    read_graphml(ist,g,dp);

    typedef boost::graph_traits<Graph_>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vip;
    vip = boost::vertices(g);
    boost::property_map<Graph_,vertex_state_str>::type stringmap=get(vertex_state_str(),g);
    for(vertex_iter vi = vip.first; vi != vip.second; ++vi)
    {
        std::vector<double> stu;
        std::stringstream sstr(stringmap[*vi]);
        std::string token;
        while(getline(sstr, token, ','))
        {
            stu.push_back(std::stod(token));
        }

        put(vertex_state_vector(), g, *vi, stu);
    }

    goal_ready = false;
    start_ready = false;
}

double ompl::geometric::URPRM::m_distance(std::vector<double> q1, std::vector<double> q2)
{
    double distance_=0;
    for(unsigned int i=0;i<q1.size();i++)
    {
        distance_+=(q1[i]-q2[i])*(q1[i]-q2[i]);
    }
    return sqrt(distance_);
}

void ompl::geometric::URPRM::adjust_graph()
{
    const base::State *start_s = pis_.nextStart();
    const base::State *goal_s = pis_.nextGoal();

    if(!start_s)
    {
        std::cout<<"invalid start_s return"<<std::endl;
        return;
    }

    if(!goal_s)
    {
        std::cout<<"invalid goal_s return"<<std::endl;
        return;
    }

    std::vector<double> start_node;
    std::vector<double> goal_node;
    const ompl::base::RealVectorStateSpace::StateType *start_state = start_s->as<ompl::base::RealVectorStateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType *goal_state = goal_s->as<ompl::base::RealVectorStateSpace::StateType>();

    for(unsigned int i=0;i<si_->getStateDimension();i++)
    {
        start_node.push_back(start_state->values[i]);
        goal_node.push_back(goal_state->values[i]);
    }

    double goal_min_distance = 9999;
    double start_min_distance = 9999;
    int goal_min_distance_index = -1;
    int start_min_distance_index = -1;
    typedef boost::graph_traits<Graph_>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vip;
    vip = boost::vertices(g);
    boost::property_map<Graph_,vertex_state_vector>::type vectormap=get(vertex_state_vector(),g);
    for(vertex_iter vi = vip.first; vi != vip.second; ++vi)
    {
        std::vector<double> vec_temp;
        for(unsigned int i=0;i<si_->getStateDimension();i++)
        {
            vec_temp.push_back(vectormap[*vi][i]);
        }

        double start_distance = m_distance(start_node,vec_temp);
        double goal_distance = m_distance(goal_node,vec_temp);

        if(start_distance<start_min_distance)
        {
            base::State       *state_temp = si_->allocState();
            ompl::base::RealVectorStateSpace::StateType *rstate = static_cast<ompl::base::RealVectorStateSpace::StateType*>(state_temp);
            for (unsigned int j = 0 ; j < si_->getStateDimension() ; ++j)
            {
                rstate->values[j] = vec_temp[j];
            }

            if(si_->checkMotion(start_state,rstate))
            {
                start_ready = true;
                start_min_distance = start_distance;
                start_min_distance_index = *vi;
            }
        }

        if(goal_distance<goal_min_distance)
        {
            base::State       *state_temp = si_->allocState();
            ompl::base::RealVectorStateSpace::StateType *rstate = static_cast<ompl::base::RealVectorStateSpace::StateType*>(state_temp);
            for (unsigned int j = 0 ; j < si_->getStateDimension() ; ++j)
            {
                rstate->values[j] = vec_temp[j];
            }

            if(si_->checkMotion(goal_state,rstate))
            {
                goal_ready = true;
                goal_min_distance = goal_distance;
                goal_min_distance_index = *vi;
            }
        }
    }

    if(goal_ready && start_ready)
    {
        boost::add_edge(boost::num_vertices(g),start_min_distance_index,start_min_distance,g);
        boost::add_edge(boost::num_vertices(g),goal_min_distance_index,goal_min_distance,g);

        put(vertex_state_vector(), g, boost::num_vertices(g)-2 , start_node);
        put(vertex_state_vector(), g, boost::num_vertices(g)-1 , goal_node);
    }
}

ompl::geometric::URPRM::~URPRM()
{}

void ompl::geometric::URPRM::clear()
{
    Planner::clear();
}

void ompl::geometric::URPRM::setup()
{
    Planner::setup();
}

ompl::base::PlannerStatus ompl::geometric::URPRM::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    adjust_graph();

    double  approxdif = 0;
    bool solved = false;
    bool approximate = false;

    if(!goal_ready)
    {
        std::cout<<"INVALID GOAL!"<<std::endl;
        return base::PlannerStatus(base::PlannerStatus::INVALID_GOAL);
    }

    if(!start_ready)
    {
        std::cout<<"INVALID START!"<<std::endl;
        return base::PlannerStatus(base::PlannerStatus::INVALID_START);
    }

    int ipot_st, ipot_end;
    ipot_st  = boost::num_vertices(g)-2;
    ipot_end = boost::num_vertices(g)-1;

    std::vector<vertex_descriptor> p(boost::num_vertices(g));
    std::vector<double> d(boost::num_vertices(g));
    vertex_descriptor s = vertex(ipot_st, g);

    boost::dijkstra_shortest_paths(g, s,
        predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
        distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));

    int t = ipot_end;
    path_boost_UR.clear();
    for (; t != ipot_st; t = p[t])
        path_boost_UR.push_back(t);
    path_boost_UR.push_back(ipot_st);
    std::reverse(path_boost_UR.begin(), path_boost_UR.end());

    //--------------------------------------------------------
//    for(unsigned int i=0;i<path_boost_UR.size();i++)
//    {
//        std::cout<<path_boost_UR[i]<<"   ";
//    }
//    std::cout<<std::endl<<std::endl;
    //--------------------------------------------------------

    PathGeometric *path = new PathGeometric(si_);
    boost::property_map<Graph_,vertex_state_vector>::type vectormap=get(vertex_state_vector(),g);
    for (unsigned int i = 0 ; i < path_boost_UR.size() ; i++)
    {
        base::State       *state_temp = si_->allocState();
        ompl::base::RealVectorStateSpace::StateType *rstate = static_cast<ompl::base::RealVectorStateSpace::StateType*>(state_temp);
        //ompl::base::RealVectorStateSpace::StateType *rstate = state_temp->as<ompl::base::RealVectorStateSpace::StateType>();
        for (unsigned int j = 0 ; j < si_->getStateDimension() ; ++j)
        {
            rstate->values[j] = vectormap[path_boost_UR[i]][j];
        }
        path->append(rstate);
    }

    PathGeometric *path_new = new PathGeometric(si_);
    optimizationPath(path,path_new);

    //--------------------------------------------------------
//    boost::property_map<Graph_,vertex_state_str>::type stringmap=get(vertex_state_str(),g);
//    for(unsigned int m=0;m<path_index_UR.size();m++)
//    {
//        std::cout<<"path id "<<path_index_UR[m];
//        std::cout<<stringmap[path_index_UR[m]]<<std::endl;
//    }

//    for(unsigned int k=0;k<path_new->getStateCount();k++)
//    {
//        ompl::base::RealVectorStateSpace::StateType *rstate = static_cast<ompl::base::RealVectorStateSpace::StateType*>(path_new->getState(k));
//        std::cout<<rstate->values[0]<<","<<rstate->values[1]<<","<<rstate->values[2]<<","
//                 <<rstate->values[3]<<","<<rstate->values[4]<<","<<rstate->values[5]<<std::endl;
//    }
//    std::cout<<std::endl;
    //--------------------------------------------------------

    pdef_->addSolutionPath(base::PathPtr(path_new), approximate, approxdif, getName());
    solved = true;

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::URPRM::optimizationPath(PathGeometric *path_old,PathGeometric *path_new)
{
    std::cout<<"optimizationPath......"<<std::endl;

    std::vector<base::State*> old_path_states = path_old->getStates();
    path_new->append(old_path_states[0]);
    //--------------------------------------------------------
    path_index_UR.clear();
    path_index_UR.push_back(path_boost_UR[0]);
    //--------------------------------------------------------
    unsigned int index = 0;
    while(index != (old_path_states.size()-1))
    {
        unsigned int i;
        for(i=(old_path_states.size()-1);i>(index+1);i--)
        {
            if(si_->checkMotion(path_new->getState(path_new->getStateCount()-1),old_path_states[i]))
            {
                path_new->append(old_path_states[i]);
                //--------------------------------------------------------
                path_index_UR.push_back(path_boost_UR[i]);
                //--------------------------------------------------------
                index = i;
                break;
            }
        }
        if(i==(index+1))
        {
            path_new->append(old_path_states[i]);
            //--------------------------------------------------------
            path_index_UR.push_back(path_boost_UR[i]);
            //--------------------------------------------------------
            index = i;
        }
    }

    std::cout<<"original path length: "<<old_path_states.size()<<std::endl;
    std::cout<<"optimization path length: "<<path_new->getStateCount()<<std::endl;
}
