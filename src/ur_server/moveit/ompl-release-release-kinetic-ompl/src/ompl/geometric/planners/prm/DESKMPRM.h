#ifndef OMPL_GEOMETRIC_PLANNERS_DESKM_PRM_
#define OMPL_GEOMETRIC_PLANNERS_DESKM_PRM_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include <iostream>
#include <utility>
#include <fstream>
#include <vector>
#include <sstream>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphml.hpp>


namespace ompl
{
    namespace geometric
    {
        class DESKMPRM : public base::Planner
        {
        public:
            /** \brief Constructor */
            DESKMPRM(const base::SpaceInformationPtr &si);

            virtual ~DESKMPRM();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            virtual void setup();

            void adjust_graph();
            double m_distance(std::vector<double> q1,std::vector<double> q2);
            void optimizationPath(PathGeometric *path_old,PathGeometric *path_new);

            struct vertex_state_str {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_state_vector {
                typedef boost::vertex_property_tag kind;
            };

            typedef boost::adjacency_list<boost::vecS,boost::vecS,boost::undirectedS,
                    boost::property < vertex_state_str, std::string,
                    boost::property < vertex_state_vector, std::vector<double> > >,
                    boost::property < boost::edge_weight_t, double> > Graph_;
            typedef boost::graph_traits <Graph_>::vertex_descriptor vertex_descriptor;

            Graph_ g;

            bool start_ready;
            bool goal_ready;
        };

    }
}

#endif
