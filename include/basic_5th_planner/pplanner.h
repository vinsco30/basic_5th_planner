
/*FCL for collision*/
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/ccd/motion.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"

/*Octomap Libraries*/
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>

/*miscellaneous*/
#include "Eigen/Dense"
#include <boost/chrono.hpp>
#include <unistd.h>

typedef struct POSITION {
  double x;
  double y;
  double z;
} POSITION;

typedef struct ORIENTATION {
    double w;
    double x;
    double y;
    double z;
}ORIENTATION;

typedef struct POSE {
    POSITION position;
    ORIENTATION orientation;
    int parent = -1;;
}POSE;

class PATH_PLANNER {

    public:
        PATH_PLANNER();
        
        /*Planner initialization, int planner define the kind of planner (A*, RRT, RRT* ecc.)*/
        void init_planner( int max_samples, const double* xbounds, const double* ybounds, const double* zbounds );
        bool plan( std::vector<POSE>& poses );

        bool isPoseValid( const POSE& p1, const POSE& p2, const double step = 0.1 );
        bool check_state( const double * s );

        void set_start_state( const POSE& s ) {
            _start = s;
            _start_state_set = true;
        }

        void set_goal_state( const POSE& g ) {
            _goal = g;
            _goal_state_set = true;
        }

        void set_robot_geometry( float box_dim[3] ) {
            _robot_obj = std::shared_ptr<fcl::CollisionGeometry>( new fcl::Box( box_dim[0], box_dim[1], box_dim[2] ) );
            _robot_object_set = true;
        }

        void set_robot_geometry( float radius ) {
            _robot_obj = std::shared_ptr<fcl::CollisionGeometry>( new fcl::Sphere( radius ) );
            _robot_object_set = true;
        }

        void set_octo_tree( octomap::OcTree* t ) {
            fcl::OcTree* tree = new fcl::OcTree( std::shared_ptr<const octomap::OcTree>( t ) );
            _tree_obj = std::shared_ptr<fcl::CollisionGeometry>( tree );
        }

    
    private:

        POSE _start;
        POSE _goal;

        std::shared_ptr<fcl::CollisionGeometry> _tree_obj;
        std::shared_ptr<fcl::CollisionGeometry> _robot_obj;

        /*Params*/
        double _x_bounds[2];
        double _y_bounds[2];
        double _z_bounds[2];
        int _max_samples;
        double P_goal = 0.2;
        double check_step = 0.1;
        double goal_dist = 2.0;

        bool _start_state_set = false;
        bool _goal_state_set = false;
        bool _robot_object_set = false;
        bool _success;
        int samples = 0;

        POSE sample() {
            if ( ( double )rand() / RAND_MAX < P_goal ) {
                return { _goal.position.x, _goal.position.y, _goal.position.z,
                    _goal.orientation.w, _goal.orientation.x, _goal.orientation.y, _goal.orientation.z, _goal.parent };
            }
            else {
                return { static_cast<double>( rand() % (int)( _x_bounds[1]-_x_bounds[0] ) ), 
                         static_cast<double>( rand() % (int)( _y_bounds[1]-_y_bounds[0] ) ),
                         static_cast<double>( rand() % (int)( _z_bounds[1]-_z_bounds[0] ) ),
                         1.0, 0.0, 0.0, 0.0, -1 };
            }
        }

        int nearest( POSE& randomPose, const std::vector<POSE>& poses ) {
            int nearestIndex = 0;
            double minDist = distance( poses[0], randomPose );
            for( int i=1; i<poses.size(); i++ ) {
                double dist = distance( poses[i], randomPose );
                if( dist < minDist ) {
                    minDist = dist;
                    nearestIndex = i;
                }
            }
            return nearestIndex;
        }

        double distance( const POSE& a, const POSE& b ) {
            return sqrt( pow( a.position.x - b.position.x, 2 ) +
                         pow( a.position.y - b.position.y, 2 ) +
                         pow( a.position.z - b.position.z, 2 ) );
        }

        POSE steer( const POSE& nearestPose, const POSE& randomPose, double epsilon = 10.0 ) {
            double theta = atan2( randomPose.position.y - nearestPose.position.y, 
                            randomPose.position.x - nearestPose.position.x );
            
            return { nearestPose.position.x + epsilon * cos( theta ), nearestPose.position.y + epsilon * sin( theta ), 
                     nearestPose.position.z + epsilon * ( randomPose.position.z - nearestPose.position.z ) / 
                     distance( nearestPose, randomPose ),
                     1.0, 0.0, 0.0, 0.0, -1 };;
        }



};
