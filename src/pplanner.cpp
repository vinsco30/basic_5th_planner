#include "pplanner.h"

PATH_PLANNER::PATH_PLANNER() {
    // Constructor implementation
    std::cout << "Path Planner initialized." << std::endl;
}

bool PATH_PLANNER::isPoseValid( const POSE& p1, const POSE& p2, const double step ) {
    // Check if the second pose is within bounds

    if( p2.position.x < _x_bounds[0] || p2.position.x > _x_bounds[1]) {
        std::cout << "X POSE fault: " << p2.position.x << " - " << _x_bounds[0] << ", " << _x_bounds[1] << std::endl;
        return false;
    }
    
    if( p2.position.y < _y_bounds[0] || p2.position.y > _y_bounds[1]) {
        std::cout << "Y POSE fault: " << p2.position.y << " - " << _y_bounds[0] << ", " << _y_bounds[1] << std::endl;
        return false;
    }
    
    if( p2.position.z < _z_bounds[0] || p2.position.z > _z_bounds[1]) {
        std::cout << "Z POSE fault: " << p2.position.z << " - " << _z_bounds[0] << ", " << _z_bounds[1] << std::endl;
        return false;
    }

    bool current_reachable = false;

    POSE pose = steer( p1, p2, step );

    while( distance( pose, p2 ) > step ) {
        // Check if the step pose is within bounds

        if( pose.position.x < _x_bounds[0] || pose.position.x > _x_bounds[1]) {
            std::cout << "X POSE STEP fault: " << pose.position.x << " - " << _x_bounds[0] << ", " << _x_bounds[1] << std::endl;
            return false;
        }
        
        if( pose.position.y < _y_bounds[0] || pose.position.y > _y_bounds[1]) {
            std::cout << "Y POSE STEP fault: " << pose.position.y << " - " << _y_bounds[0] << ", " << _y_bounds[1] << std::endl;
            return false;
        }
        
        if( pose.position.z < _z_bounds[0] || pose.position.z > _z_bounds[1]) {
            std::cout << "Z POSE STEP fault: " << pose.position.z << " - " << _z_bounds[0] << ", " << _z_bounds[1] << std::endl;
            return false;
        }
        
        if( _tree_obj ) {

            fcl::Vec3f translation( pose.position.x, pose.position.y, pose.position.z );
            fcl::Quaternion3f rotation( pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z );

            fcl::CollisionObject treeObj(( _tree_obj ));
            // fcl::CollisionObject robotObject( _robot_obj ); 

            // robotObject.setTransform(rotation, translation);
            // fcl::CollisionRequest requestType(1,false,1,false);
            // fcl::CollisionResult collisionResult;
            // fcl::collide(&robotObject, &treeObj, requestType, collisionResult);
            
            // if( collisionResult.isCollision() ) return false; 
            pose = steer( pose, p2, step );
        } 
        else {
            return true;
        }  
    }
    current_reachable = true;
    return true;
}

void PATH_PLANNER::init_planner(int max_samples, const double* xbounds, const double* ybounds, const double* zbounds) {
    
    // Initialize the planner with bounds
    std::cout << "Initializing planner with bounds: "
              << "X: [" << xbounds[0] << ", " << xbounds[1] << "], "
              << "Y: [" << ybounds[0] << ", " << ybounds[1] << "], "
              << "Z: [" << zbounds[0] << ", " << zbounds[1] << "]" << std::endl;

    _x_bounds[0] = xbounds[0];
    _x_bounds[1] = xbounds[1];
    _y_bounds[0] = ybounds[0];
    _y_bounds[1] = ybounds[1];
    _z_bounds[0] = zbounds[0];
    _z_bounds[1] = zbounds[1];
    
    _max_samples = max_samples;
}

bool PATH_PLANNER::plan( std::vector<POSE>& poses ) {
    // Implement the planning logic here
    std::cout << "Planning with max samples: " << _max_samples << std::endl;
    poses.push_back(_start);

    for( int i=0; i<_max_samples; ++i ) {
        POSE randomPose = sample();
        int nearestIndex = nearest( randomPose, poses );
        POSE newPose = steer( poses[nearestIndex], randomPose );

        if( isPoseValid(poses[nearestIndex], newPose) )
        {
            newPose.parent = nearestIndex;
            poses.push_back( newPose );

            if( distance( newPose, _goal ) < goal_dist && isPoseValid( newPose, _goal ) ) {
                _goal.parent = poses.size()-1;
                poses.push_back( _goal );
                _success = true;
                samples = i+1;
                std::cout << "Goal reached!" << std::endl;
                return true;

            }
        }
    }
    // For now, just return true to indicate success
    return false;
}

// int main(int argc, char* argv[]) {
// 	std::cout << "Starting Path Planner node..." << std::endl;
// 	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

//     PATH_PLANNER pp;
//     double xbounds[2] = {0.0, 20.0};
//     double ybounds[2] = {0.0, 10.0};
//     double zbounds[2] = {0.0, 2.0};
//     pp.init_planner(1000, xbounds, ybounds, zbounds);

// 	return 0;
// }

