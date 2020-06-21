#include "Astar_searcher.h"

#include <cmath>
#include<cassert>
using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    // center point of the cube (at the index) is the coord.
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}



inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */
   // assume we can go 6 direction.
   // logic is same with that in matlab version.

   std::vector<int> x_dir = {1, 0, -1};
   std::vector<int> y_dir = {1, 0, -1};
   std::vector<int> z_dir = {1, 0, -1};
//  mark this node as closd.
    //currentPtr->id = -1;
    
   for (auto x_d: x_dir) {
       for(auto y_d: y_dir) {
           for(auto z_d: z_dir) {
                if(( x_d != y_d || x_d != z_d  )|| x_d != 0) {
                    auto x_idx = currentPtr-> index(0);
                    auto y_idx = currentPtr-> index(1);
                    auto z_idx = currentPtr-> index(2);

                    auto nx_idx = x_idx + x_d;
                    auto ny_idx = y_idx + y_d;
                    auto nz_idx = z_idx + z_d;
                    if(isFree(nx_idx, ny_idx, nz_idx)) {

                        
                        auto n_ptr = GridNodeMap[nx_idx][ny_idx][nz_idx];

                       assert(n_ptr != currentPtr);
                        neighborPtrSets.push_back(n_ptr);
                       // same edgeCost computation method as jps do
                       auto neighborIdx = n_ptr->index;
                       
                        edgeCostSets.push_back(
                            sqrt(
                            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
                            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
                            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2)) ) 
                            );
                        // update gn and hn outside of the method.
                        //n_ptr->gn = currentPtr->gn + getEuclidean(currentPtr, n_ptr);
                        //double heu = getHeu(n_ptr, );
                        

                        
                    } else {
                        //ignore the cell.
                    }


                }else {
                    // this is current node.
                }
           }
           
       }
   }


}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */

    return getEuclideanIndex(node1, node2);
}

//TODO refactor this func, it use a copy-style  parameter
double AstarPathFinder::computeDiagonalDistance(vector<double> arr) {
    if (arr.size() == 0)
        return 0.0;
    if (arr.size() == 1)
        return arr[0];

    double tmp = *(std::min_element(arr.begin(), arr.end()));
    double dim = arr.size();
    double diagonal_dis = sqrt(tmp*tmp * dim);

    arr.erase(std::remove(arr.begin(), arr.end(), tmp), arr.end());
    return diagonal_dis + computeDiagonalDistance(arr);

}

double AstarPathFinder::getDiagonalHeuxxxxxx(GridNodePtr start, GridNodePtr end) {
    // firstly find the maximum cute in the 3d space between start and end
    // then the max square
    // finally the coord diff
    ROS_INFO("Entering a star diagonalHeu. xxxxxxxxxxxxxxxxx");

    double x_diff = end->index(0) - start->index(0);
    double y_diff = end->index(1) - start->index(1);
    double z_diff = end->index(2) - start->index(2);

    std::vector<double> diff;
    diff.push_back(x_diff);
    diff.push_back(y_diff);
    diff.push_back(z_diff);

    return computeDiagonalDistance(diff);

    /*double tmp1 = *std::min_element(diff.begin(), diff.end());

    double heu1 = sqrt(pow(tmp1,2) * 3);
    
    diff.erase(std::remove(diff.begin(), diff.end(), tmp1), diff.end());
    if (diff.size() )
    assert(diff.size() == 2);
    
    double tmp2 = *std::min_element(std::begin(diff), std::end(diff));
    double heu2 = sqrt(pow(tmp2,2) * 2);

    diff.erase(std::remove(diff.begin(), diff.end(), tmp2), diff.end());

    if(diff.size() != 1) {
        cerr << " diff size is : " << diff.size() << std::endl;
    }
    assert(diff.size() ==1);

    double heu3 = diff[0];
    
    return heu1 + heu2 + heu3;*/

}

double AstarPathFinder::getEuclideanIndex(GridNodePtr start, GridNodePtr end) {
    ROS_INFO("Entering get euclidean by index.");
    auto start_idx = start->index;
    return sqrt(
        (start_idx(0) - end->index(0)) * (start_idx(0) - end->index(0)) +
        (start_idx(1) - end->index(1)) * (start_idx(1) - end->index(1)) +
        (start_idx(2) - end->index(2)) * (start_idx(2) - end->index(2)) );

}

double AstarPathFinder::getEuclidean(GridNodePtr start, GridNodePtr end) {
    // square_root( square_diff_x  + square_diff_y + square_diff_z )
    ROS_DEBUG("Entering euclidean heu computation.");
    return sqrt( pow((end->coord(0) - start->coord(0)),2) 
                + pow((end->coord(1) - start->coord(1)),2) 
                + pow((end->coord(2) - start->coord(2)),2));
}

double AstarPathFinder::getManhattanHeu(GridNodePtr start, GridNodePtr end) {
    // abs_diff_x + abs_diff_y + abs_diff_z
    return fabs(end->index(0) - start->index(0)) +
           fabs(end->index(1) - start->index(1)) +
           fabs(end->index(2) - start->index(2));

}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    
    double delta = sqrt(3);

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    startPtr->cameFrom = NULL;
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    int total_released_node = 0;
    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
       auto item_itr = openSet.begin();
       
       // do tie break or not
       if(true) {
            // for all nodes with same fScore and a small score to it to break the `tie`.
            auto pair_iter = openSet.equal_range(item_itr->second->fScore);
            
            int no_same_f = 0;
            for(auto it = pair_iter.first; it != pair_iter.second;it++) {
                if( it != item_itr) {
                    ROS_INFO("change the fScore of %d node with same fscore as the first one did." , no_same_f);
                    it->second->fScore = it->second->fScore + delta;
                }    

                no_same_f ++;
            }
            if(no_same_f > 1)
                ROS_INFO("There total %d node with same fScore.", no_same_f);

       }
       
       
       //
       //  remove this node from the openSet, change it's id to -1.
       //openSet.erase(item_itr, openSet.end());
       item_itr->second-> id  = -1;
       //ROS_INFO("Before remove the begin node, size of open is:%d", openSet.size());
       openSet.erase(item_itr);
       total_released_node ++;
        //ROS_DEBUG("after remove the begin node, size of open is:%d", openSet.size());
       currentPtr = (item_itr->second);

        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            assert(terminatePtr->cameFrom != NULL);
            assert(terminatePtr->cameFrom != terminatePtr);

            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the successors.
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     
        ROS_INFO_STREAM("ALL neighbor size is: " << neighborPtrSets.size() << "\n");
        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            neighborPtr = neighborPtrSets[i];
            auto neighborCost = edgeCostSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                neighborPtr -> gScore = currentPtr->gScore  + neighborCost;
                auto heu_score = getHeu(neighborPtr, endPtr);
                neighborPtr -> fScore = neighborPtr-> gScore + heu_score;
    
                neighborPtr->cameFrom = currentPtr;
                neighborPtr -> id = 1; 
                //startPtr -> coord = start_pt;
                openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                continue;
            }
            else if( neighborPtr-> id == 1){ 
                //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
               if ((currentPtr->gScore + neighborCost ) < neighborPtr->gScore) {

                    neighborPtr->gScore = currentPtr->gScore + neighborCost;
                    // update neighborPtr in openSet.
                    auto p_iter = openSet.equal_range(neighborPtr->fScore);

                    // remove from the openSet, as we will update the fScore.
                    bool find = false;
                    for(auto it = p_iter.first; it != p_iter.second; it++) {
                        //assert(it->second->index != nullptr);
                        //assert(neighborPtr->index != nullptr);

                        if ( (it->second->index -  neighborPtr->index).norm() == 0 ) {
                            // this is the target `neighbor` GridNodePtr  in the multimap.
                            openSet.erase(it);
                            find  = true;
                            break;
                        } 
                    }

                    if (!find) {
                       
                        ROS_INFO("the neighbor is not in open set while it's id equal to 1 ;");
                        
                    }
                    
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr , endPtr);
                    // update parrent-children linke
                    neighborPtr->cameFrom = currentPtr;
                    openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                    

               }
               
                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                do nothing.
                *        
                */
                continue;
            }
        }      
    }
    ROS_INFO("There are total %d node being expanded.", total_released_node);
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

  

bool AstarPathFinder::hasCircle(GridNodePtr end_ptr, GridNodePtr* c_point) {
    GridNodePtr fast = NULL;
    GridNodePtr slow = NULL;
    if (end_ptr == NULL) {
        return false;
    } else {
        slow = end_ptr;
        fast = end_ptr->cameFrom;
        while(fast != NULL && fast != slow) {
            slow = slow->cameFrom;
            fast = fast->cameFrom;
            if (fast != NULL)
                fast = fast->cameFrom;
        }
        if( fast != NULL && fast == slow) {
            *c_point = slow;
            return true;
        }
        return false;
    }

}

vector<Vector3d> AstarPathFinder::getPath(Vector3d start) 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
   GridNodePtr c_point;
   bool hasc =  hasCircle(terminatePtr, &c_point);
   if(hasc) {
       ROS_INFO("there is circle in the visited(closed) node link");
       // add to path until met the c_point;
       GridNodePtr cur = terminatePtr;
       while(cur != c_point) {
           path.push_back(cur->coord);
           cur = cur->cameFrom;
       }
       ROS_INFO("There are total %d nodes in path: ", path.size()) ;
       
       return path;
   }

   if(terminatePtr != NULL) {
       std::cout  << "TerminatePtr index(x,y,z): " 
        << terminatePtr->index(0) 
        << terminatePtr->index(1) 
        << terminatePtr->index(2)         
        << endl;
       GridNodePtr curr = terminatePtr; 
       int no = 1;
       while(curr != NULL) {
           gridPath.push_back(curr);
           //assert(curr != curr->cameFrom);
           curr = curr->cameFrom;
           
           
           if (no > 40000) 
            break;
           no += 1;
           //curr = curr->cameFrom;
           
       }
       
   }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}