#include <chrono>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <glog/logging.h>

#include "smartcar/hybrid_astar.h"
#include "smartcar/node.h"


HybridAStar::HybridAStar(){
    _astar = new AStar();
}

HybridAStar::~HybridAStar(){
    for(auto node: _closeset){
        if(node.second){
            delete node.second;
            node.second = nullptr;
        }
    }
    for(auto node: _openset){
        if(node.second){
            delete node.second;
            node.second = nullptr;
        }
    }
    delete _astar;
}

void HybridAStar::initialize(Car* car, GridMap* map,
                             bool enable_backward,
                             double time_tolerance,
                             double step_size,
                             double turning_penalty_factor,
                             double goal_tolerance_dist,
                             double goal_tolerance_theta){
    _enable_backward = enable_backward;
    _car = car;
    _map = map;
    _time_tolerance = time_tolerance;
    _step_size = step_size;
    _turning_penalty_factor = turning_penalty_factor;
    _goal_tolerance_theta = goal_tolerance_theta;
    _goal_tolerance_dist = goal_tolerance_dist;
    _astar->buildGraph(_map->getData(),
                       _map->getWidth(), 
                       _map->getHeight(),
                       _car->width() / (2 * _map->getResolution()));
}

double HybridAStar::computeHCost(const Pose2D& start, const Pose2D& goal){
    // it is faster to use 2d heuristic instead of holonomic-with-obstacles
    //到目标的直线距离作为启发函数
    double dx = start.x() - goal.x();
    double dy = start.y() - goal.y();
    return std::sqrt(dx*dx + dy*dy);
    /* int sr, sc,er,ec;
    _map->xy2rc(start.x(), start.y(), sr, sc);
    _map->xy2rc(goal.x(), goal.y(), er, ec);
    Cell s(sr, sc), e(er, ec);
    std::vector<Cell> astar_path;
    if(!_astar->findPath(s, e, astar_path)){
        return _map->getWidth()*_map->getHeight()*_map->getResolution();//return an impossible result;
    }
    double l = 0.0, dx = 0, dy = 0;
    for(int i = 1; i < astar_path.size(); i++){
        dx = astar_path[i]._col - astar_path[i-1]._col; 
        dy = astar_path[i]._row - astar_path[i-1]._row; 
        l += std::sqrt(dx*dx + dy*dy);
    }
    return l*_map->getResolution(); *///directly using astar_path.size() will slow down 4 times!
}

void HybridAStar::tracePath(Node* const e,  Node* const s, std::vector<Pose2D>* path){
    Node* end = e;
    std::vector<Node*> nodes;
    while(end != s){
        nodes.push_back(end);
        end = end->precessor();
    }
    for(auto it = nodes.end()-1; it != nodes.begin()-1; it--){
        path->insert(path->end(), (*it)->subpath().begin(), (*it)->subpath().end());
    }
}

bool HybridAStar::isReachedGoal(const Pose2D& pos, const Pose2D& goal){
    if(std::abs(pos.x() - goal.x()) < _goal_tolerance_dist
        && std::abs(pos.y() - goal.y()) < _goal_tolerance_dist
        && std::abs(pos.theta() - goal.theta()) < _goal_tolerance_theta) {
        return true;
    }
    return false;
}

//规划部分
bool HybridAStar::plan(const Pose2D& start, const Pose2D& goal, std::vector<Pose2D>& path){
    auto cmp = [](const Node* a, const Node* b) {
         return a->totalCost() > b->totalCost();
    };
    std::priority_queue<Node*,
        std::vector<Node*>, decltype(cmp)> openset_quene(cmp);  //创建openlist，优先队列
    double hcost = computeHCost(start, goal)*_heuristic_factor; 

    Node *snode = new Node(hcost, 0, hcost, nullptr, start, {});  //start_node
    setNodeID(snode);
    openset_quene.push(snode);  //放入起点
    _openset[snode->getID()] = snode;

    auto ts = std::chrono::system_clock::now();

    double nonholonomic_cost = 0.0, holonomic_cost = 0.0;
    while(!openset_quene.empty()){
        LOG(ERROR)<<_openset.size();
        Node* node = openset_quene.top();  //找到openlist中最大值的节点，弹出
        openset_quene.pop();
        _openset.erase(node->getID());
        _closeset[node->getID()] = node;  //放入closelist
        std::vector<Pose2D> shot_path;
        if(_enable_backward)
        {   //如果允许倒车，使用reedsShepp；否则使用dubins
            if(reedsSheppShot(node->pose(), goal, shot_path, nonholonomic_cost))
            {
                // add these points to solution, and trace back the past
                tracePath(node , snode, &path);
                path.insert(path.end(), shot_path.begin(), shot_path.end());
                return true;
            }
        }
        else
        {
            if(dubinsShot(node->pose(), goal, shot_path, nonholonomic_cost))
            {
                // add these points to solution, and trace back the past
                tracePath(node , snode, &path);  //当前点到起点寻路（找父节点）
                path.insert(path.end(), shot_path.begin(), shot_path.end());
                std::cout << "dubins" << std::endl;
                return true;
            }
        }
        
        if(isReachedGoal(node->pose(), goal)){  //是否到达终点
            tracePath(node , snode, &path);
            std::cout << "goal" << std::endl;
            return true;
        }
        auto te = std::chrono::system_clock::now();
        // if(std::chrono::duration<double>(te - ts).count() > _time_tolerance) return false;

        std::map<SUBPATH_TYPE, std::vector<Pose2D>> frontier = neighbors(node->pose());
        int type_num = _enable_backward ? 6 : 3;     //左转、右转、直走
        for(int i = 0; i < type_num; ++i){           //遍历下一步节点
            SUBPATH_TYPE type = static_cast<SUBPATH_TYPE>(i);
            auto& sub_path = frontier[type];
            if(sub_path.empty()) continue;
            int idx = computeNodeID(sub_path.back());
            if(_closeset.find(idx) != _closeset.end()){
                continue;
            }
            double cost_so_far;  //已走过路程的cost
            bool to_insert = false;
            
            if(type==STRAIGHT_FORWORD){
                cost_so_far = node->costSoFar() + _step_size;
            }else if(type==STRAIGHT_BACKWORD){
                cost_so_far = node->costSoFar() + _step_size*_backward_penalty_factor;
            }else if(type==LEFT_FORWORD || type==RIGHT_FORWORD){
                cost_so_far = node->costSoFar() + _step_size*_turning_penalty_factor;
            }else{
                cost_so_far = node->costSoFar() + _step_size*_turning_penalty_factor*_backward_penalty_factor;
            }

            if(_openset.find(idx) == _openset.end()){
                to_insert = true;
            }else if(cost_so_far < _openset[idx]->costSoFar()){
                // deal with circle path 避免绕圈
                _openset[idx]->setCostSoFar(cost_so_far);
                _openset[idx]->setPrecessor(node);
                _openset[idx]->setSubpath(sub_path);
                _openset[idx]->setTotalCost(cost_so_far+_openset[idx]->heuristicCost());
            }
            
            if(to_insert){               
                holonomic_cost = computeHCost(sub_path.back(), goal);

                if(holonomic_cost<_map->getHeight()*_map->getWidth()){
                    // LOG(ERROR)<<type<<","<<nonholonomic_cost<<","<<holonomic_cost<<","<<cost_so_far;
                    hcost = std::max(holonomic_cost, nonholonomic_cost);  
                    Node* n = new Node(hcost, cost_so_far, hcost+cost_so_far, node, sub_path.back(), sub_path);
                    setNodeID(n);
                    openset_quene.push(n);  //将下一个节点加入openlist
                    _openset[n->getID()] = n;
                }
            }
        }
    }

    return false;
}

void HybridAStar::setNodeID(Node* node){
    node->setID(computeNodeID(node->pose()));
}

int HybridAStar::computeNodeID(const Pose2D& pose){
    int r, c, t;
    const double theta_resolution = 1.0;
    _map->xy2rc(pose.x(), pose.y(), r, c);
    double yaw = normalizeTheta(pose.theta());
    t = static_cast<int>((yaw + M_PI) * 57.29 / 10);//theta resolution：10°
    return _map->getWidth()*_map->getHeight()*t + r*_map->getWidth()+c;
}

double HybridAStar::length(const std::vector<Pose2D>& path){    //没用到
    double s = 0.0;
    for(int i = 1; i < path.size(); ++i){
        s += std::sqrt(std::pow(path[i].x()-path[i-1].x(), 2)
           + std::pow(path[i].y()-path[i-1].y(), 2));
    }
    return s;
}

std::vector<Pose2D> HybridAStar::subpath(const SUBPATH_TYPE& type,
                                const Pose2D& start,
                                const double step,/*straight forward length or turning theta*/
                                const double res,
                                const double turning_radius){
    std::vector<Pose2D> path;
    double lx, ly, ltheta, cx, cy;
    CHECK(step>0 && res>0);
    if(type == STRAIGHT_FORWORD || type == STRAIGHT_BACKWORD){
        for(double ss = 0; ; ){
            if(type == STRAIGHT_FORWORD){
                ss += res;
                if(ss > step) break;
            }else{
                ss -= res;
                if(ss <  -step) break;
            }
            lx = start.x() + std::cos(start.theta())*ss;
            ly = start.y() + std::sin(start.theta())*ss;
            ltheta = start.theta();
            path.push_back(Pose2D(lx, ly, ltheta));
        }
    }else if(type == LEFT_FORWORD || type == LEFT_BACKWARD){
        cx = start.x() + std::cos(start.theta()+M_PI_2)*turning_radius;
        cy = start.y() + std::sin(start.theta()+M_PI_2)*turning_radius;
        double ang_start = std::atan2(start.y()-cy, start.x()-cx);
        double ang_end = type == LEFT_FORWORD ? ang_start + step :ang_start-step;
        for(double t = ang_start; ;){
            if(type==LEFT_FORWORD){
                t += res;
                if(t > ang_end) break;
            }else{
                t -= res;
                if(t < ang_end) break;
            }
            lx = cx + std::cos(t)*turning_radius;
            ly = cy + std::sin(t)*turning_radius;
            ltheta =  t + M_PI_2;
            path.push_back(Pose2D(lx, ly, ltheta));
        }
    }else if(type == RIGHT_FORWORD || type == RIGHT_BACKWARD){
        cx = start.x() + std::cos(start.theta()-M_PI_2)*turning_radius;
        cy = start.y() + std::sin(start.theta()-M_PI_2)*turning_radius;
        double ang_start = std::atan2(start.y()-cy, start.x()-cx);
        double ang_end = type == RIGHT_FORWORD ? ang_start - step : ang_start + step;
        for(double t = ang_start; ;){
            if(type==RIGHT_FORWORD){
                t -= res;
                if(t < ang_end) break;
            }else{
                t += res;
                if(t > ang_end) break;
            }
            lx = cx + std::cos(t)*turning_radius;
            ly = cy + std::sin(t)*turning_radius;
            ltheta =  t - M_PI_2;
            path.push_back(Pose2D(lx, ly, ltheta));
        }
    }
    return path;
}

// return the neighbors of current pose, 6 directions
std::map<SUBPATH_TYPE, std::vector<Pose2D>> HybridAStar::neighbors(const Pose2D& pose){
    const double resolution = _map->getResolution();

    std::map<SUBPATH_TYPE, std::vector<Pose2D>> sub_paths;
    std::vector<SUBPATH_TYPE> types = {};
    if(_enable_backward){
        types = {LEFT_FORWORD, STRAIGHT_FORWORD, RIGHT_FORWORD,
                 LEFT_BACKWARD, STRAIGHT_BACKWORD, RIGHT_BACKWARD};
    }else{
        types = {LEFT_FORWORD, STRAIGHT_FORWORD, RIGHT_FORWORD};
    }
    for(const auto& type: types){
        std::vector<Pose2D> path = {};
        if(type==STRAIGHT_FORWORD || type==STRAIGHT_BACKWORD){
            path = subpath(type, pose, _step_size, resolution, _car->minTurningRadius());
        }else{
            path = subpath(type, pose, _step_size / _car->minTurningRadius(),
                resolution, _car->minTurningRadius());
        }
        if(checkPath(path)){
            sub_paths[type] = path;
        }
    }
    return sub_paths;
}

bool HybridAStar::checkPath(std::vector<Pose2D>& path){  //碰撞检查
    /* cv::Mat xxx = _map->drawResult(*_car, path);
    cv::imshow("sss", xxx);
    cv::waitKey(0); */
    for(const auto& p: path)
    {
        if(_map->isCollision(*_car, p)) return false;
    }
    return true;
}

bool HybridAStar::reedsSheppShot(const Pose2D& start,
                                 const Pose2D& goal,
                                 std::vector<Pose2D>& rs_path,
                                 double& len){
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>(_car->minTurningRadius()));
    const unsigned int num_pts = 100;
    ob::ScopedState<> from(space), to(space), s(space);
    from[0] = start.x(); from[1] = start.y(); from[2] = start.theta();
    to[0] = goal.x(); to[1] = goal.y(); to[2] = goal.theta();
    std::vector<double> reals;
    len = space->distance(from(), to());    
    for (unsigned int i=0; i<=num_pts; ++i){
        space->interpolate(from(), to(), (double)i/num_pts, s());
        reals = s.reals();
        rs_path.push_back(Pose2D(reals[0], reals[1], reals[2]));
    }
    
    if(checkPath(rs_path)){
        return true;
    }else{
        return false;
    }
}

bool HybridAStar::dubinsShot(const Pose2D& start,
                                 const Pose2D& goal,
                                 std::vector<Pose2D>& rs_path,
                                 double& len){
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    ob::StateSpacePtr space(std::make_shared<ob::DubinsStateSpace>(_car->minTurningRadius()));
    const unsigned int num_pts = 100;
    ob::ScopedState<> from(space), to(space), s(space);
    from[0] = start.x(); from[1] = start.y(); from[2] = start.theta();
    to[0] = goal.x(); to[1] = goal.y(); to[2] = goal.theta();
    std::vector<double> reals;
    len = space->distance(from(), to());    
    for (unsigned int i=0; i<=num_pts; ++i)
    {
        space->interpolate(from(), to(), (double)i/num_pts, s());
        reals = s.reals();
        rs_path.push_back(Pose2D(reals[0], reals[1], reals[2]));
    }
    
    if(checkPath(rs_path))
    {
        return true;
    }
    else
    {
        return false;
    }
}

double HybridAStar::normalizeTheta(double theta){
    if (theta >= -M_PI && theta < M_PI)
        return theta;
    double multiplier = std::floor(theta / (2*M_PI));
    theta = theta - multiplier*2*M_PI;
    if (theta >= M_PI)
        theta -= 2*M_PI;
    if (theta < -M_PI)
        theta += 2*M_PI;
    return theta;
}
