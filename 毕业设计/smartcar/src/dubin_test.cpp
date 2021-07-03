#include "smartcar/dubins.hpp"
#include <iostream>
using namespace std;

int printConfiguration(double q[3], double x, void* user_data)
{
    printf("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
    return 0;
}

int main()
{
    double q0[] = { 0,0,0 };   //起点
    double q1[] = { 4,4,3.142 };   //终点
    double turning_radius = 1.0;
    DubinsPath path;
    dubins_shortest_path( &path, q0, q1, turning_radius);
    cout << path.qi << endl;

    dubins_path_sample_many( &path, 0.5, printConfiguration, NULL);  //step_dis=0.5
    double dist;
    dist = dubins_path_length(&path);
    cout << dist << endl;
    return 0;
}