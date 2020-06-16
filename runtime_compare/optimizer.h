//
//  optimizer.h
//  space-searching
//
//  Created by Tianqi on 10/31/19.
//  Copyright © 2019 Tianqi. All rights reserved.

// key issue:
// 1. 2 structures includes each other;
// https://stackoverflow.com/questions/16210569/how-to-implement-two-structs-that-can-access-each-other
// https://stackoverflow.com/questions/41915049/c-field-has-incomplete-type
// 2. deepcopy of structs;
// 3. (TODO) struct quick initialization;
//

#ifndef optimizer_h
#define optimizer_h
#endif /* optimizer_h */

#include "util.h"
#include <vector>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <math.h> /* fmod */

struct edge;
struct node;

struct edge
{
    // v1 -> v2, with time and cost
    node *v1; // tail node
    node *v2; // head node
    double dt;
    long double cost;
    double action;
};

struct node
{
    double x;
    double v;
    vector<edge> in_edge;
    vector<edge> out_edge;

    long double cost = 99999;
};

struct solu
{
    vector<edge> path;
    long double cost;
    vector<double> action;
    double time;
};

void deepCopySol(solu *to, solu *from)
{
    //   https://stackoverflow.com/questions/6911688/making-a-deep-copy-of-a-struct-making-a-shallow-copy-of-a-struct
    to->path = from->path;
    to->cost = from->cost;
    to->action = from->action;
    to->time = from->time;
}; // is this true???
void deepCopyAct(vector<double> *to, vector<double> *from){

};
// deepcopy function

class dfs_optimizer
{
public:
    
    double dx;
    double dv;
    double dt;
    double w1;
    double w2;
    double delta_t_min;
    double idling_cost;
    vector<double> a_threshold;
    int m, n;
    vector<vector<node>> graph;
    node begin_node;
    node end_node;
    traffic_light single_light;
    double end_loc;
    vector<solu> self_sol;
    solu optimal_sol;
    dfs_optimizer(vector<double> init_state, vector<int> size, double dv_, double dt_, vector<double> a_threshold_, vector<double> para, traffic_light light);
    void build_graph(vector<double> loc_grid, vector<double> vel_grid);
    int findMin(vector<solu>);
    double *inside_red(double t, double *result);
    void solver();
    void DFS_green_light(node v, solu sol);
    void DFS_red_light(node v, solu sol);
};

dfs_optimizer::dfs_optimizer(vector<double> init_state, vector<int> size, double dv_, double dt_, vector<double> a_threshold_, vector<double> para, traffic_light light)
{
    dv = dv_;
    dt = dt_;
    a_threshold = a_threshold_;
    w1 = para[0];
    w2 = para[1];
    m = size[0];
    n = size[1];
    begin_node.x = init_state[0];
    begin_node.v = init_state[1];
    single_light = light;
    dx = (single_light.location - init_state[0]) / ((double)m - 1);
    delta_t_min = dx / ((double)n  * dv);
    end_loc = single_light.location + dx;
    end_node.x = end_loc;
    // cout<<"no problem here\n";
    
}

void dfs_optimizer::build_graph(vector<double> loc_grid, vector<double> vel_grid)
{

    //    build the skeleton of the graph
    node v;
    double delta_v;
    double delta_t;
    double a;
    long double cost;
    edge e;
    vector<node> row;

    
    for (int i = 0; i < m; i++)
    {

        // build all nodes first
        for (int j = 0; j < n + 1; j++)
        {

            v.x = loc_grid[i];
            v.v = vel_grid[j];
            row.push_back(v);
        }

        graph.push_back(row);
        row.clear();
    }
    //    after the skeleton, start building edges
    //    first layer initialize
    for (int i = 0; i < n+1; i++)
    {
        
        if (graph[0][i].v > 0)
        {
            delta_v = graph[0][i].v - begin_node.v;
            delta_t = 2 * dx / (graph[0][i].v + begin_node.v);
            a = delta_v / delta_t;
            if (a >= a_threshold[0] and a <= a_threshold[1])
            {
                cost = delta_t * w1 / delta_t_min + abs(a / a_threshold[1]) * w2;
                e.dt = delta_t;
                e.cost = cost;
                e.v1 = &begin_node;
                e.v2 = &graph[0][i];
                e.action = a;
                // e = {begin_node, graph[0][i].v, delta_t, cost};
                // add the edge between begin_node and node in the first layer
                begin_node.out_edge.push_back(e);
                graph[0][i].in_edge.push_back(e);
                cout<<"v1: ("<<e.v1->x<<", "<<e.v1->v<<") -> ("<<e.v2->x<<", "<<e.v2->v<<"): c = "<<e.cost<<"\n";
            }
        }
    }

    // for (i = 0; i < n; i++)
    // {
    //     cout<<i<<" :"<<graph[0][i].in_edge.size()<<", ";
    // }
    
    //    later layers, calculate all edges before the intersection
    for (int i = 0; i < m-1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            if (graph[i][j].in_edge.size() > 0)
            {
                //  this node is reachable from initial node
                for (int k = 0; k < n + 1; k++)
                {
                    
                    if (!(i < m - 3 && k == 0))
                    {
                    
                        delta_v = graph[i + 1][k].v - graph[i][j].v;

                        delta_t = 2 * dx / (graph[i + 1][k].v + graph[i][j].v);
                        a = delta_v / delta_t;
                        if (a >= a_threshold[0] && a <= a_threshold[1])
                        {
                            cost = delta_t * w1 / delta_t_min + abs(a / a_threshold[1]) * w2;
                            e.dt = delta_t;
                            e.cost = cost;
                            e.v1 = &graph[i][j];
                            e.v2 = &graph[i + 1][k];
                            e.action = a;
                            graph[i][j].out_edge.push_back(e);
                            graph[i + 1][k].in_edge.push_back(e);
                            cout<<"v1: ("<<e.v1->x<<", "<<e.v1->v<<") -> ("<<e.v2->x<<", "<<e.v2->v<<"): c = "<<e.cost<<"\n";
                        }
                    }
                }
            }
        }
    }
    // for (int i = 0; i < m; i++){
    //     for (int j = 0; j < n + 1; j++){
    //         cout<<"(x: "<<graph[i][j].x<<", v: "<<graph[i][j].v<<"); ";

    //     }
    // }
}

double *dfs_optimizer::inside_red(double time, double *result)
{
    //generate a pointer used by https://www.geeksforgeeks.org/return-local-array-c-function/
    // a dynamic allocated memory
    // static int result[2];

    double t = fmod(single_light.t0 + time, single_light.total_dur); // TODO

    if (t < single_light.yel_dur + single_light.gre_dur)
    {
        // not in red light
        result[0] = -1;
        result[1] = t;
    }
    else
    {
        // red light
        result[0] = 1;
        result[1] = single_light.total_dur - t;
    }

    // ? cout<<result[0];
    return result;
}

void dfs_optimizer::DFS_green_light(node v, solu sol)
{

    if (v.x < begin_node.x + dx)
    {
        // arrives the initial point
        double is_red[2];
        double *ptr_red = inside_red(sol.time, is_red);

        if (ptr_red[0] < 0)
        {
            // green light
            self_sol.push_back(sol);
        }
        return;
    }
    else
    {
        if (v.in_edge.size() > 0)
        {
            solu new_sol;
            for (int i = 0; i < v.in_edge.size(); i++)
            {

                // struct copy in cpp
                deepCopySol(&new_sol, &sol);
                // update to move to next node
                new_sol.time += v.in_edge[i].dt;
                new_sol.cost += v.in_edge[i].cost;
                // make the order correct
                new_sol.action.insert(new_sol.action.begin(), v.in_edge[i].action);
                new_sol.path.insert(new_sol.path.begin(), v.in_edge[i]);
                DFS_green_light((*v.in_edge[i].v1), new_sol);
            }
        }
    }
}

void dfs_optimizer::DFS_red_light(node v, solu sol)
{

    // cout << "entered function!  \n";
    if (v.x < begin_node.x + dx)
    {
        // arrives the initial point
        double is_red[2];
        double *ptr_red = inside_red(sol.time, is_red);

        // cout << "entered A part! x = "<<v.x<<", v = "<<v.v<<" \n";
        if (ptr_red[0] > 0)
        {
            // red light
        
            idling_cost = ptr_red[1] * 1; // idling punish, 1 is propotion
            sol.cost += idling_cost;
            self_sol.push_back(sol);
        }
        return;
    }
    else
    {
        // cout << "entered B part \n";
        // cout << "check again... "<<v.in_edge.size()<<"\n";
        if (v.in_edge.size() > 0)
        {
            solu new_sol;
            // cout << "entered B.0 part \n";
            for (int i = 0; i < v.in_edge.size(); i++)
            {

                // struct copy in cpp
                
                deepCopySol(&new_sol, &sol);
                // update to move to next node
                new_sol.time += v.in_edge[i].dt;
                new_sol.cost += v.in_edge[i].cost;
                // make the order correct
                new_sol.action.insert(new_sol.action.begin(), v.in_edge[i].action);
                new_sol.path.insert(new_sol.path.begin(), v.in_edge[i]);

                // update to move to next node
                // new_sol.time = sol.time + v.in_edge[i].dt;
                // new_sol.cost = sol.cost + v.in_edge[i].cost;
                // // make the order correct
                // new_sol.action = sol.action;
                // new_sol.action.insert(new_sol.action.begin(), v.in_edge[i].action);
                // new_sol.path = sol.path;
                // new_sol.path.insert(new_sol.path.begin(), v.in_edge[i]);
                // for (int j = 0; j < new_sol.path.size(); j++){
                //     cout<<"v.x = "<<new_sol.path[j].v1->x<<", v.v = "<<new_sol.path[j].v1->v<<"; ";
                // }
                // cout<<"v.x = "<<new_sol.path.back().v2->x<<", v.v = "<<new_sol.path.back().v2->v<<"\n";
                // // cout<<"this edge is v1: (x = "<<v.in_edge[i].v1->x<<", v = "<<v.in_edge[i].v1->v<<"), v2: (x = "<<v.in_edge[i].v2->x<<", v = "<<v.in_edge[i].v2->v<<")\n";
                // // cout<<"v.x = "<<v.x<<", v.v = "<<v.v<<"\n";
                // cout<<"new_sol.time = "<<new_sol.time<<"\n";

                DFS_red_light((*v.in_edge[i].v1), new_sol);
            }
        }
        else
        {
            // cout << "skipped!\n";
        }
        // cout << "B part finished\n";
    }
}

int dfs_optimizer::findMin(vector<solu> sol)
{
    solu best_sol;
    best_sol = sol[0];
    int index = 0;
    for (int i = 1; i < sol.size(); i++)
    {
        if (best_sol.cost > sol[i].cost)
        {
            best_sol = sol[i];
            index = i;
        }
    }
    return index;
}

void dfs_optimizer::solver()
{
    vector<double> loc_grid = linspace(begin_node.x + dx, single_light.location + dx, m);
    vector<double> vel_grid = linspace(0, ((double)n) * dv, n + 1);

    // build the graph
    dfs_optimizer::build_graph(loc_grid, vel_grid);

    cout<<"no problem here\n";
    // initial the solution
    solu init_sol;
    init_sol.time = 0;
    init_sol.cost = 0;

    // store the solutions
    vector<solu> Total_sol;

    cout<<"m = "<<m <<"\n";

    // red light solutions, at the intersection loc, v=0;
    DFS_red_light(graph[m - 2][0], init_sol);
    // pick up solution with lowest cost if solu for red exists
    if (self_sol.size() > 0)
    {
        Total_sol.push_back(self_sol[findMin(self_sol)]);
    }
    cout << "passed red light!";
    // green light solutions, at the intersec loc, v > 0;
    for (int i = 1; i < graph[m - 2].size(); i++)
    {
        self_sol.clear();
        DFS_green_light(graph[m - 2][i], init_sol);

        if (self_sol.size() > 0)
        {
            Total_sol.push_back(self_sol[findMin(self_sol)]);
        }
    }
    cout << "passed green light, total sol num = " << Total_sol.size();
    // Total_sol sovles the solution for states at the intersection

    // after we know the all optimal solutions in front of traffic light, then we move on to discover the step after passing the light, the final step

    vector<solu> Final_solution;
    node v;
    node pre_node;
    double delta_v, delta_t;
    double acc;
    long double step_cost;
    solu opt_node_sol;
    edge ee;
    for (int i = 0; i < graph.back().size(); i++)
    {   
        cout<<"final v = "<<graph.back()[i].v<<", ";

        if (graph.back()[i].v != 0)
        {
            v = graph.back()[i];

            // a double check
            opt_node_sol.cost = 99999;
            // dynamic programming in assuring the last step in traffic light
            // check every node at the intersection, get heuristic of v

            for (int j = 0; j < Total_sol.size(); j++)
            {   
                pre_node = *Total_sol[j].path.back().v2;
                delta_v = v.v - pre_node.v;
                delta_t = 2 * dx / (v.v + pre_node.v);
                acc = delta_v / delta_t;

                if (acc > a_threshold[0] && acc < a_threshold[1])
                {
                    step_cost = delta_t * w1 / delta_t_min + abs(acc / a_threshold[1]) * w2;
                    if (step_cost + Total_sol[j].cost < v.cost)
                    {
                        ee.dt = delta_t;
                        ee.cost = step_cost;
                        ee.v1 = &pre_node;
                        ee.v2 = &v;
                        // ee = {pre_node, v, delta_t, step_cost};
                        v.cost = step_cost + Total_sol[j].cost;
                        opt_node_sol = Total_sol[j];
                        opt_node_sol.path.push_back(ee);
                        opt_node_sol.cost += step_cost;
                        opt_node_sol.action.push_back(acc);
                        opt_node_sol.time += delta_t;
                    }
                }
            }

            // check if the opt_node_sol is updated, it is valid for v
            if (opt_node_sol.cost < 99999)
            {
                cout<<"output value! cost ="<<opt_node_sol.cost<<"\n";
                Final_solution.push_back(opt_node_sol);
            }
            else{

                cout<<"not output value, cost = "<<opt_node_sol.cost<<" \n";
            }
            
        }
    }

    cout << "solution obtined, final sol #"<<Final_solution.size()<<" \n";

    optimal_sol = Final_solution[findMin(Final_solution)];

    for (int i = 0; i < optimal_sol.action.size(); i++)
    {
        cout << optimal_sol.action[i] << " ";
    }
    cout << graph.size();
    return;
}
