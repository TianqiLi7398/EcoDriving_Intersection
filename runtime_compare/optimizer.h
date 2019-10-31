//
//  optimizer.h
//  space-searching
//
//  Created by Tianqi on 10/31/19.
//  Copyright © 2019 Tianqi. All rights reserved.
//

#ifndef optimizer_h
#define optimizer_h


#endif /* optimizer_h */

#include "util.h"
#include <vector>
#include <cstdlib>
#include <cmath>





struct edge{
    double v1;
    double v2;
    double dt;
    double cost;
};

struct node {
    double x;
    double v;
    vector<edge> in_edge;
    vector<edge> out_edge;
    vector<double> action;
};

class dfs_optimizer{
public:
    int i, j, k;
    double dx;
    double dv;
    double dt;
    double w1;
    double w2;
    double delta_t_min;
    vector<double> a_threshold;
    int m,n;
    vector<vector<node>> graph;
    node begin_node;
    node end_node;
    traffic_light single_light;
    double end_loc;
    
    dfs_optimizer(vector<double> init_state, vector<int> size, double dv_, double dt_, vector<double> a_threshold_, vector<double> para, traffic_light light);
    void build_graph(vector<double> loc_grid, vector<double> vel_grid);
    double solver();
};

dfs_optimizer::dfs_optimizer(vector<double> init_state, vector<int> size, double dv_, double dt_, vector<double> a_threshold_, vector<double> para, traffic_light light){
    dv = dv_;
    dt = dt_;
    a_threshold = a_threshold_;
    w1 = para[0];
    w2 = para[1];
    m = size[0];
    n = size[1];
    begin_node.x = init_state[0];
    begin_node.v = init_state[1];
    end_node.x = end_loc;
    dx = (end_loc - init_state[0])/ (double) m;
    delta_t_min = dx/(((double) n-1.0) * dv);
    single_light = light;
    end_loc = single_light.location + dx;
}

void dfs_optimizer::build_graph(vector<double> loc_grid, vector<double> vel_grid){
    
//    build the skeleton of the graph
    for(i = 0; i < m; i++){
        vector<node> row;
        for(j = 0; j < n; j++){
            node v;
            v.x = loc_grid[i];
            v.v = vel_grid[j];
            row.push_back(v);
        }
        graph.push_back(row);
    }
    
//    after the skeleton, start building edges
//    first layer initialize
    for(i = 0; i < n; i++){
        if(graph[0][i].v > 0){
            double delta_v = graph[0][i].v - begin_node.v;
            double delta_t = 2 * dx / (graph[0][i].v + begin_node.v);
            double a = delta_v/delta_t;
            if (a >= a_threshold[0] and a <= a_threshold[1]){
                double cost = delta_t * w1 / delta_t_min + abs(a/a_threshold[1]) * w2;
                edge e = {begin_node.v, graph[0][i].v, delta_t, cost};
                
//                add the edge between begin_node and node in the first layer
                begin_node.out_edge.push_back(e);
                graph[0][i].in_edge.push_back(e);
            }
        }
    }
    
//    later layers, calculate all edges before the intersection
    for (i = 0; i < m; i ++){
        for (j = 0; j < n; j++){
            if(graph[i][j].in_edge.size() > 0){
//                    this node is reachable from initial node
                for (k = 0; k < n; k++){
                    if (!(k < n-1 && graph[i+1][k].v == 0)){
                        double delta_v = graph[i+1][k].v - graph[i][j].v;
                        
                        double delta_t = 2 * dx / (graph[i+1][k].v + graph[i][j].v);
                        double a = delta_v / delta_t;
                        if (a >= a_threshold[0] and a <= a_threshold[1]){
                            double cost = cost = delta_t * w1 / delta_t_min + abs(a/a_threshold[1]) * w2;
                            edge e = {graph[i][j].v, graph[i+1][k].v, delta_t, cost};
                            graph[i][j].out_edge.push_back(e);
                            graph[i+1][k].in_edge.push_back(e);
                        }
                    }
                }
            }
        }
    }
}

double dfs_optimizer::solver(){
    vector<double> loc_grid = linspace(begin_node.x, single_light.location, m);
    vector<double> vel_grid = linspace(0, ((double)n - 1) * dv, n);
    
    dfs_optimizer::build_graph(loc_grid, vel_grid);
    
    
    return 0;
}



