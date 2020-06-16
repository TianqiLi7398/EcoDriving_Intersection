#include "optimizer.h"
#include<iostream>
#include <ctime>
#include <vector>


using namespace std;


int main(){
    double t0 = 40;
    // int timeset_[3] = { 10, 20, 30 };
    // vector<int> timeset(timeset_, timeset_+3);
    vector<double> timeset{20, 5, 25};
    double location = 50;
    traffic_light light1(t0, timeset, location);
    cout<<"the light is "<<(int) light1.give_clock(5)<<"\n";
    
    vector<double> init_state = {0, 15};
    vector<int> size = {6, 22};
    double dv_ = 1;
    double dt_ = .1;
    vector<double> a_threshold_ = {-5, 8};
    vector<double> para = {.125, .125};
    dfs_optimizer an_optimizer(init_state, size, dv_, dt_, a_threshold_, para, light1);
    
    an_optimizer.solver();
    
    
    
    return 0;
}
