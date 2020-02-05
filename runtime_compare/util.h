//
//  util.h
//  space-searching
//
//  Created by Tianqi on 10/31/19.
//  Copyright © 2019 Tianqi. All rights reserved.
//

#ifndef util_h
#define util_h


#endif /* util_h */

#include <iostream>
#include <vector>
#include <math.h>       /* fmod */

using namespace std;


class traffic_light{
public:
    
    double t0, red_dur, gre_dur, yel_dur, total_dur;
    double location;
    int give_clock(int t);
    traffic_light(){
    
    }
    traffic_light(double init_time, vector<double> timeset, double loc);
};

traffic_light::traffic_light(double init_time, vector<double> timeset, double loc){
    t0 = init_time;
    red_dur = timeset[0];
    yel_dur = timeset[1];
    gre_dur = timeset[2];
    total_dur = red_dur + yel_dur + gre_dur;
    location = loc;
};

int traffic_light::give_clock(int t){
    t = fmod(t+ t0, total_dur); // get the residual of the total period time
    if (t < gre_dur){
        return 1; // green light
    }
    else if (t < gre_dur + yel_dur){
        return 2; // yellow light
    } else{
        return 3; // red light
    }
};


//class car{
//public:
//    car();
//    double x0;
//    double v0;
//    double v_max;
//    double v_min;
//    vector<int> dynamics(vector<int> s, );
//};

// generate the linespace vector between double (a,b) with length of m
vector<double> linspace(double a, double b, int m){
    vector<double> output;
    double delta = (b-a)/(double) (m-1);
    for (int i = 0; i < m; i++){
        output.push_back(a + i*delta);
    }
    return output;
}
