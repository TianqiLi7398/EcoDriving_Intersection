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

using namespace std;


class traffic_light{
public:
    
    int t0, red_dur, gre_dur, yel_dur, total_dur;
    int location;
    int give_clock(int t);
    traffic_light(){
    
    }
    traffic_light(int init_time, vector<int> timeset, int loc);
};

traffic_light::traffic_light(int init_time, vector<int> timeset, int loc){
    t0 = init_time;
    red_dur = timeset[0];
    yel_dur = timeset[1];
    gre_dur = timeset[2];
    total_dur = red_dur + yel_dur + gre_dur;
    location = loc;
};

int traffic_light::give_clock(int t){
    t = (t + t0) % total_dur;
    if (t < gre_dur){
        return 1;
    }
    else if (t < gre_dur + yel_dur){
        return 2;
    } else{
        return 3;
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
vector<double> linspace(double a, double b, int m){
    vector<double> output;
    double delta = (b-a)/(double) (m-1);
    for (int i = 0; i < m; i++){
        output.push_back(a + i*delta);
    }
    return output;
}
