#include <iostream>
#include <vector>

using namespace std;
int i;

class traffic_light{
public:
    traffic_light(int init_time, vector<int> timeset, int loc);
    int t0, red_dur, gre_dur, yel_dur, total_dur;
    int location;
    int give_clock(int t);
};

traffic_light::traffic_light(int init_time, vector<int> timeset, int loc){
    t0 = init_time;
    red_dur = timeset[0];
    yel_dur = timeset[1];
    gre_dur = timeset[2];
    total_dur = red_dur + yel_dur + gre_dur;
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
