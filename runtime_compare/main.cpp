#include "util.h"
#include<iostream>
#include <ctime>
#include <vector>


using namespace std;

int main(){
    int t0 = 40;
    vector<int> timeset{ 10, 20, 30 };
    int location = 50;
    traffic_light light1 = traffic_light(t0, timeset, location);
    cout<<"the light is "<<(int) light1.give_clock(20);



    return 0;
}
