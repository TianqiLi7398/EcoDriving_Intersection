# MDP desgin for intersection problem

## 1. Traffic light pattern
The traffic light is the fixed pattern (green -> yellow -> red), and 3 different patterns are studied
a. Green, Yellow, Red: (25, 5, 26)
b. Green, Yellow, Red: (20, 3, 15)


## 2. Files
### a. library files
`util.py`: class of `traffic_light`, `vehicle`, `opt_vehicle`;
`optimizer`: class of full info car planner `def_optimizer`;
`sto_optimizer_update.py`: class of partial info car planner `stochastic_light`.

### b. function files
`create_csv.py`: input light pattern and init velocity, generate `.csv` files of MDP;
`monte_carlo_trail.py`: input light pattern and init velocity, based on the `.csv` files of MDP, do Monte Carlo trail in traffic signal time offset t_0, and save resuts in `.csv` files.
