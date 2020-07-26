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

`smart_controller.py`: class of human model controll called `smart_control`.

### b. function files
`create_csv.py`: input light pattern and init velocity, generate `.csv` files of MDP;

`monte_carlo_trail.py`: input light pattern and init velocity, based on the `.csv` files of MDP, do Monte Carlo trail in traffic signal time offset t_0, and save resuts in `_monte_carlo.csv` files.

`cost_table_gen.py`: generate the cost table of (v, a) given fuel cost model as a `.csv` file.

`mdp_job.ada_job`: job files of TAMU HPRC.

`process.py`: with the result file in `_monte_carlo.csv`, output a plot of mean v.s. Monte Carlo trials.

### c. result files
`main.py`: plot the behaviors of 3 controllers, `det, sto, dum`.

`optimize_main.py`: plot the behaviors of `det`.

`fuel_eco.py`: shows the fuel economy fuel/distance of the fuel model.

`plan_plot.py`: plot all behaviors of a controller with changing time offset t_0 in one picture.

### d. debug files

debug by behavior: `sto_main.py`, `det_main.py`, `dummmy_main.py`.
