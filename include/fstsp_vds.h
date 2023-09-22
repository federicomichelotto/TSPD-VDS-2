#ifndef FSTSP_H
#define FSTSP_H

#include <cplex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <float.h>
#include <sys/types.h>
#include <dirent.h>

#define NEIGHBORHOOD_SIZE 100


// Data Structures
typedef struct
{
    char output_name[100]; // ouput filename
    int seed;               // Seed given to cplex
    int run;                // number of run with this seed (automatically detected)
    int verbose;            // verbosity level [0,4]
    int ticks;              // flag: 0->seconds, 1->ticks
    int interactive;        // 0: none plot is displayed, 1: all plots are printed
    int saveplots;          // 1: save all plots
    int savefinalplot;       // 1: save plot of the final solution, 0 otherwise
    int heur;               // 0: exact method; 1: heuristic method
    double uav_min_speed;
    double uav_max_speed;
    int select_max_range_speed;    // 1: use drone max range speed
    int select_max_speed;   // 1: use drone max speed
    // POIKONEN INSTANCES
    int poikonen;           // 1: use the parser for the poikonen instances
    int alpha;              // For Poikonen instances: drone_speed = alpha * truck_speed
    int MHD;                // 1: max_endurance, truck_only customers, launch_time, retrieval_time

    int ha;                 // 1: use the parser for the ha instances
    double endurance;       // max endurance expressed in seconds, 0 => infinite endurance

    int murray_chu;         // 1: use the parser for the murray_chu instances

    int neighborhood_size; // neighborhood size of each node
    int iterations;         // #iterations for the genetic algorithm
} parameter;

typedef struct
{   // Node
    int id;             // number of the node (e.g. 1, 2, 3, ..., n)
    double x;           // x coordinate
    double y;           // y coordinate
    double weight;      // parcel weight associated to each node
    int truck_only;  // 1: truck-only customer
    int * neighbors; // array of neighbors wrt to the truck traveling time
} node_struct;

typedef struct
{   // Edge in the circuit

    double dist; // Weight of the edge
    int prev;    // Starting node
    int next;    // Ending node
    int flag;    // Inside circuit or not
} edge;

typedef struct {
    int ID;
    double time;
    int length;
    int* leg;
    int flag_feasible_drone_leg_exist;
    int flag_active;
} feasibleTruckLeg;

typedef struct {
    int count;
    feasibleTruckLeg* legs;
} feasibleTruckLegs_ik;


typedef struct
{
    char instance_path[1000]; // instance path
    char instance_name[200];  // instance name
    // Input data
    int dimension;  // Number of nodes of the problem
    node_struct *nodes;    // List of nodes
    int *truck_seq; // truck sequence
    int *drone_seq; // drone sequence

    double **truck_dists; // matrix of the truck travel distances for each (i,j)
    double **truck_times; // matrix of the truck travel times for each (i,j)

    double ***min_time_drone;      // 3D matrix of the minimum drone's travel times for each drone leg i-j-k
    double ***min_feas_time_drone; // 3D matrix of the minimum drone's travel times for each drone leg i-j-k

    double ***max_time_drone;      // 3D matrix of the maximum drone's travel times for each drone leg i-j-k
    double ***max_feas_time_drone; // 3D matrix of the maximum drone's travel times for each drone leg i-j-k

    double *max_i_drone_leg_times;   // array of the of maximum drone leg travel time that start from i
    double **max_ij_drone_leg_times; // array of the of maximum drone leg travel time that start from i and serve node j

    double **drone_dists; // matrix of the drone travel distances for each (i,j)
    //double **drone_power; // matrix of the drone power consumptions for each payload and for each admissible speed

    int number_feasible_truck_legs;
    // double *feasible_truck_legs_times;
    // int **feasible_truck_legs;
    // int **feasible_truck_legs_ik_begin;
    // int **feasible_truck_legs_ik_number;

    // int *reduced_truck_legs;

    // int reduced_number_feasible_truck_legs;
    // int *reduced_feasible_truck_legs;
    // int **reduced_feasible_truck_legs_ik_begin;
    // int **reduced_feasible_truck_legs_ik_number;

    /// new //
    // int**** feasible_truck_legs_ik; 
    // int** number_feasible_truck_legs_ik; 

    feasibleTruckLegs_ik** feasible_truck_legs2;
    /////////

    double **power_consumption; // matrix of power consumption w.r.t. the parcel weight and the speed
    parameter param;            // Parameters of the instance

    double time_limit; // Specifies the maximum time allowed within the execution
    double timestamp_start;
    double timestamp_finish;

    double timestamp_last_plot;
    int plot_counter;

    int model_type;   // Specifies the compact model to use
    double z_best;    // Value of the best solution available (incumbent)
    double best_lb;   // best lower bound
    double *best_sol; // Best xstar found
    int cols;         // #columns in cplex
    FILE *gnuplotPipe;

} instance;

typedef struct
{
    instance *inst;
    int ecount;
    int *elist;
    double *x;
    CPXCALLBACKCONTEXTptr context;
} doit_fn_input;

// Enumerations
enum verbose_level
{
    QUIET = 0,
    NORMAL = 1,
    VERBOSE = 2,
    NERD = 3,
    DEBUG = 4
};

static const char *verbose_name[] = {
    "QUIET",
    "NORMAL",
    "VERBOSE",
    "NERD",
    "DEBUG"
};

// *** TSP solver *** //

// Exact model builder
void build_model(CPXENVptr env, CPXLPptr lp, instance *inst, double bigM);

// Retrieve the position of the variable
int xTruck_pos(int i, int j, instance *inst);
int xDrone_pos(int i, int j, instance *inst);
int yT_pos(int i, instance *inst);
int yD_pos(int i, instance *inst);
int yC_pos(int i, instance *inst);
int a_pos(int i, instance *inst);
int z_pos(int i, int j, int k, instance *inst);
int alpha_pos(int i, instance *inst);
int beta_pos(int i, instance *inst);
int u_pos(int i, instance *inst);

// Retrieve the distance among each node of the instance
double dist(int i, int j, instance *inst);
double dist_GEO(int i, int j, instance *inst);
double dist_EUC_2D(int i, int j, instance *inst);
double euclDist(double xx1, double xx2, double yy1, double yy2);
double dist_MAN_2D(int i, int j, instance *inst);


// retrieve the drone and truck sequences from the xstar vector returned by cplex
void gather_solution(instance *inst, const double *xstar);

// retrieve the max drone leg time for any landing node given the takeoff node and the customer served by the drone
void retrieve_max_drone_leg_times(instance *inst);
void retrieve_max_ij_drone_leg_times(instance *inst);

// Callback
// static int CPXPUBLIC callback_driver(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);
// static int CPXPUBLIC callback_candidate(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);
// static int CPXPUBLIC callback_relaxation(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);
// int doit_fn_concorde(double cutval, int cutcount, int *cut, void *void_context);

// Data Structures
typedef struct
{
    int seed;        // Seed given to cplex
    int verbose;     // verbosity level [0,4]
    int ticks;       // flag: 0->seconds, 1->ticks
    int interactive; // 0: none plot is displayed, 1: all plots are printed
    int saveplots;   // 0: save only the final plot, 1: save all plots
} parameter_TSP;

typedef struct
{   // Node
    int id;   // number of the node (e.g. 1, 2, 3, ..., n)
    double x; // x coordinate
    double y; // y coordinate
} node_TSP;

typedef struct
{
    char instance_path[1000]; // Path of the file

    // Input data
    int dimension;    // Number of nodes of the problem
    node_TSP *nodes;  // List of nodes
    int *succ;        // array of nodes' successors
    double **weights; // edge weights

    parameter_TSP param; // Parameters of the instance

    double time_limit; // Specifies the maximum time allowed within the execution
    double timestamp_start;
    double timestamp_finish;

    double z_best;    // Value of the best solution available (incumbent)
    double best_lb;   // best lower bound
    double *best_sol; // Best xstar found
    int cols;         // #columns in cplex
    FILE *gnuplotPipe;

} instance_TSP;

int tsp_solver(instance_TSP *inst);

void basic_model_directed_TSP(CPXENVptr env, CPXLPptr lp, instance_TSP *inst);

void GG_lazy_2sec(CPXENVptr env, CPXLPptr lp, instance_TSP *inst);

double gather_solution_TSP(instance_TSP *inst, const double *xstar, int type);

int xpos(int i, int j, instance_TSP *inst);
int xpos_dir(int i, int j, instance_TSP *inst);
int upos(int i, instance_TSP *inst);
int ypos(int i, int j, instance_TSP *inst);

int generate_TSP_instance(instance *inst, instance_TSP *inst_tsp);
void post_tsp_sol(CPXENVptr env, CPXLPptr lp, instance_TSP *inst_tsp, instance *inst);
void save_and_plot_solution_TSP(instance_TSP *inst);

double optimal_solver(instance *inst, instance_TSP *inst_tsp, int iter);

#endif //FSTSP_H