#include <nlopt.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <float.h>

#define UAV_MIN_SPEED 0.1
#define UAV_MAX_SPEED 40.0                // [m / s]
#define UAV_WEIGHT_LIMIT 2.26796185004536 // [kg]  max UAV parcel weight = 5 lbs
#define UAV_BATTERY_CAPACITY 500000       // [J]
#define NLOPT_TOLERANCE 1e-4

#define CRUISE_ALTITUDE 50.0 //[m]
#define TAKEOFF_SPEED 10.0   // [m / s]
#define LANDING_SPEED 5.0    // [m / s]

#define TRUCK_DELIVERY_TIME 30.0 // [s]
#define UAV_DELIVERY_TIME 60.0   // [s]
#define UAV_LAUNCH_TIME 60.0     // [s]
#define UAV_RETRIEVAL_TIME 30.0  // [s]
#define UAV_CRUISE_ALTITUDE 50.0 // [m]
#define UAV_ASCENDING_SPEED 10.0 // [m/s]
#define UAV_DESCENDING_SPEED 5.0 // [m/s]
#define UAV_ROTATION_TIME 0.5    // [s]

#define POIKONEN_UAV_ENDURANCE 20.000000
#define POIKONEN_UAV_LAUNCH_TIME 1.0
#define POIKONEN_UAV_RETRIEVAL_TIME 1.0

// DRONE PARAMETERS //
#define K1 0.8554
#define K2 0.3051
#define C1 2.8037
#define C2 0.3177
#define C4 0.0296
#define C5 0.0279
#define ALPHA 10.0
#define G 9.8 // [m / s ^ 2]
#define W 1.5 // UAV frame weight[kg]

static const double takeoff_time = UAV_CRUISE_ALTITUDE / UAV_ASCENDING_SPEED + UAV_ROTATION_TIME;
static const double landing_time = UAV_CRUISE_ALTITUDE / UAV_DESCENDING_SPEED;
static const double extra_time = 2 * takeoff_time + 2 * landing_time + UAV_DELIVERY_TIME;

static const double q5 = C5 * pow(cos(ALPHA * M_PI / 180), 2);
static const double c12 = C1 + C2;
static const double c45 = pow(C4, 2) + pow(C5 * pow(cos(ALPHA * M_PI / 180), 2), 2);

// Power consumption for hovering with no payload
static const double hovering_power = (C1 + C2) * pow(W * G, (3.0f / 2.0f));

// DATA STRUCTURES FOR NLOPT //

typedef struct
{
    double s1, s2; // speeds
} my_variables;

typedef struct
{
    double B;                                            // battery capacity 
    double TIME;										 // target time when used by compute_min_energy_time()
    double d1, d2;                                       // distances
    double w;                                            // parcel weight
    double *best_obj_feas, *best_s1_feas, *best_s2_feas; // best feasible sol
} my_params;

double vertical_power(double speed, double weight);
double vertical_energy(double speed, double delta_altitude, double weight);
double cruise_power(double speed, double weight);
double cruise_energy(double speed, double weight, double dist);
double cruise_energy_time(double dist, double time, double weight);

// min drone leg time
double obj_min_drone_leg_time(unsigned n, const double *x, double *grad, void *data);
double constraint_min_drone_leg_time(unsigned n, const double *x, double *grad, void *data);
int compute_min_time(double d1, double d2, double w, double B, double init_s1, double init_s2, double uav_min_speed, double uav_max_speed, double *min_t, double *min_feas_t, double *s1, double *s2, double *s1_feas, double *s2_feas);

// max drone leg time
double obj_max_drone_leg_time(unsigned n, const double *x, double *grad, void *data);
double constraint1_max_drone_leg_time(unsigned n, const double *x, double *grad, void *data);
double constraint2_max_drone_leg_time(unsigned n, const double *x, double *grad, void *data);
int compute_max_time(double d1, double d2, double w, double B, double init_s1, double init_s2, double uav_min_speed, double uav_max_speed, double *max_t, double *max_feas_t, double *s1, double *s2, double *s1_feas, double *s2_feas);

// max range speed
double obj_max_range_speed(unsigned n, const double *x, double *grad, void *data);
double compute_max_range_speed(double w, double uav_min_speed, double uav_max_speed);

// min energy time
double compute_min_energy_time(double d1, double d2, double w, double time, double init_t1, double init_t2, double uav_min_speed, double uav_max_speed, double energy_battery);

// min_max_flag = 0: lb is not feasible, ub is feasible
// min_max_flag = 1: lb is feasibile, ub is not feasible
// compute the smallest (or largest) feasible travel time between lb_time and ub_time
double bisection(double d1, double d2, double w, double B, double lb_time, double ub_time, int min_max_flag, double uav_min_speed, double uav_max_speed);
