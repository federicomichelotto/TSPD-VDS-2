#ifndef UTILS_H
#define UTILS_H

#include <assert.h>

#include "fstsp_vds.h"


void parse_command_line(int argc, char **argv, instance *inst);
void parse_instance(instance *inst);
void parse_locations(instance *inst);
void parse_truck_travel_data(instance *inst);
void allocate_mem_arrays(instance *inst);
void allocate_mem_truck_times_and_dists(instance *inst);
void allocate_mem_drone_min_max_times(instance *inst);
void compute_drone_distances(instance *inst);
void parse_min_max_drone_legs_times(instance *inst);

void compute_min_max_drone_legs_times(instance *inst);

void compute_neighborhoods(instance *inst);
void compute_k_closest_nodes(instance *inst, int node, int k);


// Poikonen instances
void parse_instance_poikonen(instance *inst);
void parse_locations_poikonen(instance *inst);
void compute_truck_travel_times_poikonen(instance *inst);
void compute_drone_travel_times_poikonen(instance *inst);

//Ha instances
void parse_instance_ha(instance *inst);
void parse_locations_ha(instance *inst, double * truck_speed, double *drone_speed, double * max_endurance);
void compute_truck_travel_times_ha(instance *inst, double truck_speed);
void compute_drone_travel_times_ha(instance *inst, double drone_speed, double max_endurance);


void print_command_line(instance *inst);
void print_instance(instance *inst);

void check_format(char *param);
void initialize_instance(instance *inst);
void free_instance(instance *inst);

void print_help();
void print_error(const char *err);
void print_error_status(const char *err, int e);
void print_message(const char *msg);

void save_and_plot_solution(instance *inst, int iter);
void save_and_plot_solution_succ(instance *inst, int *truck_succ, int *drone_succ, double objval);
void save_and_plot_solution_general(instance *inst, int *truck_succ, int *drone_succ, int iter);

int generate_path(char *path, char *folder, char *type, const char *model, char *filename, int seed, char *extension);
int generate_csv_record(char *path, char* output_name, char *instance_name, int seed, int model, int run, double z_best, double time_elapsed, double time_incumbent, int ticks);

void createInstanceFolders(instance *inst);
int IsPathExist(const char *s);
void getTimeStamp(double *ts);

void compute_opt_speeds(instance *inst);
void cruise_power_consumption(double w, int nspeed, double *speeds, double *P);
double vertical_power_consumption(double w, double speed);
double min_time(double B, double *power_w, double *power_0, double min_speed, double granularity, int nspeed, double *speeds, double *T1, double *T2, int *opt_idx_s1, int *opt_idx_s2);
double max_time(double B, double *power_w, double *power_0, double max_speed, double granularity, int nspeed, double *speeds, double *T1, double *T2, int *opt_idx_s1, int *opt_idx_s2);

void setRunNumber(instance *inst);
int countDir(char *dir_path);

int count_all_feasible_truck_legs(instance *inst);
int count_truck_legs(instance *inst, int *R, int size_R, int last_truck_leg_cust, double truck_leg_cost, int i, int j, int k, int *unique_truck_legs,
                     int* positive_idx, int * size_pos_idx);

int count_all_feasible_truck_legs_no_recursion(instance *inst);
int count_truck_legs_no_recursion_save(instance *inst, const int i, const int k, int*** feasible_truck_legs, double** feasible_truck_legs_times, const int init_truck_legs_count);
void remove_non_optimal_truck_legs(instance *inst, int** feasible_truck_legs, const double* feasible_truck_legs_times, const int truck_legs_count);

int compute_truck_legs(instance* inst);
int compute_truck_legs2(instance* inst);


int count_truck_legs_no_recursion(instance *inst, int i, int j, int k);
int compare_truck_legs(const void* a, const void* b);
int compare_int(const void* a, const void* b);
int compare_int_asc(const void* a, const void* b);
int compare_feasible_truck_legs(const void* a, const void* b);


#endif //UTILS_H
