#ifndef GENETIC_H
#define GENETIC_H

#include <cplex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <float.h>

//#define MAX_TRUCK_LEG_SIZE_EVAL 8 //8


typedef struct
{
    int *seq;        // chromosome
    int dimension;   // chromosome length
    int *truck_seq;  // optimal truck sequence derived from seq
    int *drone_seq;  // optimal drone sequence derived from seq
    int *truck_succ; // optimal truck successors derived from seq
    int *drone_succ; // optimal drone successors derived from seq
    double makespan; // makespan
    double avg_dist;  // diversity contribution w.r.t. to the distances to its closer neighbors
    int fitness_rank;
    int diversity_rank;
    double biased_fitness;
    int iter_left;  // the individual cannot be removed before than iter_left iterations
    int timeout; // if timeout = k, the individual cannot be selected as parent for the crossover operation for k iterations
} individual;

void initialize_individual(individual *ind, int dimension);
individual *create_random_individual(instance *inst);
individual *create_NN_individual(instance *inst, int options);
void free_individual(individual * ind);

individual *crossover(instance *inst, individual *ind1, individual *ind2);
individual *crossover_EM(instance *inst, individual *p1, individual *p2);
individual *crossover2(instance *inst, individual *p1, individual *p2);
individual *crossover_light(instance *inst, individual *p1, individual *p2);
individual *crossover_light2(instance *inst, individual *p1, individual *p2);

individual *crossover_light_2points(instance *inst, individual *p1, individual *p2);
individual *crossover_greedy(instance *inst, individual *p1, individual *p2);
individual *crossover_heavy(instance * inst, individual * p1, individual * p2);



individual *binary_tournament(individual **population, int population_size);
double compute_avg_dist(individual **population, int population_size, int index_ind, int n_neighbors);

void compute_fitness_ranking(individual **population, int population_size);
void compute_diversity_ranking(individual **population, int population_size);
void compute_biased_fitness_ranking(individual **population, int population_size);


double compute_sequence_min_time(instance *inst, int *seq, int seq_dim, int *truck_seq, int *drone_seq);
double compute_sequence_min_time_light(instance *inst, int *seq, int seq_dim, int *truck_seq, int *drone_seq,int max_truck_leg_size);
void generate_random_sequence(int *rand_seq, instance *inst);

void get_precedence_seq(int *truck_seq, int *drone_seq, int seq_dim, int *output_seq);
void get_successors(int *truck_succ, int *drone_succ, int dim, int *truck_seq, int *drone_seq, int end_value);
double compute_score(instance *inst, int *truck_seq, int *drone_seq, int end_value, int flag_print);

void nearest_neighbours(instance *inst, int *seq, int options);

// LOCAL SEARCH METHODS
double two_opt(instance *inst, int *seq, int seq_dim);
double optimize_truck_legs(instance *inst, int *truck_seq, int *drone_seq);
double reverse_drone_leg(instance *inst, int *truck_seq, int *drone_seq);
double swap_customers(instance *inst, individual *ind);
double partial_fstsp(instance *inst, int *input_seq, int offset, int dim_input_seq, int *opt_truck_seq, int *opt_drone_seq, int incl_depots);

void local_search(instance *inst, individual *ind);

// distance measure
int hamming_dist(individual *ind1, individual *ind2);

// greedy insertion
double greedy_insertion(instance *inst, int end_value, individual*offspring, int node);

#endif //GENETIC_H