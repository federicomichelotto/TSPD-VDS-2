#include "../include/utils.h"
#include "../include/genetic.h"
#include "../include/leg_formulation.h"

extern int max_truck_leg_size_eval;
extern int max_truck_leg_size_found;


int main(int argc, char **argv)
{
    double time_elapsed;
    instance inst;         // FSTSP-VSD instance of the problem
    instance_TSP inst_tsp; // TSP instance of the problem

    initialize_instance(&inst);

    srand(inst.param.seed);

    struct timespec timestamp;
    if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
        print_error("Error clock_gettime");
    inst.timestamp_start = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);

    double timestamp_incumbent = inst.timestamp_start;

    parse_command_line(argc, argv, &inst);
    if (inst.param.poikonen == 1) // Poikonen instances
        parse_instance_poikonen(&inst);
    else if (inst.param.ha == 1) // Ha instances
        parse_instance_ha(&inst);
    else // Murray instances
        parse_instance(&inst);

    //print_instance(&inst);

    // create the folders related to this run
    createInstanceFolders(&inst);

    if (!inst.param.heur) // exact method
    {
        // generate a TSP instance from the FSTSP instance
        generate_TSP_instance(&inst, &inst_tsp);
        // compute the optimal TSP solution
        tsp_solver(&inst_tsp);
        // compute the optimal FSTPS-VSD solution
        optimal_solver(&inst, &inst_tsp, 0) ? print_error("optimal_solver() ") : print_message("All went good inside optimal_solver()");

        // CHECK SCORE
        // printf("score computed: %f\n", compute_score(inst.truck_seq, inst.drone_seq, &inst));
        // int truck_seq[inst.dimension];
        // int drone_seq[inst.dimension];
        // int output_seq[inst.dimension];
        // get_precedence_seq(inst.truck_seq, inst.drone_seq, inst.dimension, output_seq);
        // double score = compute_sequence_min_time(output_seq, &inst, truck_seq, drone_seq);
        // printf("score DP: %f\n", score);
    }
    else // heuristic method
    {

        // TEST GENETIC ROUTINES
        int population_size1 = 200;
        // int n_close1 = 5; //10
        int n_close1 = 20; //20 -> 30
        int n_offsprings1 = 40;
        double default_penalty_coeff1 = 0.9; //0.9
        double penalty_coeff1 = default_penalty_coeff1;
        int n_restarts = 0;
        int max_iterations_wo_improv = 400; //400

        int default_radius = 0.04; //0.04
        int radius = default_radius;

        int last_restart = 0;

        //int iterations = 2000;

        int count = 0; // #iteration without improvement
        double last_obj = DBL_MAX;
        double curr_obj = DBL_MAX;

        // POPULATION 1
        individual **population1; // population of individuals (array of individual pointers)
        population1 = (individual **)malloc(population_size1 * sizeof(individual *));
        // diversity and fitness rankings population
        individual **diversity_ranking1 = (individual **)malloc(population_size1 * sizeof(individual *)); // individuals (index) with a smaller similarity measure have a smaller rank
        individual **fitness_ranking1 = (individual **)malloc(population_size1 * sizeof(individual *));   // individuals (index) with a smaller makespan have a smaller rank

        int count_left = 0;

        // initial populations
        for (int i = 0; i < population_size1; i++)
        {
            population1[i] = create_random_individual(&inst);
        }

        // compute the diversity ranking
        memcpy(fitness_ranking1, population1, population_size1 * sizeof(individual *));
        memcpy(diversity_ranking1, population1, population_size1 * sizeof(individual *));

        // compute avg distance of and individual to its closest neighbors
        for (int k = 0; k < population_size1; k++)
            population1[k]->avg_dist = compute_avg_dist(population1, population_size1, k, n_close1);
        // fitness rank
        compute_fitness_ranking(fitness_ranking1, population_size1);
        for (int k = 0; k < population_size1; k++)
            fitness_ranking1[k]->fitness_rank = k;
        // diversity rank
        compute_diversity_ranking(diversity_ranking1, population_size1);
        for (int k = 0; k < population_size1; k++)
            diversity_ranking1[k]->diversity_rank = k;

        // biased fitness
        for (int k = 0; k < population_size1; k++)
            population1[k]->biased_fitness = population1[k]->fitness_rank + penalty_coeff1 * population1[k]->diversity_rank;

        compute_biased_fitness_ranking(population1, population_size1);

        // iterative phase
        for (int iter = 0; iter < inst.param.iterations; iter++)
        {

            int nDuplicates = 0;

            // check duplicates in the populations
            if (iter)
            {
                int c2 = 0;
                for (int k = 0; k < population_size1; k++)
                {
                    for (int w = 0; w < population_size1; w++)
                    {
                        if (k == w)
                            continue;
                        if (hamming_dist(population1[k], population1[w]) <= (2 * inst.dimension) * radius)
                        {
                            if (population1[k] == fitness_ranking1[0])
                                continue;
                            if (rand() % 2)
                                continue;

                            nDuplicates++;


                            int div_ind = -1;
                            for (int j = 0; j < population_size1; j++)
                            {
                                if (population1[k] == diversity_ranking1[j])
                                {
                                    div_ind = j;
                                    break;
                                }
                            }

                            individual *p1 = binary_tournament(population1, population_size1);
                            while (p1 == population1[k])
                                p1 = binary_tournament(population1, population_size1);
                            individual *p2 = binary_tournament(population1, population_size1);
                            while (p2 == population1[k] || p2 == p1)
                                p2 = binary_tournament(population1, population_size1);


                            individual *offspring;
                            if (rand() % 4 == 0)
                                offspring = crossover_greedy(&inst, p1, p2);
                            else
                                offspring = crossover_heavy(&inst, p1, p2);

                            local_search(&inst, offspring);
                            free(population1[k]);
                            population1[k] = offspring;

                            if (div_ind != -1)
                            {
                                diversity_ranking1[div_ind] = population1[k];
                            }
                        }
                    }
                }
            }

            // COMPUTE the AVG DISTANCE between each individual and its n_close1 closest individuals

            memcpy(fitness_ranking1, population1, population_size1 * sizeof(individual *));
            memcpy(diversity_ranking1, population1, population_size1 * sizeof(individual *));

            // compute avg distance of and individual to its closest neighbors
            for (int k = 0; k < population_size1; k++)
                population1[k]->avg_dist = compute_avg_dist(population1, population_size1, k, n_close1);
            // fitness rank

            compute_fitness_ranking(fitness_ranking1, population_size1);

            //printf("Population sorted by fitness:\n");
            double avg_dist2 = 0.0;
            double avg_fitness2 = 0.0;
            for (int k = 0; k < population_size1; k++)
            {
                if (k < 10)
                {
                    avg_dist2 += fitness_ranking1[k]->avg_dist;
                    avg_fitness2 += fitness_ranking1[k]->makespan;
                }
                fitness_ranking1[k]->fitness_rank = k;
                //printf("[%d]: %f \n", k, fitness_ranking[k]->makespan);
            }
            avg_dist2 /= 10;
            avg_fitness2 /= 10;
            //printf("\t makespan | avg dist(10): %f | avg fitness(10): %f \n", avg_dist2, avg_fitness2);
            // diversity rank
            avg_dist2 = 0.0;
            avg_fitness2 = 0.0;
            compute_diversity_ranking(diversity_ranking1, population_size1);
            //printf("Population sorted by avg distance to closest neighbors:\n");
            for (int k = 0; k < population_size1; k++)
            {
                if (k < 10)
                {
                    avg_dist2 += diversity_ranking1[k]->avg_dist;
                    avg_fitness2 += diversity_ranking1[k]->makespan;
                }
                diversity_ranking1[k]->diversity_rank = k;
                //printf("[%d]: %f \n", k, diversity_ranking1[k]->avg_dist);
            }
            avg_dist2 /= 10;
            avg_fitness2 /= 10;
            //printf("\t distance | avg dist(10): %f | avg fitness(10): %f \n", avg_dist2, avg_fitness2);

            // biased fitness
            for (int k = 0; k < population_size1; k++)
                population1[k]->biased_fitness = population1[k]->fitness_rank + penalty_coeff1 * population1[k]->diversity_rank;

            compute_biased_fitness_ranking(population1, population_size1);
            //printf("\t diversity_ranking1[0]: fitness = %f , distance2champ = %d\n", diversity_ranking1[0]->makespan, hamming_dist(diversity_ranking1[0], fitness_ranking1[0]));

            double avg_dist_top50_1 = 0.0;
            double avg_fitness_top50_1 = 0.0;
            double avg_dist_last50_1 = 0.0;
            double avg_fitness_last50_1 = 0.0;
            for (int k = 0; k < population_size1; k++)
            {
                if (k < population_size1 / 2)
                {
                    avg_dist_top50_1 += population1[k]->avg_dist;
                    avg_fitness_top50_1 += population1[k]->makespan;
                }
                else
                {
                    avg_dist_last50_1 += population1[k]->avg_dist;
                    avg_fitness_last50_1 += population1[k]->makespan;
                }
            }
            avg_fitness_top50_1 = avg_fitness_top50_1 / (population_size1 / 2);
            avg_dist_top50_1 = avg_dist_top50_1 / (population_size1 / 2);
            avg_fitness_last50_1 = avg_fitness_last50_1 / (population_size1 - (int)(population_size1 / 2));
            avg_dist_last50_1 = avg_dist_last50_1 / (population_size1 - (int)(population_size1 / 2));
            // printf("\t TOP 50 %%    | avg fitness: %f  avg dist: %f \n", avg_fitness_top50_1, avg_dist_top50_1);
            // printf("\t BOTTOM 50 %% | avg fitness: %f  avg dist: %f \n", avg_fitness_last50_1, avg_dist_last50_1);

            for (int k = 0; k < n_offsprings1; k++)
            {
                if (population1[population_size1 - n_offsprings1 + k] == fitness_ranking1[0])
                    continue;
                free_individual(population1[population_size1 - n_offsprings1 + k]);
                individual *p1 = binary_tournament(population1, population_size1 - n_offsprings1);
                individual *p2 = binary_tournament(population1, population_size1 - n_offsprings1);
                while (p1 == p2)
                    p2 = binary_tournament(population1, population_size1 - n_offsprings1);
                individual *offspring;
                if (rand() % 4 == 0)
                    offspring = crossover_greedy(&inst, p1, p2);
                else
                    offspring = crossover_heavy(&inst, p1, p2);
                local_search(&inst, offspring);
                population1[population_size1 - n_offsprings1 + k] = offspring;

            }

            // compute the diversity ranking
            memcpy(fitness_ranking1, population1, population_size1 * sizeof(individual *));
            memcpy(diversity_ranking1, population1, population_size1 * sizeof(individual *));

            // compute avg distance of and individual to its closest neighbors
            for (int k = 0; k < population_size1; k++)
                population1[k]->avg_dist = compute_avg_dist(population1, population_size1, k, n_close1);
            // fitness rank
            compute_fitness_ranking(fitness_ranking1, population_size1);
            for (int k = 0; k < population_size1; k++)
                fitness_ranking1[k]->fitness_rank = k;
            // diversity rank
            compute_diversity_ranking(diversity_ranking1, population_size1);
            for (int k = 0; k < population_size1; k++)
                diversity_ranking1[k]->diversity_rank = k;

            // biased fitness
            for (int k = 0; k < population_size1; k++)
                population1[k]->biased_fitness = population1[k]->fitness_rank + penalty_coeff1 * population1[k]->diversity_rank;

            compute_biased_fitness_ranking(population1, population_size1);
            //printf("\t diversity_ranking1[0]: fitness = %f , distance2champ = %d\n", diversity_ranking1[0]->makespan, hamming_dist(diversity_ranking1[0], fitness_ranking1[0]));


            if (fabs(compute_score(&inst, fitness_ranking1[0]->truck_seq, fitness_ranking1[0]->drone_seq, inst.dimension - 1, 0) - fitness_ranking1[0]->makespan) > 1e-7)
            {
                printf("*** ACHTUNG!!! compute_score() gives a different result!\n");
            }

            // printf("* Iterations without improvement: %d | #restarts: %d\n", count, n_restarts);
            printf("[%d / %d] BEST INDIVIDUAL MAKESPAN: %f\n", iter, inst.param.iterations, fitness_ranking1[0]->makespan);

            if ( last_obj - fitness_ranking1[0]->makespan < 1e-2 )
            {
                count++;
            }
            else
            {
                count = 0;
                last_obj = curr_obj;
                curr_obj = fitness_ranking1[0]->makespan;
                inst.z_best = curr_obj;
                save_and_plot_solution_general(&inst, fitness_ranking1[0]->truck_seq, fitness_ranking1[0]->drone_seq, iter);
                if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
                    print_error("Error clock_gettime");
                timestamp_incumbent = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);

            }


            if (count == max_iterations_wo_improv)
            {
                for (int i = 0; i < population_size1; i++)
                {
                    if (population1[i] == fitness_ranking1[0]) {
                        continue;
                    }
                    if (i < (int)population_size1 * 0.25) {
                        if (rand() % 2) {
                            free_individual(population1[i]);
                            population1[i] = create_random_individual(&inst);
                        }
                        continue;
                    }
                    if (i > (int)population_size1 * 0.75) {
                        free_individual(population1[i]);
                        population1[i] = create_random_individual(&inst);
                        continue;
                    }
                    if (rand() % 4) {
                        free_individual(population1[i]);
                        population1[i] = create_random_individual(&inst);
                    }

                }
                count = 0;
                n_restarts++;
                last_restart = iter;
                radius = default_radius;
                // flag_greedy = 1;
                // max_greedy_wo_improv = 25;
                printf("Iterations without improvement: %d -> Restart #%d\n", max_iterations_wo_improv, n_restarts);
            }

        }

        printf("\nTRUCK SEQ: %d", fitness_ranking1[0]->truck_seq[0]);
        for (int i = 1; i < inst.dimension; i++)
        {
            printf(" -> %d", fitness_ranking1[0]->truck_seq[i]);
            if (fitness_ranking1[0]->truck_seq[i] == inst.dimension - 1)
                break;
        }
        printf("\nDRONE SEQ: %d", fitness_ranking1[0]->drone_seq[0]);
        for (int i = 1; i < inst.dimension; i++)
        {
            printf(" -> %d", fitness_ranking1[0]->drone_seq[i]);
            if (fitness_ranking1[0]->drone_seq[i] == inst.dimension - 1)
                break;
        }
        printf("\n\n");
        compute_score(&inst, fitness_ranking1[0]->truck_seq, fitness_ranking1[0]->drone_seq, inst.dimension - 1, 1);

    }


    if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
        print_error("Error clock_gettime");
    inst.timestamp_finish = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);

    double time_incumbent = timestamp_incumbent - inst.timestamp_start;

    time_elapsed = inst.timestamp_finish - inst.timestamp_start;
    inst.param.ticks ? printf("Elapsed time in ticks: %f\n", time_elapsed) : printf("Elapsed time: %f seconds\n", time_elapsed);
    printf("Best objective: %f\n", inst.z_best);

    // generate_csv_record("../output", inst.param.output_name, inst.instance_name, inst.param.seed, inst.model_type, inst.param.run, inst.z_best, time_elapsed, time_incumbent, inst.param.ticks);
// Free the memory used by the instance
//free_instance(&inst);
    return 0;
}
