#include "../include/utils.h"
#include "../include/genetic.h"
#include "../include/min_max_opt.h"

int max_truck_leg_size_eval = 10; //8, 10
int max_truck_leg_size_found = 0;

void get_successors(int *truck_succ, int *drone_succ, int dim, int *truck_seq, int *drone_seq, int end_value)
{
    // init
    for (int i = 0; i < dim; i++)
    {
        truck_succ[i] = -1;
        drone_succ[i] = -1;
    }
    // truck
    for (int i = 0; i < dim; i++)
    {
        if (truck_seq[i] == end_value)
            break;
        truck_succ[truck_seq[i]] = truck_seq[i + 1];
    }
    // drone
    for (int i = 0; i < dim; i++)
    {
        if (drone_seq[i] == end_value)
            break;
        drone_succ[drone_seq[i]] = drone_seq[i + 1];
    }
}

void initialize_individual(individual *ind, int dimension)
{
    ind->dimension = dimension;
    ind->seq = (int *)malloc(dimension * sizeof(int));
    ind->truck_seq = (int *)malloc(dimension * sizeof(int));
    ind->drone_seq = (int *)malloc(dimension * sizeof(int));
    ind->truck_succ = (int *)malloc(dimension * sizeof(int));
    ind->drone_succ = (int *)malloc(dimension * sizeof(int));
    ind->makespan = 0;
    ind->avg_dist = 0;
    ind->fitness_rank = 0;
    ind->diversity_rank = 0;
    ind->biased_fitness = 0;
    ind->iter_left = 0;
    ind->timeout = 0;
}

individual *create_random_individual(instance *inst)
{
    individual *ind = (individual *)malloc(sizeof(individual));
    initialize_individual(ind, inst->dimension);
    // ind->dimension = inst->dimension;
    // ind->seq = (int *)malloc(inst->dimension * sizeof(int));
    // ind->truck_seq = (int *)malloc(inst->dimension * sizeof(int));
    // ind->drone_seq = (int *)malloc(inst->dimension * sizeof(int));
    // ind->truck_succ = (int *)malloc(inst->dimension * sizeof(int));
    // ind->drone_succ = (int *)malloc(inst->dimension * sizeof(int));
    generate_random_sequence(ind->seq, inst);
    ind->makespan = compute_sequence_min_time(inst, ind->seq, ind->dimension, ind->truck_seq, ind->drone_seq);
    get_successors(ind->truck_succ, ind->drone_succ, ind->dimension, ind->truck_seq, ind->drone_seq, ind->dimension - 1);
    return ind;
}

individual *create_NN_individual(instance *inst, int options)
{

    individual *ind = (individual *)malloc(sizeof(individual));
    initialize_individual(ind, inst->dimension);
    // ind->dimension = inst->dimension;
    // ind->seq = (int *)malloc(inst->dimension * sizeof(int));
    // ind->truck_seq = (int *)malloc(inst->dimension * sizeof(int));
    // ind->drone_seq = (int *)malloc(inst->dimension * sizeof(int));
    // ind->truck_succ = (int *)malloc(inst->dimension * sizeof(int));
    // ind->drone_succ = (int *)malloc(inst->dimension * sizeof(int));
    nearest_neighbours(inst, ind->seq, options);
    ind->makespan = compute_sequence_min_time(inst, ind->seq, ind->dimension, ind->truck_seq, ind->drone_seq);
    get_successors(ind->truck_succ, ind->drone_succ, ind->dimension, ind->truck_seq, ind->drone_seq, ind->dimension - 1);
    return ind;
}

void free_individual(individual *ind)
{
    free(ind->seq);
    free(ind->truck_seq);
    free(ind->drone_seq);
    free(ind->truck_succ);
    free(ind->drone_succ);
    free(ind);
}

int hamming_dist(individual *ind1, individual *ind2)
{
    if (ind1->dimension != ind2->dimension)
        print_error("Error in hamming_dist(): ind1 and ind2 have different dimensions.\n");
    int avg_dist = 0; // diversity contribution to return
    for (int i = 0; i < ind1->dimension - 1; i++)
    {
        // truck successors
        if (ind1->truck_succ[i] == -1 && ind2->truck_succ[i] != -1)
            avg_dist++;
        else if (ind1->truck_succ[i] != -1 && ind2->truck_succ[i] == -1)
            avg_dist++;
        else if (ind1->truck_succ[i] != ind2->truck_succ[i])
            avg_dist++;
        // drone successors
        if (ind1->drone_succ[i] == -1 && ind2->drone_succ[i] != -1)
            avg_dist++;
        else if (ind1->drone_succ[i] != -1 && ind2->drone_succ[i] == -1)
            avg_dist++;
        else if (ind1->drone_succ[i] != ind2->drone_succ[i])
            avg_dist++;
    }
    return avg_dist;
}

individual *crossover(instance *inst, individual *p1, individual *p2)
{
    if (p1->dimension != p2->dimension)
    {
        printf("p1 = %d, p2=%d\n", p1->dimension, p2->dimension);
        print_error("Error in crossover(): p1 and p2 have different dimensions.\n");
    }
    individual *offspring = (individual *)malloc(sizeof(individual));
    offspring->seq = (int *)malloc(inst->dimension * sizeof(int));
    offspring->truck_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->truck_succ = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_succ = (int *)malloc(p1->dimension * sizeof(int));
    // define the cutoff point
    int cutoff = 15 + rand() % 70;
    cutoff = (int)(cutoff * p1->dimension / 100.0);
    int flag[p1->dimension];
    for (int i = 0; i < p1->dimension; i++)
        flag[i] = 0;

    if (rand() % 2)
    {
        // copy genetic material from p1
        for (int i = 0; i < cutoff; i++)
        {
            offspring->seq[i] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        // copy genetic material from p2
        int dim = cutoff; // current chromosome dimension
        for (int i = cutoff; i < p1->dimension; i++)
        {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }
        offspring->dimension = dim;
    }
    else
    {
        // copy genetic material from p2
        for (int i = p1->dimension - cutoff; i < p1->dimension; i++)
        {
            offspring->seq[i] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }

        // copy genetic material from p1
        int dim = 0;
        for (int i = 0; i < p1->dimension - cutoff; i++)
        {
            if (flag[p1->seq[i]])
                continue;
            offspring->seq[dim++] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        memmove(offspring->seq + dim, offspring->seq + p1->dimension - cutoff, cutoff * sizeof(int));
        offspring->dimension = cutoff + dim;
    }

    // add the remeaning nodes in the seq
    if (offspring->dimension < p1->dimension)
    {
        for (int i = 1; i < p1->dimension - 1; i++)
        {
            if (flag[i]) // node already present
                continue;

            offspring->dimension++;
            // node i must be added
            // compute the min time for each possible position for the new node
            double min_t = DBL_MAX;
            int *opt_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *opt_truck_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *opt_drone_seq = (int *)malloc(offspring->dimension * sizeof(int));

            int *new_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_truck_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_drone_seq = (int *)malloc(offspring->dimension * sizeof(int));
            for (int pos = 1; pos < offspring->dimension - 1; pos++)
            {
                // try node i at position pos
                memcpy(new_seq, offspring->seq, pos * sizeof(int));
                memcpy(new_seq + pos + 1, offspring->seq + pos, (offspring->dimension - 1 - pos) * sizeof(int));
                new_seq[pos] = i;
                double t = compute_sequence_min_time(inst, new_seq, offspring->dimension, new_truck_seq, new_drone_seq);
                if (t < min_t)
                {
                    min_t = t;
                    memcpy(opt_seq, new_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_truck_seq, new_truck_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_drone_seq, new_drone_seq, offspring->dimension * sizeof(int));
                }
            }
            // update offspring
            free(offspring->seq);
            free(offspring->truck_seq);
            free(offspring->drone_seq);
            offspring->seq = opt_seq;
            offspring->truck_seq = opt_truck_seq;
            offspring->drone_seq = opt_drone_seq;
            offspring->makespan = min_t;

            //double delta = swap_customers(inst, offspring);
            //offspring->makespan -= delta;
            // if (delta > 0)
            //     printf("* offspring improved with swap_customers() by %f\n", delta);
        }
    }

    offspring->makespan = compute_sequence_min_time(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq);

    get_successors(offspring->truck_succ, offspring->drone_succ, offspring->dimension, offspring->truck_seq, offspring->drone_seq, offspring->dimension - 1);

    if (fabs(offspring->makespan - compute_sequence_min_time(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq)) > 1e-4)
    {
        printf("!!! ACHTUNG !!! min_t != compute_sequence_min_time() \n");
    }
    return offspring;
}

individual *crossover_EM(instance *inst, individual *p1, individual *p2)
{
    if (p1->dimension != p2->dimension)
        print_error("Error in crossover(): p1 and p2 have different dimensions.\n");
    individual *offspring = (individual *)malloc(sizeof(individual));
    offspring->seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->truck_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->truck_succ = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_succ = (int *)malloc(p1->dimension * sizeof(int));
    // define the cutoff point
    int cutoff = 15 + rand() % 70;
    cutoff = (int)(cutoff * p1->dimension / 100.0);
    int flag[p1->dimension];
    for (int i = 0; i < p1->dimension; i++)
        flag[i] = 0;

    // copy genetic material from p1
    for (int i = 0; i < cutoff; i++)
    {
        offspring->seq[i] = p1->seq[i];
        flag[p1->seq[i]] = 1;
    }
    // copy genetic material from p2
    int dim = cutoff; // current chromosome dimension
    for (int i = cutoff; i < p1->dimension; i++)
    {
        if (flag[p2->seq[i]])
            continue;
        offspring->seq[dim++] = p2->seq[i];
        flag[p2->seq[i]] = 1;
    }

    offspring->dimension = dim;

    // add the remeaning nodes in the seq
    if (offspring->dimension < p1->dimension)
    {
        int R[p1->dimension - offspring->dimension]; // array of the remaining elements after the combination phase
        int nR = 0;
        for (int i = 0; i < p1->dimension; i++)
        {
            if (flag[i] == 0)
                R[nR++] = i;
        }

        for (int i = 0; i < nR; i++)
        {
            // select randomly the node to add in the offspring
            int rand_idx = rand() % (p1->dimension - offspring->dimension);
            int node = R[rand_idx];
            R[rand_idx] = R[p1->dimension - offspring->dimension - 1];

            double min_t = inst->truck_times[0][node] + inst->truck_times[node][offspring->seq[node]];
            int min_pos = 1;
            for (int pos = 2; pos < dim - 1; pos++)
            {
                double tmp = inst->truck_times[offspring->seq[pos - 1]][node] + inst->truck_times[node][offspring->seq[pos]];
                if (tmp < min_t)
                {
                    min_t = tmp;
                    min_pos = pos;
                }
            }
            memmove(offspring->seq + min_pos + 1, offspring->seq + min_pos, (offspring->dimension - min_pos) * sizeof(int));
            offspring->seq[min_pos] = node;
            offspring->dimension++;
        }

    }
    offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, max_truck_leg_size_eval);
    get_successors(offspring->truck_succ, offspring->drone_succ, offspring->dimension, offspring->truck_seq, offspring->drone_seq, offspring->dimension - 1);
    return offspring;

}

individual *crossover_light(instance * inst, individual * p1, individual * p2)
{
    if (p1->dimension != p2->dimension)
    {
        printf("p1 = %d, p2=%d\n", p1->dimension, p2->dimension);
        print_error("Error in crossover(): p1 and p2 have different dimensions.\n");
    }
    individual *offspring = (individual *)malloc(sizeof(individual));
    offspring->seq = (int *)malloc(inst->dimension * sizeof(int));
    offspring->truck_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->truck_succ = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_succ = (int *)malloc(p1->dimension * sizeof(int));

    int max_truck_leg_size = max_truck_leg_size_eval;
    // define the cutoff point
    // int cutoff = 10 + rand() % 80;
    int cutoff = 5 + rand() % 90;
    cutoff = (int)(cutoff * p1->dimension / 100.0);
    int flag[p1->dimension];
    for (int i = 0; i < p1->dimension; i++)
        flag[i] = 0;

    int r = rand() % 3;

    switch (r) {
    case 0:
    {
        // copy genetic material from p1
        for (int i = 0; i < cutoff; i++)
        {
            offspring->seq[i] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        // copy genetic material from p2
        int dim = cutoff; // current chromosome dimension
        int start_p2 = inst->nodes[p1->seq[cutoff - 1]].neighbors[rand() % inst->param.neighborhood_size];
        // find node [start_p2] in p2->seq
        int ind_start_p2;
        for (ind_start_p2 = 0 ; ind_start_p2 < inst->dimension; ind_start_p2++) {
            if (p2->seq[ind_start_p2] == start_p2)
                break;
        }
        for (int i = ind_start_p2; i < p2->dimension; i++)
        {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;
            if (dim == p2->dimension)
                break;
        }
        offspring->dimension = dim;
        break;
    }
    case 1:
    {
        for (int i = p1->dimension - cutoff; i < p1->dimension; i++)
        {
            offspring->seq[i] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }

        // copy genetic material from p1
        int dim = 0;
        for (int i = 0; i < p1->dimension - cutoff; i++)
        {
            if (flag[p1->seq[i]])
                continue;
            offspring->seq[dim++] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        memmove(offspring->seq + dim, offspring->seq + p1->dimension - cutoff, cutoff * sizeof(int));
        offspring->dimension = cutoff + dim;
        break;
    }
    case 2:
    {
        // copy genetic materal of parent p1 from A to B
        // int a = 10 + rand() % 40;
        // int b = 50 + rand () % 40;
        int a = 10 + rand() % 60;
        int b = a + 5 + rand () % 65;
        a = (int)(a * p1->dimension / 100.0);
        b = (int)(b * p1->dimension / 100.0);
        if (b > inst->dimension - 1)
            b = inst->dimension - 1;

        for (int i = a; i < b; i++)
        {
            flag[p1->seq[i]] = 1;
        }

        int dim = 0;
        for (int i = 0; i < a; i++) {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }
        for (int i = a; i < b; i++) {
            offspring->seq[dim++] = p1->seq[i];
        }
        for (int i = b; i < p2->dimension; i++) {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;

        }



        offspring->dimension = dim;
        break;
    }
    }


    // if (rand() % 2)
    // {
    //     // copy genetic material from p1
    //     for (int i = 0; i < cutoff; i++)
    //     {
    //         offspring->seq[i] = p1->seq[i];
    //         flag[p1->seq[i]] = 1;
    //     }
    //     // copy genetic material from p2
    //     int dim = cutoff; // current chromosome dimension
    //     for (int i = cutoff; i < p1->dimension; i++)
    //     {
    //         if (flag[p2->seq[i]])
    //             continue;
    //         offspring->seq[dim++] = p2->seq[i];
    //         flag[p2->seq[i]] = 1;
    //     }
    //     offspring->dimension = dim;
    // }
    // else
    // {
    //     /*        // copy genetic material from p2
    //             for (int i = 0; i < cutoff; i++)
    //             {
    //                 offspring->seq[i] = p2->seq[i];
    //                 flag[p2->seq[i]] = 1;
    //             }
    //             // copy genetic material from p2
    //             int dim = cutoff; // current chromosome dimension
    //             for (int i = cutoff; i < p1->dimension; i++)
    //             {
    //                 if (flag[p1->seq[i]])
    //                     continue;
    //                 offspring->seq[dim++] = p1->seq[i];
    //                 flag[p1->seq[i]] = 1;
    //             }
    //             offspring->dimension = dim;*/

    //     // copy genetic material from p2
    //     for (int i = p1->dimension - cutoff; i < p1->dimension; i++)
    //     {
    //         offspring->seq[i] = p2->seq[i];
    //         flag[p2->seq[i]] = 1;
    //     }

    //     // copy genetic material from p1
    //     int dim = 0;
    //     for (int i = 0; i < p1->dimension - cutoff; i++)
    //     {
    //         if (flag[p1->seq[i]])
    //             continue;
    //         offspring->seq[dim++] = p1->seq[i];
    //         flag[p1->seq[i]] = 1;
    //     }
    //     memmove(offspring->seq + dim, offspring->seq + p1->dimension - cutoff, cutoff * sizeof(int));
    //     offspring->dimension = cutoff + dim;
    // }

    // for (int i = 0; i < p1->dimension; i++)
    // {
    //     printf("(%d)%d, ", i, flag[i]);
    // }
    // printf("\n");

    // add the remeaning nodes in the seq
    if (offspring->dimension < p1->dimension)
    {
        int R[p1->dimension - offspring->dimension]; // array of the remaining elements after the combination phase
        int nR = 0;
        for (int i = 0; i < p1->dimension; i++)
        {
            if (flag[i] == 0)
                R[nR++] = i;
        }
        // for (int i = 0; i < p1->dimension - offspring->dimension; i++)
        // {
        //     printf("%d, ", R[i]);
        // }
        // printf("\t nR = %d \n", nR);

        for (int i = 0; i < nR; i++)
        {
            // if (flag[i]) // node already present
            //     continue;

            // select randomly the node to add in the offspring
            int rand_idx = rand() % (p1->dimension - offspring->dimension);
            int node = R[rand_idx];
            R[rand_idx] = R[p1->dimension - offspring->dimension - 1];

            // find the position with the shortest distances between the predecessor and the sucessor nodes
            int min_pos = 1;
            double min_d = inst->min_feas_time_drone[offspring->seq[0]][node][offspring->seq[1]];
            for (int pos = 2; pos < offspring->dimension - 1; pos++)
            {
                double d = inst->min_feas_time_drone[offspring->seq[pos - 1]][node][offspring->seq[pos]];
                if (d < min_d)
                {
                    min_d = d;
                    min_pos = pos;
                }
            }
            int start_pos = min_pos - 15;
            if (start_pos < 1)
                start_pos = 1;
            int end_pos = start_pos + 30;
            if (end_pos > offspring->dimension - 2)
                end_pos = offspring->dimension - 2;
            // start_pos = 1;
            // end_pos = offspring->dimension - 2;

            offspring->dimension++;
            // node i must be added
            // compute the min time for each possible position for the new node
            double min_t = DBL_MAX;
            int *opt_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *opt_truck_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *opt_drone_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_truck_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_drone_seq = (int *)malloc(offspring->dimension * sizeof(int));

            for (int pos = start_pos; pos <= end_pos; pos++)
            {
                // try node i at position pos
                memcpy(new_seq, offspring->seq, pos * sizeof(int));
                memcpy(new_seq + pos + 1, offspring->seq + pos, (offspring->dimension - 1 - pos) * sizeof(int));
                new_seq[pos] = node;
                double t = compute_sequence_min_time_light(inst, new_seq, offspring->dimension, new_truck_seq, new_drone_seq, max_truck_leg_size);

                if (t < min_t)
                {
                    min_t = t;
                    memcpy(opt_seq, new_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_truck_seq, new_truck_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_drone_seq, new_drone_seq, offspring->dimension * sizeof(int));
                }
            }
            // update offspring
            free(offspring->seq);
            free(offspring->truck_seq);
            free(offspring->drone_seq);
            offspring->seq = opt_seq;
            offspring->truck_seq = opt_truck_seq;
            offspring->drone_seq = opt_drone_seq;
            offspring->makespan = min_t;

            double delta = swap_customers(inst, offspring);
            offspring->makespan -= delta;
            // if (delta > 0)
            //     printf("* offspring improved with swap_customers() by %f\n", delta);
        }
    }
    offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, max_truck_leg_size);
    get_successors(offspring->truck_succ, offspring->drone_succ, offspring->dimension, offspring->truck_seq, offspring->drone_seq, offspring->dimension - 1);

    // if (fabs(offspring->makespan - compute_sequence_min_time(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq)) > 1e-4)
    // {
    //     printf("!!! ACHTUNG !!! min_t != compute_sequence_min_time() \n");
    // }
    return offspring;
}

individual *crossover_light2(instance * inst, individual * p1, individual * p2)
{
    if (p1->dimension != p2->dimension)
    {
        printf("p1 = %d, p2=%d\n", p1->dimension, p2->dimension);
        print_error("Error in crossover(): p1 and p2 have different dimensions.\n");
    }
    individual *offspring = (individual *)malloc(sizeof(individual));
    offspring->seq = (int *)malloc(inst->dimension * sizeof(int));
    offspring->truck_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->truck_succ = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_succ = (int *)malloc(p1->dimension * sizeof(int));
    // define the cutoff point
    int cutoff = 15 + rand() % 70;
    cutoff = (int)(cutoff * p1->dimension / 100.0);
    int flag[p1->dimension];
    for (int i = 0; i < p1->dimension; i++)
        flag[i] = 0;

    if (rand() % 2)
    {
        // copy genetic material from p1
        for (int i = 0; i < cutoff; i++)
        {
            offspring->seq[i] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        // copy genetic material from p2
        int dim = cutoff; // current chromosome dimension
        int start_p2 = inst->nodes[p1->seq[cutoff - 1]].neighbors[rand() % inst->param.neighborhood_size];
        // find node [start_p2] in p2->seq
        int ind_start_p2;
        for (ind_start_p2 = 0 ; ind_start_p2 < inst->dimension; ind_start_p2++) {
            if (p2->seq[ind_start_p2] == start_p2)
                break;
        }
        for (int i = ind_start_p2; i < p2->dimension; i++)
        {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;
            if (dim == p2->dimension)
                break;
        }
        offspring->dimension = dim;
    }
    else
    {
        for (int i = p1->dimension - cutoff; i < p1->dimension; i++)
        {
            offspring->seq[i] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }

        // copy genetic material from p1
        int dim = 0;
        for (int i = 0; i < p1->dimension - cutoff; i++)
        {
            if (flag[p1->seq[i]])
                continue;
            offspring->seq[dim++] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        memmove(offspring->seq + dim, offspring->seq + p1->dimension - cutoff, cutoff * sizeof(int));
        offspring->dimension = cutoff + dim;
    }


    // for (int i = 0; i < p1->dimension; i++)
    // {
    //     printf("(%d)%d, ", i, flag[i]);
    // }
    // printf("\n");

    // compute_seq_min_time_light()
    // get_successors()

    // add the remeaning nodes in the seq
    if (offspring->dimension < p1->dimension)
    {
        int R[p1->dimension - offspring->dimension]; // array of the remaining elements after the combination phase
        int nR = 0;
        for (int i = 0; i < p1->dimension; i++)
        {
            if (flag[i] == 0)
                R[nR++] = i;
        }
        // for (int i = 0; i < p1->dimension - offspring->dimension; i++)
        // {
        //     printf("%d, ", R[i]);
        // }
        // printf("\t nR = %d \n", nR);

        for (int i = 0; i < nR; i++)
        {
            // if (flag[i]) // node already present
            //     continue;

            // select randomly the node to add in the offspring
            int rand_idx = rand() % (p1->dimension - offspring->dimension);
            int node = R[rand_idx];
            R[rand_idx] = R[p1->dimension - offspring->dimension - 1];

            // find the position with the shortest distances between the predecessor and the sucessor nodes
            int min_pos = 1;
            double min_tt = inst->truck_times[offspring->seq[0]][node] + inst->truck_times[node][offspring->seq[1]];
            if (inst->truck_times[node][offspring->seq[2]] < inst->truck_times[node][offspring->seq[1]])
                min_tt = inst->truck_times[offspring->seq[0]][node] + inst->truck_times[node][offspring->seq[2]];
            for (int pos = 2; pos < offspring->dimension - 1; pos++)
            {
                double t, t1, t2, t3;
                t1 = inst->truck_times[offspring->seq[pos - 1]][node] + inst->truck_times[node][offspring->seq[pos]];
                t2 = inst->truck_times[offspring->seq[pos - 2]][node] + inst->truck_times[node][offspring->seq[pos]];
                if (t1 < t2)
                    t = t1;
                else
                    t = t2;
                if (pos < offspring->dimension - 2) {
                    t3 = inst->truck_times[offspring->seq[pos - 1]][node] + inst->truck_times[node][offspring->seq[pos + 1]];
                    if (t3 < t)
                        t = t3;
                }

                if (t < min_tt)
                {
                    min_tt = t;
                    min_pos = pos;
                }
            }
            int start_pos = min_pos - 20;
            if (start_pos < 1)
                start_pos = 1;
            int end_pos = start_pos + 40;
            if (end_pos > offspring->dimension - 2)
                end_pos = offspring->dimension - 2;
            start_pos = 1;
            end_pos = offspring->dimension - 2;

            offspring->dimension++;
            // node i must be added
            // compute the min time for each possible position for the new node
            double min_t = DBL_MAX;
            int *opt_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *opt_truck_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *opt_drone_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_truck_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_drone_seq = (int *)malloc(offspring->dimension * sizeof(int));

            for (int pos = start_pos; pos <= end_pos; pos++)
            {
                // try node i at position pos
                memcpy(new_seq, offspring->seq, pos * sizeof(int));
                memcpy(new_seq + pos + 1, offspring->seq + pos, (offspring->dimension - 1 - pos) * sizeof(int));
                new_seq[pos] = node;
                double t = compute_sequence_min_time_light(inst, new_seq, offspring->dimension, new_truck_seq, new_drone_seq, max_truck_leg_size_eval);
                if (t < min_t)
                {
                    min_t = t;
                    memcpy(opt_seq, new_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_truck_seq, new_truck_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_drone_seq, new_drone_seq, offspring->dimension * sizeof(int));
                }
            }
            // update offspring
            free(offspring->seq);
            free(offspring->truck_seq);
            free(offspring->drone_seq);
            offspring->seq = opt_seq;
            offspring->truck_seq = opt_truck_seq;
            offspring->drone_seq = opt_drone_seq;
            offspring->makespan = min_t;

            double delta = swap_customers(inst, offspring);
            offspring->makespan -= delta;
            // if (delta > 0)
            //     printf("* offspring improved with swap_customers() by %f\n", delta);
        }
    }
    offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, max_truck_leg_size_eval);
    get_successors(offspring->truck_succ, offspring->drone_succ, offspring->dimension, offspring->truck_seq, offspring->drone_seq, offspring->dimension - 1);

    // if (fabs(offspring->makespan - compute_sequence_min_time(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq)) > 1e-4)
    // {
    //     printf("!!! ACHTUNG !!! min_t != compute_sequence_min_time() \n");
    // }
    return offspring;
}


individual *crossover_greedy(instance * inst, individual * p1, individual * p2)
{
    if (p1->dimension != p2->dimension)
    {
        printf("p1 = %d, p2=%d\n", p1->dimension, p2->dimension);
        print_error("Error in crossover(): p1 and p2 have different dimensions.\n");
    }
    individual *offspring = (individual *)malloc(sizeof(individual));
    offspring->seq = (int *)malloc(inst->dimension * sizeof(int));
    offspring->truck_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->truck_succ = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_succ = (int *)malloc(p1->dimension * sizeof(int));
    // define the cutoff point
    // int cutoff = 5 + rand() % 90;
    int cutoff = 10 + rand() % 80;
    cutoff = (int)(cutoff * p1->dimension / 100.0);
    int flag[p1->dimension];
    for (int i = 0; i < p1->dimension; i++)
        flag[i] = 0;
    int r = rand() % 3;
    // r = 2;

    switch (r) {
    case 0:
    {
        // copy genetic material from p1
        for (int i = 0; i < cutoff; i++)
        {
            offspring->seq[i] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        // copy genetic material from p2
        int dim = cutoff; // current chromosome dimension
        int start_p2 = inst->nodes[p1->seq[cutoff - 1]].neighbors[rand() % inst->param.neighborhood_size];
        // find node [start_p2] in p2->seq
        int ind_start_p2;
        for (ind_start_p2 = 0 ; ind_start_p2 < inst->dimension; ind_start_p2++) {
            if (p2->seq[ind_start_p2] == start_p2)
                break;
        }
        for (int i = ind_start_p2; i < p2->dimension; i++)
        {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;
            if (dim == p2->dimension)
                break;
        }
        offspring->dimension = dim;
        break;
    }
    case 1:
    {
        for (int i = p1->dimension - cutoff; i < p1->dimension; i++)
        {
            offspring->seq[i] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }

        // copy genetic material from p1
        int dim = 0;
        for (int i = 0; i < p1->dimension - cutoff; i++)
        {
            if (flag[p1->seq[i]])
                continue;
            offspring->seq[dim++] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        memmove(offspring->seq + dim, offspring->seq + p1->dimension - cutoff, cutoff * sizeof(int));
        offspring->dimension = cutoff + dim;
        break;
    }
    case 2:
    {
        // copy genetic materal of parent p1 from A to B
        // int a = 15 + rand() % 30;
        // int b = 55 + rand () % 30;
        int a = rand() % p1->dimension;
        int b = rand() % p1->dimension;
        // int a = 10 + rand() % 80;
        // int b = 10 + rand() % 80;
        if (a == b)
        {
            a = 0.5 * (rand() % p1->dimension);
            b = a + 1 + 0.5 * (rand() % p1->dimension);
        }
        if (a > b) {
            int tmp = a;
            a = b;
            b = tmp;
        }
        if ( (b - a) < 10 * p1->dimension / 100.0)
        {
            a -= 10 * p1->dimension / 100.0;
            b += 10 * p1->dimension / 100.0;
        }
        if (a < 0)
            a = 0;
        if (b > p1->dimension - 1)
            b = p1->dimension - 1;

        for (int i = a; i < b; i++)
        {
            flag[p1->seq[i]] = 1;
        }

        int dim = 0;
        for (int i = 0; i < a; i++) {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }
        for (int i = a; i < b; i++) {
            offspring->seq[dim++] = p1->seq[i];
        }
        for (int i = b; i < p2->dimension; i++) {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;

        }
        offspring->dimension = dim;
        break;
    }
    }

    offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, max_truck_leg_size_eval);
    // local_search(inst, offspring);

    // add the remeaning nodes in the seq
    if (offspring->dimension < p1->dimension)
    {
        // offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, max_truck_leg_size_eval);
        // local_search(inst, offspring);
        int R[p1->dimension - offspring->dimension]; // array of the remaining elements after the combination phase
        int nR = 0;
        for (int i = 0; i < p1->dimension; i++)
        {
            if (flag[i] == 0)
                R[nR++] = i;
        }

        for (int i = 0; i < nR; i++)
        {
            // if (flag[i]) // node already present
            //     continue;

            // select randomly the node to add in the offspring
            int rand_idx = rand() % (p1->dimension - offspring->dimension);
            int node = R[rand_idx];
            R[rand_idx] = R[p1->dimension - offspring->dimension - 1];

            int g = 0;
            if (offspring->drone_seq[2] != p1->seq[p1->dimension - 1]) {
                double e = greedy_insertion(inst, p1->seq[p1->dimension - 1], offspring, node);
                if (offspring->makespan + e > 0.0f) {
                    offspring->makespan += e;
                    offspring->dimension++;
                    get_precedence_seq(offspring->truck_seq, offspring->drone_seq, offspring->dimension, offspring->seq);
                }
                else {
                    g = 1;
                }
            }
            else {
                g = 1;
            }

            if (g) {
                offspring->dimension++;
                double min_t = DBL_MAX;
                int *opt_seq = (int *)malloc(p1->dimension * sizeof(int));
                int *opt_truck_seq = (int *)malloc(p1->dimension * sizeof(int));
                int *opt_drone_seq = (int *)malloc(p1->dimension * sizeof(int));
                int *new_seq = (int *)malloc(p1->dimension * sizeof(int));
                int *new_truck_seq = (int *)malloc(p1->dimension * sizeof(int));
                int *new_drone_seq = (int *)malloc(p1->dimension * sizeof(int));

                // find the position with the shortest distances between the predecessor and the sucessor nodes
                // int min_pos = 1;
                // double min_d = inst->drone_dists[offspring->seq[0]][node] /*+ inst->truck_times[node][offspring->seq[1]]*/;
                // for (int pos = 2; pos < offspring->dimension - 1; pos++)
                // {
                //     double d = inst->drone_dists[offspring->seq[pos - 1]][node] /*+ inst->truck_times[node][offspring->seq[pos]]*/;
                //     if (d < min_d)
                //     {
                //         min_d = d;
                //         min_pos = pos;
                //     }
                // }
                // int start_pos = min_pos - 20;
                // int start_pos = 1 + rand() % (offspring->dimension - 3);
                // if (start_pos + 40 > offspring->dimension - 2)
                //     start_pos = offspring->dimension - 2 - 40;
                // if (start_pos < 1)
                //     start_pos = 1;

                // int end_pos = start_pos + 40;
                // if (end_pos > offspring->dimension - 2)
                //     end_pos = offspring->dimension - 2;

                int start_pos = 1;
                int end_pos = offspring->dimension - 2;

                for (int pos = start_pos; pos <= end_pos; pos++)
                {
                    // try node i at position pos
                    memcpy(new_seq, offspring->seq, pos * sizeof(int));
                    memcpy(new_seq + pos + 1, offspring->seq + pos, (offspring->dimension - 1 - pos) * sizeof(int));
                    new_seq[pos] = node;
                    double t = compute_sequence_min_time_light(inst, new_seq, offspring->dimension, new_truck_seq, new_drone_seq, max_truck_leg_size_eval);
                    if (t < min_t)
                    {
                        min_t = t;
                        memcpy(opt_seq, new_seq, offspring->dimension * sizeof(int));
                        memcpy(opt_truck_seq, new_truck_seq, offspring->dimension * sizeof(int));
                        memcpy(opt_drone_seq, new_drone_seq, offspring->dimension * sizeof(int));
                    }
                }
                // printf("*** i'm here! min_t: %f \n", min_t);

                // update offspring
                free(offspring->seq);
                free(offspring->truck_seq);
                free(offspring->drone_seq);
                offspring->seq = opt_seq;
                offspring->truck_seq = opt_truck_seq;
                offspring->drone_seq = opt_drone_seq;
                offspring->makespan = min_t;

                // double delta = swap_customers(inst, offspring);
                // offspring->makespan -= delta;
                //printf("*** i'm here2! \n");

            }
            // double delta = swap_customers(inst, offspring);
            // offspring->makespan -= delta;

            // printf("*** e: %f\n", e);
            // printf("*** AFTER INSERTION NODE %d: offspring->truck_seq: ", node);
            // for (int k = 0; k < offspring->dimension; k++)
            //     printf("%d, ", offspring->truck_seq[k]);
            // printf("\n");

            // printf("*** AFTER INSERTION NODE %d: offspring->drone_seq: ", node);
            // for (int k = 0; k < offspring->dimension; k++)
            //     printf("%d, ", offspring->drone_seq[k]);
            // printf("\n");

        }

        // printf("*** END FOR \n");
        //get_precedence_seq(offspring->truck_seq, offspring->drone_seq, offspring->dimension, offspring->seq);
    }
    // else {
    //     //     offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, max_truck_leg_size_eval);
    // }
    get_successors(offspring->truck_succ, offspring->drone_succ, offspring->dimension, offspring->truck_seq, offspring->drone_seq, p1->seq[p1->dimension - 1]);

    // if (fabs(offspring->makespan - compute_sequence_min_time(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq)) > 1e-4)
    // {
    //     printf("!!! ACHTUNG !!! min_t != compute_sequence_min_time() \n");
    // }

    return offspring;
}

individual *crossover_heavy(instance * inst, individual * p1, individual * p2)
{
    if (p1->dimension != p2->dimension)
    {
        printf("p1 = %d, p2=%d\n", p1->dimension, p2->dimension);
        print_error("Error in crossover(): p1 and p2 have different dimensions.\n");
    }
    individual *offspring = (individual *)malloc(sizeof(individual));
    offspring->seq = (int *)malloc(inst->dimension * sizeof(int));
    offspring->truck_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->truck_succ = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_succ = (int *)malloc(p1->dimension * sizeof(int));
    // define the cutoff point
    // int cutoff = 5 + rand() % 90;
    int cutoff = 10 + rand() % 80;
    cutoff = (int)(cutoff * p1->dimension / 100.0);
    int flag[p1->dimension];
    for (int i = 0; i < p1->dimension; i++)
        flag[i] = 0;
    int r = rand() % 3;
    // r = 2;

    switch (r) {
    case 0:
    {
        // copy genetic material from p1
        for (int i = 0; i < cutoff; i++)
        {
            offspring->seq[i] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        // copy genetic material from p2
        int dim = cutoff; // current chromosome dimension
        int start_p2 = inst->nodes[p1->seq[cutoff - 1]].neighbors[rand() % inst->param.neighborhood_size];
        // find node [start_p2] in p2->seq
        int ind_start_p2;
        for (ind_start_p2 = 0 ; ind_start_p2 < inst->dimension; ind_start_p2++) {
            if (p2->seq[ind_start_p2] == start_p2)
                break;
        }
        for (int i = ind_start_p2; i < p2->dimension; i++)
        {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;
            if (dim == p2->dimension)
                break;
        }
        offspring->dimension = dim;
        break;
    }
    case 1:
    {
        for (int i = p1->dimension - cutoff; i < p1->dimension; i++)
        {
            offspring->seq[i] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }

        // copy genetic material from p1
        int dim = 0;
        for (int i = 0; i < p1->dimension - cutoff; i++)
        {
            if (flag[p1->seq[i]])
                continue;
            offspring->seq[dim++] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        memmove(offspring->seq + dim, offspring->seq + p1->dimension - cutoff, cutoff * sizeof(int));
        offspring->dimension = cutoff + dim;
        break;
    }
    case 2:
    {
        // copy genetic materal of parent p1 from A to B
        // int a = 1 + rand() % 100;
        // int b = 1 + rand() % 100;
        // if (a == b)
        // {
        // if (rand() % 2)
        // a -= 5;
        // b += 5;
        // }
        int a = rand() % p1->dimension;
        int b = rand() % p1->dimension;
        // int a = 10 + rand() % 80;
        // int b = 10 + rand() % 80;
        if (a == b)
        {
            a = 0.5 * (rand() % p1->dimension);
            b = a + 1 + 0.5 * (rand() % p1->dimension);
        }
        if (a > b) {
            int tmp = a;
            a = b;
            b = tmp;
        }
        if ( (b - a) < 10 * p1->dimension / 100.0)
        {
            a -= 10 * p1->dimension / 100.0;
            b += 10 * p1->dimension / 100.0;
        }
        if (a < 0)
            a = 0;
        if (b > p1->dimension - 1)
            b = p1->dimension - 1;

        for (int i = a; i < b; i++)
        {
            flag[p1->seq[i]] = 1;
        }

        int dim = 0;
        for (int i = 0; i < a; i++) {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }
        for (int i = a; i < b; i++) {
            offspring->seq[dim++] = p1->seq[i];
        }
        for (int i = b; i < p2->dimension; i++) {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;

        }
        offspring->dimension = dim;
        break;
    }
    }

    // offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, 5);
    // // double delta = swap_customers(inst, offspring);
    // // offspring->makespan -= delta;
    // // local_search(inst, offspring);

    // add the remeaning nodes in the seq
    if (offspring->dimension < p1->dimension)
    {
        int R[p1->dimension - offspring->dimension]; // array of the remaining elements after the combination phase
        int nR = 0;
        for (int i = 0; i < p1->dimension; i++)
        {
            if (flag[i] == 0)
                R[nR++] = i;
        }

        for (int i = 0; i < nR; i++)
        {
            // if (flag[i]) // node already present
            //     continue;

            // select randomly the node to add in the offspring
            int rand_idx = rand() % (p1->dimension - offspring->dimension);
            int node = R[rand_idx];
            R[rand_idx] = R[p1->dimension - offspring->dimension - 1];

            offspring->dimension++;
            double min_t = DBL_MAX;
            int *opt_seq = (int *)malloc(p1->dimension * sizeof(int));
            int *opt_truck_seq = (int *)malloc(p1->dimension * sizeof(int));
            int *opt_drone_seq = (int *)malloc(p1->dimension * sizeof(int));
            int *new_seq = (int *)malloc(p1->dimension * sizeof(int));
            int *new_truck_seq = (int *)malloc(p1->dimension * sizeof(int));
            int *new_drone_seq = (int *)malloc(p1->dimension * sizeof(int));

            // find the position with the shortest distances between the predecessor and the sucessor nodes
            // int min_pos = 1;
            // double min_d = inst->drone_dists[offspring->seq[0]][node] /*+ inst->truck_times[node][offspring->seq[1]]*/;
            // for (int pos = 2; pos < offspring->dimension - 1; pos++)
            // {
            //     double d = inst->drone_dists[offspring->seq[pos - 1]][node] /*+ inst->truck_times[node][offspring->seq[pos]]*/;
            //     if (d < min_d)
            //     {
            //         min_d = d;
            //         min_pos = pos;
            //     }
            // }
            // int start_pos = min_pos - 20;
            int start_pos = 1 + rand() % (offspring->dimension - 3);
            if (start_pos + 40 > offspring->dimension - 2)
                start_pos = offspring->dimension - 2 - 40;
            if (start_pos < 1)
                start_pos = 1;

            int end_pos = start_pos + 40;
            if (end_pos > offspring->dimension - 2)
                end_pos = offspring->dimension - 2;

            start_pos = 1;
            end_pos = offspring->dimension - 2;

            for (int pos = start_pos; pos <= end_pos; pos++)
            {
                // try node i at position pos
                memcpy(new_seq, offspring->seq, pos * sizeof(int));
                memcpy(new_seq + pos + 1, offspring->seq + pos, (offspring->dimension - 1 - pos) * sizeof(int));
                new_seq[pos] = node;
                double t = compute_sequence_min_time_light(inst, new_seq, offspring->dimension, new_truck_seq, new_drone_seq, max_truck_leg_size_eval);
                if (t < min_t)
                {
                    min_t = t;
                    memcpy(opt_seq, new_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_truck_seq, new_truck_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_drone_seq, new_drone_seq, offspring->dimension * sizeof(int));
                }
            }
            // printf("*** i'm here! min_t: %f \n", min_t);

            // update offspring
            free(offspring->seq);
            free(offspring->truck_seq);
            free(offspring->drone_seq);
            offspring->seq = opt_seq;
            offspring->truck_seq = opt_truck_seq;
            offspring->drone_seq = opt_drone_seq;
            offspring->makespan = min_t;

            // double delta = swap_customers(inst, offspring);
            // offspring->makespan -= delta;
            //printf("*** i'm here2! \n");


            // double delta = swap_customers(inst, offspring);
            // offspring->makespan -= delta;

            // printf("*** e: %f\n", e);
            // printf("*** AFTER INSERTION NODE %d: offspring->truck_seq: ", node);
            // for (int k = 0; k < offspring->dimension; k++)
            //     printf("%d, ", offspring->truck_seq[k]);
            // printf("\n");

            // printf("*** AFTER INSERTION NODE %d: offspring->drone_seq: ", node);
            // for (int k = 0; k < offspring->dimension; k++)
            //     printf("%d, ", offspring->drone_seq[k]);
            // printf("\n");

        }

        // printf("*** END FOR \n");
        //get_precedence_seq(offspring->truck_seq, offspring->drone_seq, offspring->dimension, offspring->seq);
    }
    else {
        offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, max_truck_leg_size_eval);
    }
    get_successors(offspring->truck_succ, offspring->drone_succ, offspring->dimension, offspring->truck_seq, offspring->drone_seq, offspring->dimension - 1);

    // if (fabs(offspring->makespan - compute_sequence_min_time(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq)) > 1e-4)
    // {
    //     printf("!!! ACHTUNG !!! min_t != compute_sequence_min_time() \n");
    // }

    return offspring;
}

/*
individual *crossover_light(instance *inst, individual *p1, individual *p2)
{
    if (p1->dimension != p2->dimension)
    {
        printf("p1 = %d, p2=%d\n", p1->dimension, p2->dimension);
        print_error("Error in crossover(): p1 and p2 have different dimensions.\n");
    }
    individual *offspring = (individual *)malloc(sizeof(individual));
    offspring->seq = (int *)malloc(inst->dimension * sizeof(int));
    offspring->truck_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->truck_succ = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_succ = (int *)malloc(p1->dimension * sizeof(int));
    // define the cutoff point
    int cutoff = 15 + rand() % 70;
    cutoff = (int)(cutoff * p1->dimension / 100.0);
    int flag[p1->dimension];
    for (int i = 0; i < p1->dimension; i++)
        flag[i] = 0;

    if (rand() % 2)
    {
        // copy genetic material from p1
        for (int i = 0; i < cutoff; i++)
        {
            offspring->seq[i] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        // copy genetic material from p2
        int dim = cutoff; // current chromosome dimension
        for (int i = cutoff; i < p1->dimension; i++)
        {
            if (flag[p2->seq[i]])
                continue;
            offspring->seq[dim++] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }
        offspring->dimension = dim;
    }
    else
    {
        // copy genetic material from p2
        for (int i = p1->dimension - cutoff; i < p1->dimension; i++)
        {
            offspring->seq[i] = p2->seq[i];
            flag[p2->seq[i]] = 1;
        }

        // copy genetic material from p1
        int dim = 0;
        for (int i = 0; i < p1->dimension - cutoff; i++)
        {
            if (flag[p1->seq[i]])
                continue;
            offspring->seq[dim++] = p1->seq[i];
            flag[p1->seq[i]] = 1;
        }
        memmove(offspring->seq + dim, offspring->seq + p1->dimension - cutoff, cutoff * sizeof(int));
        offspring->dimension = cutoff + dim;
    }

    // for (int i = 0; i < p1->dimension; i++)
    // {
    //     printf("(%d)%d, ", i, flag[i]);
    // }
    // printf("\n");

    // add the remeaning nodes in the seq
    if (offspring->dimension < p1->dimension)
    {
        int R[p1->dimension - offspring->dimension]; // array of the remaining elements after the combination phase
        int nR = 0;
        for (int i = 0; i < p1->dimension; i++)
        {
            if (flag[i] == 0)
                R[nR++] = i;
        }
        // for (int i = 0; i < p1->dimension - offspring->dimension; i++)
        // {
        //     printf("%d, ", R[i]);
        // }
        // printf("\t nR = %d \n", nR);

        for (int i = 0; i < nR; i++)
        {
            // if (flag[i]) // node already present
            //     continue;

            // select randomly the node to add in the offspring
            int rand_idx = rand() % (p1->dimension - offspring->dimension);
            int node = R[rand_idx];
            R[rand_idx] = R[p1->dimension - offspring->dimension - 1];

            offspring->dimension++;
            // node i must be added
            // compute the min time for each possible position for the new node
            double min_t = DBL_MAX;
            int *opt_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *opt_truck_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *opt_drone_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_truck_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_drone_seq = (int *)malloc(offspring->dimension * sizeof(int));

            // compute the nDists positions with the shortest drone travel distances
            int nDists = 10;
            if (offspring->dimension - 2 < nDists)
                nDists = offspring->dimension - 2;
            if (nDists < 0)
                nDists = 1;
            double dists[nDists];
            int positions[nDists];
            for (int j = 0; j < nDists; j++)
            {
                dists[j] = DBL_MAX;
                positions[j] = -1;
            }
            for (int pos = 1; pos < offspring->dimension - 1; pos++)
            {
                // compute the distance: seq[pos-1]->node + node->seq[pos]
                double d = inst->drone_dists[offspring->seq[pos - 1]][node] + inst->drone_dists[node][offspring->seq[pos]];
                if (d < dists[nDists - 1])
                {
                    double maxD = d;
                    int max_k = nDists - 1;
                    for (int k = 0; k < nDists - 1; k++)
                    {
                        if (dists[k] > maxD)
                        {
                            maxD = dists[k];
                            max_k = k;
                        }
                    }
                    if (max_k != nDists - 1)
                    {
                        dists[nDists - 1] = dists[max_k];
                        positions[nDists - 1] = positions[max_k];
                    }
                    dists[max_k] = d;
                    positions[max_k] = pos;
                }
                // for (int k = 0; k < nDists; k++)
                // {
                //     printf("*pos:%d -> d:%f\n", positions[k], dists[k]);
                // }
                // printf("------------------\n");
            }
            // for (int k = 0; k < nDists; k++)
            // {
            //     printf("pos:%d -> d:%f\n", positions[k], dists[k]);
            // }

            //int pos = positions[rand() % nDists];
            //printf("pos: %d\n", pos);
            // memcpy(new_seq, offspring->seq, pos * sizeof(int));
            // memcpy(new_seq + pos + 1, offspring->seq + pos, (offspring->dimension - 1 - pos) * sizeof(int));
            // new_seq[pos] = node;

            // compute_sequence_min_time_light(inst, new_seq, offspring->dimension, new_truck_seq, new_drone_seq);
            // memcpy(opt_seq, new_seq, offspring->dimension * sizeof(int));
            // memcpy(opt_truck_seq, new_truck_seq, offspring->dimension * sizeof(int));
            // memcpy(opt_drone_seq, new_drone_seq, offspring->dimension * sizeof(int));

            for (int pos_i = 0; pos_i < nDists; pos_i++)
            {
                // try node i at position pos
                int pos = positions[pos_i];
                memcpy(new_seq, offspring->seq, pos * sizeof(int));
                memcpy(new_seq + pos + 1, offspring->seq + pos, (offspring->dimension - 1 - pos) * sizeof(int));
                new_seq[pos] = node;
                double t = compute_sequence_min_time_light(inst, new_seq, offspring->dimension, new_truck_seq, new_drone_seq);
                if (t < min_t)
                {
                    min_t = t;
                    memcpy(opt_seq, new_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_truck_seq, new_truck_seq, offspring->dimension * sizeof(int));
                    memcpy(opt_drone_seq, new_drone_seq, offspring->dimension * sizeof(int));
                }
            }
            // update offspring
            free(offspring->seq);
            free(offspring->truck_seq);
            free(offspring->drone_seq);
            offspring->seq = opt_seq;
            offspring->truck_seq = opt_truck_seq;
            offspring->drone_seq = opt_drone_seq;
            offspring->makespan = min_t;

            double delta = swap_customers(inst, offspring);
            offspring->makespan -= delta;
            // if (delta > 0)
            //     printf("* offspring improved with swap_customers() by %f\n", delta);
        }
    }

    offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq);
    get_successors(offspring->truck_succ, offspring->drone_succ, offspring->dimension, offspring->truck_seq, offspring->drone_seq, offspring->dimension - 1);

    // if (fabs(offspring->makespan - compute_sequence_min_time(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq)) > 1e-4)
    // {
    //     printf("!!! ACHTUNG !!! min_t != compute_sequence_min_time() \n");
    // }
    return offspring;
}
*/


individual *crossover2(instance * inst, individual * p1, individual * p2)
{
    if (p1->dimension != p2->dimension)
    {
        printf("p1 = %d, p2=%d\n", p1->dimension, p2->dimension);
        print_error("Error in crossover(): p1 and p2 have different dimensions.\n");
    }
    individual *offspring = (individual *)malloc(sizeof(individual));
    offspring->seq = (int *)malloc(inst->dimension * sizeof(int));
    offspring->truck_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_seq = (int *)malloc(p1->dimension * sizeof(int));
    offspring->truck_succ = (int *)malloc(p1->dimension * sizeof(int));
    offspring->drone_succ = (int *)malloc(p1->dimension * sizeof(int));
    // define the cutoff point
    int cutoff1 = 15 + rand() % 50;
    int cutoff2 = 1 + cutoff1 + rand() % 60;
    if (cutoff2 > 80)
        cutoff2 = 80;
    cutoff1 = (int)(cutoff1 * p1->dimension / 100.0);
    cutoff2 = (int)(cutoff2 * p2->dimension / 100.0);
    int flag[p1->dimension];
    for (int i = 0; i < p1->dimension; i++)
        flag[i] = 0;

    int *s2 = (int *)malloc(p1->dimension * sizeof(int)); // tmp sequence for the middle sequence
    int *s3 = (int *)malloc(p1->dimension * sizeof(int)); // tmp sequence for the final sequence

    // copy genetic material from p1
    // inital sequence
    int l2 = 0;
    for (int i = cutoff1; i <= cutoff2; i++)
    {
        s2[l2++] = p1->seq[i];
        flag[p1->seq[i]] = 1;
    }

    // copy genetic material from p2
    // middle sequence
    int l1 = 0;
    for (int i = 0; i < cutoff1; i++)
    {
        if (flag[p2->seq[i]])
            continue;
        offspring->seq[l1++] = p2->seq[i];
        flag[p2->seq[i]] = 1;
    }

    // copy genetic material from p2
    // final sequence
    int l3 = 0;
    for (int i = cutoff2 + 1; i < p1->dimension; i++)
    {
        if (flag[p2->seq[i]])
            continue;
        s3[l3++] = p2->seq[i];
        flag[p2->seq[i]] = 1;
    }

    // check if it seems convenient to reverse the middle sequence (s2) or not
    // if (inst->truck_times[offspring->seq[l1 - 1]][s2[0]] + inst->truck_times[s2[l2 - 1]][s3[0]] > inst->truck_times[offspring->seq[l1 - 1]][s2[l2 - 1]] + inst->truck_times[s2[0]][s3[0]])
    // {
    //     // add s2 reversed
    //     for (int i = 0; i < l2; i++)
    //         offspring->seq[l1 + i] = s2[l2 - 1 - i];
    // }
    // else
    {
        // add s2
        for (int i = 0; i < l2; i++)
            offspring->seq[l1 + i] = s2[i];
    }
    // add s3
    for (int i = 0; i < l3; i++)
        offspring->seq[l1 + l2 + i] = s3[i];
    offspring->dimension = l1 + l2 + l3;

    // add the remeaning nodes in the seq
    if (offspring->dimension < p1->dimension)
    {
        for (int i = 1; i < p1->dimension - 1; i++)
        {
            if (flag[i]) // node already present
                continue;

            offspring->dimension++;
            // node i must be added
            // compute the min time for each possible position for the new node
            double min_t = DBL_MAX;
            int *opt_seq = (int *)malloc(offspring->dimension * sizeof(int));
            int *new_seq = (int *)malloc(offspring->dimension * sizeof(int));
            for (int pos = 1; pos < offspring->dimension - 1; pos++)
            {
                // try node i at position pos
                memcpy(new_seq, offspring->seq, pos * sizeof(int));
                memcpy(new_seq + pos + 1, offspring->seq + pos, (offspring->dimension - 1 - pos) * sizeof(int));
                new_seq[pos] = i;
                double t = compute_sequence_min_time_light(inst, new_seq, offspring->dimension, NULL, NULL, max_truck_leg_size_eval);
                if (t < min_t)
                {
                    min_t = t;
                    memcpy(opt_seq, new_seq, offspring->dimension * sizeof(int));
                    // memcpy(opt_truck_seq, new_truck_seq, offspring->dimension * sizeof(int));
                    // memcpy(opt_drone_seq, new_drone_seq, offspring->dimension * sizeof(int));
                }
            }
            // update offspring
            free(offspring->seq);
            // free(offspring->truck_seq);
            // free(offspring->drone_seq);
            offspring->seq = opt_seq;
            // offspring->truck_seq = opt_truck_seq;
            // offspring->drone_seq = opt_drone_seq;
            offspring->makespan = min_t;

            // double delta = swap_customers(inst, offspring);
            // offspring->makespan -= delta;
            // if (delta > 0)
            //     printf("* offspring improved with swap_customers() by %f\n", delta);
        }
    }

    offspring->makespan = compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, max_truck_leg_size_eval);

    get_successors(offspring->truck_succ, offspring->drone_succ, offspring->dimension, offspring->truck_seq, offspring->drone_seq, offspring->dimension - 1);

    if (fabs(offspring->makespan - compute_sequence_min_time_light(inst, offspring->seq, offspring->dimension, offspring->truck_seq, offspring->drone_seq, max_truck_leg_size_eval)) > 1e-4)
    {
        printf("!!! ACHTUNG !!! min_t != compute_sequence_min_time() \n");
    }
    free(s2);
    free(s3);
    return offspring;
}

individual *binary_tournament(individual **population, int population_size)
{
    int i = rand() % (population_size);
    int j = rand() % (population_size);
    if (i == j)
        j++;
    if (j == population_size)
        j -= 2;

    if (population[i]->makespan < population[j]->makespan)
        return population[i];
    else
        return population[j];
}

double compute_avg_dist(individual **population, int population_size, int index_ind, int n_neighbors)
{
    if (population_size < 1)
        print_error("Error in compute_avg_dist(): population_size < 1\n");
    if (n_neighbors > population_size - 1 || n_neighbors < 1)
        n_neighbors = 1;
    int closest_dist[n_neighbors];
    for (int i = 0; i < n_neighbors; i++)
        closest_dist[i] = INT_MAX;
    for (int i = 0; i < population_size; i++)
    {
        if (i == index_ind)
            continue;
        int d = hamming_dist(population[index_ind], population[i]);
        int count = 0; // count how many elements shift
        for (int j = 0; j < n_neighbors; j++)
        {
            if (d >= closest_dist[n_neighbors - 1 - j])
                break;
            count++;
        }
        if (count == 0)
            continue;
        // shift
        memmove(closest_dist + (n_neighbors - count) + 1, closest_dist + (n_neighbors - count), (count - 1) * sizeof(int));
        closest_dist[n_neighbors - count] = d;
    }

    double avg_dist = 0.0;
    for (int i = 0; i < n_neighbors; i++)
    {
        avg_dist += closest_dist[i];
    }
    avg_dist /= n_neighbors;
    return avg_dist;
}

int cmp_rank_fitness(const void *a, const void *b)
{
    if ((*(individual **)a)->makespan > (*(individual **)b)->makespan)
        return 1;
    if ((*(individual **)a)->makespan < (*(individual **)b)->makespan)
        return -1;
    return 0;
}

void compute_fitness_ranking(individual **population, int population_size)
{
    qsort(population, population_size, sizeof(individual *), cmp_rank_fitness);
}

int cmp_rank_diversity(const void *a, const void *b)
{
    if ((*(individual **)a)->avg_dist < (*(individual **)b)->avg_dist)
        return 1;
    if ((*(individual **)a)->avg_dist > (*(individual **)b)->avg_dist)
        return -1;
    return 0;
}

void compute_diversity_ranking(individual **population, int population_size)
{
    qsort(population, population_size, sizeof(individual *), cmp_rank_diversity);
}

int cmp_rank_biased_fitness(const void *a, const void *b)
{
    if ((*(individual **)a)->biased_fitness > (*(individual **)b)->biased_fitness)
        return 1;
    if ((*(individual **)a)->biased_fitness < (*(individual **)b)->biased_fitness)
        return -1;
    return 0;
}

void compute_biased_fitness_ranking(individual **population, int population_size)
{
    qsort(population, population_size, sizeof(individual *), cmp_rank_biased_fitness);
}


void local_search(instance * inst, individual * ind)
{
    if (rand() % 2)
    {
        while (1)
        {
            double old_makespan = ind->makespan;
            double delta = optimize_truck_legs(inst, ind->truck_seq, ind->drone_seq);
            delta += reverse_drone_leg(inst, ind->truck_seq, ind->drone_seq);
            delta += swap_customers(inst, ind);
            // update the individual makespan
            ind->makespan -= delta;
            // update ind sequence
            get_precedence_seq(ind->truck_seq, ind->drone_seq, ind->dimension, ind->seq);
            // if there is not an improvement exit
            if (delta < 1e-9)
                break;
            // compute the optimal time associated to the new sequence

            // if there is not an improvement exit
            ind->makespan = compute_sequence_min_time_light(inst, ind->seq, ind->dimension, ind->truck_seq, ind->drone_seq, max_truck_leg_size_eval);
            if (old_makespan - ind->makespan < 1e-9)
                break;
        }
    }
    else
    {
        while (1)
        {
            double old_makespan = ind->makespan;
            double delta = optimize_truck_legs(inst, ind->truck_seq, ind->drone_seq);
            delta += swap_customers(inst, ind);
            delta += reverse_drone_leg(inst, ind->truck_seq, ind->drone_seq);
            // update the individual makespan
            ind->makespan -= delta;
            // update ind sequence
            get_precedence_seq(ind->truck_seq, ind->drone_seq, ind->dimension, ind->seq);
            // if there is not an improvement exit
            if (delta < 1e-9)
                break;
            // compute the optimal time associated to the new sequence
            ind->makespan = compute_sequence_min_time_light(inst, ind->seq, ind->dimension, ind->truck_seq, ind->drone_seq, max_truck_leg_size_eval);
            // if there is not an improvement exit
            if (old_makespan - ind->makespan < 1e-9)
                break;
        }
    }

    // update individual sequence
    // get_precedence_seq(ind->truck_seq, ind->drone_seq, ind->dimension, ind->seq);
    // update successors arrays
    get_successors(ind->truck_succ, ind->drone_succ, ind->dimension, ind->truck_seq, ind->drone_seq, inst->dimension - 1);
}

// seq: rules of precedence
// if two nodes i,j are visited by the same vehicle
// then i must appear before j in the sequence seq
// double compute_sequence_min_time(int *seq, instance *inst)
// {
//     // array of minimum arrival times T
//     // T[i] = minimum arrival time at node i is visited by the truck
//     // (since the last node must be visited by the truck, this assumption is ok)
//     double T[inst->dimension];

//     // initialize the arrival times T[i] equal to the arrival times related to the case in which the customers are served only by the truck
//     T[0] = 0.0;
//     for (int i = 0; i < inst->dimension - 1; i++)
//     {
//         T[i + 1] = T[i] + inst->truck_times[seq[i]][seq[i + 1]];
//     }

//     T[0] = 0.0;
//     for (int i = 0; i < inst->dimension - 1; i++)
//     {
//         // move 1: move the truck with the drone onboard by one position
//         if (T[i] + inst->truck_times[seq[i]][seq[i + 1]] < T[i + 1])
//         {
//             T[i + 1] = T[i] + inst->truck_times[seq[i]][seq[i + 1]];
//         }

//         // move 2: launch the drone from the current node i, to visit node j, and rejoin the truck with the drone at node k
//         //printf("\ni = %d (%d)\n", i, seq[i]);
//         double truck_t = 0.0;
//         for (int k = i + 2; k < inst->dimension; k++)
//         {
//             if (k == i + 2)
//                 truck_t += inst->truck_times[seq[i]][seq[k]];
//             else
//                 truck_t += inst->truck_times[seq[k - 1]][seq[k]];

//             if (truck_t > inst->max_i_drone_leg_times[seq[i]])
//                 break; // does not exist a drone leg that start from node i that can last up to the time truck_t
//             double truck_tj = truck_t;
//             //printf("\n\tk = %d(%d): ", k, seq[k]);
//             for (int j = i + 1; j < k; j++)
//             {
//                 if (inst->nodes[seq[j]].weight > UAV_WEIGHT_LIMIT) // node j cannot be a drone customer
//                     continue;
//                 // adapt the travel time of the truck path, since now the customer served by the drone is j
//                 if (j > i + 1)
//                 {
//                     truck_tj = truck_tj - inst->truck_times[seq[j - 2]][seq[j]] - inst->truck_times[seq[j]][seq[j + 1]];
//                     truck_tj = truck_tj + inst->truck_times[seq[j - 2]][seq[j - 1]] + inst->truck_times[seq[j - 1]][seq[j + 1]];
//                 }
//                 // check if the drone leg i-->j-->k is feasible
//                 if (truck_tj > inst->max_feas_time_drone[seq[i]][seq[j]][seq[k]]) // the drone leg i-->j-->k is infeasible
//                     continue;
//                 // // drone leg is feasible
//                 //printf("%d(%d) ", j, seq[j]);
//                 double tmp = inst->min_feas_time_drone[seq[i]][seq[j]][seq[k]];
//                 if (tmp < truck_tj)
//                     tmp = truck_tj;
//                 if (T[i] + tmp < T[k])
//                     T[k] = T[i] + tmp;
//             }
//         }
//     }
//     // for (int i = 0; i < inst->dimension; i++)
//     //     printf("T[%d] = %f\n", seq[i], T[i]);
//     return T[inst->dimension - 1];
// }

double compute_score(instance * inst, int *truck_seq, int *drone_seq, int end_value, int flag_print)
{
    if (truck_seq[0] != drone_seq[0])
        print_error("Error in compute_score(): truck_seq[0] != drone_seq[0]\n");

    double obj = 0.0;
    int t_i = 0;
    int d_i = 0;
    while (truck_seq[t_i] != end_value)
    {
        if (truck_seq[t_i + 1] != drone_seq[d_i + 1]) // drone leg found
        {
            int d_j = d_i + 1;
            int d_k = d_i + 2;

            if (flag_print) {
                printf("DRONE LEG: %d --> %d --> %d", drone_seq[d_i], drone_seq[d_j], drone_seq[d_k]);
                printf("\t min_t: %.2f", inst->min_feas_time_drone[drone_seq[d_i]][drone_seq[d_j]][drone_seq[d_k]]);
                printf("\t max_t: %.2f\n", inst->max_feas_time_drone[drone_seq[d_i]][drone_seq[d_j]][drone_seq[d_k]]);
            }

            // compute the truck travel time to traverse the path i -> k
            int tmp = t_i;
            double truck_t_ik = 0.0;
            if (flag_print)
                printf("TRUCK LEG: %d ", truck_seq[t_i]);
            while (truck_seq[tmp] != drone_seq[d_k])
            {
                if (flag_print)
                    printf("-> %d ", truck_seq[tmp + 1]);
                truck_t_ik += inst->truck_times[truck_seq[tmp]][truck_seq[tmp + 1]];
                tmp++;
            }
            if (flag_print)
                printf("\t \t t: %.2f\n\n", truck_t_ik);
            int t_k = tmp; // truck index associated to the rendezvous node k

            // compute the minimum feasible travel times related to the drone legs i --> j --> k and k --> j --> i
            if (truck_t_ik > inst->max_feas_time_drone[drone_seq[d_i]][drone_seq[d_j]][drone_seq[d_k]])
            {
                return -1;
            }
            double t_ijk = inst->min_feas_time_drone[drone_seq[d_i]][drone_seq[d_j]][drone_seq[d_k]];
            if (t_ijk < truck_t_ik)
                t_ijk = truck_t_ik;
            obj += t_ijk;
            if (inst->param.poikonen == 0 && inst->param.ha == 0)
                obj += UAV_LAUNCH_TIME + UAV_RETRIEVAL_TIME;
            if (inst->param.poikonen == 1 && inst->param.MHD == 1)
                obj += POIKONEN_UAV_LAUNCH_TIME + POIKONEN_UAV_RETRIEVAL_TIME;
            if (inst->param.ha == 1) {
                if (truck_seq[t_i] == 0)
                    obj += UAV_LAUNCH_TIME;
                else
                    obj += 2 * UAV_LAUNCH_TIME;
            }


            t_i = t_k;
            d_i = d_k;
        }
        else
        {
            if (flag_print)
                printf("COMBINED EDGE: %d -> %d \t \t t: %.2f\n\n", truck_seq[t_i], truck_seq[t_i + 1],  inst->truck_times[truck_seq[t_i]][truck_seq[t_i + 1]]);
            obj += inst->truck_times[truck_seq[t_i]][truck_seq[t_i + 1]];
            t_i++;
            d_i++;
        }
    }
    return obj;
}

double compute_sequence_min_time(instance * inst, int *seq, int seq_dim, int *truck_seq, int *drone_seq)
{
    if (seq_dim > inst->dimension)
        print_error("Error in compute_sequence_min_time(): seq_dim > inst->dimension\n");
    // array of minimum arrival times T
    // T[i] = minimum arrival time at node i, when i is visited by the truck
    // (since the last node must be visited by the truck, this assumption is ok)
    double T[seq_dim];
    double truck_only[seq_dim];

    // move1[k] = 1 => optimal move that ends in k is of type 1 (truck moves forward with the drone onboard)
    // move1[k] = 0 => optimal move that ends in k is of type 2 (drone leg which ends in k)
    int move1[seq_dim];
    // move2_drone_cust[k] = j if j is the drone customer visited by the drone leg associated to the landing node k
    // move2_takeoff_node[k] = i if i is the takeoff node from which it starts the drone leg associated to the landing node k
    int move2_drone_cust[seq_dim];
    int move2_takeoff_node[seq_dim];

    double extra_t = 0.0;
    if (inst->param.poikonen == 0 && inst->param.ha == 0)
        extra_t = UAV_LAUNCH_TIME + UAV_RETRIEVAL_TIME;
    if (inst->param.ha == 1)
        extra_t = 2 * UAV_LAUNCH_TIME;
    if (inst->param.poikonen == 1 && inst->param.MHD == 1)
        extra_t = POIKONEN_UAV_LAUNCH_TIME + POIKONEN_UAV_RETRIEVAL_TIME;

    // initialize arrival times T[i] equal to the arrival times related to the truck only case
    T[0] = 0.0;
    truck_only[0] = T[0];
    for (int i = 0; i < seq_dim - 1; i++)
    {
        T[i + 1] = T[i] + inst->truck_times[seq[i]][seq[i + 1]];
        truck_only[i + 1] = T[i + 1];
        move1[i + 1] = 1;
    }

    for (int i = 0; i < seq_dim - 1; i++)
    {
        // move 1: move the truck with the drone onboard by one position
        if (T[i] + inst->truck_times[seq[i]][seq[i + 1]] < T[i + 1])
        {
            T[i + 1] = T[i] + inst->truck_times[seq[i]][seq[i + 1]];
            move1[i + 1] = 1;
        }

        // move 2: launch the drone from the current node i, to visit node j, and rejoin the truck with the drone at node k
        //** printf("\ni = %d (%d)\n", i, seq[i]);

        for (int j = i + 1; j < seq_dim - 1; j++)
        {
            if (inst->param.poikonen == 0 && inst->param.ha == 0 && inst->nodes[seq[j]].weight > UAV_WEIGHT_LIMIT) // node j cannot be a drone customer
                continue;
            if (inst->nodes[seq[j]].truck_only == 1)
                continue;
            //** printf("\n\tj = %d(%d) : ", j, seq[j]);
            //double truck_t = 0.0;
            double truck_t = truck_only[j + 1] - truck_only[i] - inst->truck_times[seq[j - 1]][seq[j]] - inst->truck_times[seq[j]][seq[j + 1]] + inst->truck_times[seq[j - 1]][seq[j + 1]];
            for (int k = j + 1; k < seq_dim; k++)
            {
                // update truck travel time
                // if (k == j + 1)
                // {
                //     for (int w = i + 1; w < j; w++)
                //     {
                //         truck_t += inst->truck_times[seq[w - 1]][seq[w]];
                //     }
                //     truck_t += inst->truck_times[seq[j - 1]][seq[k]];
                // }
                // else
                //     truck_t += inst->truck_times[seq[k - 1]][seq[k]];
                if (k > j + 1)
                    truck_t += inst->truck_times[seq[k - 1]][seq[k]];

                // if the truck_t > max_k {drone travel time i-->j-->k}   STOP
                if (inst->param.poikonen == 0 && inst->param.ha == 0 && truck_t > inst->max_ij_drone_leg_times[seq[i]][seq[j]])
                {
                    // does not exist a landing node k associated to a feasible drone leg i-->j-->k
                    break;
                }
                // check if the drone leg i-->j-->k is feasible
                if (truck_t > inst->max_feas_time_drone[seq[i]][seq[j]][seq[k]]) // the drone leg i-->j-->k is infeasible
                {
                    continue;
                }

                double tmp = inst->min_feas_time_drone[seq[i]][seq[j]][seq[k]];

                if (tmp < truck_t)
                    tmp = truck_t;

                if (inst->param.ha == 0) {
                    if (T[i] + tmp + extra_t < T[k])
                    {
                        T[k] = T[i] + tmp + extra_t;
                        move1[k] = 0;
                        move2_drone_cust[k] = j;
                        move2_takeoff_node[k] = i;
                    }
                }
                else {
                    if (seq[i] == 0) {
                        if (T[i] + tmp + extra_t / 2 < T[k])
                        {
                            T[k] = T[i] + tmp + extra_t / 2;
                            move1[k] = 0;
                            move2_drone_cust[k] = j;
                            move2_takeoff_node[k] = i;
                        }
                    }
                    else {
                        if (T[i] + tmp + extra_t < T[k])
                        {
                            T[k] = T[i] + tmp + extra_t;
                            move1[k] = 0;
                            move2_drone_cust[k] = j;
                            move2_takeoff_node[k] = i;
                        }
                    }
                }
                if (tmp == truck_t) {
                    break;
                }

            }
        }
    }

    if (truck_seq == NULL || drone_seq == NULL)
        return T[seq_dim - 1];

    // store truck and drone sequences if truck_seq and drone_seq are not NULL
    int truck_idx = seq_dim - 1;
    int drone_idx = seq_dim - 1;
    truck_seq[truck_idx--] = seq[seq_dim - 1];
    drone_seq[drone_idx--] = seq[seq_dim - 1];

    for (int k = seq_dim - 1; k > 0; k--)
    {
        if (move1[k] == 1)
        {
            truck_seq[truck_idx--] = seq[k - 1];
            drone_seq[drone_idx--] = seq[k - 1];
        }
        else
        {
            drone_seq[drone_idx--] = seq[move2_drone_cust[k]];
            drone_seq[drone_idx--] = seq[move2_takeoff_node[k]];
            int i = k - 1;
            while (i > move2_takeoff_node[k])
            {
                if (i != move2_drone_cust[k])
                {
                    truck_seq[truck_idx--] = seq[i];
                }
                i--;
            }
            truck_seq[truck_idx--] = seq[move2_takeoff_node[k]];
            // go directly to k = move2_takeoff_node[k]
            k = move2_takeoff_node[k] + 1;
        }
    }

    // shift the sequences such that 0 is at the beginning
    memmove(truck_seq, truck_seq + truck_idx + 1, (seq_dim - truck_idx - 1) * sizeof(int));
    memmove(drone_seq, drone_seq + drone_idx + 1, (seq_dim - drone_idx - 1) * sizeof(int));
    return T[seq_dim - 1];
}

double compute_sequence_min_time_light(instance * inst, int *seq, int seq_dim, int *truck_seq, int *drone_seq, int max_truck_leg_size)
{
    if (seq_dim > inst->dimension)
        print_error("Error in compute_sequence_min_time(): seq_dim > inst->dimension\n");
    // array of minimum arrival times T
    // T[i] = minimum arrival time at node i, when i is visited by the truck
    // (since the last node must be visited by the truck, this assumption is ok)
    double T[seq_dim];
    double truck_only[seq_dim];

    // move1[k] = 1 => optimal move that ends in k is of type 1 (truck moves forward with the drone onboard)
    // move1[k] = 0 => optimal move that ends in k is of type 2 (drone leg which ends in k)
    int move1[seq_dim];
    // move2_drone_cust[k] = j if j is the drone customer visited by the drone leg associated to the landing node k
    // move2_takeoff_node[k] = i if i is the takeoff node from which it starts the drone leg associated to the landing node k
    int move2_drone_cust[seq_dim];
    int move2_takeoff_node[seq_dim];

    double extra_t = 0.0;
    if (inst->param.poikonen == 0 && inst->param.ha == 0)
        extra_t = UAV_LAUNCH_TIME + UAV_RETRIEVAL_TIME;
    if (inst->param.ha == 1)
        extra_t = 2 * UAV_LAUNCH_TIME;
    if (inst->param.poikonen == 1 && inst->param.MHD == 1)
        extra_t = POIKONEN_UAV_LAUNCH_TIME + POIKONEN_UAV_RETRIEVAL_TIME;

    // initialize arrival times T[i] equal to the arrival times related to the truck only case
    T[0] = 0.0;
    truck_only[0] = T[0];
    for (int i = 0; i < seq_dim - 1; i++)
    {
        T[i + 1] = T[i] + inst->truck_times[seq[i]][seq[i + 1]];
        truck_only[i + 1] = T[i + 1];
        move1[i + 1] = 1;
    }

    for (int i = 0; i < seq_dim - 1; i++)
    {
        // move 1: move the truck with the drone onboard by one position
        if (T[i] + inst->truck_times[seq[i]][seq[i + 1]] < T[i + 1])
        {
            T[i + 1] = T[i] + inst->truck_times[seq[i]][seq[i + 1]];
            move1[i + 1] = 1;
        }

        // move 2: launch the drone from the current node i, to visit node j, and rejoin the truck with the drone at node k
        //** printf("\ni = %d (%d)\n", i, seq[i]);

        for (int j = i + 1; j < seq_dim - 1; j++)
        {
            if (j > i + max_truck_leg_size - 1)
                break;
            if (inst->param.poikonen == 0 && inst->param.ha == 0 && inst->nodes[seq[j]].weight > UAV_WEIGHT_LIMIT) // node j cannot be a drone customer
                continue;
            if (inst->nodes[seq[j]].truck_only == 1)
                continue;
            //** printf("\n\tj = %d(%d) : ", j, seq[j]);
            double truck_t = truck_only[j + 1] - truck_only[i] - inst->truck_times[seq[j - 1]][seq[j]] - inst->truck_times[seq[j]][seq[j + 1]] + inst->truck_times[seq[j - 1]][seq[j + 1]];
            for (int k = j + 1; k < seq_dim; k++)
            {
                if (k > i + max_truck_leg_size)
                    break;

                // update truck travel time
                // if (k == j + 1)
                // {
                //     for (int w = i + 1; w < j; w++)
                //     {
                //         truck_t += inst->truck_times[seq[w - 1]][seq[w]];
                //     }
                //     truck_t += inst->truck_times[seq[j - 1]][seq[k]];
                // }
                // else
                //     truck_t += inst->truck_times[seq[k - 1]][seq[k]];
                if (k > j + 1)
                    truck_t += inst->truck_times[seq[k - 1]][seq[k]];

                // if the truck_t > max_k {drone travel time i-->j-->k}   STOP
                if (inst->param.poikonen == 0 && inst->param.ha == 0 && truck_t > inst->max_ij_drone_leg_times[seq[i]][seq[j]])
                {
                    // does not exist a landing node k associated to a feasible drone leg i-->j-->k
                    break;
                }
                // check if the drone leg i-->j-->k is feasible
                if (truck_t > inst->max_feas_time_drone[seq[i]][seq[j]][seq[k]]) // the drone leg i-->j-->k is infeasible
                {
                    continue;
                }
                if (k - i - 2 > max_truck_leg_size_found) {
                    max_truck_leg_size_found = k - i - 2;
                }
                double tmp = inst->min_feas_time_drone[seq[i]][seq[j]][seq[k]];

                if (tmp < truck_t) // the drone must wait
                    tmp = truck_t;

                if (inst->param.ha == 0) {
                    if (T[i] + tmp + extra_t < T[k])
                    {
                        T[k] = T[i] + tmp + extra_t;
                        move1[k] = 0;
                        move2_drone_cust[k] = j;
                        move2_takeoff_node[k] = i;
                    }
                }
                else {
                    if (seq[i] == 0) {
                        if (T[i] + tmp + extra_t / 2 < T[k])
                        {
                            T[k] = T[i] + tmp + extra_t / 2;
                            move1[k] = 0;
                            move2_drone_cust[k] = j;
                            move2_takeoff_node[k] = i;
                        }
                    }
                    else {
                        if (T[i] + tmp + extra_t < T[k])
                        {
                            T[k] = T[i] + tmp + extra_t;
                            move1[k] = 0;
                            move2_drone_cust[k] = j;
                            move2_takeoff_node[k] = i;
                        }
                    }
                }
                if (tmp == truck_t) {
                    break;
                }
            }
        }
    }

    if (truck_seq == NULL || drone_seq == NULL)
        return T[seq_dim - 1];

    // store truck and drone sequences if truck_seq and drone_seq are not NULL
    int truck_idx = seq_dim - 1;
    int drone_idx = seq_dim - 1;
    truck_seq[truck_idx--] = seq[seq_dim - 1];
    drone_seq[drone_idx--] = seq[seq_dim - 1];

    for (int k = seq_dim - 1; k > 0; k--)
    {
        if (move1[k] == 1)
        {
            truck_seq[truck_idx--] = seq[k - 1];
            drone_seq[drone_idx--] = seq[k - 1];
        }
        else
        {
            drone_seq[drone_idx--] = seq[move2_drone_cust[k]];
            drone_seq[drone_idx--] = seq[move2_takeoff_node[k]];
            int i = k - 1;
            while (i > move2_takeoff_node[k])
            {
                if (i != move2_drone_cust[k])
                {
                    truck_seq[truck_idx--] = seq[i];
                }
                i--;
            }
            truck_seq[truck_idx--] = seq[move2_takeoff_node[k]];
            // go directly to k = move2_takeoff_node[k]
            k = move2_takeoff_node[k] + 1;
        }
    }

    // shift the sequences such that 0 is at the beginning
    memmove(truck_seq, truck_seq + truck_idx + 1, (seq_dim - truck_idx - 1) * sizeof(int));
    memmove(drone_seq, drone_seq + drone_idx + 1, (seq_dim - drone_idx - 1) * sizeof(int));
    return T[seq_dim - 1];
}

void generate_random_sequence(int *rand_seq, instance * inst)
{
    // fix starting and ending node
    rand_seq[0] = 0;
    rand_seq[inst->dimension - 1] = inst->dimension - 1;

    int numbers[inst->dimension - 2];
    for (int i = 0; i < inst->dimension - 2; i++)
    {
        numbers[i] = i + 1;
    }

    for (int i = 0; i < inst->dimension - 2; i++)
    {
        int idx = rand() % (inst->dimension - 2 - i);
        rand_seq[i + 1] = numbers[idx];
        numbers[idx] = numbers[inst->dimension - 3 - i];
    }
}

// revert the operation with the greatest time gain (if it exists)
// let us call C the current operation that we are evaluating
// case 1: the operation C is not preceded or succeeded by other operations
// case 2: the operation C is not preceded by another operation, but it is succeded by another operation
// case 3: the operation C is preceded by another operation, but it is not succeded by another operation
// case 4: the operation C is both preceded and succeded by another operation
double opt_drone_legs_direction(int *truck_succ, int *drone_succ, instance * inst)
{
    int last_takeoff_node = -1;
    // a drone leg that starts from node 0 must not be inverted
    int i = drone_succ[0];
    int prev_truck_node = 0;
    // check if from node 0 it starts a drone leg
    if (drone_succ[0] != truck_succ[0])
    {
        last_takeoff_node = 0;
        i = drone_succ[drone_succ[0]];
        while (truck_succ[prev_truck_node] != i)
        {
            prev_truck_node = truck_succ[prev_truck_node];
        }
    }

    double delta_max = 0;
    // a drone leg that ends in the final node must not be inverted
    while (drone_succ[i] != inst->dimension - 1 && drone_succ[drone_succ[i]] != inst->dimension - 1)
    {
        if (drone_succ[i] != truck_succ[i]) // drone leg found
        {
            // printf("DRONE LEG FOUND: %d --> %d --> %d\n", i, drone_succ[i], drone_succ[drone_succ[i]]);
            // printf("{prev_truck_node = %d}\n", prev_truck_node);
            // drone leg: i --> j --> k
            // we want to invert it: k --> j --> i
            int j = drone_succ[i];
            int k = drone_succ[j];

            int new_prev_truck_node; // next value to assign to the variable prev_truck_node at the end of the if
            // compute the truck travel time to traverse the path k -> i and the path i -> k
            int tmp_i = i;
            double truck_t_ki = 0.0;
            double truck_t_ik = 0.0;
            while (tmp_i != k)
            {
                truck_t_ik += inst->truck_times[tmp_i][truck_succ[tmp_i]];
                truck_t_ki += inst->truck_times[truck_succ[tmp_i]][tmp_i];
                new_prev_truck_node = tmp_i;
                tmp_i = truck_succ[tmp_i];
            }
            // check if the drone leg k --> j --> i is feasible
            //printf("\ttruck_t_ki = %f\n", truck_t_ki);
            //printf("\max_feas_time_drone[k][j][i] = %f\n", inst->max_feas_time_drone[k][j][i]);
            if (truck_t_ki > inst->max_feas_time_drone[k][j][i])
            {
                //printf("*** drone leg k --> j --> i is infeasible ***\n");
                prev_truck_node = new_prev_truck_node;
                last_takeoff_node = i;
                i = k;
                continue;
            }
            // compute the minimum feasible travel times related to the drone legs i --> j --> k and k --> j --> i
            double t_ijk = inst->min_feas_time_drone[i][j][k];
            double t_kji = inst->min_feas_time_drone[k][j][i];
            if (t_ijk < truck_t_ik)
                t_ijk = truck_t_ik;
            if (t_kji < truck_t_ki)
                t_kji = truck_t_ki;
            // printf("\tt_ijk = %f\n", t_ijk);
            // printf("\tt_kji = %f\n", t_kji);
            // printf("{last_takeoff_node = %d}\n", last_takeoff_node);
            if (last_takeoff_node == -1 || drone_succ[drone_succ[last_takeoff_node]] != i) // case 1|2
            {
                if (drone_succ[k] == truck_succ[k]) // case 1
                {
                    //printf("\tCASE 1\n");
                    double delta = -inst->truck_times[prev_truck_node][i] - t_ijk - inst->truck_times[k][truck_succ[k]] + inst->truck_times[prev_truck_node][k] + t_kji + inst->truck_times[i][truck_succ[k]];
                    //printf("\tdelta = %f\n", delta);
                    if (delta < 0)
                    {
                        // it is convenient to revert the drone leg
                        // printf("*** it is convenient to reverse the drone leg %d --> %d --> %d\n", i, j, k);
                        // printf("\tdelta = %f\n", delta);
                        if (delta < delta_max)
                        {
                            delta_max = delta;
                        }
                    }
                }
                else // case 2
                {
                    //printf("\tCASE 2\n");
                    // from k it begins the drone leg: k --> u --> v
                    int u = drone_succ[k];
                    int v = drone_succ[u];
                    if (inst->min_feas_time_drone[i][u][v] < 1e-6)
                    {
                        //printf("*** drone leg i --> u --> v is infeasible ***\n");
                        prev_truck_node = new_prev_truck_node;
                        last_takeoff_node = i;
                        i = k;
                        continue;
                    }
                    double truck_t_kv = 0.0;
                    int tmp_k = k;
                    while (tmp_k != v)
                    {
                        truck_t_kv += inst->truck_times[tmp_k][truck_succ[tmp_k]];

                        tmp_k = truck_succ[tmp_k];
                    }
                    double truck_t_iv = truck_t_kv - inst->truck_times[k][truck_succ[k]] + inst->truck_times[i][truck_succ[k]];
                    // check if the drone leg can be inverted
                    //printf("\ttruck_t_iv = %f\n", truck_t_iv);
                    //printf("\max_feas_time_drone[i][u][v] = %f\n", inst->max_feas_time_drone[i][u][v]);
                    if (truck_t_iv > inst->max_feas_time_drone[i][u][v])
                    {
                        //printf("*** drone leg i --> u --> v is infeasible ***\n");
                        prev_truck_node = new_prev_truck_node;
                        last_takeoff_node = i;
                        i = k;
                        continue;
                    }
                    // compute the minimum feasible travel times related to the drone legs k --> u --> v and i --> u --> v
                    double t_kuv = inst->min_feas_time_drone[k][u][v];
                    double t_iuv = inst->min_feas_time_drone[i][u][v];
                    if (t_kuv < truck_t_kv)
                        t_kuv = truck_t_kv;
                    if (t_iuv < truck_t_iv)
                        t_iuv = truck_t_iv;
                    //printf("\tt_kuv = %f\n", t_kuv);
                    //printf("\tt_iuv = %f\n", t_iuv);
                    double delta = -inst->truck_times[prev_truck_node][i] - t_ijk - t_kuv + inst->truck_times[prev_truck_node][k] + t_kji + t_iuv;
                    //printf("\tdelta = %f\n", delta);
                    if (delta < 0)
                    {
                        // it is convenient to revert the drone leg
                        //printf("*** it is convenient to reverse the drone leg %d --> %d --> %d\n", i, j, k);
                        //printf("\tdelta = %f\n", delta);
                        if (delta < delta_max)
                        {
                            delta_max = delta;
                        }
                    }
                }
            }
            else // case 3|4
            {
                // a drone lands at node i
                // lets call this drone leg: m --> n --> i
                int m = last_takeoff_node;
                int n = drone_succ[m];
                if (inst->min_feas_time_drone[m][n][k] < 1e-6)
                {
                    //printf("*** drone leg m --> n --> k is infeasible ***\n");
                    prev_truck_node = new_prev_truck_node;
                    last_takeoff_node = i;
                    i = k;
                    continue;
                }

                double truck_t_mi = 0.0;
                int tmp_m = m;
                while (tmp_m != i)
                {
                    truck_t_mi += inst->truck_times[tmp_m][truck_succ[tmp_m]];
                    tmp_m = truck_succ[tmp_m];
                }
                double truck_t_mk = truck_t_mi - inst->truck_times[prev_truck_node][i] + inst->truck_times[prev_truck_node][k];
                // check if the drone leg can be inverted
                // printf("\ttruck_t_mi = %f\n", truck_t_mi);
                // printf("\ttruck_t_mk = %f\n", truck_t_mk);
                // printf("\max_feas_time_drone[m][n][i] = %f\n", inst->max_feas_time_drone[m][n][i]);
                if (truck_t_mk > inst->max_feas_time_drone[m][n][k])
                {
                    //printf("*** drone leg m --> n --> k is infeasible ***\n");
                    prev_truck_node = new_prev_truck_node;
                    last_takeoff_node = i;
                    i = k;
                    continue;
                }
                // compute the minimum feasible travel times related to the drone legs m --> n --> i and m --> n --> k
                double t_mni = inst->min_feas_time_drone[m][n][i];
                double t_mnk = inst->min_feas_time_drone[m][n][k];
                if (t_mni < truck_t_mi)
                    t_mni = truck_t_mi;
                if (t_mnk < truck_t_mk)
                    t_mnk = truck_t_mk;
                // printf("\tt_mni = %f\n", t_mni);
                // printf("\tt_mnk = %f\n", t_mnk);
                if (drone_succ[k] == truck_succ[k]) // case 3
                {
                    //printf("\tCASE 3\n");
                    double delta = -t_mni - t_ijk - inst->truck_times[k][drone_succ[k]] + t_mnk + t_kji + inst->truck_times[i][drone_succ[k]];
                    //printf("\tdelta = %f\n", delta);
                    if (delta < 0)
                    {
                        // it is convenient to revert the drone leg
                        // printf("*** it is convenient to reverse the drone leg %d --> %d --> %d\n", i, j, k);
                        // printf("\tdelta = %f\n", delta);
                        if (delta < delta_max)
                        {
                            delta_max = delta;
                        }
                    }
                }
                else // case 4
                {
                    //printf("\tCASE 4\n");
                    // from k it begins the drone leg: k --> u --> v
                    int u = drone_succ[k];
                    int v = drone_succ[u];
                    if (inst->min_feas_time_drone[i][u][v] < 1e-6)
                    {
                        //printf("*** drone leg i --> u --> v is infeasible ***\n");
                        prev_truck_node = new_prev_truck_node;
                        last_takeoff_node = i;
                        i = k;
                        continue;
                    }
                    double truck_t_kv = 0.0;
                    int tmp = k;
                    while (tmp != v)
                    {
                        truck_t_kv += inst->truck_times[tmp][truck_succ[tmp]];
                        tmp = truck_succ[tmp];
                    }
                    double truck_t_iv = truck_t_kv - inst->truck_times[k][truck_succ[k]] + inst->truck_times[i][truck_succ[k]];
                    // printf("\ttruck_t_kv = %f\n", truck_t_kv);
                    // printf("\ttruck_t_iv = %f\n", truck_t_iv);
                    // printf("\max_feas_time_drone[i][u][v] = %f\n", inst->max_feas_time_drone[i][u][v]);
                    // check if the drone leg can be inverted
                    if (truck_t_iv > inst->max_feas_time_drone[i][u][v])
                    {
                        //printf("*** drone leg i --> u --> v is infeasible ***\n");
                        prev_truck_node = new_prev_truck_node;
                        last_takeoff_node = i;
                        i = k;
                        continue;
                    }
                    // compute the minimum feasible travel times related to the drone legs k --> u --> v and i --> u --> v
                    double t_kuv = inst->min_feas_time_drone[k][u][v];
                    double t_iuv = inst->min_feas_time_drone[i][u][v];
                    if (t_kuv < truck_t_kv)
                        t_kuv = truck_t_kv;
                    if (t_iuv < truck_t_iv)
                        t_iuv = truck_t_iv;
                    // printf("\tt_kuv = %f\n", t_kuv);
                    // printf("\tt_iuv = %f\n", t_iuv);
                    double delta = -t_mni - t_ijk - t_kuv + t_mnk + t_kji + t_iuv;
                    //printf("\tdelta = %f\n", delta);
                    if (delta < 0)
                    {
                        // it is convenient to revert the drone leg
                        // printf("*** it is convenient to reverse the drone leg %d --> %d --> %d\n", i, j, k);
                        // printf("\tdelta = %f\n", delta);
                        if (delta < delta_max)
                        {
                            delta_max = delta;
                        }
                    }
                }
            }
            // set the last node served by the truck
            prev_truck_node = new_prev_truck_node;
            // set the last node from which the drone took off
            last_takeoff_node = i;
            // advance i
            i = k;
        }
        else
        {
            // advance the node
            prev_truck_node = i;
            i = drone_succ[i];
        }
    }
    return delta_max;
}

double two_opt(instance * inst, int *seq, int seq_dim)
{
    int *seq2rev; // reversed sequence associated to the 2-opt with the greatest delta
    int i_star = -1;
    int j_star = -1;
    double max_delta = 0.0;
    for (int i = 0; i < seq_dim - 1; i++)
    {
        if (seq[i + 1] == seq[seq_dim - 1]) // the path seq[i+1] to seq[j] must be invertible
            continue;
        for (int j = i + 1; j < seq_dim - 1; j++)
        {
            if (seq[i + 1] == seq[j])
                continue;
            double delta = inst->truck_times[seq[i]][seq[i + 1]] + inst->truck_times[seq[j]][seq[j + 1]];
            delta -= inst->truck_times[seq[i]][seq[j]] + inst->truck_times[seq[i + 1]][seq[j + 1]];
            if (delta > 0) // check if inverting the path seq[i+1] ---> seq[j] is advantageous
            {
                int *tmp = (int *)malloc((j - i) * sizeof(int)); // array where to store the reversed seq from seq[j] to seq[i+1]
                for (int k = i + 1; k < j; k++)
                {
                    delta += inst->truck_times[seq[k]][seq[k + 1]] - inst->truck_times[seq[k + 1]][seq[k]];
                    tmp[j - k] = seq[k];
                }
                tmp[0] = seq[j];
                if (delta > max_delta) // it is convenient to apply the move
                {
                    if (i_star != -1)
                        free(seq2rev);
                    seq2rev = tmp;
                    max_delta = delta;
                    i_star = i;
                    j_star = j;
                }
                else
                    free(tmp);
            }
        }
    }
    // apply the 2-opt move with the greatest delta
    if (i_star != -1)
    {
        memcpy(seq + i_star + 1, seq2rev, (j_star - i_star) * sizeof(int));
        free(seq2rev);
    }
    return max_delta;
}

double optimize_truck_legs(instance * inst, int *truck_seq, int *drone_seq)
{
    int *original_truck_seq = (int *)malloc(inst->dimension * sizeof(int));
    int *original_drone_seq = (int *)malloc(inst->dimension * sizeof(int));
    memcpy(original_truck_seq, truck_seq, inst->dimension * sizeof(int));
    memcpy(original_drone_seq, drone_seq, inst->dimension * sizeof(int));
    // printf("ORIGINAL TRUCK SEQ: \n");
    // for (int i = 0; i < inst->dimension; i++)
    //     printf("%d,", truck_seq[i]);
    // printf("\nCOPIED TRUCK SEQ: \n");
    // for (int i = 0; i < inst->dimension; i++)
    //     printf("%d,", original_truck_seq[i]);
    // END DELETE
    double delta = 0.0;

    // directly optimize truck legs with 2 or 3 truck-only customers
    int t = 0; // current truck index
    int d = 0; // current drone index
    while (truck_seq[t] != inst->dimension - 1)
    {
        //printf("t = %d\n", t);
        if (truck_seq[t + 1] == drone_seq[d + 1]) // combined arc
        {
            // advance the node
            d++;
            t++;
            continue;
        }
        if (truck_seq[t + 1] == drone_seq[d + 2]) // truck leg without truck customers
        {
            d += 2;
            t++;
            continue;
        }
        // truck leg with at least one truck customer
        int count = 0;                // #edges in the truck leg
        double init_truck_time = 0.0; // retrieve the truck leg travel time
        while (truck_seq[t + count] != drone_seq[d + 2])
        {
            init_truck_time += inst->truck_times[truck_seq[t + count]][truck_seq[t + count + 1]];
            count++;
        }
        //printf("truck leg travel time %d -> %d : %f\n", drone_seq[d], drone_seq[d + 2], init_truck_time);
        // advance the indices
        if (count == 2)
        {
            // // advance the indices
            d += 2;
            t += count;
            continue;
        }
        else if (count == 3) // 2 truck customers
        {
            // check if swapping the nodes, the travel time decreases
            double truck_time2 = inst->truck_times[truck_seq[t]][truck_seq[t + 2]] + inst->truck_times[truck_seq[t + 2]][truck_seq[t + 1]] + inst->truck_times[truck_seq[t + 1]][truck_seq[t + 3]];
            if (truck_time2 < init_truck_time)
            {
                //printf("*** swap2!! ***\n");
                int tmp = truck_seq[t + 1];
                truck_seq[t + 1] = truck_seq[t + 2];
                truck_seq[t + 2] = tmp;
                // there is an improvement only if the truck waits at the rendezvous node
                if (init_truck_time > inst->min_feas_time_drone[drone_seq[d]][drone_seq[d + 1]][drone_seq[d + 2]])
                {
                    double gain = init_truck_time - inst->min_feas_time_drone[drone_seq[d]][drone_seq[d + 1]][drone_seq[d + 2]]; // truck waiting time at the rendezvous node w.r.t. the min time drone leg
                    if (gain > init_truck_time - truck_time2)
                        gain = init_truck_time - truck_time2;
                    delta += gain;
                }
            }
        }
        else if (count == 4) // 3 truck customers
        {
            int min_i;
            double times[5];
            // 1 -> 3 -> 2
            times[0] = inst->truck_times[truck_seq[t]][truck_seq[t + 1]] + inst->truck_times[truck_seq[t + 1]][truck_seq[t + 3]] + inst->truck_times[truck_seq[t + 3]][truck_seq[t + 2]] + inst->truck_times[truck_seq[t + 2]][truck_seq[t + 4]];
            min_i = 0;
            // 2 -> 1 -> 3
            times[1] = inst->truck_times[truck_seq[t]][truck_seq[t + 2]] + inst->truck_times[truck_seq[t + 2]][truck_seq[t + 1]] + inst->truck_times[truck_seq[t + 1]][truck_seq[t + 3]] + inst->truck_times[truck_seq[t + 3]][truck_seq[t + 4]];
            if (times[1] < times[min_i])
                min_i = 1;
            // 2 -> 3 -> 1
            times[2] = inst->truck_times[truck_seq[t]][truck_seq[t + 2]] + inst->truck_times[truck_seq[t + 2]][truck_seq[t + 3]] + inst->truck_times[truck_seq[t + 3]][truck_seq[t + 1]] + inst->truck_times[truck_seq[t + 1]][truck_seq[t + 4]];
            if (times[2] < times[min_i])
                min_i = 2;
            // 3 -> 1 -> 2
            times[3] = inst->truck_times[truck_seq[t]][truck_seq[t + 3]] + inst->truck_times[truck_seq[t + 3]][truck_seq[t + 1]] + inst->truck_times[truck_seq[t + 1]][truck_seq[t + 2]] + inst->truck_times[truck_seq[t + 2]][truck_seq[t + 4]];
            if (times[3] < times[min_i])
                min_i = 3;
            // 3 -> 2 -> 1
            times[4] = inst->truck_times[truck_seq[t]][truck_seq[t + 3]] + inst->truck_times[truck_seq[t + 3]][truck_seq[t + 2]] + inst->truck_times[truck_seq[t + 2]][truck_seq[t + 1]] + inst->truck_times[truck_seq[t + 1]][truck_seq[t + 4]];
            if (times[4] < times[min_i])
                min_i = 4;
            if (times[min_i] < init_truck_time)
            {
                //printf("*** swap3!! ***\n");
                switch (min_i)
                {
                case 0:
                {
                    double tmp = truck_seq[t + 2];
                    truck_seq[t + 2] = truck_seq[t + 3];
                    truck_seq[t + 3] = tmp;
                    break;
                }
                case 1:
                {
                    double tmp = truck_seq[t + 1];
                    truck_seq[t + 1] = truck_seq[t + 2];
                    truck_seq[t + 2] = tmp;
                    break;
                }
                case 2:
                {
                    double tmp = truck_seq[t + 1];
                    truck_seq[t + 1] = truck_seq[t + 2];
                    truck_seq[t + 2] = truck_seq[t + 3];
                    truck_seq[t + 3] = tmp;
                    break;
                }
                case 3:
                {
                    double tmp = truck_seq[t + 1];
                    truck_seq[t + 1] = truck_seq[t + 3];
                    truck_seq[t + 3] = truck_seq[t + 2];
                    truck_seq[t + 2] = tmp;
                    break;
                }
                case 4:
                {
                    double tmp = truck_seq[t + 1];
                    truck_seq[t + 1] = truck_seq[t + 3];
                    truck_seq[t + 3] = tmp;
                    break;
                }
                } // end switch
                // there is an improvement only if the truck waits at the rendezvous node
                if (init_truck_time > inst->min_feas_time_drone[drone_seq[d]][drone_seq[d + 1]][drone_seq[d + 2]])
                {
                    // truck waiting time at the rendezvous node w.r.t. the min time drone leg
                    double gain = init_truck_time - inst->min_feas_time_drone[drone_seq[d]][drone_seq[d + 1]][drone_seq[d + 2]];
                    if (gain > init_truck_time - times[min_i])
                        gain = init_truck_time - times[min_i];
                    delta += gain;
                }
            }
        }
        else // more than 3 truck customers
        {
            // optimize the truck legs with 2-opt moves
            //printf("*** MORE THAN 3 TRUCK CUSTOMERS (%d) ***\n", count - 1);
            double truck_time2 = init_truck_time;
            double move_gain = 0.0;
            do
            {
                move_gain = two_opt(inst, truck_seq + t, count + 1);
                if (move_gain > 0.0)
                {
                    //printf("2-opt delta = %f\n", move_gain);
                    truck_time2 -= move_gain;
                }
            } while (move_gain > 0.0);
            // there is an improvement only if the truck waits at the rendezvous node w.r.t. the min time drone leg
            if (init_truck_time > inst->min_feas_time_drone[drone_seq[d]][drone_seq[d + 1]][drone_seq[d + 2]])
            {
                // truck waiting time at the rendezvous node w.r.t. the min time drone leg
                double gain = init_truck_time - inst->min_feas_time_drone[drone_seq[d]][drone_seq[d + 1]][drone_seq[d + 2]];
                if (gain > init_truck_time - truck_time2)
                    gain = init_truck_time - truck_time2;
                delta += gain;
            }
        }

        // // advance the indices
        d += 2;
        t += count;
    }

    double obj = compute_score(inst, original_truck_seq, original_drone_seq, inst->dimension - 1, 0);
    //printf("INITIAL COST: %f \n", obj);
    // printf("\nOPTIMIZED TRUCK SEQ:\n");
    // for (int k = 0; k < inst->dimension; k++)
    //     printf("%d,", truck_seq[k]);
    // printf("\nOPTIMIZED DRONE SEQ:\n");
    // for (int k = 0; k < inst->dimension; k++)
    //     printf("%d,", drone_seq[k]);
    // printf("\n");
    // check new cost
    double new_obj = compute_score(inst, truck_seq, drone_seq, inst->dimension - 1, 0);
    if (fabs(delta - (obj - new_obj)) > 1e-7)
    {
        printf("*** ACHTUNG! *** \n");
        printf("\nORIGINAL TRUCK SEQ:\n");
        for (int k = 0; k < inst->dimension; k++)
            printf("%d,", original_truck_seq[k]);
        printf("\nORIGINAL DRONE SEQ:\n");
        for (int k = 0; k < inst->dimension; k++)
            printf("%d,", original_drone_seq[k]);
        printf("delta = %f\n", delta);
        printf("COMPUTED delta = %f\n", obj - new_obj);
    }
    free(original_truck_seq);
    free(original_drone_seq);
    return delta;
}

// revert the operation with the greatest time gain (if it exists)
// let us call C the current operation that we are evaluating
// case 1: the operation C is not preceded or succeeded by other operations
// case 2: the operation C is not preceded by another operation, but it is succeded by another operation
// case 3: the operation C is preceded by another operation, but it is not succeded by another operation
// case 4: the operation C is both preceded and succeded by another operation
double reverse_drone_leg(instance * inst, int *truck_seq, int *drone_seq)
{
    // a drone leg that starts from node 0 must not be inverted
    // check if from node 0 it starts a drone leg
    int t_i = 0;                                  // index node i in the truck sequence
    int d_i = 0;                                  // index node i in the drone sequence
    if (drone_seq[d_i + 1] != truck_seq[t_i + 1]) // drone leg that starts from 0
    {
        t_i++;
        d_i += 2;
        while (truck_seq[t_i] != drone_seq[d_i])
        {
            t_i++;
        }
    }
    else
    {
        t_i++;
        d_i++;
    }
    // at this point: truck_seq[t_i] = drone_seq[d_i]

    // truck and drone indices of the takeoff and landing nodes related to the drone leg with the greatest gain
    int best_t_i = -1;
    int best_t_k = -1;
    int best_d_i = -1;
    int best_d_k = -1;

    double delta_max = 0;
    // a drone leg that ends in the final node must not be inverted
    while (truck_seq[t_i + 1] != inst->dimension - 1 && drone_seq[d_i + 2] != inst->dimension - 1)
    {
        if (truck_seq[t_i] != drone_seq[d_i])
            print_error("*** truck_seq[t_i] != drone_seq[d_i] ***\n");
        // printf("drone_seq[d_i] = %d\n", drone_seq[d_i]);
        // printf("truck_seq[t_i] = %d\n", truck_seq[t_i]);
        if (truck_seq[t_i + 1] != drone_seq[d_i + 1]) // drone leg found
        {
            // printf("DRONE LEG FOUND: %d --> %d --> %d\n", drone_seq[d_i], drone_seq[d_i + 1], drone_seq[d_i + 2]);
            // printf("{prev_truck_node = %d}\n", prev_truck_node);
            // drone leg: i --> j --> k
            // we want to invert it: k --> j --> i
            int d_j = d_i + 1;
            int d_k = d_i + 2;

            // compute the truck travel time to traverse the path k -> i and the path i -> k
            int tmp_i = t_i;
            double truck_t_ki = 0.0;
            double truck_t_ik = 0.0;
            while (truck_seq[tmp_i] != drone_seq[d_k])
            {
                truck_t_ik += inst->truck_times[truck_seq[tmp_i]][truck_seq[tmp_i + 1]];
                truck_t_ki += inst->truck_times[truck_seq[tmp_i + 1]][truck_seq[tmp_i]];
                tmp_i++;
            }
            int t_k = tmp_i; // truck index associated to the rendezvous node k

            // check if the drone leg k --> j --> i is feasible
            //printf("\ttruck_t_ki = %f\n", truck_t_ki);
            //printf("\max_feas_time_drone[k][j][i] = %f\n", inst->max_feas_time_drone[k][j][i]);
            if (truck_t_ki > inst->max_feas_time_drone[drone_seq[d_k]][drone_seq[d_j]][drone_seq[d_i]])
            {
                //printf("*** drone leg k --> j --> i is infeasible ***\n");
                // jump to the rendezvous node
                t_i = t_k;
                d_i = d_k;
                continue;
            }
            // compute the minimum feasible travel times related to the drone legs i --> j --> k and k --> j --> i
            double t_ijk = inst->min_feas_time_drone[drone_seq[d_i]][drone_seq[d_j]][drone_seq[d_k]];
            double t_kji = inst->min_feas_time_drone[drone_seq[d_k]][drone_seq[d_j]][drone_seq[d_i]];
            if (t_ijk < truck_t_ik)
                t_ijk = truck_t_ik;
            if (t_kji < truck_t_ki)
                t_kji = truck_t_ki;
            // printf("\tt_ijk = %f\n", t_ijk);
            // printf("\tt_kji = %f\n", t_kji);
            // printf("{last_takeoff_node = %d}\n", last_takeoff_node);
            if (drone_seq[d_i - 1] == truck_seq[t_i - 1]) // case 1|2
            {
                if (drone_seq[d_k + 1] == truck_seq[t_k + 1]) // case 1
                {
                    // printf("\tCASE 1\n");
                    // delta = time gained reversing the drone leg
                    double delta = inst->truck_times[truck_seq[t_i - 1]][truck_seq[t_i]] + t_ijk + inst->truck_times[truck_seq[t_k]][truck_seq[t_k + 1]];
                    delta = delta - inst->truck_times[truck_seq[t_i - 1]][truck_seq[t_k]] - t_kji - inst->truck_times[truck_seq[t_i]][truck_seq[t_k + 1]];
                    //printf("\tdelta = %f\n", delta);
                    if (delta > 0)
                    {
                        // it is convenient to revert the drone leg
                        // printf("*** it is convenient to reverse the drone leg %d --> %d --> %d\n", drone_seq[d_i], drone_seq[d_j], drone_seq[d_k]);
                        // printf("\tdelta = %f\n", delta);
                        if (delta > delta_max)
                        {
                            delta_max = delta;
                            best_t_i = t_i;
                            best_t_k = t_k;
                            best_d_i = d_i;
                            best_d_k = d_k;
                        }
                    }
                }
                else // case 2
                {
                    // printf("\tCASE 2\n");
                    // from k it begins the drone leg: k --> u --> v
                    int d_u = d_k + 1;
                    int d_v = d_k + 2;
                    if (inst->min_feas_time_drone[drone_seq[d_i]][drone_seq[d_u]][drone_seq[d_v]] < 1e-6)
                    {
                        //printf("*** drone leg i --> u --> v is infeasible ***\n");
                        // jump to the rendezvous node
                        t_i = t_k;
                        d_i = d_k;
                        continue;
                    }
                    // compute the truck path length to go from k to v
                    double truck_t_kv = 0.0;
                    int tmp_k = t_k;
                    while (truck_seq[tmp_k] != drone_seq[d_v])
                    {
                        truck_t_kv += inst->truck_times[truck_seq[tmp_k]][truck_seq[tmp_k + 1]];
                        tmp_k++;
                    }
                    double truck_t_iv = truck_t_kv - inst->truck_times[truck_seq[t_k]][truck_seq[t_k + 1]] + inst->truck_times[truck_seq[t_i]][truck_seq[t_k + 1]];
                    // check if the drone leg can be inverted
                    //printf("\ttruck_t_iv = %f\n", truck_t_iv);
                    //printf("\max_feas_time_drone[i][u][v] = %f\n", inst->max_feas_time_drone[i][u][v]);
                    if (truck_t_iv > inst->max_feas_time_drone[drone_seq[d_i]][drone_seq[d_u]][drone_seq[d_v]])
                    {
                        //printf("*** drone leg i --> u --> v is infeasible ***\n");
                        // jump to the rendezvous node
                        t_i = t_k;
                        d_i = d_k;
                        continue;
                    }
                    // compute the minimum feasible travel times related to the drone legs k --> u --> v and i --> u --> v
                    double t_kuv = inst->min_feas_time_drone[drone_seq[d_k]][drone_seq[d_u]][drone_seq[d_v]];
                    double t_iuv = inst->min_feas_time_drone[drone_seq[d_i]][drone_seq[d_u]][drone_seq[d_v]];
                    if (t_kuv < truck_t_kv)
                        t_kuv = truck_t_kv;
                    if (t_iuv < truck_t_iv)
                        t_iuv = truck_t_iv;
                    //printf("\tt_kuv = %f\n", t_kuv);
                    //printf("\tt_iuv = %f\n", t_iuv);
                    //double delta = -inst->truck_times[prev_truck_node][i] - t_ijk - t_kuv + inst->truck_times[prev_truck_node][k] + t_kji + t_iuv;
                    double delta = inst->truck_times[truck_seq[t_i - 1]][truck_seq[t_i]] + t_ijk + t_kuv;
                    delta = delta - inst->truck_times[truck_seq[t_i - 1]][truck_seq[t_k]] - t_kji - t_iuv;
                    //printf("\tdelta = %f\n", delta);
                    if (delta > 0)
                    {
                        // it is convenient to revert the drone leg
                        // printf("*** it is convenient to reverse the drone leg %d --> %d --> %d\n", drone_seq[d_i], drone_seq[d_j], drone_seq[d_k]);
                        // printf("\tdelta = %f\n", delta);
                        if (delta > delta_max)
                        {
                            delta_max = delta;
                            best_t_i = t_i;
                            best_t_k = t_k;
                            best_d_i = d_i;
                            best_d_k = d_k;
                        }
                    }
                }
            }
            else // case 3|4
            {
                // a drone lands at node i
                // lets call this drone leg: m --> n --> i
                int d_n = d_i - 1;
                int d_m = d_i - 2;
                if (inst->min_feas_time_drone[drone_seq[d_m]][drone_seq[d_n]][drone_seq[d_i]] < 1e-6)
                {
                    //printf("*** drone leg m --> n --> k is infeasible ***\n");
                    // jump to the rendezvous node
                    t_i = t_k;
                    d_i = d_k;
                    continue;
                }

                double truck_t_mi = 0.0;
                int tmp = t_i;
                while (truck_seq[tmp] != drone_seq[d_m])
                {
                    truck_t_mi += inst->truck_times[truck_seq[tmp - 1]][truck_seq[tmp]];
                    tmp--;
                }
                double truck_t_mk = truck_t_mi - inst->truck_times[truck_seq[t_i - 1]][truck_seq[t_i]] + inst->truck_times[truck_seq[t_i - 1]][truck_seq[t_k]];
                // check if the drone leg can be inverted
                // printf("\ttruck_t_mi = %f\n", truck_t_mi);
                // printf("\ttruck_t_mk = %f\n", truck_t_mk);
                // printf("\max_feas_time_drone[m][n][i] = %f\n", inst->max_feas_time_drone[m][n][i]);
                if (truck_t_mk > inst->max_feas_time_drone[drone_seq[d_m]][drone_seq[d_n]][drone_seq[d_k]])
                {
                    //printf("*** drone leg m --> n --> k is infeasible ***\n");
                    // jump to the rendezvous node
                    t_i = t_k;
                    d_i = d_k;
                    continue;
                }
                // compute the minimum feasible travel times related to the drone legs m --> n --> i and m --> n --> k
                double t_mni = inst->min_feas_time_drone[drone_seq[d_m]][drone_seq[d_n]][drone_seq[d_i]];
                double t_mnk = inst->min_feas_time_drone[drone_seq[d_m]][drone_seq[d_n]][drone_seq[d_k]];
                if (t_mni < truck_t_mi)
                    t_mni = truck_t_mi;
                if (t_mnk < truck_t_mk)
                    t_mnk = truck_t_mk;
                // printf("\tt_mni = %f\n", t_mni);
                // printf("\tt_mnk = %f\n", t_mnk);
                if (drone_seq[d_k + 1] == truck_seq[t_k + 1]) // case 3
                {
                    // printf("\tCASE 3\n");
                    //double delta = -t_mni - t_ijk - inst->truck_times[k][drone_succ[k]] + t_mnk + t_kji + inst->truck_times[i][drone_succ[k]];
                    double delta = t_mni + t_ijk + inst->truck_times[truck_seq[t_k]][truck_seq[t_k + 1]];
                    delta = delta - t_mnk - t_kji - inst->truck_times[truck_seq[t_i]][truck_seq[t_k + 1]];
                    //printf("\tdelta = %f\n", delta);
                    if (delta > 0)
                    {
                        // it is convenient to revert the drone leg
                        // printf("*** it is convenient to reverse the drone leg %d --> %d --> %d\n", drone_seq[d_i], drone_seq[d_j], drone_seq[d_k]);
                        // printf("\tdelta = %f\n", delta);
                        if (delta > delta_max)
                        {
                            delta_max = delta;
                            best_t_i = t_i;
                            best_t_k = t_k;
                            best_d_i = d_i;
                            best_d_k = d_k;
                        }
                    }
                }
                else // case 4
                {
                    // printf("\tCASE 4\n");
                    // from k it begins the drone leg: k --> u --> v
                    int d_u = d_k + 1;
                    int d_v = d_k + 2;
                    if (inst->min_feas_time_drone[drone_seq[d_i]][drone_seq[d_u]][drone_seq[d_v]] < 1e-6)
                    {
                        //printf("*** drone leg i --> u --> v is infeasible ***\n");
                        // jump to the rendezvous node
                        t_i = t_k;
                        d_i = d_k;
                        continue;
                    }
                    double truck_t_kv = 0.0;
                    int tmp_k = t_k;
                    while (truck_seq[tmp_k] != drone_seq[d_v])
                    {
                        truck_t_kv += inst->truck_times[truck_seq[tmp_k]][truck_seq[tmp_k + 1]];
                        tmp_k++;
                    }
                    double truck_t_iv = truck_t_kv - inst->truck_times[truck_seq[t_k]][truck_seq[t_k + 1]] + inst->truck_times[truck_seq[t_i]][truck_seq[t_k + 1]];
                    // printf("\ttruck_t_kv = %f\n", truck_t_kv);
                    // printf("\ttruck_t_iv = %f\n", truck_t_iv);
                    // printf("\max_feas_time_drone[i][u][v] = %f\n", inst->max_feas_time_drone[i][u][v]);
                    // check if the drone leg can be inverted
                    if (truck_t_iv > inst->max_feas_time_drone[drone_seq[d_i]][drone_seq[d_u]][drone_seq[d_v]])
                    {
                        //printf("*** drone leg i --> u --> v is infeasible ***\n");
                        // jump to the rendezvous node
                        t_i = t_k;
                        d_i = d_k;
                        continue;
                    }
                    // compute the minimum feasible travel times related to the drone legs k --> u --> v and i --> u --> v
                    double t_kuv = inst->min_feas_time_drone[drone_seq[d_k]][drone_seq[d_u]][drone_seq[d_v]];
                    double t_iuv = inst->min_feas_time_drone[drone_seq[d_i]][drone_seq[d_u]][drone_seq[d_v]];
                    if (t_kuv < truck_t_kv)
                        t_kuv = truck_t_kv;
                    if (t_iuv < truck_t_iv)
                        t_iuv = truck_t_iv;
                    // printf("\tt_kuv = %f\n", t_kuv);
                    // printf("\tt_iuv = %f\n", t_iuv);
                    double delta = t_mni + t_ijk + t_kuv;
                    delta = delta - t_mnk - t_kji - t_iuv;
                    //printf("\tdelta = %f\n", delta);
                    if (delta > 0)
                    {
                        // it is convenient to revert the drone leg
                        // printf("*** it is convenient to reverse the drone leg %d --> %d --> %d\n", drone_seq[d_i], drone_seq[d_j], drone_seq[d_k]);
                        // printf("\tdelta = %f\n", delta);
                        if (delta > delta_max)
                        {
                            delta_max = delta;
                            best_t_i = t_i;
                            best_t_k = t_k;
                            best_d_i = d_i;
                            best_d_k = d_k;
                        }
                    }
                }
            }
            // jump to the rendezvous node
            t_i = t_k;
            d_i = d_k;
        }
        else
        {
            // advance the node
            d_i++;
            t_i++;
        }
    }
    if (delta_max > 0) // reverse the most convenient drone leg found (if any)
    {
        // reverse the drone leg
        int tmp = drone_seq[best_d_i];
        drone_seq[best_d_i] = drone_seq[best_d_k];
        drone_seq[best_d_k] = tmp;
        // reverse the truck leg
        int left = best_t_i;
        int right = best_t_k;
        while (left < right)
        {
            tmp = truck_seq[left];
            truck_seq[left] = truck_seq[right];
            truck_seq[right] = tmp;
            left++;
            right--;
        }
    }
    return delta_max;
}

void nearest_neighbours(instance * inst, int *seq, int options)
{
    if (options < 1)
        options = 1;
    // increase the number of options by 1, in order to handle an extra case

    // boolean array : selected[i] = 1 if node i has been already selected
    int *selected = (int *)calloc(inst->dimension, sizeof(int));
    int current = 0; // Index of the current node
    seq[0] = current;
    selected[current] = 1;

    seq[inst->dimension - 1] = inst->dimension - 1;
    selected[inst->dimension - 1] = 1;

    // flag = 1 if in the last iteration the current node has been updated (0 otherwise)
    int flag = 1;

    // Build the circuit adding inst->dimension - 2 nodes (the first and the last node are fixed)
    for (int count = 1; count < inst->dimension - 1; count++)
    {

        // Check the number of nodes not yet selected
        // options must be smaller than or equal to number of available nodes
        if (inst->dimension - 1 - count < options)
        {
            options = inst->dimension - 1 - count;
        }

        double min_time[options]; // Minimum truck travel times
        int min_node[options];    // Closest nodes indices

        for (int i = 0; i < options; ++i)
        {
            min_time[i] = DBL_MAX;
            min_node[i] = -1;
        }

        // select the closest (options) nodes w.r.t. to the current one
        for (int i = 1; i < inst->dimension - 1; i++)
        {
            if (selected[i] == 0) // node i has not been selected yet
            {
                double time = inst->truck_times[current][i];
                if (time < min_time[options - 1])
                {
                    int k;
                    for (k = options - 2; k >= 0; k--)
                    {   // if options == 1 => it does not enter in the loop => k+1 = 0: OK
                        if (time >= min_time[k])
                            break; // node i is the (k+1)-th nearest node (currently)
                    }

                    // the selected node is the (k+1)-th closest node
                    // right-shift the elements that are farthest by one
                    for (int j = options - 2; j > k; j--)
                    {   // if options == 1 => k=-1, j=-1 => it does not enter in the loop
                        min_time[j + 1] = min_time[j];
                        min_node[j + 1] = min_node[j];
                    }
                    min_time[k + 1] = time;
                    min_node[k + 1] = i;
                }
            }
        }

        // Minimum node random selection
        int h = rand() % options;
        seq[count] = min_node[h];
        selected[min_node[h]] = 1;

        // Update the current node with prob 0.5
        // If in the last iteration the node has not been updated, it must be updated in this iteration
        if (flag && rand() % 2)
        {
            flag = 0;
        }
        else
        {
            current = min_node[h];
            flag = 1;
        }
    }

    free(selected);
}

void get_precedence_seq(int *truck_seq, int *drone_seq, int seq_dim, int *output_seq)
{
    if (drone_seq[0] != truck_seq[0]) {
        printf("drone_seq[0]: %d, truck_seq[0]: %d", drone_seq[0], truck_seq[0]);
        print_error("Error in get_precedence_seq(): different first node\n");
    }
    output_seq[0] = truck_seq[0];
    int i = 1; // where to store the next element in the sequence
    int t = 0; // current truck index
    int d = 0; // current drone index
    while (i < seq_dim)
    {
        if (truck_seq[t + 1] == drone_seq[d + 1]) // combined arc
        {
            output_seq[i++] = truck_seq[t + 1];
            // advance the drone and the truck to the next node
            t++;
            d++;
            continue;
        }
        // DRONE LEG
        // copy drone customer
        output_seq[i++] = drone_seq[d + 1];
        d++;
        // copy truck customers
        while (truck_seq[t + 1] != drone_seq[d + 1])
        {
            if (t == seq_dim)
                print_error("Error in get_precedence_seq(): t == seq_dim\n");
            output_seq[i++] = truck_seq[t + 1];
            t++;
        }
        // copy rendezvous node
        output_seq[i++] = truck_seq[t + 1];
        t++;
        d++;
    }
}


// double swap_customers(instance * inst, individual * ind)
// {
//     // printf("*TRUCK SEQ:");
//     // for (int j = 0; j < ind->dimension; j++)
//     //     printf("%d,", ind->truck_seq[j]);
//     // printf("\n*DRONE SEQ:");
//     // for (int j = 0; j < ind->dimension; j++)
//     //     printf("%d,", ind->drone_seq[j]);
//     // printf("\n");
//     double tot_delta = 0.0;
//     int n_moves = 0;
//     int t = 0; // truck seq index
//     int d = 0; // drone seq index
//     // truck and drone indices of the last customer visited both by the truck and the drone (combined customer or rendezvous node)
//     int t_start = -1;
//     int d_start = -1;
//     while (ind->truck_seq[t] != inst->dimension - 1)
//     {
//         if (ind->truck_seq[t + 1] == ind->drone_seq[d + 1]) // combined edge
//         {
//             t_start = t;
//             d_start = d;
//             t++;
//             d++;
//             //printf("*combined edge\n");
//             continue;
//         }
//         // operation found
//         // compute the truck index of the rendezvous node
//         int rendezvous_index = t + 1;
//         while (ind->truck_seq[rendezvous_index] != ind->drone_seq[d + 2])
//             rendezvous_index++;
//         if (t == 0)
//         {
//             t_start = 0;
//             d_start = 0;
//             t = rendezvous_index;
//             d += 2;
//             continue;
//         }

//         int *subseq_truck = (int *)malloc((rendezvous_index - t_start + 1) * sizeof(int));
//         int *subseq_drone = (int *)malloc((d + 2 - d_start + 1) * sizeof(int));
//         memcpy(subseq_truck, ind->truck_seq + t_start, (rendezvous_index - t_start + 1) * sizeof(int));
//         memcpy(subseq_drone, ind->drone_seq + d_start, (d + 2 - d_start + 1) * sizeof(int));
//         // printf("t = %d, d = %d\n", t, d);
//         // printf("t_start = %d, d_start = %d\n", t_start, d_start);
//         // printf("TRUCK SUBSEQ:");
//         // for (int j = 0; j < (rendezvous_index - t_start + 1); j++)
//         //     printf("%d,", subseq_truck[j]);
//         // printf("\nDRONE SUBSEQ:");
//         // for (int j = 0; j < (d + 2 - d_start + 1); j++)
//         //     printf("%d,", subseq_drone[j]);
//         // printf("\n");
//         // swap the current node with each one of its neighbors
//         int t_sub = t - t_start;
//         int d_sub = d - d_start;
//         // compute the score of the original subsequence
//         double init_score = compute_score(inst, subseq_truck, subseq_drone, ind->drone_seq[d + 2]);
//         double min_score = init_score;
//         int option = 0;

//         if (ind->truck_seq[t - 1] == ind->drone_seq[d - 1])
//         {
//             if (t > 1)
//             {
//                 // case 1: previous combined customer
//                 subseq_truck[t_sub] = ind->truck_seq[t - 1];
//                 subseq_truck[t_sub - 1] = ind->truck_seq[t];
//                 subseq_drone[d_sub] = ind->drone_seq[d - 1];
//                 subseq_drone[d_sub - 1] = ind->drone_seq[d];
//                 // compute new score
//                 double score = compute_score(inst, subseq_truck, subseq_drone, ind->drone_seq[d + 2]);
//                 if (score > 0 && score < min_score)
//                 {
//                     min_score = score;
//                     option = 1;
//                 }
//                 // restore subseqs
//                 subseq_truck[t_sub] = ind->truck_seq[t];
//                 subseq_truck[t_sub - 1] = ind->truck_seq[t - 1];
//                 subseq_drone[d_sub] = ind->drone_seq[d];
//                 subseq_drone[d_sub - 1] = ind->drone_seq[d - 1];
//             }
//         }
//         else
//         {
//             if (ind->truck_seq[t - 1] != ind->drone_seq[d - 2])
//             {
//                 // case 2: previous truck customer
//                 if (t > 1)
//                 {
//                     subseq_truck[t_sub] = ind->truck_seq[t - 1];
//                     subseq_truck[t_sub - 1] = ind->truck_seq[t];
//                     subseq_drone[d_sub] = subseq_truck[t_sub];
//                     // compute new score
//                     double score = compute_score(inst, subseq_truck, subseq_drone, ind->drone_seq[d + 2]);
//                     if (score > 0 && score < min_score)
//                     {
//                         min_score = score;
//                         option = 2;
//                     }
//                     // restore subseqs
//                     subseq_truck[t_sub] = ind->truck_seq[t];
//                     subseq_truck[t_sub - 1] = ind->truck_seq[t - 1];
//                     subseq_drone[d_sub] = ind->drone_seq[d];
//                 }
//                 // case 3: previous drone customer
//                 {
//                     subseq_drone[d_sub] = ind->drone_seq[d - 1];
//                     subseq_drone[d_sub - 1] = ind->drone_seq[d];
//                     subseq_truck[t_sub] = subseq_drone[d_sub];
//                     // compute new score
//                     double score = compute_score(inst, subseq_truck, subseq_drone, ind->drone_seq[d + 2]);
//                     if (score > 0 && score < min_score)
//                     {
//                         min_score = score;
//                         option = 3;
//                     }
//                     // restore subseqs
//                     subseq_truck[t_sub] = ind->truck_seq[t];
//                     subseq_drone[d_sub] = ind->drone_seq[d];
//                     subseq_drone[d_sub - 1] = ind->drone_seq[d - 1];
//                 }
//             }
//         }

//         if (ind->truck_seq[t + 1] == ind->drone_seq[d + 1])
//         {
//             // case 4: successive combined customer
//             if (ind->truck_seq[t + 1] != inst->dimension - 1)
//             {
//                 subseq_truck[t_sub] = ind->truck_seq[t + 1];
//                 subseq_truck[t_sub + 1] = ind->truck_seq[t];
//                 subseq_drone[d_sub] = ind->drone_seq[d + 1];
//                 subseq_drone[d_sub + 1] = ind->drone_seq[d];
//                 // compute new score
//                 double score = compute_score(inst, subseq_truck, subseq_drone, ind->drone_seq[d + 2]);
//                 if (score > 0 && score < min_score)
//                 {
//                     min_score = score;
//                     option = 4;
//                 }
//                 // restore subseqs
//                 subseq_truck[t_sub] = ind->truck_seq[t];
//                 subseq_truck[t_sub + 1] = ind->truck_seq[t + 1];
//                 subseq_drone[d_sub] = ind->drone_seq[d];
//                 subseq_drone[d_sub + 1] = ind->drone_seq[d + 1];
//             }
//         }
//         else
//         {
//             if (ind->truck_seq[t + 1] != ind->drone_seq[d + 2])
//             {
//                 // case 5: successive truck customer
//                 {
//                     subseq_truck[t_sub] = ind->truck_seq[t + 1];
//                     subseq_truck[t_sub + 1] = ind->truck_seq[t];
//                     subseq_drone[d_sub] = subseq_truck[t_sub];
//                     // compute new score
//                     double score = compute_score(inst, subseq_truck, subseq_drone, ind->drone_seq[d + 2]);
//                     if (score > 0 && score < min_score)
//                     {
//                         min_score = score;
//                         option = 5;
//                     }
//                     // restore subseqs
//                     subseq_truck[t_sub] = ind->truck_seq[t];
//                     subseq_truck[t_sub + 1] = ind->truck_seq[t + 1];
//                     subseq_drone[d_sub] = ind->drone_seq[d];
//                 }
//                 // case 6: successive drone customer
//                 {
//                     subseq_drone[d_sub] = ind->drone_seq[d + 1];
//                     subseq_drone[d_sub + 1] = ind->drone_seq[d];
//                     subseq_truck[t_sub] = subseq_drone[d_sub];
//                     // compute new score
//                     double score = compute_score(inst, subseq_truck, subseq_drone, ind->drone_seq[d + 2]);
//                     if (score > 0 && score < min_score)
//                     {
//                         min_score = score;
//                         option = 6;
//                     }
//                     // restore subseqs
//                     subseq_truck[t_sub] = ind->truck_seq[t];
//                     subseq_drone[d_sub] = ind->drone_seq[d];
//                     subseq_drone[d_sub + 1] = ind->drone_seq[d + 1];
//                 }
//             }
//         }

//         switch (option)
//         {
//         case 1:
//             n_moves++;
//             tot_delta += init_score - min_score;
//             ind->truck_seq[t] = subseq_truck[t_sub - 1];
//             ind->truck_seq[t - 1] = subseq_truck[t_sub];
//             ind->drone_seq[d] = subseq_drone[d_sub - 1];
//             ind->drone_seq[d - 1] = subseq_drone[d_sub];
//             break;
//         case 2:
//             n_moves++;
//             tot_delta += init_score - min_score;
//             ind->truck_seq[t] = subseq_truck[t_sub - 1];
//             ind->truck_seq[t - 1] = subseq_truck[t_sub];
//             ind->drone_seq[d] = ind->truck_seq[t];
//             break;

//         case 3:
//             n_moves++;
//             tot_delta += init_score - min_score;
//             ind->drone_seq[d] = subseq_drone[d_sub - 1];
//             ind->drone_seq[d - 1] = subseq_drone[d_sub];
//             ind->truck_seq[t] = ind->drone_seq[d];
//             break;
//         case 4:
//             n_moves++;
//             tot_delta += init_score - min_score;
//             ind->truck_seq[t] = subseq_truck[t_sub + 1];
//             ind->truck_seq[t + 1] = subseq_truck[t_sub];
//             ind->drone_seq[d] = subseq_drone[d_sub + 1];
//             ind->drone_seq[d + 1] = subseq_drone[d_sub];
//             break;
//         case 5:
//             n_moves++;
//             tot_delta += init_score - min_score;
//             ind->truck_seq[t] = subseq_truck[t_sub + 1];
//             ind->truck_seq[t + 1] = subseq_truck[t_sub];
//             ind->drone_seq[d] = ind->truck_seq[t];
//             break;
//         case 6:
//             n_moves++;
//             tot_delta += init_score - min_score;
//             ind->drone_seq[d] = subseq_drone[d_sub + 1];
//             ind->drone_seq[d + 1] = subseq_drone[d_sub];
//             ind->truck_seq[t] = ind->drone_seq[d];
//             break;
//         }

//         t_start = t;
//         d_start = d;
//         t = rendezvous_index;
//         d += 2;
//         free(subseq_truck);
//         free(subseq_drone);
//     }
//     if (n_moves)
//     {
//         get_precedence_seq(ind->truck_seq, ind->drone_seq, ind->dimension, ind->seq);
//         get_successors(ind->truck_succ, ind->drone_succ, ind->dimension, ind->truck_seq, ind->drone_seq, inst->dimension - 1);
//     }
//     return tot_delta;
// }


double swap_customers(instance *inst, individual *ind)
{
    // printf("*TRUCK SEQ:");
    // for (int j = 0; j < ind->dimension; j++)
    //     printf("%d,", ind->truck_seq[j]);
    // printf("\n*DRONE SEQ:");
    // for (int j = 0; j < ind->dimension; j++)
    //     printf("%d,", ind->drone_seq[j]);
    // printf("\n");
    double tot_delta = 0.0;
    int n_moves = 0;
    int t = 0; // truck seq index
    int d = 0; // drone seq index
    // truck and drone indices of the last customer visited both by the truck and the drone (combined customer or rendezvous node)
    int t_start = -1;
    int d_start = -1;
    while (ind->truck_seq[t] != inst->dimension - 1)
    {
        if (ind->truck_seq[t + 1] == ind->drone_seq[d + 1]) // combined edge
        {
            t_start = t;
            d_start = d;
            t++;
            d++;
            //printf("*combined edge\n");
            continue;
        }
        // operation found
        // compute the truck index of the rendezvous node
        int rendezvous_index = t + 1;
        while (ind->truck_seq[rendezvous_index] != ind->drone_seq[d + 2])
            rendezvous_index++;
        if (t == 0)
        {
            t_start = 0;
            d_start = 0;
            t = rendezvous_index;
            d += 2;
            continue;
        }

        int *subseq_truck_1 = (int *)malloc((rendezvous_index - t_start + 1) * sizeof(int));
        int *subseq_drone_1 = (int *)malloc((d + 2 - d_start + 1) * sizeof(int));
        memcpy(subseq_truck_1, ind->truck_seq + t_start, (rendezvous_index - t_start + 1) * sizeof(int));
        memcpy(subseq_drone_1, ind->drone_seq + d_start, (d + 2 - d_start + 1) * sizeof(int));

        // printf("t = %d, d = %d\n", t, d);
        // printf("t_start = %d, d_start = %d\n", t_start, d_start);
        // printf("TRUCK SUBSEQ:");
        // for (int j = 0; j < (rendezvous_index - t_start + 1); j++)
        //     printf("%d,", subseq_truck_1[j]);
        // printf("\nDRONE SUBSEQ:");
        // for (int j = 0; j < (d + 2 - d_start + 1); j++)
        //     printf("%d,", subseq_drone_1[j]);
        // printf("\n");
        // swap the current node with each one of its neighbors
        int t_sub = t - t_start;
        int d_sub = d - d_start;
        // compute the score of the original subsequence
        double init_score = compute_score(inst, subseq_truck_1, subseq_drone_1, ind->drone_seq[d + 2], 0);
        double min_score = init_score;
        int option = 0;

        // check if the solution cost can be improved by swapping the takeoff node with one of its neighbours  //

        if (ind->truck_seq[t - 1] == ind->drone_seq[d - 1])
        {
            if (t > 1)
            {
                // case 1: previous combined customer
                subseq_truck_1[t_sub] = ind->truck_seq[t - 1];
                subseq_truck_1[t_sub - 1] = ind->truck_seq[t];
                subseq_drone_1[d_sub] = ind->drone_seq[d - 1];
                subseq_drone_1[d_sub - 1] = ind->drone_seq[d];
                // compute new score
                double score = compute_score(inst, subseq_truck_1, subseq_drone_1, ind->drone_seq[d + 2], 0);
                if (score > 0 && score < min_score)
                {
                    min_score = score;
                    option = 1;
                }
                // restore subseqs
                subseq_truck_1[t_sub] = ind->truck_seq[t];
                subseq_truck_1[t_sub - 1] = ind->truck_seq[t - 1];
                subseq_drone_1[d_sub] = ind->drone_seq[d];
                subseq_drone_1[d_sub - 1] = ind->drone_seq[d - 1];
            }
        }
        else
        {
            if (ind->truck_seq[t - 1] != ind->drone_seq[d - 2])
            {
                // case 2: previous truck customer
                if (t > 1)
                {
                    subseq_truck_1[t_sub] = ind->truck_seq[t - 1];
                    subseq_truck_1[t_sub - 1] = ind->truck_seq[t];
                    subseq_drone_1[d_sub] = subseq_truck_1[t_sub];
                    // compute new score
                    double score = compute_score(inst, subseq_truck_1, subseq_drone_1, ind->drone_seq[d + 2], 0);
                    if (score > 0 && score < min_score)
                    {
                        min_score = score;
                        option = 2;
                    }
                    // restore subseqs
                    subseq_truck_1[t_sub] = ind->truck_seq[t];
                    subseq_truck_1[t_sub - 1] = ind->truck_seq[t - 1];
                    subseq_drone_1[d_sub] = ind->drone_seq[d];
                }
                // case 3: previous drone customer
                {
                    subseq_drone_1[d_sub] = ind->drone_seq[d - 1];
                    subseq_drone_1[d_sub - 1] = ind->drone_seq[d];
                    subseq_truck_1[t_sub] = subseq_drone_1[d_sub];
                    // compute new score
                    double score = compute_score(inst, subseq_truck_1, subseq_drone_1, ind->drone_seq[d + 2], 0);
                    if (score > 0 && score < min_score)
                    {
                        min_score = score;
                        option = 3;
                    }
                    // restore subseqs
                    subseq_truck_1[t_sub] = ind->truck_seq[t];
                    subseq_drone_1[d_sub] = ind->drone_seq[d];
                    subseq_drone_1[d_sub - 1] = ind->drone_seq[d - 1];
                }
            }
        }

        // if (ind->truck_seq[t + 1] == ind->drone_seq[d + 1])
        // {
        //     // case 4: successive combined customer
        //     if (ind->truck_seq[t + 1] != inst->dimension - 1)
        //     {
        //         subseq_truck_1[t_sub] = ind->truck_seq[t + 1];
        //         subseq_truck_1[t_sub + 1] = ind->truck_seq[t];
        //         subseq_drone_1[d_sub] = ind->drone_seq[d + 1];
        //         subseq_drone_1[d_sub + 1] = ind->drone_seq[d];
        //         // compute new score
        //         double score = compute_score(inst, subseq_truck_1, subseq_drone_1, ind->drone_seq[d + 2]);
        //         if (score > 0 && score < min_score)
        //         {
        //             min_score = score;
        //             option = 4;
        //         }
        //         // restore subseqs
        //         subseq_truck_1[t_sub] = ind->truck_seq[t];
        //         subseq_truck_1[t_sub + 1] = ind->truck_seq[t + 1];
        //         subseq_drone_1[d_sub] = ind->drone_seq[d];
        //         subseq_drone_1[d_sub + 1] = ind->drone_seq[d + 1];
        //     }
        // }
        // else
        // {
        if (ind->truck_seq[t + 1] != ind->drone_seq[d + 2])
        {
            // case 5: successive truck customer
            {
                subseq_truck_1[t_sub] = ind->truck_seq[t + 1];
                subseq_truck_1[t_sub + 1] = ind->truck_seq[t];
                subseq_drone_1[d_sub] = subseq_truck_1[t_sub];
                // compute new score
                double score = compute_score(inst, subseq_truck_1, subseq_drone_1, ind->drone_seq[d + 2], 0);
                if (score > 0 && score < min_score)
                {
                    min_score = score;
                    option = 5;
                }
                // restore subseqs
                subseq_truck_1[t_sub] = ind->truck_seq[t];
                subseq_truck_1[t_sub + 1] = ind->truck_seq[t + 1];
                subseq_drone_1[d_sub] = ind->drone_seq[d];
            }

        }
        //}

        // case 6: successive drone customer
        {
            subseq_drone_1[d_sub] = ind->drone_seq[d + 1];
            subseq_drone_1[d_sub + 1] = ind->drone_seq[d];
            subseq_truck_1[t_sub] = subseq_drone_1[d_sub];
            // compute new score
            double score = compute_score(inst, subseq_truck_1, subseq_drone_1, ind->drone_seq[d + 2], 0);
            if (score > 0 && score < min_score)
            {
                min_score = score;
                option = 6;
            }
            // restore subseqs
            subseq_truck_1[t_sub] = ind->truck_seq[t];
            subseq_drone_1[d_sub] = ind->drone_seq[d];
            subseq_drone_1[d_sub + 1] = ind->drone_seq[d + 1];
        }

        // check if the solution cost can be improved by swapping the rendezvous node with one of its neighbours  //
        int t_end = rendezvous_index; // truck index of the rendezvous node involved in the swap
        int d_end = d + 2;            // drone index of the rendezvous node involved in the swap

        int *subseq_truck_2 = (int *)malloc(ind->dimension * sizeof(int));
        int *subseq_drone_2 = (int *)malloc(ind->dimension * sizeof(int));

        double init_score2, min_score2;

        if (ind->truck_seq[rendezvous_index] != inst->dimension - 1)
        {
            t_end++;                                            // truck index of the last node involved in the swap
            d_end++;                                            // drone index of the last node involved in the swap
            if (ind->truck_seq[t_end] == ind->drone_seq[d_end]) // after the current operation there is a combined edge
            {
                if (ind->truck_seq[t_end] != inst->dimension - 1)
                {
                    t_end++;
                    d_end++;
                }
            }

            // check if from ind->truck_seq[t_end-1] it starts an operation
            // if after the current operation there is a combined customer, the swap between the rendezvous node and this node involves also the successive customer or operation
            if (ind->truck_seq[t_end] != ind->drone_seq[d_end]) // after the current operation there is another operation
            {
                d_end++;
                while (ind->truck_seq[t_end] != ind->drone_seq[d_end])
                {
                    t_end++;
                    if (t_end > ind->dimension - 1)
                        print_error("t_end > ind->dimension!");
                }
            }

            //subseq_truck_2 = (int *)realloc(subseq_truck_2, (t_end - t + 1) * sizeof(int));
            //subseq_drone_2 = (int *)realloc(subseq_drone_2, (d_end - d + 1) * sizeof(int));
            //subseq_truck_2 = (int *)malloc((t_end - t + 1) * sizeof(int));
            //subseq_drone_2 = (int *)malloc((d_end - d + 1) * sizeof(int));
            memcpy(subseq_truck_2, ind->truck_seq + t, (t_end - t + 1) * sizeof(int));
            memcpy(subseq_drone_2, ind->drone_seq + d, (d_end - d + 1) * sizeof(int));

            // printf("t = %d, d = %d\n", t, d);
            // printf("t_end = %d, d_end = %d\n", t_end, d_end);
            // printf("TRUCK SUBSEQ:");
            // for (int j = 0; j < t_end - t + 1; j++)
            //     printf("%d,", subseq_truck_2[j]);
            // printf("\nDRONE SUBSEQ:");
            // for (int j = 0; j < d_end - d + 1; j++)
            //     printf("%d,", subseq_drone_2[j]);
            // printf("\n");

            init_score2 = compute_score(inst, subseq_truck_2, subseq_drone_2, ind->truck_seq[t_end], 0);
            min_score2 = init_score2;

            if (ind->truck_seq[rendezvous_index + 1] == ind->drone_seq[d + 3])
            {
                // operation followed by a combined edge
                if (ind->truck_seq[rendezvous_index + 1] != inst->dimension - 1)
                {
                    // case 7: swap the rendezvous node with the combined customer after the rendezvous node
                    subseq_truck_2[rendezvous_index - t] = ind->truck_seq[rendezvous_index + 1];
                    subseq_truck_2[rendezvous_index + 1 - t] = ind->truck_seq[rendezvous_index];
                    subseq_drone_2[2] = ind->drone_seq[d + 3];
                    subseq_drone_2[3] = ind->drone_seq[d + 2];

                    // compute new score
                    double score = compute_score(inst, subseq_truck_2, subseq_drone_2, ind->truck_seq[t_end], 0);
                    if (score > 0 && score < min_score2)
                    {
                        min_score2 = score;
                        option = 7;
                    }
                    //printf("init_score2: %f -> score: %f\n", init_score2, score);
                    // restore subseqs
                    subseq_truck_2[rendezvous_index - t] = ind->truck_seq[rendezvous_index];
                    subseq_truck_2[rendezvous_index + 1 - t] = ind->truck_seq[rendezvous_index + 1];
                    subseq_drone_2[2] = ind->drone_seq[d + 2];
                    subseq_drone_2[3] = ind->drone_seq[d + 3];
                }
            }
            else // operation followed by an operation
            {

                // case 10: swap the rendezvous node with the drone customer of the successive operation
                {
                    subseq_truck_2[rendezvous_index - t] = ind->drone_seq[d + 3];
                    subseq_drone_2[2] = ind->drone_seq[d + 3];
                    subseq_drone_2[3] = ind->drone_seq[d + 2];
                    // compute new score
                    double score = compute_score(inst, subseq_truck_2, subseq_drone_2, ind->truck_seq[t_end], 0);
                    if (score > 0 && score < min_score2)
                    {
                        min_score2 = score;
                        option = 10;
                    }
                    // restore subseqs
                    subseq_truck_2[rendezvous_index - t] = ind->truck_seq[rendezvous_index];
                    subseq_drone_2[2] = ind->drone_seq[d + 2];
                    subseq_drone_2[3] = ind->drone_seq[d + 3];
                }
                // case 11: swap the rendezvous node with the first truck customer of the successive operation (if it exists)
                if (ind->truck_seq[rendezvous_index + 1] != ind->drone_seq[d + 4])
                {
                    subseq_truck_2[rendezvous_index - t] = ind->truck_seq[rendezvous_index + 1];
                    subseq_truck_2[rendezvous_index + 1 - t] = ind->truck_seq[rendezvous_index];
                    subseq_drone_2[2] = ind->truck_seq[rendezvous_index + 1];
                    // compute new score
                    double score = compute_score(inst, subseq_truck_2, subseq_drone_2, ind->truck_seq[t_end], 0);
                    if (score > 0 && score < min_score2)
                    {
                        min_score2 = score;
                        option = 11;
                    }
                    // restore subseqs
                    subseq_truck_2[rendezvous_index - t] = ind->truck_seq[rendezvous_index];
                    subseq_truck_2[rendezvous_index + 1 - t] = ind->truck_seq[rendezvous_index + 1];
                    subseq_drone_2[2] = ind->drone_seq[d + 2];
                }
            }
            // case 8: swap the rendezvous node with the drone customer of the current operation
            {
                subseq_truck_2[rendezvous_index - t] = ind->drone_seq[d + 1];
                subseq_drone_2[2] = ind->drone_seq[d + 1];
                subseq_drone_2[1] = ind->drone_seq[d + 2];
                // compute new score
                double score = compute_score(inst, subseq_truck_2, subseq_drone_2, ind->truck_seq[t_end], 0);
                if (score > 0 && score < min_score2)
                {
                    min_score2 = score;
                    option = 8;
                }
                // restore subseqs
                subseq_truck_2[rendezvous_index - t] = ind->truck_seq[rendezvous_index];
                subseq_drone_2[1] = ind->drone_seq[d + 1];
                subseq_drone_2[2] = ind->drone_seq[d + 2];
            }
            // case 9: swap the rendezvous node with the last truck customer of the current operation (if it exists)
            if (ind->truck_seq[rendezvous_index - 1] != ind->drone_seq[d])
            {
                subseq_truck_2[rendezvous_index - t] = ind->truck_seq[rendezvous_index - 1];
                subseq_truck_2[rendezvous_index - 1 - t] = ind->truck_seq[rendezvous_index];
                subseq_drone_2[2] = ind->truck_seq[rendezvous_index - 1];
                // compute new score
                double score = compute_score(inst, subseq_truck_2, subseq_drone_2, ind->truck_seq[t_end], 0);
                if (score > 0 && score < min_score2)
                {
                    min_score2 = score;
                    option = 9;
                }
                // restore subseqs
                subseq_truck_2[rendezvous_index - t] = ind->truck_seq[rendezvous_index];
                subseq_truck_2[rendezvous_index - 1 - t] = ind->truck_seq[rendezvous_index - 1];
                subseq_drone_2[2] = ind->drone_seq[d + 2];
            }
        }

        switch (option)
        {
        case 1:
            n_moves++;
            tot_delta += init_score - min_score;
            ind->truck_seq[t] = subseq_truck_1[t_sub - 1];
            ind->truck_seq[t - 1] = subseq_truck_1[t_sub];
            ind->drone_seq[d] = subseq_drone_1[d_sub - 1];
            ind->drone_seq[d - 1] = subseq_drone_1[d_sub];
            break;
        case 2:
            n_moves++;
            tot_delta += init_score - min_score;
            ind->truck_seq[t] = subseq_truck_1[t_sub - 1];
            ind->truck_seq[t - 1] = subseq_truck_1[t_sub];
            ind->drone_seq[d] = ind->truck_seq[t];
            break;

        case 3:
            n_moves++;
            tot_delta += init_score - min_score;
            ind->drone_seq[d] = subseq_drone_1[d_sub - 1];
            ind->drone_seq[d - 1] = subseq_drone_1[d_sub];
            ind->truck_seq[t] = ind->drone_seq[d];
            break;
        case 4:
            n_moves++;
            tot_delta += init_score - min_score;
            ind->truck_seq[t] = subseq_truck_1[t_sub + 1];
            ind->truck_seq[t + 1] = subseq_truck_1[t_sub];
            ind->drone_seq[d] = subseq_drone_1[d_sub + 1];
            ind->drone_seq[d + 1] = subseq_drone_1[d_sub];
            break;
        case 5:
            n_moves++;
            tot_delta += init_score - min_score;
            ind->truck_seq[t] = subseq_truck_1[t_sub + 1];
            ind->truck_seq[t + 1] = subseq_truck_1[t_sub];
            ind->drone_seq[d] = ind->truck_seq[t];
            break;
        case 6:
            n_moves++;
            tot_delta += init_score - min_score;
            ind->drone_seq[d] = subseq_drone_1[d_sub + 1];
            ind->drone_seq[d + 1] = subseq_drone_1[d_sub];
            ind->truck_seq[t] = ind->drone_seq[d];
            break;
        case 7:
            n_moves++;
            tot_delta += init_score2 - min_score2;
            ind->truck_seq[rendezvous_index] = subseq_truck_2[rendezvous_index + 1 - t];
            ind->truck_seq[rendezvous_index + 1] = subseq_truck_2[rendezvous_index - t];
            ind->drone_seq[d + 2] = subseq_drone_2[3];
            ind->drone_seq[d + 3] = subseq_drone_2[2];
            break;
        case 8:
            n_moves++;
            tot_delta += init_score2 - min_score2;
            ind->truck_seq[rendezvous_index] = subseq_drone_2[1];
            ind->drone_seq[d + 1] = subseq_drone_2[2];
            ind->drone_seq[d + 2] = subseq_drone_2[1];
            break;
        case 9:
            n_moves++;
            tot_delta += init_score2 - min_score2;
            ind->truck_seq[rendezvous_index - 1] = subseq_truck_2[rendezvous_index - t];
            ind->truck_seq[rendezvous_index] = subseq_truck_2[rendezvous_index - 1 - t];
            ind->drone_seq[d + 2] = subseq_truck_2[rendezvous_index - 1 - t];
            break;
        case 10:
            n_moves++;
            tot_delta += init_score2 - min_score2;
            ind->truck_seq[rendezvous_index] = subseq_drone_2[3];
            ind->drone_seq[d + 2] = subseq_drone_2[3];
            ind->drone_seq[d + 3] = subseq_drone_2[2];
            break;
        case 11:
            n_moves++;
            tot_delta += init_score2 - min_score2;
            ind->truck_seq[rendezvous_index] = subseq_truck_2[rendezvous_index + 1 - t];
            ind->truck_seq[rendezvous_index + 1] = subseq_truck_2[rendezvous_index - t];
            ind->drone_seq[d + 2] = subseq_truck_2[rendezvous_index + 1 - t];
            break;
        }

        t_start = t;
        d_start = d;
        t = rendezvous_index;
        d += 2;
        free(subseq_truck_1);
        free(subseq_drone_1);
        free(subseq_truck_2);
        free(subseq_drone_2);
    }
    if (n_moves)
    {
        get_precedence_seq(ind->truck_seq, ind->drone_seq, ind->dimension, ind->seq);
        get_successors(ind->truck_succ, ind->drone_succ, ind->dimension, ind->truck_seq, ind->drone_seq, inst->dimension - 1);
    }
    return tot_delta;
}


// optimize the subsequence input_seq[offset] ... input_seq[offset+dim_input_seq]
double partial_fstsp(instance * inst, int *input_seq, int offset, int dim_input_seq, int *opt_truck_seq, int *opt_drone_seq, int incl_depots)
{
    // check if dim_input_seq is feasible, otherwise set dim_input_seq to max feasible value
    if (offset + dim_input_seq > inst->dimension)
        dim_input_seq = inst->dimension - offset;

    int *seq;
    int dim_seq = dim_input_seq;
    // include initial and final depots if incl_depots = 1
    if (incl_depots)
    {
        int count = 0;
        if (offset > 0) // initial depot is not in the sequence input_seq
            count++;
        if (offset + dim_input_seq < inst->dimension) // final depot is not in the sequence input_seq
            count++;
        dim_seq += count;
    }

    seq = (int *)calloc(dim_seq, sizeof(int));

    // add the depots to the sequence if required
    if (dim_seq > dim_input_seq) // add in the sequence the depots
    {
        int j = 0;
        if (offset == 0)
            j++;
        seq[0] = 0;
        for (int i = 1; i < dim_seq - 1; i++)
        {
            seq[i] = input_seq[offset + j];
            j++;
        }
        seq[dim_seq - 1] = inst->dimension - 1;
    }
    else
    {
        for (int i = 0; i < dim_seq; i++)
            seq[i] = input_seq[offset + i];
    }

    // map between index and nodes
    int map[dim_seq];
    for (int i = 0; i < dim_seq; i++)
        map[i] = seq[i];

    // initialize the new instance
    instance new_inst;
    initialize_instance(&new_inst);
    new_inst.param.run = inst->param.run;
    new_inst.dimension = dim_seq;
    new_inst.nodes = (node_struct *)calloc(new_inst.dimension, sizeof(node_struct));

    // allocate memory for the the drone/truck structures
    allocate_mem_arrays(&new_inst);

    // copy nodes info
    for (int i = 0; i < new_inst.dimension; i++)
    {
        new_inst.nodes[i].id = inst->nodes[seq[i]].id;
        new_inst.nodes[i].x = inst->nodes[seq[i]].x;
        new_inst.nodes[i].y = inst->nodes[seq[i]].y;
        new_inst.nodes[i].weight = inst->nodes[seq[i]].weight;
    }

    // copy truck travel times for the new instance
    for (int i = 0; i < new_inst.dimension; i++)
    {
        for (int j = 0; j < new_inst.dimension; j++)
        {
            if (i == j)
                continue;
            new_inst.truck_times[i][j] = inst->truck_times[seq[i]][seq[j]];
            new_inst.truck_dists[i][j] = inst->truck_dists[seq[i]][seq[j]];
        }
    }

    // copy min max drone times for the new instance
    for (int i = 0; i < new_inst.dimension; i++)
    {
        for (int j = 0; j < new_inst.dimension; j++)
        {
            if (i == j)
                continue;
            for (int k = 0; k < new_inst.dimension; k++)
            {
                if (i == k || j == k)
                    continue;
                new_inst.min_feas_time_drone[i][j][k] = inst->min_feas_time_drone[seq[i]][seq[j]][seq[k]];
                new_inst.max_feas_time_drone[i][j][k] = inst->max_feas_time_drone[seq[i]][seq[j]][seq[k]];
            }
        }
    }
    // Solve the FSTSP-VDS for this subinstance
    optimal_solver(&new_inst, NULL, 1);
    // copy (and map) the optimal truck/drone sequences
    for (int i = 0; i < dim_seq; i++)
    {
        opt_truck_seq[i] = map[new_inst.truck_seq[i]];
        opt_drone_seq[i] = map[new_inst.drone_seq[i]];
    }
    free(seq);
    return new_inst.z_best;
}


/*int greedy_insertion(instance *inst, int end_value, int *truck_seq, int* drone_seq, int node){
    double extra_time = DBL_MAX;
    int i = 1;
    int best_i = i;
    // check the EXTRA TIME required to insert node [node] between node_t and truck_succ[node_t]
    while (truck_seq[i] != end_value) {
        double extra_tmp = inst->truck_times[truck_seq[i]][node] + inst->truck_times[node][truck_seq[i + 1]] - inst->truck_times[truck_seq[i]][truck_seq[i + 1]];
        if (extra_tmp < extra_time) {
            extra_time = extra_tmp;
            best_i = i;
        }
        i++;
        if (i == inst->dimension)
            print_error("[greedy_insertion] i==inst->dimension");
    }
    return best_i;
}*/

/*
double greedy_insertion(instance *inst, int end_value, int *truck_seq, int* drone_seq, int node) {
    if (truck_seq[0] != drone_seq[0])
        print_error("[greedy_insertion] truck_seq[0] != drone_seq[0]");
    double extra_time = DBL_MAX;
    int i = 1;
    int best_i = i;
    int j = 1; // drone index
    if ( truck_seq[i] != drone_seq[j])  // drone_seq[j] is drone customer
        j++;
    // INVARIANT: at the beginning of every iteration, drone_seq[j] is a combined customer
    int best_j = j;
    // check the EXTRA TIME required to insert node [node] between node_t and truck_succ[node_t]
    while (truck_seq[i] != end_value) {
        double extra_tmp = inst->truck_times[truck_seq[i - 1]][node] + inst->truck_times[node][truck_seq[i]] - inst->truck_times[truck_seq[i - 1]][truck_seq[i]];
        if (extra_tmp < extra_time) {
            extra_time = extra_tmp;
            best_i = i;
            best_j = j;
        }
        if ( truck_seq[i] == drone_seq[j]) {
            if ( truck_seq[i + 1] == drone_seq[j + 1]) // drone_seq[j] is combined customer
                j++;
            else //drone_seq[j] is drone customer
                j += 2;
        }
        i++;
        if (i == inst->dimension || j == inst->dimension)
            print_error("[greedy_insertion] i==inst->dimension");
    }
    // truck_seq[i] == end_value
    // drone_seq[j] == end_value
    // printf("*** [greedy_insertion] minimum extra_time: %f \n", extra_time);
    // printf("*** [greedy_insertion] best_i: %d, best_j: %d  \n", best_i, best_j);

    if (truck_seq[best_i - 1] == drone_seq[best_j - 1] && truck_seq[best_i] ==  drone_seq[best_j]) { // combined edge
        // shift also drone_seq
        //memmove(drone_seq + best_j + 1 , drone_seq + best_j, (inst->dimension - best_j - 1 ) * sizeof(int));
        memmove(drone_seq + best_j + 1 , drone_seq + best_j, (j - best_j + 1 ) * sizeof(int));
        drone_seq[best_j] = node;
    }
    //memmove(truck_seq + best_i + 1 , truck_seq + best_i, (inst->dimension - best_i - 1 ) * sizeof(int));
    memmove(truck_seq + best_i + 1 , truck_seq + best_i, (i - best_i + 1 ) * sizeof(int));
    truck_seq[best_i] = node;

    return extra_time;
}
*/


double greedy_insertion(instance *inst, int end_value, individual *offspring, int node) {
    int * truck_seq = offspring->truck_seq;
    int * drone_seq = offspring->drone_seq;

    if (truck_seq[0] != drone_seq[0])
        print_error("[greedy_insertion] truck_seq[0] != drone_seq[0]");
    double extra_t = DBL_MAX;

    int init_curr_t_leg = 0;
    int init_curr_d_leg = 0;

    int end_curr_t_leg = 0;
    int end_curr_d_leg = 0;

    int init_prev_t_op = -1;
    int init_prev_d_op = -1;

    int i = 0;
    int j = 0;

    int best_pos_i = -1;
    int best_pos_j = -1;
    int move_type = -1;

    double max_delta = DBL_MAX;

    while (truck_seq[i] != end_value) {
        if (truck_seq[i + 1] != drone_seq[j + 1]) { //drone leg found
            init_curr_t_leg = i;
            init_curr_d_leg = j;

            i++;
            j += 2;

            // check rendezvouz node in the truck seq
            while (truck_seq[i] != drone_seq[j]) {
                i++;
            }
            end_curr_t_leg = i;
            end_curr_d_leg = j;

            // printf("init_prev_t_op: %d, init_curr_t_leg: %d, end_curr_t_leg: %d\n", init_prev_t_op, init_curr_t_leg, end_curr_t_leg);
            // printf("init_prev_d_op: %d, init_curr_d_leg: %d, end_curr_d_leg: %d\n", init_prev_d_op, init_curr_d_leg, end_curr_d_leg);
            // printf("truck_seq: ");
            // for (int k = 0; k < inst->dimension; k++) {
            //     printf("%d, ", truck_seq[k]);
            // }
            // printf("\n");
            // printf("drone_seq: ");
            // for (int k = 0; k < inst->dimension; k++) {
            //     printf("%d, ", drone_seq[k]);
            // }
            // printf("\n");

            // save current operational time to perform the current operation
            double op_time = compute_score(inst, truck_seq + init_curr_t_leg, drone_seq + init_curr_d_leg, drone_seq[end_curr_d_leg], 0);


            // add [node] in the truck seq as truck-only customer in the most convenient position
            {
                int * tmp_t_seq = (int*) calloc(end_curr_t_leg - init_curr_t_leg + 2, sizeof(int));
                int * tmp_d_seq = (int*) calloc(end_curr_d_leg - init_curr_d_leg + 2, sizeof(int));
                // copy drone leg sequence
                memcpy(tmp_d_seq, drone_seq + init_curr_d_leg, (end_curr_d_leg - init_curr_d_leg + 1) * sizeof(int));
                // copy truck leg sequence
                memcpy(tmp_t_seq, truck_seq + init_curr_t_leg, (end_curr_t_leg - init_curr_t_leg + 1) * sizeof(int));

                // printf("*** op_time: %f\n", op_time);

                // printf("truck_seq: ");
                // for (int k = 0; k < end_curr_t_leg - init_curr_t_leg + 1; k++) {
                //     printf("%d, ", tmp_t_seq[k]);
                // }
                // printf("\n");


                double min_time = op_time;
                for (int pos = 1; pos < end_curr_t_leg - init_curr_t_leg + 1; pos++) {

                    // shift truck leg sequence, put in position pos the new node
                    memcpy(tmp_t_seq, truck_seq + init_curr_t_leg, (pos)* sizeof(int));
                    tmp_t_seq[pos] = node;
                    memcpy(tmp_t_seq + pos + 1, truck_seq + init_curr_t_leg + pos, (end_curr_t_leg - init_curr_t_leg + 1 - pos)* sizeof(int));

                    // printf("new_truck_seq: ");
                    // for (int k = 0; k < end_curr_t_leg - init_curr_t_leg + 2; k++) {
                    //     printf("%d, ", tmp_t_seq[k]);
                    // }

                    double tmp_time = compute_score(inst, tmp_t_seq, tmp_d_seq, drone_seq[end_curr_d_leg], 0);
                    if (tmp_time > 0.0f) {
                        // printf("*** tmp_time: %f\n", tmp_time);

                        if (tmp_time - op_time < extra_t) {
                            extra_t = tmp_time - op_time;
                            best_pos_i = init_curr_t_leg + pos;
                            move_type = 1;
                        }
                    }
                }
                free(tmp_t_seq);
                free(tmp_d_seq);

            }

            // add [node] as drone customer, put the drone customer in the truck sequence as truck-only customer in the most convenient position
            {
                int * tmp_t_seq = (int*) calloc(end_curr_t_leg - init_curr_t_leg + 2, sizeof(int));
                int * tmp_d_seq = (int*) calloc(end_curr_d_leg - init_curr_d_leg + 2, sizeof(int));
                // copy drone leg sequence
                memcpy(tmp_d_seq, drone_seq + init_curr_d_leg, (end_curr_d_leg - init_curr_d_leg + 1) * sizeof(int));
                // copy truck leg sequence
                memcpy(tmp_t_seq, truck_seq + init_curr_t_leg, (end_curr_t_leg - init_curr_t_leg + 1) * sizeof(int));

                // printf("original_truck_seq: ");
                // for (int k = 0; k < end_curr_t_leg - init_curr_t_leg + 2; k++) {
                //     printf("%d, ", tmp_t_seq[k]);
                // }
                // printf("\n");

                // printf("original_drone_seq: ");
                // for (int k = 0; k < end_curr_d_leg - init_curr_d_leg + 2; k++) {
                //     printf("%d, ", tmp_d_seq[k]);
                // }
                // printf("\n");

                // set [node] as the drone customer
                int original_drone_cust = drone_seq[init_curr_d_leg + 1];
                tmp_d_seq[1] = node;
                // check score for each position of [original_drone_cust] in the truck leg
                for (int pos = 1; pos < end_curr_t_leg - init_curr_t_leg + 1; pos++) {
                    // shift truck leg sequence, put in position pos the new node
                    memcpy(tmp_t_seq, truck_seq + init_curr_t_leg, (pos)* sizeof(int));
                    tmp_t_seq[pos] = original_drone_cust;
                    memcpy(tmp_t_seq + pos + 1, truck_seq + init_curr_t_leg + pos, (end_curr_t_leg - init_curr_t_leg + 1 - pos)* sizeof(int));


                    // printf("new_truck_seq: ");
                    // for (int k = 0; k < end_curr_t_leg - init_curr_t_leg + 2; k++) {
                    //     printf("%d, ", tmp_t_seq[k]);
                    // }
                    // printf("\n");

                    // printf("new_drone_seq: ");
                    // for (int k = 0; k < end_curr_d_leg - init_curr_d_leg + 2; k++) {
                    //     printf("%d, ", tmp_d_seq[k]);
                    // }
                    // printf("\n");


                    double tmp_time = compute_score(inst, tmp_t_seq, tmp_d_seq, drone_seq[end_curr_d_leg], 0);
                    if (tmp_time > 0.0f) {
                        if (tmp_time - op_time < extra_t) {
                            extra_t = tmp_time - op_time;
                            best_pos_i = init_curr_t_leg + pos;
                            best_pos_j = init_curr_d_leg + 1;
                            move_type = 2;
                        }
                    }
                }
                free(tmp_t_seq);
                free(tmp_d_seq);

            }

            if (init_prev_t_op != -1) {
                double seq_time = compute_score(inst, truck_seq + init_prev_t_op, drone_seq + init_prev_d_op, drone_seq[end_curr_d_leg], 0);

                // use [node] to build a combined edge that connect the last operation with the current one
                {
                    int * tmp_t_seq = (int*) calloc(end_curr_t_leg - init_prev_t_op + 2, sizeof(int));
                    int * tmp_d_seq = (int*) calloc(end_curr_d_leg - init_prev_d_op + 2, sizeof(int));
                    // modify truck leg sequence
                    memcpy(tmp_t_seq, truck_seq + init_prev_t_op, (init_curr_t_leg - init_prev_t_op) * sizeof(int));
                    tmp_t_seq[init_curr_t_leg - init_prev_t_op] = node;
                    memcpy(tmp_t_seq + (init_curr_t_leg - init_prev_t_op) + 1, truck_seq + init_curr_t_leg, (end_curr_t_leg - init_curr_t_leg + 1) * sizeof(int));
                    // modify drone leg sequence
                    memcpy(tmp_d_seq, drone_seq + init_prev_d_op, (init_curr_d_leg - init_prev_d_op) * sizeof(int));
                    tmp_d_seq[init_curr_d_leg - init_prev_d_op] = node;
                    memcpy(tmp_d_seq + (init_curr_d_leg - init_prev_d_op) + 1, drone_seq + init_curr_d_leg, (end_curr_d_leg - init_curr_d_leg + 1) * sizeof(int));




                    double tmp_time = compute_score(inst, tmp_t_seq, tmp_d_seq, drone_seq[end_curr_d_leg], 0);
                    if (tmp_time > 0.0f) {

                        if (tmp_time - seq_time < extra_t) {
                            extra_t = tmp_time - seq_time;
                            best_pos_i = init_curr_t_leg;
                            best_pos_j = init_curr_d_leg;
                            move_type = 3;
                        }
                    }

                    // check also in pos+1
                    tmp_t_seq[init_curr_t_leg - init_prev_t_op] = truck_seq[init_curr_t_leg];
                    tmp_t_seq[init_curr_t_leg - init_prev_t_op + 1] = node;
                    tmp_d_seq[init_curr_d_leg - init_prev_d_op] = drone_seq[init_curr_d_leg];
                    tmp_d_seq[init_curr_d_leg - init_prev_d_op + 1] = node;

                    tmp_time = compute_score(inst, tmp_t_seq, tmp_d_seq, drone_seq[end_curr_d_leg], 0);
                    if (tmp_time > 0.0f) {
                        if (tmp_time - seq_time < extra_t) {
                            extra_t = tmp_time - seq_time;
                            best_pos_i = init_curr_t_leg + 1;
                            best_pos_j = init_curr_d_leg + 1;
                            move_type = 3;
                        }
                    }
                    free(tmp_t_seq);
                    free(tmp_d_seq);
                }

                // set [node] as launch customer for the current operation
                {
                    int * tmp_t_seq = (int*) calloc(end_curr_t_leg - init_prev_t_op + 2, sizeof(int));
                    int * tmp_d_seq = (int*) calloc(end_curr_d_leg - init_prev_d_op + 1, sizeof(int));

                    // copy truck seq, set [node] as the current launch node
                    memcpy(tmp_t_seq, truck_seq + init_prev_t_op, (init_curr_t_leg - init_prev_t_op) * sizeof(int));
                    tmp_t_seq[init_curr_t_leg - init_prev_t_op] = node;
                    memcpy(tmp_t_seq + (init_curr_t_leg - init_prev_t_op) + 1, truck_seq + init_curr_t_leg, (end_curr_t_leg - init_curr_t_leg + 1) * sizeof(int));

                    // copy drone seq as it is
                    memcpy(tmp_d_seq, drone_seq + init_prev_d_op, (end_curr_d_leg - init_prev_d_op + 1) * sizeof(int));
                    tmp_d_seq[init_curr_d_leg - init_prev_d_op] = node;

                    double tmp_time = compute_score(inst, tmp_t_seq, tmp_d_seq, drone_seq[end_curr_d_leg], 0);
                    if (tmp_time > 0.0f) {
                        if (tmp_time - seq_time < extra_t) {
                            extra_t = tmp_time - seq_time;
                            best_pos_i = init_curr_t_leg;
                            best_pos_j = init_curr_d_leg;
                            move_type = 5;
                        }
                    }

                    free(tmp_t_seq);
                    free(tmp_d_seq);

                }

                // printf("original_truck_seq: ");
                // for (int k = 0; k < end_curr_t_leg - init_prev_t_op + 1; k++) {
                //     printf("%d, ", truck_seq[init_prev_t_op + k]);
                // }
                // printf("\n");

                // printf("original_drone_seq: ");
                // for (int k = 0; k < end_curr_d_leg - init_prev_d_op + 1; k++) {
                //     printf("%d, ", drone_seq[init_prev_d_op + k]);
                // }
                // printf("\n * adding node %d\n", node);

                // printf("new_truck_seq: ");
                // for (int k = 0; k < end_curr_t_leg - init_prev_t_op + 2; k++) {
                //     printf("%d, ", tmp_t_seq[k]);
                // }
                // printf("\n");

                // printf("new_drone_seq: ");
                // for (int k = 0; k < end_curr_d_leg - init_prev_d_op + 2; k++) {
                //     printf("%d, ", tmp_d_seq[k]);
                // }
                // printf("\n");

            }

            if (truck_seq[end_curr_t_leg] != end_value) {
                // set [node] as the rendezvouz node of the current operation
                int end_next_t_leg = end_curr_t_leg + 1;
                int end_next_d_leg = end_curr_d_leg + 1;

                if (truck_seq[end_next_t_leg] != drone_seq[end_next_d_leg]) {
                    // there is another operation after the current one
                    end_next_d_leg++;
                    while (truck_seq[end_next_t_leg] != drone_seq[end_next_d_leg]) {
                        if (end_next_t_leg == inst->dimension)
                            print_error("[greedy_insertion] end_next_t_leg == inst->dimension");
                        end_next_t_leg++;
                    }
                }
                int * tmp_t_seq = (int*) calloc(end_next_t_leg - init_curr_t_leg + 2, sizeof(int));
                int * tmp_d_seq = (int*) calloc(end_next_d_leg - init_curr_d_leg + 1, sizeof(int));

                // copy truck seq, set [node] as the current rendezvouz node
                memcpy(tmp_t_seq, truck_seq + init_curr_t_leg, (end_curr_t_leg - init_curr_t_leg + 1) * sizeof(int));
                tmp_t_seq[end_curr_t_leg + 1 - init_curr_t_leg] = node;
                memcpy(tmp_t_seq + (end_curr_t_leg - init_curr_t_leg) + 2, truck_seq + end_curr_t_leg + 1, (end_next_t_leg - end_curr_t_leg) * sizeof(int));

                // copy drone seq as it is
                memcpy(tmp_d_seq, drone_seq + init_curr_d_leg, (end_next_d_leg - init_curr_d_leg + 1) * sizeof(int));
                tmp_d_seq[end_curr_d_leg - init_curr_d_leg] = node;

                double tmp_time = compute_score(inst, tmp_t_seq, tmp_d_seq, drone_seq[end_next_d_leg], 0);
                if (tmp_time > 0.0f) {
                    double seq_time = compute_score(inst, truck_seq + init_curr_t_leg, drone_seq + init_curr_d_leg, drone_seq[end_next_d_leg], 0);
                    if (tmp_time - seq_time < extra_t) {
                        extra_t = tmp_time - seq_time;
                        best_pos_i = end_curr_t_leg + 1;
                        best_pos_j = end_curr_d_leg;
                        move_type = 5;
                    }
                }


                // printf("original_truck_seq: ");
                // for (int k = 0; k < end_next_t_leg - init_curr_t_leg + 1; k++) {
                //     printf("%d, ", truck_seq[init_curr_t_leg + k]);
                // }
                // printf("\n");

                // printf("original_drone_seq: ");
                // for (int k = 0; k < end_next_d_leg - init_curr_d_leg + 1; k++) {
                //     printf("%d, ", drone_seq[init_curr_d_leg + k]);
                // }
                // printf("\n * adding node %d\n", node);

                // printf("new_truck_seq: ");
                // for (int k = 0; k < end_next_t_leg - init_curr_t_leg + 2; k++) {
                //     printf("%d, ", tmp_t_seq[k]);
                // }
                // printf("\n");

                // printf("new_drone_seq: ");
                // for (int k = 0; k < end_next_d_leg - init_curr_d_leg + 1; k++) {
                //     printf("%d, ", tmp_d_seq[k]);
                // }
                // printf("\n");
                // print_error("*** i'm here\n");

            }


            init_prev_t_op = init_curr_t_leg;
            init_prev_d_op = init_curr_d_leg;

        }
        else { // combined edge

            init_prev_t_op = i;
            init_prev_d_op = j;

            i++;
            j++;
            double tmp_extra_time = inst->truck_times[truck_seq[i - 1]][node] + inst->truck_times[node][truck_seq[i]] - inst->truck_times[truck_seq[i - 1]][truck_seq[i]] ;
            if (tmp_extra_time < extra_t)
            {
                extra_t = tmp_extra_time;
                best_pos_i = i;
                best_pos_j = j;
                move_type = 0;
            }

            // DOES NOT PRODUCE GOOD RESULTS
            // build operation:
            // truck_seq: truck_seq[i - 1] -> truck_seq[i+k]
            // drone_seq: inst->drone_seq[j-1] --> node --> inst->drone_seq[j+k]

            double tmp_truck_time = 0.0f;
            for (int k = 0; truck_seq[i + k] != end_value; k++) {
                if (drone_seq[j + k] != truck_seq[i + k])
                    break;
                tmp_truck_time += inst->truck_times[truck_seq[i - 1 + k]][truck_seq[i + k]];
                int tmp_d_seq[3] = {drone_seq[j - 1], node, drone_seq[j + k]};
                double tmp_time = compute_score(inst, truck_seq + i - 1, tmp_d_seq, tmp_d_seq[2], 0);
                if (tmp_time > 0.0f) {
                    if (tmp_time - tmp_truck_time < extra_t)
                    {
                        extra_t = tmp_time - tmp_truck_time;
                        best_pos_j = j;
                        best_pos_i = k;
                        move_type = 4;
                    }
                }
            }



        }

    }
    int end_t_seq = i;
    int end_d_seq = j;

    // printf("best_pos: %d\n", best_pos);
    // printf("extra_time: %f\n", extra_time);

    switch (move_type) {
    case 0:
    {

        // printf("*** MOVE TYPE 0, node %d\n", node);

        memmove(truck_seq + best_pos_i + 1 , truck_seq + best_pos_i, (end_t_seq - best_pos_i + 1 ) * sizeof(int));
        memmove(drone_seq + best_pos_j + 1 , drone_seq + best_pos_j, (end_d_seq - best_pos_j + 1 ) * sizeof(int));
        truck_seq[best_pos_i] = node;
        drone_seq[best_pos_j] = node;


        // printf("new_truck_seq: ");
        // for (int k = 0; k <= end_t_seq + 1; k++) {
        //     printf("%d, ", truck_seq[k]);
        // }
        // printf("\n");
        // printf("new_drone_seq: ");
        // for (int k = 0; k <= end_d_seq + 1; k++) {
        //     printf("%d, ", drone_seq[k]);
        // }
        // printf("\n");


        //print_error("*** I'M HERE ***");
        break;

    }
    case 1:
    {
        memmove(truck_seq + best_pos_i + 1 , truck_seq + best_pos_i, (end_t_seq - best_pos_i + 1 ) * sizeof(int));
        truck_seq[best_pos_i] = node;
        break;
    }
    case 2:
    {
        memmove(truck_seq + best_pos_i + 1 , truck_seq + best_pos_i, (end_t_seq - best_pos_i + 1 ) * sizeof(int));
        truck_seq[best_pos_i] = drone_seq[best_pos_j];
        drone_seq[best_pos_j] = node;
        break;
    }
    case 3:
    {
        memmove(truck_seq + best_pos_i + 1 , truck_seq + best_pos_i, (end_t_seq - best_pos_i + 1 ) * sizeof(int));
        memmove(drone_seq + best_pos_j + 1 , drone_seq + best_pos_j, (end_d_seq - best_pos_j + 1 ) * sizeof(int));
        truck_seq[best_pos_i] = node;
        drone_seq[best_pos_j] = node;

        // printf("*** MOVE 3 ***\n");
        // printf("new_truck_seq: ");
        // for (int k = 0; k <= end_t_seq + 1; k++) {
        //     printf("%d, ", truck_seq[k]);
        // }
        // printf("\n");
        // printf("new_drone_seq: ");
        // for (int k = 0; k <= end_d_seq + 1; k++) {
        //     printf("%d, ", drone_seq[k]);
        // }
        // printf("\n");


        // print_error("*** I'M HERE ***");
        break;
    }
    case 4:
    {
        int k = best_pos_i;
        memmove(drone_seq + best_pos_j + 1, drone_seq + best_pos_j + k, (end_d_seq - best_pos_j - k + 1 ) * sizeof(int));
        drone_seq[best_pos_j] = node;
        break;
    }
    case 5: {
        memmove(truck_seq + best_pos_i + 1 , truck_seq + best_pos_i, (end_t_seq - best_pos_i + 1 ) * sizeof(int));
        truck_seq[best_pos_i] = node;
        drone_seq[best_pos_j] = node;
        break;
    }
    default:
        //  didn't find any valid insertion
        return -(offspring->makespan + 1.0f);
        break;
    }

    // end switch


    //print_error("*** I'M HERE \n");
    return extra_t;
}

