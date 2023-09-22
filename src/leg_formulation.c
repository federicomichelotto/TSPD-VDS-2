#include <cplex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <float.h>
#include <limits.h>

#include "../include/utils.h"
#include "../include/min_max_opt.h"
#include "../include/leg_formulation.h"

#define FLAG_REDUCED_TRUCK_LEGS 0

int number_unique_truck_legs = 0;

// retrieve the position of the decision var. x(i,j) in the cplex vector
int x_pos_leg(int i, int j, instance *inst)
{
    if (i < 0 || j < 0)
        print_error("Error in x_pos_leg: negative indexes are not valid!");
    if (i >= inst->dimension || j >= inst->dimension)
        print_error("Error in x_pos_leg: indexes exceeding the dimension are not valid!");
    return i * inst->dimension + j;
}

// retrieve the position of the decision var. y(l) in the cplex vector
int y_pos_leg(int l, instance *inst)
{
    if (l < 0 || l >= inst->number_feasible_truck_legs)
        print_error("Error in y_pos_leg: negative indexes are not valid!");
    return x_pos_leg(inst->dimension - 1, inst->dimension - 1, inst) + 1 + l;
}

// retrieve the position of the decision var. z(i,j,k) in the cplex vector
int z_pos_leg(int i, int j, int k, instance *inst)
{
    if (i < 0 || j < 0 || k < 0)
        print_error("Error in z_pos_leg: negative indexes are not valid!");
    if (i >= inst->dimension || j >= inst->dimension || k >= inst->dimension)
        print_error("Error in z_pos_leg: indexes exceeding the dimension are not valid!");
    return y_pos_leg(inst->number_feasible_truck_legs - 1, inst) + 1 + i * inst->dimension * inst->dimension + j * inst->dimension + k;
}

// retrieve the position of the decision var. w(k) in the cplex vector
int w_pos_leg(int k, instance *inst)
{
    if (k < 0 || k >= inst->dimension)
        print_error("Error in w_pos_leg: negative indexes are not valid!");
    return z_pos_leg(inst->dimension - 1, inst->dimension - 1, inst->dimension - 1, inst) + 1 + k;
}

// retrieve the position of the decision var. u(i) in the cplex vector
int u_pos_leg(int i, instance *inst)
{
    if (i < 0 || i >= inst->dimension)
        print_error("Error in u_pos_leg: negative indexes are not valid!");
    return w_pos_leg(inst->dimension - 1, inst) + 1 + i;
}

// retrieve the position of the decision var. u(i) in the cplex vector
int b_pos_leg(int i, instance *inst)
{
    if (i < 0 || i >= inst->dimension)
        print_error("Error in b_pos_leg: negative indexes are not valid!");
    return u_pos_leg(inst->dimension - 1, inst) + 1 + i;
}


void build_leg_based_model(CPXENVptr env, CPXLPptr lp, instance *inst, double min_cost_TSP)
{
    char binary = 'B';     // B => binary variable flag
    char integer = 'I';    // I => integer variable flag
    char continuous = 'C'; // C => continuous variable flag

    // cname: columns' names (column = variable)
    char **cname = (char **)calloc(1, sizeof(char *)); // array of strings to store the column names
    cname[0] = (char *)calloc(100, sizeof(char));

    // rname: rows' names (row = constraint)
    char **rname = (char **)calloc(1, sizeof(char *)); // array of strings to store the row names
    rname[0] = (char *)calloc(100, sizeof(char));

    // ************************ VARIABLES ************************ //

    // Add a binary variable 'x' for each edge (i,j)
    // x(i,j) = 1 if edge (i,j) is selected as combined edge
    for (int i = 0; i < inst->dimension; i++)
    {
        for (int j = 0; j < inst->dimension; j++)
        {
            sprintf(cname[0], "x(%d,%d)", i, j);
            double obj = inst->truck_times[i][j];
            double lb = 0.0;
            double ub = 1.0;
            if (i == j)
                ub = 0.0;
            if (i == inst->dimension - 1 || j == 0)
                ub = 0.0;
            if (inst->truck_times[i][j] == DBL_MAX) {
                ub = 0.0;
            }

            if (CPXnewcols(env, lp, 1, &obj, &lb, &ub, &binary, cname))
                print_error("wrong CPXnewcols on x variables");
            if (CPXgetnumcols(env, lp) - 1 != x_pos_leg(i, j, inst))
                print_error("wrong position for x variables");
        }
    }


    // Add a binary variable 'y' for each feasible truck leg l
    // y(l) = 1 if drone leg l is selected
    {
        for (int i = 0; i < inst->dimension - 1; i++) {
            for (int k = 1; k < inst->dimension; k++) {
                for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                    printf("[%d][%d]{%.2f} leg(%d) ", i, k, inst->feasible_truck_legs2[i][k].legs[leg_id].time, inst->feasible_truck_legs2[i][k].legs[leg_id].ID);
                    int used_customers[inst->dimension];
                    for (int n = 0; n < inst->dimension; n++)
                        used_customers[n] = 0;
                    for (int c = 0; c < inst->feasible_truck_legs2[i][k].legs[leg_id].length; c++) {
                        printf("%d ", inst->feasible_truck_legs2[i][k].legs[leg_id].leg[c]);
                        used_customers[inst->feasible_truck_legs2[i][k].legs[leg_id].leg[c]] = 1;
                    }
                    printf("\n");

                    sprintf(cname[0], "y(%d)", inst->feasible_truck_legs2[i][k].legs[leg_id].ID);
                    double obj = inst->feasible_truck_legs2[i][k].legs[leg_id].time;
                    double lb = 0.0;
                    double ub = 1.0;

                    if (inst->feasible_truck_legs2[i][k].legs[leg_id].flag_feasible_drone_leg_exist == 0)
                        ub = 0.0;

                    // check if there is a better truck leg that serves the same set of customers
                    if (ub > 0.99) {
                        for (int leg_id2 = 0; leg_id2 < inst->feasible_truck_legs2[i][k].count; leg_id2++) {
                            if (leg_id2 == leg_id) continue;
                            if (inst->feasible_truck_legs2[i][k].legs[leg_id2].length != inst->feasible_truck_legs2[i][k].legs[leg_id].length)
                                continue;
                            if (inst->feasible_truck_legs2[i][k].legs[leg_id2].flag_feasible_drone_leg_exist == 0)
                                continue;
                            int count_equal = 0;
                            for (int c = 0; c < inst->feasible_truck_legs2[i][k].legs[leg_id2].length; c++) {
                                if (used_customers[inst->feasible_truck_legs2[i][k].legs[leg_id2].leg[c]] == 1) {
                                    count_equal++;
                                }
                            }
                            if (count_equal != inst->feasible_truck_legs2[i][k].legs[leg_id].length)
                                continue;
                            if (inst->feasible_truck_legs2[i][k].legs[leg_id2].time < inst->feasible_truck_legs2[i][k].legs[leg_id].time - 1e-6) {
                                ub = 0.0;
                                break;
                            }
                            // in case of parity, prefer the truck legs with the lower ID
                            if (leg_id2 > leg_id && fabs(inst->feasible_truck_legs2[i][k].legs[leg_id2].time - inst->feasible_truck_legs2[i][k].legs[leg_id].time) < 1e-6) {
                                ub = 0.0;
                                break;
                            }
                        }
                    }
                    if (ub > 0.99) {
                        inst->feasible_truck_legs2[i][k].legs[leg_id].flag_active = 1;
                        number_unique_truck_legs++;
                    }
                    if (CPXnewcols(env, lp, 1, &obj, &lb, &ub, &binary, cname))
                        print_error("wrong CPXnewcols on y variables");
                    if (CPXgetnumcols(env, lp) - 1 != y_pos_leg(inst->feasible_truck_legs2[i][k].legs[leg_id].ID, inst))
                        print_error("wrong position for y variables");
                }
            }
        }
    }

    // Add a binary variable 'z' for drone leg (i,j,k)
    // z(i,j,k) = 1 if drone leg i-->j-->k is selected
    for (int i = 0; i < inst->dimension; i++)
    {
        for (int j = 0; j < inst->dimension; j++)
        {
            for (int k = 0; k < inst->dimension; k++)
            {
                sprintf(cname[0], "z(%d,%d,%d)", i, j, k);
                double obj = 2.0;
                if (inst->param.MHD == 0)
                    obj = 0.0;
                if (i == 0)
                    obj = 1.0;
                double lb = 0.0;
                double ub = 1.0;
                if (i == k)
                    ub = 0.0;
                if (j == i || j == k) // the intermediate node must be different from the starting and ending node
                    ub = 0.0;
                if (j == 0 || j == inst->dimension - 1) // the intermediate node cannot be a depot
                    ub = 0.0;
                if (inst->min_time_drone[i][j][k] == DBL_MAX) // drone leg i-->j-->k is not available
                    ub = 0.0;
                if (i == 0 && k == inst->dimension - 1)
                    ub = 0.0;

                if (CPXnewcols(env, lp, 1, &obj, &lb, &ub, &binary, cname))
                    print_error("wrong CPXnewcols on z variables");
                if (CPXgetnumcols(env, lp) - 1 != z_pos_leg(i, j, k, inst))
                    print_error("wrong position for z variables");
            }
        }
    }

    // Add a real variable 'w' for each node k
    // w(k) = waiting time spent at node k
    for (int k = 0; k < inst->dimension; k++) {
        sprintf(cname[0], "w(%d)", k);
        double obj = 1.0;
        double lb = 0.0;
        double ub = min_cost_TSP;
        if (k == 0)
            ub = 0.0;
        if (CPXnewcols(env, lp, 1, &obj, &lb, &ub, &continuous, cname))
            print_error("wrong CPXnewcols on w variables");
        if (CPXgetnumcols(env, lp) - 1 != w_pos_leg(k, inst))
            print_error("wrong position for w variables");
    }

    // Add an integer variable 'u' for each node i
    // u(i) = visit order of node i in the solution
    for (int i = 0; i < inst->dimension; i++) {
        sprintf(cname[0], "u(%d)", i);
        double obj = 0.0;
        double lb = 1;
        double ub = inst->dimension - 2;
        if (i == 0) {
            lb = 0;
            ub = 0;
        }
        if (i == inst->dimension - 1) {
            // lb = 1; //1 OR inst->dimension - 1
            // ub = inst->dimension - 1;
            lb = 0; //1 OR inst->dimension - 1
            ub = 0;
        }
        if (CPXnewcols(env, lp, 1, &obj, &lb, &ub, &integer, cname))
            print_error("wrong CPXnewcols on u variables");
        if (CPXgetnumcols(env, lp) - 1 != u_pos_leg(i, inst))
            print_error("wrong position for u variables");
    }

    // Add an integer variable 'b' (budget) for each node i
    // b(i) = budget node i, when i is a drone customer
    // for (int i = 0; i < inst->dimension; i++) {
    //     sprintf(cname[0], "b(%d)", i);
    //     double obj = 0.0;
    //     double lb = 0.0;
    //     double ub = 1.0;
    //     // if (i == 0 || i < inst->dimension || inst->nodes[i].truck_only) {
    //     //     ub = 0.0;
    //     // }
    //     if (i == 8)
    //         ub = 0.0;
    //     if (CPXnewcols(env, lp, 1, &obj, &lb, &ub, &continuous, cname))
    //         print_error("wrong CPXnewcols on b variables");
    //     if (CPXgetnumcols(env, lp) - 1 != b_pos_leg(i, inst))
    //         print_error("wrong position for b variables");
    // }

    // ************************ CONSTRAINTS ************************ //

    // truck only customers
    for (int j = 1; j < inst->dimension - 1; j++)
    {
        if (inst->nodes[j].truck_only == 0) continue;
        int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
        double rhs = 0.0;
        char sense = 'E';
        sprintf(rname[0], "truck_only(%d)", j);
        if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
            print_error("wrong CPXnewrows [truck_only]");
        for (int i = 0; i < inst->dimension - 1; i++) {
            for (int k = 1; k < inst->dimension; k++) {
                if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
                    print_error("wrong CPXchgcoef [truck_only_z]");
            }
        }
    }

    // (2b) each customer is served exactly once
    for (int j = 1; j < inst->dimension - 1; j++)
    {
        int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
        double rhs = 1.0;
        char sense = 'E';
        sprintf(rname[0], "serve(%d)", j);
        if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
            print_error("wrong CPXnewrows [serve_j]");
        for (int i = 0; i < inst->dimension - 1; i++) {
            if (i == j) continue;
            if (CPXchgcoef(env, lp, row, x_pos_leg(i, j, inst), 1.0))
                print_error("wrong CPXchgcoef [serve_x_ij]");
        }
        for (int i = 0; i < inst->dimension - 1; i++) {
            if (i == j) continue;
            for (int k = 1; k < inst->dimension; k++) {
                if (k == i || k == j) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
                    print_error("wrong CPXchgcoef [serve_z_ijk]");
                if (CPXchgcoef(env, lp, row, z_pos_leg(i, k, j, inst), 1.0))
                    print_error("wrong CPXchgcoef [serve_z_ikj]");
            }
        }
        for (int i = 0; i < inst->dimension - 1; i++) {
            if (i == j) continue;
            for (int k = 1; k < inst->dimension; k++) {
                if (k == i) continue;
                for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                    const int* truck_leg = inst->feasible_truck_legs2[i][k].legs[leg_id].leg;
                    for (int cust_id = 1; cust_id < inst->feasible_truck_legs2[i][k].legs[leg_id].length - 1; cust_id++) {
                        if (truck_leg[cust_id] == j) {
                            if (CPXchgcoef(env, lp, row, y_pos_leg(inst->feasible_truck_legs2[i][k].legs[leg_id].ID, inst), 1.0))
                                print_error("wrong CPXchgcoef [serve_j_y_l]");
                            break;
                        }
                    }
                    // if (truck_leg[inst->feasible_truck_legs2[i][k].legs[leg_id].length - 1] == j) {
                    //     if (CPXchgcoef(env, lp, row, y_pos_leg(inst->feasible_truck_legs2[i][k].legs[leg_id].ID, inst), 1.0))
                    //         print_error("wrong CPXchgcoef [serve_j_y_l]");
                    // }
                }
            }
        }
    }


    // set the number of nodes to serve
    // {
    //     int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
    //     double rhs = inst->dimension - 2;
    //     char sense = 'G';
    //     sprintf(rname[0], "serve");
    //     if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
    //         print_error("wrong CPXnewrows [serve]");
    //     for (int i = 0; i < inst->dimension - 1; i++) {
    //         for (int j = 1; j < inst->dimension; j++) {
    //             if (i == j) continue;
    //             if (CPXchgcoef(env, lp, row, x_pos_leg(i, j, inst), 1.0))
    //                 print_error("wrong CPXchgcoef [serve]");
    //         }
    //     }
    //     for (int i = 0; i < inst->dimension - 1; i++) {
    //         for (int j = 1; j < inst->dimension - 1; j++) {
    //             if (i == j) continue;
    //             for (int k = 1; k < inst->dimension; k++) {
    //                 if (k == i || k == j) continue;
    //                 if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 2.0))
    //                     print_error("wrong CPXchgcoef [serve]");
    //             }
    //         }
    //     }
    //     for (int i = 0; i < inst->dimension - 1; i++) {
    //         for (int k = 1; k < inst->dimension; k++) {
    //             if (k == i) continue;
    //             for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
    //                 int l = inst->feasible_truck_legs2[i][k].legs[leg_id].ID;
    //                 int new_coef = inst->feasible_truck_legs2[i][k].legs[leg_id].length - 2;
    //                 if (CPXchgcoef(env, lp, row, y_pos_leg(l, inst), new_coef))
    //                     print_error("wrong CPXchgcoef [serve_j_y_l]");
    //             }
    //         }
    //     }
    // }

// Dk inequalities
    for (int i = 1; i < inst->dimension - 1; i++) {
        for (int j = 1; j < inst->dimension - 1; j++) {
            if (j == i) continue;
            for (int k = 1; k < inst->dimension - 1; k++) {
                if (i == k || j == k) continue;
                int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
                double rhs = 2.0;
                char sense = 'L';
                sprintf(rname[0], "Dk_ineq(%d,%d,%d)", i, j, k);
                if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                    print_error("wrong CPXnewrows [Dk_ineq]");
                if (CPXchgcoef(env, lp, row, x_pos_leg(i, j, inst), 2.0))
                    print_error("wrong CPXchgcoef [Dk_ineq]");
                if (CPXchgcoef(env, lp, row, x_pos_leg(k, j, inst), 1.0))
                    print_error("wrong CPXchgcoef [Dk_ineq]");
                if (CPXchgcoef(env, lp, row, x_pos_leg(j, i, inst), 1.0))
                    print_error("wrong CPXchgcoef [Dk_ineq]");
                if (CPXchgcoef(env, lp, row, x_pos_leg(i, k, inst), 1.0))
                    print_error("wrong CPXchgcoef [Dk_ineq]");

                if (CPXchgcoef(env, lp, row, z_pos_leg(k, j, i, inst), 1.0))
                    print_error("wrong CPXchgcoef [Dk_ineq]");

                for (int a = 0; a < inst->dimension - 1; a++) {
                    for (int b = 1; b < inst->dimension; b++) {
                        for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[a][b].count; leg_id++) {
                            const int* truck_leg = inst->feasible_truck_legs2[a][b].legs[leg_id].leg;
                            double coeff = 0.0;
                            for (int cust_id = 0; cust_id < inst->feasible_truck_legs2[a][b].legs[leg_id].length - 1; cust_id++) {
                                if ((truck_leg[cust_id] == j && truck_leg[cust_id + 1] == i)) {
                                    coeff += 1.0;
                                    break;
                                }
                            }
                            if (coeff > 0) {
                                if (CPXchgcoef(env, lp, row, y_pos_leg(inst->feasible_truck_legs2[a][b].legs[leg_id].ID, inst), coeff))
                                    print_error("wrong CPXchgcoef [Dk_ineq]");
                            }
                        }
                    }
                }

            }
        }
    }

// (2c) link the truck legs with the drone legs
    for (int i = 0; i < inst->dimension - 1; i++) {
        for (int k = 1; k < inst->dimension; k++) {
            if (i == k) continue;
            int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
            double rhs = 0.0;
            char sense = 'E';
            sprintf(rname[0], "link_y_z(%d,%d)", i, k);
            if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                print_error("wrong CPXnewrows [link_y_z]");
            for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                if (CPXchgcoef(env, lp, row, y_pos_leg(inst->feasible_truck_legs2[i][k].legs[leg_id].ID, inst), 1.0))
                    print_error("wrong CPXchgcoef [link_y]");
            }
            for (int j = 0; j < inst->dimension; j++) {
                if (j == i || j == k) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), -1.0))
                    print_error("wrong CPXchgcoef [link_z]");
                // if (CPXchgcoef(env, lp, row, z_pos_leg(k, j, i, inst), -1.0))
                //     print_error("wrong CPXchgcoef [link_z]");

                // if (CPXchgcoef(env, lp, row, z_pos_leg(i, k, j, inst), -1.0))
                //     print_error("wrong CPXchgcoef [link_z]");
                // if (CPXchgcoef(env, lp, row, z_pos_leg(k, i, j, inst), -1.0))
                //     print_error("wrong CPXchgcoef [link_z]");

                // if (CPXchgcoef(env, lp, row, z_pos_leg(j, i, k, inst), -1.0))
                //     print_error("wrong CPXchgcoef [link_z]");
                // if (CPXchgcoef(env, lp, row, z_pos_leg(j, k, i, inst), -1.0))
                //     print_error("wrong CPXchgcoef [link_z]");
            }
        }
    }

  /*  for (int i = 0; i < inst->dimension - 1; i++) {
        for (int k = 1; k < inst->dimension; k++) {
            if (i == k) continue;
            int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
            double rhs = 1.0;
            char sense = 'L';
            sprintf(rname[0], "z_max(%d,%d)", i, k);
            if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                print_error("wrong CPXnewrows [z_max]");
            if (CPXchgcoef(env, lp, row, x_pos_leg(i, k, inst), 1.0))
                print_error("wrong CPXchgcoef [z_max]");
            if (CPXchgcoef(env, lp, row, x_pos_leg(k, i, inst), 1.0))
                print_error("wrong CPXchgcoef [z_max]");
            for (int j = 0; j < inst->dimension; j++) {
                if (j == i || j == k) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
                    print_error("wrong CPXchgcoef [z_max]");
                if (CPXchgcoef(env, lp, row, z_pos_leg(k, j, i, inst), 1.0))
                    print_error("wrong CPXchgcoef [z_max]");

                if (CPXchgcoef(env, lp, row, z_pos_leg(i, k, j, inst), 1.0))
                    print_error("wrong CPXchgcoef [z_max]");
                if (CPXchgcoef(env, lp, row, z_pos_leg(k, i, j, inst), 1.0))
                    print_error("wrong CPXchgcoef [z_max]");

                if (CPXchgcoef(env, lp, row, z_pos_leg(j, i, k, inst), 1.0))
                    print_error("wrong CPXchgcoef [z_max]");
                if (CPXchgcoef(env, lp, row, z_pos_leg(j, k, i, inst), 1.0))
                    print_error("wrong CPXchgcoef [z_max]");
            }
        }
    }*/

// (2d_1) ensure that the drone leaves the depot exactly once
    {
        int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
        double rhs = 1.0;
        char sense = 'E';
        sprintf(rname[0], "outdegree_initial_depot");
        if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
            print_error("wrong CPXnewrows [outdegree_initial_depot]");
        for (int j = 1; j < inst->dimension - 1; j++) {
            if (CPXchgcoef(env, lp, row, x_pos_leg(0, j, inst), 1.0))
                print_error("wrong CPXchgcoef [outdegree_initial_depot_x]");
        }
        for (int j = 1; j < inst->dimension - 1; j++) {
            for (int k = 1; k < inst->dimension; k++) {
                if (j == k) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(0, j, k, inst), 1.0))
                    print_error("wrong CPXchgcoef [outdegree_initial_depot_z]");
            }
        }
    }

// (2d_2) ensure that the drone returns to the depot exactly once
    {
        int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
        double rhs = 1.0;
        char sense = 'E';
        sprintf(rname[0], "indegree_final_depot");
        if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
            print_error("wrong CPXnewrows [indegree_final_depot]");
        for (int i = 0; i < inst->dimension - 1; i++) {
            if (CPXchgcoef(env, lp, row, x_pos_leg(i, inst->dimension - 1, inst), 1.0))
                print_error("wrong CPXchgcoef [indegree_final_depot_x]");
        }
        for (int i = 0; i < inst->dimension - 1; i++) {
            for (int j = 1; j < inst->dimension - 1; j++) {
                if (i == j) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, inst->dimension - 1, inst), 1.0))
                    print_error("wrong CPXchgcoef [indegree_final_depot_z]");
            }
        }
    }

// (2e) flow-conservation constraints for the drone

    for (int j = 1; j < inst->dimension - 1; j++) {
        int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
        double rhs = 0.0;
        char sense = 'E';
        sprintf(rname[0], "drone_flow_conservation(%d)", j);
        if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
            print_error("wrong CPXnewrows [drone_flow_conservation]");
        for (int i = 0; i < inst->dimension - 1; i++) {
            if (i == j) continue;
            if (CPXchgcoef(env, lp, row, x_pos_leg(i, j, inst), 1.0))
                print_error("wrong CPXchgcoef [drone_flow_conservation_x_lhs]");
            for (int w = 1; w < inst->dimension - 1; w++) {
                if (w == i || w == j) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(i, w, j, inst), 1.0))
                    print_error("wrong CPXchgcoef [drone_flow_conservation_z_lhs]");
            }
        }
        for (int k = 1; k < inst->dimension; k++) {
            if (k == j) continue;
            if (CPXchgcoef(env, lp, row, x_pos_leg(j, k, inst), -1.0))
                print_error("wrong CPXchgcoef [drone_flow_conservation_x_rhs]");
            for (int w = 1; w < inst->dimension - 1; w++) {
                if (w == j || w == k) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(j, w, k, inst), -1.0))
                    print_error("wrong CPXchgcoef [drone_flow_conservation_z_rhs]");
            }
        }
    }

// (2e+) max in flow

    // for (int j = 1; j < inst->dimension - 1; j++) {
    //     int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
    //     double rhs = 1.0;
    //     char sense = 'L';
    //     sprintf(rname[0], "max_in_flow(%d)", j);
    //     if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
    //         print_error("wrong CPXnewrows [max_in_flow]");
    //     for (int i = 0; i < inst->dimension - 1; i++) {
    //         if (i == j) continue;
    //         if (CPXchgcoef(env, lp, row, x_pos_leg(i, j, inst), 1.0))
    //             print_error("wrong CPXchgcoef [max_in_flow]");
    //         for (int w = 1; w < inst->dimension - 1; w++) {
    //             if (w == i || w == j) continue;
    //             if (CPXchgcoef(env, lp, row, z_pos_leg(i, w, j, inst), 1.0))
    //                 print_error("wrong CPXchgcoef [max_in_flow]");
    //             if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, w, inst), 1.0))
    //                 print_error("wrong CPXchgcoef [max_in_flow]");
    //         }
    //     }
    // }


// (2f) Subtour Elimination Constraints (SEC) in Miller Tucker Zemlin form
// for (int i = 0; i < inst->dimension - 1; i++) {
//     for (int k = 1; k < inst->dimension - 1; k++) {
//         if (i == k) continue;
//         int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
//         double rhs = -inst->dimension + 2;
//         char sense = 'G';
//         sprintf(rname[0], "SEC(%d,%d)", i, k);
//         if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
//             print_error("wrong CPXnewrows [SEC]");
//         if (CPXchgcoef(env, lp, row, x_pos_leg(i, k, inst), -(inst->dimension - 1)))
//             print_error("wrong CPXchgcoef [SEC_x]");
//         for (int j = 1; j < inst->dimension - 1; j++) {
//             if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), -(inst->dimension - 1)))
//                 print_error("wrong CPXchgcoef [SEC_z]");
//         }
//         if (CPXchgcoef(env, lp, row, u_pos_leg(k, inst), 1.0))
//             print_error("wrong CPXchgcoef [SEC_u_k]");
//         if (CPXchgcoef(env, lp, row, u_pos_leg(i, inst), -1.0))
//             print_error("wrong CPXchgcoef [SEC_u_i]");
//     }
// }



// (2g) compute the waiting time of the truck at each node
    for (int k = 1; k < inst->dimension; k++) {
        int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
        double rhs = 0.0;
        char sense = 'L';
        sprintf(rname[0], "waiting_time(%d)", k);
        if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
            print_error("wrong CPXnewrows [waiting_time]");
        if (CPXchgcoef(env, lp, row, w_pos_leg(k, inst), -1.0))
            print_error("wrong CPXchgcoef [waiting_time_w_k]");
        for (int i = 0; i < inst->dimension - 1; i++) {
            if (i == k) continue;
            for (int j = 1; j < inst->dimension - 1; j++) {
                if (j == i || j == k) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), inst->min_feas_time_drone[i][j][k]))
                    print_error("wrong CPXchgcoef [waiting_time_drone_time_ijk]");
            }
        }
        for (int i = 0; i < inst->dimension - 1; i++) {
            if (i == k) continue;
            for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                if (CPXchgcoef(env, lp, row, y_pos_leg(inst->feasible_truck_legs2[i][k].legs[leg_id].ID, inst), -inst->feasible_truck_legs2[i][k].legs[leg_id].time))
                    print_error("wrong CPXchgcoef [waiting_time_truck_time_l]");
            }
        }
    }


// for (int j = 1; j < inst->dimension - 1; j++) {
//     int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
//     double rhs = 1.0;
//     char sense = 'L';
//     sprintf(rname[0], "drone_cust_x_ub(%d)", j);
//     if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
//         print_error("wrong CPXnewrows [drone_cust_x_ub]");
//     for (int k = 1; k < inst->dimension; k++) {
//         if (CPXchgcoef(env, lp, row, x_pos_leg(j, k, inst), 1.0))
//             print_error("wrong CPXchgcoef [drone_cust_x_ub]");
//     }
//     for (int i = 0; i < inst->dimension - 1; i++) {
//         for (int k = 1; k < inst->dimension; k++) {
//             if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
//                 print_error("wrong CPXchgcoef [drone_cust_x_ub]");
//         }
//     }
//     for (int k = 1; k < inst->dimension; k++) {
//         for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[j][k].count; leg_id++) {
//             if (CPXchgcoef(env, lp, row, y_pos_leg(inst->feasible_truck_legs2[j][k].legs[leg_id].ID, inst), 1.0))
//                 print_error("wrong CPXchgcoef [drone_cust_x_ub]");
//         }
//     }
// }

// *** VALID INEQUALITIES *** //




// 2-SEC

    for (int i = 1; i < inst->dimension - 1; i++) {
        for (int k = 1; k < inst->dimension - 1; k++) {
            if (i == k) continue;
            int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
            double rhs = 1.0;
            char sense = 'L';
            sprintf(rname[0], "2-SEC(%d,%d)", i, k);
            if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                print_error("wrong CPXnewrows [2-SEC]");
            if (CPXchgcoef(env, lp, row, x_pos_leg(i, k, inst), 1.0))
                print_error("wrong CPXchgcoef [2-SEC_x_ik]");
            if (CPXchgcoef(env, lp, row, x_pos_leg(k, i, inst), 1.0))
                print_error("wrong CPXchgcoef [2-SEC_x_ki]");
            for (int j = 1; j < inst->dimension - 1; j++) {
                if (j == i || j == k) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
                    print_error("wrong CPXchgcoef [2-SEC_x_ijk]");
            }
            for (int j = 1; j < inst->dimension - 1; j++) {
                if (j == i || j == k) continue;
                if (CPXchgcoef(env, lp, row, z_pos_leg(k, j, i, inst), 1.0))
                    print_error("wrong CPXchgcoef [2-SEC_x_kji]");
            }
        }
    }

// 2-SEC v2
// for (int i = 0; i < inst->dimension - 1; i++) {
//     for (int k = i + 1; k < inst->dimension; k++) {
//         if (i == k) continue;
//         int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
//         double rhs = 1.0;
//         char sense = 'L';
//         sprintf(rname[0], "2-SEC(%d,%d)", i, k);
//         if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
//             print_error("wrong CPXnewrows [2-SEC]");
//         if (CPXchgcoef(env, lp, row, x_pos_leg(i, k, inst), 1.0))
//             print_error("wrong CPXchgcoef [2-SEC_x_ik]");
//         if (CPXchgcoef(env, lp, row, x_pos_leg(k, i, inst), 1.0))
//             print_error("wrong CPXchgcoef [2-SEC_x_ki]");
//         for (int j = 1; j < inst->dimension; j++) {
//             if (j == i || j == k) continue;
//             if (CPXchgcoef(env, lp, row, z_pos_leg(j, i, k, inst), 1.0))
//                 print_error("wrong CPXchgcoef [2-SEC_x_ijk]");
//             if (CPXchgcoef(env, lp, row, z_pos_leg(j, k, i, inst), 1.0))
//                 print_error("wrong CPXchgcoef [2-SEC_x_ijk]");

//             if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
//                 print_error("wrong CPXchgcoef [2-SEC_x_ijk]");
//             if (CPXchgcoef(env, lp, row, z_pos_leg(k, j, i, inst), 1.0))
//                 print_error("wrong CPXchgcoef [2-SEC_x_kji]");

//             if (CPXchgcoef(env, lp, row, z_pos_leg(i, k , j, inst), 1.0))
//                 print_error("wrong CPXchgcoef [2-SEC_x_ijk]");
//             if (CPXchgcoef(env, lp, row, z_pos_leg(k, i, j, inst), 1.0))
//                 print_error("wrong CPXchgcoef [2-SEC_x_kji]");
//         }
//     }
// }

// 2-SEC v2 |S| = 3
    /*
        for (int i = 0; i < inst->dimension - 1; i++) {
            for (int j = i + 1; j < inst->dimension - 1; j++) {
                for (int k = j + 1; k < inst->dimension; k++) {
                    int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
                    double rhs = 2.0;
                    char sense = 'L';
                    sprintf(rname[0], "3-SEC(%d,%d,%d)", i, j, k);
                    if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                        print_error("wrong CPXnewrows [3-SEC]");
                    if (CPXchgcoef(env, lp, row, x_pos_leg(i, j, inst), 1.0))
                        print_error("wrong CPXchgcoef [3-SEC_x_ij]");
                    if (CPXchgcoef(env, lp, row, x_pos_leg(i, k, inst), 1.0))
                        print_error("wrong CPXchgcoef [3-SEC_x_ik]");
                    if (CPXchgcoef(env, lp, row, x_pos_leg(j, i, inst), 1.0))
                        print_error("wrong CPXchgcoef [3-SEC_x_ji]");
                    if (CPXchgcoef(env, lp, row, x_pos_leg(j, k, inst), 1.0))
                        print_error("wrong CPXchgcoef [3-SEC_x_jk]");
                    if (CPXchgcoef(env, lp, row, x_pos_leg(k, i, inst), 1.0))
                        print_error("wrong CPXchgcoef [3-SEC_x_ki]");
                    if (CPXchgcoef(env, lp, row, x_pos_leg(k, j, inst), 1.0))
                        print_error("wrong CPXchgcoef [3-SEC_x_kj]");
                    // if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 2.0))
                    //     print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                    // if (CPXchgcoef(env, lp, row, z_pos_leg(k, j, i, inst), 2.0))
                    //     print_error("wrong CPXchgcoef [3-SEC_z_kji]");
                    // if (CPXchgcoef(env, lp, row, z_pos_leg(i, k, j, inst), 2.0))
                    //     print_error("wrong CPXchgcoef [3-SEC_z_kji]");
                    // if (CPXchgcoef(env, lp, row, z_pos_leg(k, i, j, inst), 2.0))
                    //     print_error("wrong CPXchgcoef [3-SEC_z_kji]");
                    // if (CPXchgcoef(env, lp, row, z_pos_leg(j, k, i, inst), 2.0))
                    //     print_error("wrong CPXchgcoef [3-SEC_z_kji]");
                    // if (CPXchgcoef(env, lp, row, z_pos_leg(j, i, k, inst), 2.0))
                    //     print_error("wrong CPXchgcoef [3-SEC_z_kji]");

                    for (int w = 1; w < inst->dimension; w++) {
                        if (w == i || w == j || w == k) continue;
                        if (CPXchgcoef(env, lp, row, z_pos_leg(i, w, j, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                        if (CPXchgcoef(env, lp, row, z_pos_leg(j, w, i, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");

                        if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, w, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                        if (CPXchgcoef(env, lp, row, z_pos_leg(j, i, w, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");

                        if (CPXchgcoef(env, lp, row, z_pos_leg(w, i, j, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                        if (CPXchgcoef(env, lp, row, z_pos_leg(w, j, i, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");


                        if (CPXchgcoef(env, lp, row, z_pos_leg(i, k, w, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                        if (CPXchgcoef(env, lp, row, z_pos_leg(k, i, w, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");

                        if (CPXchgcoef(env, lp, row, z_pos_leg(i, w, k, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                        if (CPXchgcoef(env, lp, row, z_pos_leg(k, w, i, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");

                        if (CPXchgcoef(env, lp, row, z_pos_leg(w, i, k, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                        if (CPXchgcoef(env, lp, row, z_pos_leg(w, k, i, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");


                        if (CPXchgcoef(env, lp, row, z_pos_leg(j, k, w, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                        if (CPXchgcoef(env, lp, row, z_pos_leg(k, j, w, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");

                        if (CPXchgcoef(env, lp, row, z_pos_leg(j, w, k, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                        if (CPXchgcoef(env, lp, row, z_pos_leg(k, w, j, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");

                        if (CPXchgcoef(env, lp, row, z_pos_leg(w, j, k, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                        if (CPXchgcoef(env, lp, row, z_pos_leg(w, k, j, inst), 1.0))
                            print_error("wrong CPXchgcoef [3-SEC_z_ijk]");
                    }
                }
            }
        }
    */

// 2-SEC depots
    for (int j = 1; j < inst->dimension - 1; j++) {
        int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
        double rhs = 1.0;
        char sense = 'L';
        sprintf(rname[0], "2-SEC_depots(%d)", j);
        if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
            print_error("wrong CPXnewrows [2-SEC_depots]");
        if (CPXchgcoef(env, lp, row, x_pos_leg(0, j, inst), 1.0))
            print_error("wrong CPXchgcoef [2-SEC_depots_0i]");
        if (CPXchgcoef(env, lp, row, x_pos_leg(j, inst->dimension - 1, inst), 1.0))
            print_error("wrong CPXchgcoef [2-SEC_depots_i0]");
        for (int w = 1; w < inst->dimension - 1; w++) {
            if (w == j) continue;
            if (CPXchgcoef(env, lp, row, z_pos_leg(0, w, j, inst), 1.0))
                print_error("wrong CPXchgcoef [2-SEC_depots_z0j]");
        }
        for (int w = 1; w < inst->dimension - 1; w++) {
            if (w == j) continue;
            if (CPXchgcoef(env, lp, row, z_pos_leg(j, w, inst->dimension - 1, inst), 1.0))
                print_error("wrong CPXchgcoef [2-SEC_depots_j0]");
        }
    }

// 2-SEC depots v2
// for (int j = 1; j < inst->dimension - 1; j++) {
//     int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
//     double rhs = 1.0;
//     char sense = 'L';
//     sprintf(rname[0], "2-SEC_depots(%d)", j);
//     if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
//         print_error("wrong CPXnewrows [2-SEC_depots]");
//     if (CPXchgcoef(env, lp, row, x_pos_leg(0, j, inst), 1.0))
//         print_error("wrong CPXchgcoef [2-SEC_depots_0i]");
//     if (CPXchgcoef(env, lp, row, x_pos_leg(j, inst->dimension - 1, inst), 1.0))
//         print_error("wrong CPXchgcoef [2-SEC_depots_i0]");
//     for (int w = 1; w < inst->dimension - 1; w++) {
//         if (w == j) continue;
//         if (CPXchgcoef(env, lp, row, z_pos_leg(0, w, j, inst), 1.0))
//             print_error("wrong CPXchgcoef [2-SEC_depots_z0j]");
//         if (CPXchgcoef(env, lp, row, z_pos_leg(j, w, inst->dimension - 1, inst), 1.0))
//             print_error("wrong CPXchgcoef [2-SEC_depots_j0]");
//     }
// }

// REMOVE INCOMPATIBLE TRUCK/DRONE LEGS
    // for (int i = 0; i < inst->dimension - 1; i++) {
    //     for (int k = 1; k < inst->dimension; k++) {
    //         if (k == i) continue;
    //         for (int l = 0; l < inst->feasible_truck_legs2[i][k].count; l++) {
    //             if (inst->feasible_truck_legs2[i][k].legs[l].flag_active == 0) continue;
    //             // compute the set of feasible drone customers for truck leg l1
    //             int used_customers[inst->dimension];
    //             for (int c = 0; c < inst->dimension; c++) {
    //                 used_customers[c] = 0;
    //             }
    //             for (int c = 0; c < inst->feasible_truck_legs2[i][k].legs[l].length; c++) {
    //                 used_customers[inst->feasible_truck_legs2[i][k].legs[l].leg[c]] = 1;
    //             }
    //             for (int j = 1; j < inst->dimension - 1; j++) {
    //                 if (j == i || j == k) continue;
    //                 if (used_customers[j] == 0) continue;
    //                 int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
    //                 double rhs = 1.0;
    //                 char sense = 'L';
    //                 sprintf(rname[0], "remove_operation(%d,%d)", l, j);
    //                 if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
    //                     print_error("wrong CPXnewrows [remove_operation]");
    //                 if (CPXchgcoef(env, lp, row, y_pos_leg(inst->feasible_truck_legs2[i][k].legs[l].ID, inst), 1.0))
    //                     print_error("wrong CPXchgcoef [remove_operation truck leg coeff]");
    //                 if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
    //                     print_error("wrong CPXchgcoef [2remove_operation drone leg coeff]");
    //             }
    //         }
    //     }
    // }

// REMOVE AN OPERATION IF THERE EXISTS A DIFFERENT BUT EQUIVALENT OPERATION WITH A SHORTER (or equal) TRAVEL TIME
    for (int i = 0; i < inst->dimension - 1; i++) {
        for (int k = 1; k < inst->dimension; k++) {
            for (int l1 = 0; l1 < inst->feasible_truck_legs2[i][k].count; l1++) {
                if (inst->feasible_truck_legs2[i][k].legs[l1].flag_active == 0) continue;
                // compute the set of feasible drone customers for truck leg l1
                int used_customers[inst->dimension];
                for (int c = 0; c < inst->dimension; c++) {
                    used_customers[c] = 0;
                }
                for (int c = 0; c < inst->feasible_truck_legs2[i][k].legs[l1].length; c++) {
                    used_customers[inst->feasible_truck_legs2[i][k].legs[l1].leg[c]] = 1;
                }
                for (int j1 = 1; j1 < inst->dimension - 1; j1++) {
                    if (inst->nodes[j1].truck_only || used_customers[j1] == 1) continue;
                    if (inst->max_feas_time_drone[i][j1][k] == DBL_MAX) continue;
                    double travel_time1 = inst->feasible_truck_legs2[i][k].legs[l1].time;
                    if (inst->min_feas_time_drone[i][j1][k] > travel_time1 + 1e-6)
                        travel_time1 = inst->min_feas_time_drone[i][j1][k];
                    used_customers[j1] = 1;
                    // check if there exists an equivalent operation with a shorter (or equal) travel time
                    for (int l2 = 0; l2 < inst->feasible_truck_legs2[i][k].count; l2++) {
                        if (l1 == l2) continue;
                        if (inst->feasible_truck_legs2[i][k].legs[l2].flag_active == 0) continue;
                        if (inst->feasible_truck_legs2[i][k].legs[l2].length != inst->feasible_truck_legs2[i][k].legs[l1].length) continue;
                        int count_used = 0;
                        for (int c2 = 0; c2 < inst->feasible_truck_legs2[i][k].legs[l2].length; c2++) {
                            if (used_customers[inst->feasible_truck_legs2[i][k].legs[l2].leg[c2]] == 0)
                                break;
                            count_used++;
                        }
                        if (count_used != inst->feasible_truck_legs2[i][k].legs[l1].length)
                            continue;

                        for (int j2 = 1; j2 < inst->dimension - 1; j2++) {
                            if (used_customers[j2] == 0) continue;
                            // found an equivalent operation
                            double travel_time2 = inst->feasible_truck_legs2[i][k].legs[l2].time;
                            if (inst->min_feas_time_drone[i][j2][k] > travel_time2 + 1e-6)
                                travel_time2 = inst->min_feas_time_drone[i][j2][k];
                            if (travel_time2 > travel_time1 + 1e-6) continue;
                            // APPLY A CUT
                            int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
                            double rhs = 1.0;
                            char sense = 'L';
                            sprintf(rname[0], "remove_operation(%d,%d)", l1, j1);
                            if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                                print_error("wrong CPXnewrows [remove_operation]");
                            if (CPXchgcoef(env, lp, row, y_pos_leg(inst->feasible_truck_legs2[i][k].legs[l1].ID, inst), 1.0))
                                print_error("wrong CPXchgcoef [remove_operation truck leg coeff]");
                            if (CPXchgcoef(env, lp, row, z_pos_leg(i, j1, k, inst), 1.0))
                                print_error("wrong CPXchgcoef [2remove_operation drone leg coeff]");
                            inst->feasible_truck_legs2[i][k].legs[l1].flag_active = 0;
                        }
                    }
                    used_customers[j1] = 0;
                }
            }
        }
    }

// REMOVE INCOMPATIBLE ARCS
    // for (int i = 0; i < inst->dimension; i++) {
    //     for (int j = 1; j < inst->dimension - 1; j++) {
    //         if (j == i) continue;
    //         for (int k = 1; k < inst->dimension; k++) {
    //             if (k == i || k == j) continue;
    //             if (inst->nodes[j].truck_only == 1) continue;
    //             int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
    //             double rhs = 1.0;
    //             char sense = 'L';
    //             sprintf(rname[0], "remove_arcs_z(%d,%d,%d)", i, j, k);
    //             if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
    //                 print_error("wrong CPXnewrows [remove_arcs_z]");
    //             if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
    //                 print_error("wrong CPXchgcoef [remove_arcs_z]");
    //             if (CPXchgcoef(env, lp, row, x_pos_leg(i, j, inst), 1.0))
    //                 print_error("wrong CPXchgcoef [remove_arcs_z]");
    //             // if (CPXchgcoef(env, lp, row, x_pos_leg(j, i, inst), 1.0))
    //             //     print_error("wrong CPXchgcoef [remove_arcs_z]");
    //             // if (CPXchgcoef(env, lp, row, x_pos_leg(j, k, inst), 1.0))
    //             //     print_error("wrong CPXchgcoef [remove_arcs_z]");
    //             if (CPXchgcoef(env, lp, row, x_pos_leg(k, j, inst), 1.0))
    //                 print_error("wrong CPXchgcoef [remove_arcs_z]");
    //         }
    //     }
    // }

// REMOVE SUB-OPTIMAL COMBINED LEGS i->...->k if there exist a drone leg that procude no waiting time
// for the truck leg i->...->k
//ACHTUNG! check if the drone customer "satisfies" the triangle inequality under the truck distance metric
    /*
        int count = 0;
        for (int i = 0; i < inst->dimension - 1; i++) {
            for (int k = 1; k < inst->dimension - 1; k++) {
                if (i == k) continue;
                for (int l = 0; l < inst->feasible_truck_legs2[i][k].count; l++) {
                    if (inst->feasible_truck_legs2[i][k].legs[l].flag_feasible_drone_leg_exist == 0)
                        continue;
                    printf("l = %d, time = %.2f\n", l, inst->feasible_truck_legs2[i][k].legs[l].time);
                    // look for drone legs that produce no waiting time for the current truck leg
                    int used_customers[inst->dimension];
                    for (int n = 0; n < inst->dimension; n++)
                        used_customers[n] = 0;
                    for (int c = 0; c < inst->feasible_truck_legs2[i][k].legs[l].length; c++) {
                        used_customers[inst->feasible_truck_legs2[i][k].legs[l].leg[c]] = 1;
                    }
                    for (int j = 1; j < inst->dimension - 1; j++) {
                        if (inst->nodes[j].truck_only || used_customers[j])
                            continue;
                        if (inst->max_feas_time_drone[i][j][k] == DBL_MAX)
                            continue;
                        printf("\tj = %d, time = %.2f\n", j, inst->min_feas_time_drone[i][j][k]);
                        // add a cut!
                        double waiting_time = inst->min_feas_time_drone[i][j][k] - inst->feasible_truck_legs2[i][k].legs[l].time;
                        if (waiting_time < 1e-6) {
                            int flag_triang_ineq_ok = 1;
                            for (int prev = 0; prev < inst->dimension - 1; prev++) {
                                for (int next = 1; next < inst->dimension; next++)
                                {
                                    if (prev == next || prev == j || next == j) continue;
                                    if (inst->truck_times[prev][next] < inst->truck_times[prev][j] + inst->truck_times[j][next] < inst->truck_times[prev][next] + 1e-6)
                                    {
                                        flag_triang_ineq_ok = 0;
                                        break;
                                    }
                                }
                            }
                            if (!flag_triang_ineq_ok)
                                continue;
                            count++;
                            printf("*** triangle inequality ok for node %d\n", j);
                            int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
                            double rhs = inst->feasible_truck_legs2[i][k].legs[l].length - 2;
                            char sense = 'L';
                            sprintf(rname[0], "remove_comb_leg_truck_leg(%d,%d)", l, j);
                            if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                                print_error("wrong CPXnewrows [remove_comb_leg_truck_leg]");

                            // for (int c = 0; c < inst->feasible_truck_legs2[i][k].legs[l].length - 1; c++) {
                            //     if (CPXchgcoef(env, lp, row, x_pos_leg(inst->feasible_truck_legs2[i][k].legs[l].leg[c], inst->feasible_truck_legs2[i][k].legs[l].leg[c + 1], inst), 1.0))
                            //         print_error("wrong CPXchgcoef [remove_comb_leg_truck_leg]");
                            // }
                            for (int c1 = 1; c1 < inst->feasible_truck_legs2[i][k].legs[l].length - 1; c1++) {
                                for (int c2 = 1; c2 < inst->feasible_truck_legs2[i][k].legs[l].length - 1; c2++) {
                                    if (c1 == c2) continue;
                                    if (CPXchgcoef(env, lp, row, x_pos_leg(inst->feasible_truck_legs2[i][k].legs[l].leg[c1], inst->feasible_truck_legs2[i][k].legs[l].leg[c2], inst), 1.0))
                                        print_error("wrong CPXchgcoef [remove_comb_leg_truck_leg]");
                                }
                            }
                            for (int c = 1; c < inst->feasible_truck_legs2[i][k].legs[l].length - 1; c++) {
                                if (CPXchgcoef(env, lp, row, x_pos_leg(inst->feasible_truck_legs2[i][k].legs[l].leg[0], inst->feasible_truck_legs2[i][k].legs[l].leg[c], inst), 1.0))
                                    print_error("wrong CPXchgcoef [remove_comb_leg_truck_leg]");
                                if (CPXchgcoef(env, lp, row, x_pos_leg(inst->feasible_truck_legs2[i][k].legs[l].leg[c], inst->feasible_truck_legs2[i][k].legs[l].leg[inst->feasible_truck_legs2[i][k].legs[l].length - 1], inst), 1.0))
                                    print_error("wrong CPXchgcoef [remove_comb_leg_truck_leg]");
                            }
                            break;
                        }
                        // else {
                        // for (int pos = 1; pos < inst->feasible_truck_legs2[i][k].legs[l].length; pos++) {
                        //     int prev_cust = inst->feasible_truck_legs2[i][k].legs[l].leg[pos - 1];
                        //     int next_cust = inst->feasible_truck_legs2[i][k].legs[l].leg[pos];
                        //     double delta = inst->truck_times[prev_cust][j] + inst->truck_times[j][next_cust] - inst->truck_times[prev_cust][next_cust];
                        //     if (waiting_time > delta + 1e-6) {
                        //         // add a cut!
                        //         int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
                        //         double rhs = 1.0;
                        //         char sense = 'L';
                        //         sprintf(rname[0], "remove_comb_leg_truck_leg2(%d,%d,%d)", l, j, pos);
                        //         if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                        //             print_error("wrong CPXnewrows [remove_comb_leg_truck_leg2]");
                        //         if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
                        //             print_error("wrong CPXchgcoef [remove_comb_leg_truck_leg2]");
                        //         if (CPXchgcoef(env, lp, row, y_pos_leg(l, inst), 1.0))
                        //             print_error("wrong CPXchgcoef [remove_comb_leg_truck_leg2]");
                        //     }
                        //     else {
                        //         // add another cut
                        //         // int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
                        //         // double rhs = inst->feasible_truck_legs2[i][k].legs[l].length;
                        //         // char sense = 'L';
                        //         // sprintf(rname[0], "remove_comb_leg_truck_leg3(%d,%d,%d)", l, j, pos);
                        //         // if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                        //         //     print_error("wrong CPXnewrows [remove_comb_leg_truck_leg3]");
                        //         // for (int c = 0; c < inst->feasible_truck_legs2[i][k].legs[l].length - 1; c++) {
                        //         //     if (c == pos - 1) {
                        //         //         if (CPXchgcoef(env, lp, row, x_pos_leg(inst->feasible_truck_legs2[i][k].legs[l].leg[c], j, inst), 1.0))
                        //         //             print_error("wrong CPXchgcoef [remove_comb_leg_truck_leg3]");
                        //         //         if (CPXchgcoef(env, lp, row, x_pos_leg(j, inst->feasible_truck_legs2[i][k].legs[l].leg[c + 1], inst), 1.0))
                        //         //             print_error("wrong CPXchgcoef [remove_comb_leg_truck_leg3]");
                        //         //         continue;
                        //         //     }
                        //         //     if (CPXchgcoef(env, lp, row, x_pos_leg(inst->feasible_truck_legs2[i][k].legs[l].leg[c], inst->feasible_truck_legs2[i][k].legs[l].leg[c + 1], inst), 1.0))
                        //         //         print_error("wrong CPXchgcoef [remove_comb_leg_truck_leg3]");
                        //         // }
                        //     }
                        // }
                        // }
                    }
                    printf("leg %d count = %d\n", l, count);
                }
            }
        }
        printf("count = %d\n",count);
    */

// add cuts: x_ij + x_jk <= 1
// if, for a drone leg i-->j-->k exists a truck leg with 0 waiting time
// for (int i = 0; i < inst->dimension - 1; i++) {
//     for (int k = 1; k < inst->dimension - 1; k++) {
//         if (i == k) continue;
//         for (int l = 0; l < inst->feasible_truck_legs2[i][k].count; l++) {
//             int count = 0;
//             if (inst->feasible_truck_legs2[i][k].legs[l].flag_feasible_drone_leg_exist == 0)
//                 continue;
//             // look for drone legs that produce no waiting time for the current truck leg
//             int used_customers[inst->dimension];
//             for (int n = 0; n < inst->dimension; n++)
//                 used_customers[n] = 0;
//             for (int c = 0; c < inst->feasible_truck_legs2[i][k].legs[l].length; c++) {
//                 used_customers[inst->feasible_truck_legs2[i][k].legs[l].leg[c]] = 1;
//             }
//             for (int j = 1; j < inst->dimension - 1; j++) {
//                 if (inst->nodes[j].truck_only || used_customers[j])
//                     continue;
//                 if (inst->max_feas_time_drone[i][j][k] == DBL_MAX)
//                     continue;
//                 double waiting_time = inst->min_feas_time_drone[i][j][k] - inst->feasible_truck_legs2[i][k].legs[l].time;
//                 if (waiting_time > 1e-6)
//                     continue;
//                 // add a cut: x_ij + x_jk <= 1
//                 int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
//                 double rhs = 1.0;
//                 char sense = 'L';
//                 sprintf(rname[0], "remove_x_ij_xjk(%d,%d,%d)", i, j, k);
//                 if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
//                     print_error("wrong CPXnewrows [remove_x_ij_xjk]");
//                 if (CPXchgcoef(env, lp, row, x_pos_leg(i, j, inst), 1.0))
//                     print_error("wrong CPXchgcoef [remove_x_ij_xjk]");
//                 if (CPXchgcoef(env, lp, row, x_pos_leg(j, k, inst), 1.0))
//                     print_error("wrong CPXchgcoef [remove_x_ij_xjk]");
//             }
//         }
//     }
// }

// for (int j = 1; j < inst->dimension - 1; j++) {
//     for (int k = 1; k < inst->dimension; k++) {
//         int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
//         double rhs = 1.0;
//         char sense = 'L';
//         sprintf(rname[0], "z_jk(%d,%d)", j, k);
//         if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
//             print_error("wrong CPXnewrows [z_jk]");
//         for (int w = 0; w < inst->dimension - 1; w++) {
//             if (CPXchgcoef(env, lp, row, z_pos_leg(w, j, k, inst), 1.0))
//                 print_error("wrong CPXchgcoef [z_wjk]");
//         }
//         for (int w = 1; w < inst->dimension; w++) {
//             if (CPXchgcoef(env, lp, row, z_pos_leg(j, k, w, inst), 1.0))
//                 print_error("wrong CPXchgcoef [z_jkw]");
//             if (CPXchgcoef(env, lp, row, z_pos_leg(k, j, w, inst), 1.0))
//                 print_error("wrong CPXchgcoef [z_jkw]");
//         }
//     }
// }

// {
//     int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
//     double rhs = (int)(inst->dimension / 2);
//     char sense = 'L';
//     sprintf(rname[0], "sum_z_ijk_UB");
//     if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
//         print_error("wrong CPXnewrows [sum_z_ijk_UB]");
//     for (int i = 0; i < inst->dimension - 1; i++) {
//         for (int j = 1; j < inst->dimension - 1; j++) {
//             for (int k = 1; k < inst->dimension; k++) {
//                 if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
//                     print_error("wrong CPXchgcoef [sum_z_ijk_UB]");
//             }
//         }
//     }
// }

// add cuts b_j <= 1 - sum{i,k} z_ijk
// for (int j = 1; j < inst->dimension - 1; j++) {
//     int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
//     double rhs = 1.0;
//     char sense = 'L';
//     sprintf(rname[0], "b_ijk(%d)", j);
//     if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
//         print_error("wrong CPXnewrows [b_ijk]");
//     if (CPXchgcoef(env, lp, row, b_pos_leg(j, inst), 1.0))
//         print_error("wrong CPXchgcoef [b_ijk]");
//     for (int i = 0; i < inst->dimension - 1; i++) {
//         for (int k = 1; k < inst->dimension; k++) {
//             // if (inst->max_feas_time_drone[i][j][k] == DBL_MAX)
//             // continue;
//             if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
//                 print_error("wrong CPXchgcoef [b_ijk]");
//         }
//     }
// }

// add cuts: (1 - sum_{ik}z_ijk) + b_j >= 1 - z_ijk
// for (int i = 0; i < inst->dimension - 1; i++) {
//     for (int k = 1; k < inst->dimension; k++) {
//         for (int j = 1; j < inst->dimension - 1; j++) {
//             // if (inst->max_feas_time_drone[i][j][k] == DBL_MAX)
//             // continue;
//             int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
//             double rhs = 0.0;
//             char sense = 'G';
//             sprintf(rname[0], "b_ijkG(%d)", j);
//             if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
//                 print_error("wrong CPXnewrows [b_ijkG]");
//             if (CPXchgcoef(env, lp, row, b_pos_leg(j, inst), 1.0))
//                 print_error("wrong CPXchgcoef [b_ijkG]");
//             // if (CPXchgcoef(env, lp, row, z_pos_leg(i, j, k, inst), 1.0))
//             // print_error("wrong CPXchgcoef [b_ijkG]");
//             for (int a = 0; a < inst->dimension - 1; a++) {
//                 for (int b = 1; b < inst->dimension; b++) {
//                     double coef = -1.0;
//                     if (a == i && b == k)
//                         coef = 0.0;
//                     if (CPXchgcoef(env, lp, row, z_pos_leg(a, j, b, inst), 0.0))
//                         print_error("wrong CPXchgcoef [b_ijkG]");
//                 }
//             }
//         }
//     }
// }

}



double gather_solution_leg_formulation(instance * inst, const double * xstar, int* truck_succ, int* drone_succ)
{
    // int N_truck_legs = inst->number_feasible_truck_legs;
    // if (FLAG_REDUCED_TRUCK_LEGS)
    //     N_truck_legs = inst->reduced_number_feasible_truck_legs;

    // printf("*** inside gather_solution_leg_formulation() ***\n");

    int visited[inst->dimension];
    int n_visited = 0;

    for (int i = 0; i < inst->dimension; i++) {
        truck_succ[i] = -1;
        drone_succ[i] = -1;
        visited[i] = 0;
    }

    // for (int k = 0; k < inst->dimension; k++) {
    //     if (inst->nodes[k].truck_only)
    //         printf("customer %d truck_only: %d\n", k, inst->nodes[k].truck_only);
    // }



    //////////////////////////////////
    int node = 0;
    visited[0] = 1;
    n_visited = 1;

    while (n_visited <= inst->dimension) {
        if (node == inst->dimension - 1)
            break;
        // printf("truck_succ[%d] = %d, node_succ[%d] = %d\n", node, truck_succ[node], node, drone_succ[node]);
        if (truck_succ[node] != -1 && drone_succ[node] != -1)
            break;
        int cycle_found = 0;
        // if (node == inst->dimension - 1 && n_visited < inst->dimension) {
        //     cycle_found = 1;
        //     goto CYCLE_FOUND;
        // }
        for (int j = 1; j < inst->dimension; j++) {
            if (j == node) continue;
            if (xstar[x_pos_leg(node, j, inst)] > 0.9) {
                // printf("* x(%d,%d) = 1\n", node, j);
                assert(truck_succ[node] == -1);
                assert(drone_succ[node] == -1);
                truck_succ[node] = j;
                drone_succ[node] = j;
                if (visited[j]) {
                    // printf("cycle found, n_visited = %d\n", n_visited);
                    cycle_found = 1;
                    goto CYCLE_FOUND;
                }
                n_visited++;
                visited[j] = 1;
                // printf(" n_visited = %d\n", n_visited);
                node = j;
                if (node == inst->dimension - 1) {
                    cycle_found = 1;
                    goto CYCLE_FOUND;
                }
                goto END_LOOP;
            }
        }
        for (int j = 1; j < inst->dimension - 1; j++) {
            if (j == node) continue;
            for (int k = 1; k < inst->dimension; k++) {
                if (k == node || k == j) continue;
                if (xstar[z_pos_leg(node, j, k, inst)] > 0.9) {
                    assert(truck_succ[node] == -1);
                    assert(drone_succ[node] == -1);
                    // printf(" z(%d,%d,%d) = 1\n", node, j, k);
                    drone_succ[node] = j;
                    visited[j] = 1;
                    n_visited++;

                    for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[node][k].count; leg_id++) {
                        int l = inst->feasible_truck_legs2[node][k].legs[leg_id].ID;
                        if (xstar[y_pos_leg(l, inst)] > 0.9) {
                            const int* truck_leg = inst->feasible_truck_legs2[node][k].legs[leg_id].leg;
                            for (int truck_cust_id = 0; truck_cust_id < inst->feasible_truck_legs2[node][k].legs[leg_id].length; truck_cust_id++) {
                                truck_succ[truck_leg[truck_cust_id]] = truck_leg[truck_cust_id + 1];
                                if (truck_leg[truck_cust_id + 1] == k)
                                    break;
                                visited[truck_leg[truck_cust_id]] = 1;
                                n_visited++;
                            }
                            break;
                        }
                    }
                    drone_succ[j] = k;
                    if (visited[k]) {
                        // printf("cycle found, n_visited = %d\n", n_visited);
                        cycle_found = 1;
                        goto CYCLE_FOUND;
                    }
                    visited[k] = 1;
                    n_visited++;
                    // printf(" n_visited = %d\n", n_visited);
                    node = k;

                    if (node == inst->dimension - 1) {
                        cycle_found = 1;
                        goto CYCLE_FOUND;
                    }
                    goto END_LOOP;
                }
            }
        }

CYCLE_FOUND:;
        if (cycle_found && n_visited < inst->dimension) {
            // find a node from which restart
            for (int i = 0; i < inst->dimension; i++) {
                if (visited[i]) continue;
                // check if i is NOT a truck/drone only customer
                for (int j = 1; j < inst->dimension; j++) {
                    if (visited[j] || j == i) continue;
                    if (xstar[x_pos_leg(i, j, inst)] > 0.9) {
                        visited[i] = 1;
                        n_visited++;
                        node = i;
                        goto END_LOOP;
                    }
                }
                for (int j = 1; j < inst->dimension - 1; j++) {
                    if (visited[j] || j == i) continue;
                    for (int k = 1; k < inst->dimension; k++) {
                        if (visited[k] || k == j || k == i) continue;
                        if (xstar[z_pos_leg(i, j, k, inst)] > 0.9) {
                            visited[i] = 1;
                            n_visited++;
                            node = i;
                            goto END_LOOP;
                        }
                    }
                }

            }
        }
END_LOOP:;
    }

    // printf("\n");
    // for (int i = 0; i < inst->dimension - 1; i++) {
    //     for (int j = 0; j < inst->dimension - 1; j++) {
    //         if (xstar[x_pos_leg(i, j, inst)] > 0.9) {
    //             printf("* x(%d,%d) = 1\n", i, j);
    //             if (truck_succ[i] != -1)
    //                 print_error("truck_succ[i] != -1");
    //             if (drone_succ[i] != -1)
    //                 print_error("drone_succ[i] != -1");
    //             truck_succ[i] = j;
    //             drone_succ[i] = j;
    //         }
    //     }
    // }

    // printf("\n");
    // for (int i = 0; i < inst->dimension - 1; i++) {
    //     for (int j = 0; j < inst->dimension - 1; j++) {
    //         for (int k = 0; k < inst->dimension; k++) {
    //             if (xstar[z_pos_leg(i, j, k, inst)] > 0.9) {
    //                 printf("* z(%d,%d,%d) = 1 min_time = %.2f, \n", i, j, k, inst->min_feas_time_drone[i][j][k]);
    //                 if (truck_succ[i] != -1)
    //                     print_error("truck_succ[i] != -1");
    //                 if (drone_succ[i] != -1)
    //                     print_error("drone_succ[i] != -1");
    //                 drone_succ[i] = j;
    //                 drone_succ[j] = k;
    //                 int truck_leg_found = 0;
    //                 for (int l = inst->feasible_truck_legs_ik_begin[i][k]; l < inst->feasible_truck_legs_ik_begin[i][k] + inst->feasible_truck_legs_ik_number[i][k]; l++) {
    //                     if (xstar[y_pos_leg(l, inst)] > 0.9) {
    //                         truck_leg_found = 1;
    //                         const int* truck_leg = inst->feasible_truck_legs[l];
    //                         int node = i;
    //                         for (int truck_cust_id = 2; truck_cust_id < truck_leg[0] + 1; truck_cust_id++) {
    //                             truck_succ[node] = truck_leg[truck_cust_id];
    //                             node = truck_leg[truck_cust_id];
    //                         }
    //                         break;
    //                     }
    //                 }
    //                 if (!truck_leg_found)
    //                     print_error("Truck leg not found\n");
    //             }
    //         }
    //     }
    // }

    // for (int i = 0; i < inst->dimension; i++) {
    //     printf("truck_succ[%d] = %d\n", i, truck_succ[i]);
    // }
    // printf("\n");
    // for (int i = 0; i < inst->dimension; i++) {
    //     printf("drone_succ[%d] = %d\n", i, drone_succ[i]);
    // }

    /*

    printf("\n");
    for (int l = 0; l < inst->number_feasible_truck_legs; l++) {
        if (xstar[y_pos_leg(l, inst)] > 0.9) {
            if (FLAG_REDUCED)
                printf("* y(%d) = 1 time = %.2f : ", l, inst->feasible_truck_legs_times[l]);
            else
                printf("* y(%d) = 1 time = %.2f : ", l, inst->feasible_truck_legs_times[l]);
            for (int cust_id = 1; cust_id < inst->feasible_truck_legs[l][0] + 1; cust_id++) {
                if (FLAG_REDUCED)
                    printf("%d ", inst->feasible_truck_legs[l][cust_id]);
                else
                    printf("%d ", inst->feasible_truck_legs[l][cust_id]);
            }
            printf("\n");
        }
    }

    printf("\n");
    for (int i = 0; i < inst->dimension; i++) {
        printf("* u(%d) = %f\n", i, xstar[u_pos_leg(i, inst)]);
    }
    printf("\n");
    for (int i = 0; i < inst->dimension; i++) {
        printf("* w(%d) = %f\n", i, xstar[w_pos_leg(i, inst)]);
    }
    //////////////////////////////////
    printf("\nGathering solution...\n");
    int visited[inst->dimension];
    for (int i = 0; i < inst->dimension; i++)
        visited[0] = 0;
    int n_visited = 0;
    inst->truck_seq[0] = 0;
    inst->drone_seq[0] = 0;

    int node = 0;
    int next_truck_pos = 1, next_drone_pos = 1;
    double obj = 0.0;
    while (node != inst->dimension - 1)
    {
        int flag = 0;
        for (int j = 1; j < inst->dimension; j++)
        {
            if (xstar[x_pos_leg(node, j, inst)] > 0.5)
            {
                printf("* x(%d,%d) = 1\n", node, j);
                flag = 1;
                inst->truck_seq[next_truck_pos++] = j;
                inst->drone_seq[next_drone_pos++] = j;
                obj += inst->truck_times[node][j];
                node = j;
                printf("obj(%d) = %.2f\n", j, obj);
                break;
            }
        }
        if (!flag) {
            for (int j = 1; j < inst->dimension; j++)
            {
                for (int k = 1; k < inst->dimension; k++)
                {
                    if (xstar[z_pos_leg(node, j, k, inst)] > 0.5)
                    {
                        printf("* z(%d,%d,%d) = 1\n", node, j, k);
                        flag = 1;
                        inst->drone_seq[next_drone_pos++] = j;
                        inst->drone_seq[next_drone_pos++] = k;
                        // find truck leg from node to k
                        if (inst->feasible_truck_legs_ik_number[node][k] == 0)
                            print_error("Error in gather_solution_leg_formulation(): no feasible truck legs from node to k\n");

                        int flag_truck_leg = 0;
                        double truck_leg_time = 0.0;
                        for (int l = inst->feasible_truck_legs_ik_begin[node][k]; l < inst->feasible_truck_legs_ik_begin[node][k] + inst->feasible_truck_legs_ik_number[node][k]; l++) {
                            if (xstar[y_pos_leg(l, inst)] > 0.5) {
                                flag_truck_leg = 1;
                                const int* truck_leg = inst->feasible_truck_legs[l];
                                for (int truck_cust_id = 2; truck_cust_id < truck_leg[0] + 1; truck_cust_id++) {
                                    inst->truck_seq[next_truck_pos++] = truck_leg[truck_cust_id];
                                    truck_leg_time += inst->truck_times[truck_leg[truck_cust_id - 1]][truck_leg[truck_cust_id]];
                                }
                                break;
                            }
                        }
                        if (flag_truck_leg == 0)
                            print_error("Error in gather_solution_leg_formulation(): truck leg not found\n");

                        const double drone_leg_time = inst->min_feas_time_drone[node][j][k];
                        assert(truck_leg_time < inst->max_feas_time_drone[node][j][k] + 1e-4);
                        if (drone_leg_time > truck_leg_time)
                            obj += drone_leg_time;
                        else
                            obj += truck_leg_time;
                        obj += POIKONEN_UAV_LAUNCH_TIME + POIKONEN_UAV_RETRIEVAL_TIME;
                        printf("obj(%d) = %.2f\n", k, obj);
                        node = k;
                        break;
                    }
                    if (flag) break;
                }
                if (flag) break;
            }
        }
        if (!flag || next_truck_pos > inst->dimension || next_drone_pos > inst->dimension)
            print_error("Error in gather_solution_leg_formulation()\n");
    }
    printf("\nSolution Path: \n");
    printf("Truck path: ");
    for (int i = 0; inst->truck_seq[i] != inst->dimension - 1; i++)
        printf("%d -> ", inst->truck_seq[i]);
    printf("0 \n");
    printf("Drone path: ");
    for (int i = 0; inst->drone_seq[i] != inst->dimension - 1; i++)
        printf("%d -> ", inst->drone_seq[i]);
    printf("0 \n");
    printf("Obj = %.2f\n", obj);

    save_and_plot_solution(inst, 0);
    */
}

double leg_formulation(instance * inst, instance_TSP * inst_tsp, int iter)
{
    printf("\nComputing the optimal TSPD solution using the LEG BASED FORMULATION...\n");
    // Open CPLEX model
    if (inst->number_feasible_truck_legs == 0) {
        printf("0 feasible truck legs found.\n");
        inst->z_best = inst_tsp->z_best;
        return inst->z_best;
    }

    struct timespec timestamp;
    if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
        print_error("Error clock_gettime");
    double timestamp_start = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);



    int error;
    CPXENVptr env = CPXopenCPLEX(&error);
    if (error)
        print_error("Error in CPXopenCPLEX() (TSPD LEG formulation)\n");
    CPXLPptr lp;
    lp = CPXcreateprob(env, &error, "TSPD LEG formulation");

    if (error)
        print_error("Error in CPXopenCPLEX() (TSPD LEG formulation)\n");
    // get timestamp
    inst->param.ticks ? CPXgetdettime(env, &inst->timestamp_start) : getTimeStamp(&inst->timestamp_start);

    // path for LP and log files
    char lp_path[1000];
    sprintf(lp_path, "../output/LP_%s_alpha_%d.lp", inst->instance_name, inst->param.alpha);
    char log_path[1000];
    sprintf(log_path, "../output/log_%s_alpha_%d.txt", inst->instance_name, inst->param.alpha);

/////////////////////
    // build model
    if (inst_tsp == NULL)
        build_leg_based_model(env, lp, inst, DBL_MAX);
    else
        build_leg_based_model(env, lp, inst, inst_tsp->z_best);


    inst->cols = CPXgetnumcols(env, lp);
    inst->best_sol = (double *)calloc(inst->cols, sizeof(double));
    if (inst_tsp != NULL)
        post_tsp_sol(env, lp, inst_tsp, inst);

    // ****** CPLEX's parameter setting ******
    if (CPXsetintparam(env, CPX_PARAM_RANDOMSEED, inst->param.seed)) // Set seed
        print_error("CPX_PARAM_RANDOMSEED error");

    if (inst->param.ticks)
    {
        if (CPXsetdblparam(env, CPX_PARAM_DETTILIM, inst->time_limit))
            print_error("CPX_PARAM_DETTILIM error");
    }
    else
    {
        if (CPXsetdblparam(env, CPX_PARAM_TILIM, inst->time_limit))
            print_error("CPX_PARAM_TILIM error");
    }

    if (CPXsetintparam(env, CPXPARAM_Parallel, CPX_PARALLEL_OPPORTUNISTIC)) // Set opportunistic mode
        // if (CPXsetintparam(env, CPXPARAM_Parallel, CPX_PARALLEL_DETERMINISTIC)) // Set opportunistic mode
        print_error("CPXPARAM_Parallel error");

    // if (CPXsetintparam(env, CPXPARAM_LPMethod, CPX_ALG_AUTOMATIC)) // Set LP method
    //     print_error("CPXPARAM_LPMethod error");


    if (CPXsetintparam(env, CPX_PARAM_THREADS, 2)) // Set number of thread
        print_error("CPX_PARAM_THREADS error");

    if (CPXsetintparam(env, CPX_PARAM_CLONELOG, -1)) // CPLEX does not clone log files. (off)
        print_error("CPXPARAM_Output_CloneLog error");

    // CPLEX's precision setting
    if (CPXsetdblparam(env, CPX_PARAM_EPINT, 0.0)) // very important if big-M is present
        print_error("CPX_PARAM_EPINT error");
    if (CPXsetdblparam(env, CPX_PARAM_EPRHS, 1e-9))
        print_error("CPX_PARAM_EPRHS error");
    if (CPXsetdblparam(env, CPX_PARAM_EPGAP, 1e-5)) // abort Cplex when relative gap below this value
        print_error("CPX_PARAM_EPGAP error");

    if (CPXsetlogfilename(env, log_path, "w")) // set log file path
        print_error("Error in CPXsetlogfilename() (TSPD LEG formulation)\n");

    // set callback function to generate SEC from candidate solutions
    CPXLONG contextid = CPX_CALLBACKCONTEXT_CANDIDATE;
    // CPXLONG contextid = CPX_CALLBACKCONTEXT_CANDIDATE | CPX_CALLBACKCONTEXT_RELAXATION;
    if (CPXcallbacksetfunc(env, lp, contextid, callback_driver, inst))
        print_error("CPXcallbacksetfunc() error");

    if (CPXmipopt(env, lp))
        print_error("CPXmipopt() error");
    if (CPXwriteprob(env, lp, lp_path, NULL)) // write LP model
        print_error("Error in CPXwriteprob() (TSPD LEG formulation)\n");

    // solution status of the problem
    int lpstat = CPXgetstat(env, lp);
    printf("CPLEX status: %d\n", lpstat);
    if (lpstat == 103)
        print_error("Infeasible problem.");
    if (lpstat == 107 || lpstat == 108) {
        printf("Time limit exceeded; no integer solution found.");
    }

    printf("Lower Bound = %.4f\n", inst->best_lb);

    // Use the optimal solution found by CPLEX
    if (CPXgetx(env, lp, inst->best_sol, 0, inst->cols - 1))
        print_error("CPXgetx() error");

    CPXgetobjval(env, lp, &inst->z_best);      // Best objective value
    CPXgetbestobjval(env, lp, &inst->best_lb); // Best lower bound

    printf("\nSOLUTION -----------------------------------------------\n");

    int truck_succ[inst->dimension];
    int drone_succ[inst->dimension];
    gather_solution_leg_formulation(inst, inst->best_sol, truck_succ, drone_succ);
    printf("Print Solution...\n");
    for (int i = 0; i < inst->dimension; i++) {
        printf("truck_succ[%d] = %d\n", i, truck_succ[i]);
    }
    printf("\n");
    for (int i = 0; i < inst->dimension; i++) {
        printf("drone_succ[%d] = %d\n", i, drone_succ[i]);
    }
    printf("\n");
    for (int i = 0; i < inst->dimension - 1; i++) {
        for (int j = 0; j < inst->dimension ; j++) {
            if (inst->best_sol[x_pos_leg(i, j, inst)] > 0.5) {
                printf("x[%d][%d] = %.2f, time = %.2f\n", i, j, inst->best_sol[x_pos_leg(i, j, inst)], inst->truck_times[i][j]);
            }
        }
    }
    printf("\n");
    for (int i = 0; i < inst->dimension - 1; i++) {
        for (int j = 0; j < inst->dimension ; j++) {
            for (int k = 0; k < inst->dimension; k++) {
                if (inst->best_sol[z_pos_leg(i, j, k, inst)] > 0.5) {
                    printf("z[%d][%d][%d] = %.2f, time = %.2f\n", i, j, k, inst->best_sol[z_pos_leg(i, j, k, inst)], inst->min_feas_time_drone[i][j][k]);
                }
            }
        }
    }
    printf("\n");
    for (int i = 0; i < inst->dimension - 1; i++) {
        for (int k = 0; k < inst->dimension; k++) {
            for (int l = 0; l < inst->feasible_truck_legs2[i][k].count; l++) {
                int leg_id = inst->feasible_truck_legs2[i][k].legs[l].ID;
                if (inst->best_sol[y_pos_leg(leg_id, inst)] > 0.01) {
                    printf("y[%d][%d]{%d} = %.2f (", i, k, l, inst->best_sol[y_pos_leg(leg_id, inst)]);
                    int* truck_leg = inst->feasible_truck_legs2[i][k].legs[l].leg;
                    for (int cust = 0; cust < inst->feasible_truck_legs2[i][k].legs[l].length; cust++)
                        printf("%d,", truck_leg[cust]);
                    printf("), , time = %.2f\n", inst->feasible_truck_legs2[i][k].legs[l].time);
                }
            }
        }
    }
// for (int i = 0; i < inst->dimension - 1; i++) {
//     printf("b[%d] = %.2f\n", i, inst->best_sol[b_pos_leg(i, inst)]);
// }
    save_and_plot_solution_succ(inst, truck_succ, drone_succ, inst->z_best);



// int *comp = NULL;
// int *succ = NULL;
// int ncomp = 0; // number of connected components
// int *length_comp = NULL;

// // Retrieve the connected components of the current solution
// findConnectedComponents(inst->best_sol, inst, &succ, &comp, &ncomp, &length_comp);

// printf("Print connected components...\n");
// for (int i = 0; i < inst->dimension; i++) {
//     printf("comp[%d] = %d\n", i, comp[i]);
// }
// // printf("\n");
// // for (int i = 0; i < inst->dimension; i++) {
// //     printf("succ[%d] = %d\n", i, succ[i]);
// // }
// printf("\n");
// for (int C = 0; C < ncomp; C++) {
//     printf("length comp %d = %d\n", C, length_comp[C]);
// }
    if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
        print_error("Error clock_gettime");
    double timestamp_finish = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);
    double time_elapsed = timestamp_finish - timestamp_start;

    FILE *csv;
    char output_path[1000];
    sprintf(output_path, "../output/%s.csv", inst->param.output_name);
    int flag_new_file = 0;
    if (!IsPathExist(output_path))
        flag_new_file = 1;
    csv = fopen(output_path, "a");
    if (csv == NULL)
        print_error("generate_csv_record fopen() error");

    if (lpstat == 107 || lpstat == 108)
        fprintf(csv, "%s, %d, %f, %f, %f, %d, %d, TIME LIMIT\n", inst->instance_name, inst->param.alpha, inst->best_lb, inst->z_best, time_elapsed, inst->number_feasible_truck_legs, number_unique_truck_legs);
    else
        fprintf(csv, "%s, %d, %f, %f, %f, %d, %d, OPTIMAL\n", inst->instance_name, inst->param.alpha, inst->best_lb, inst->z_best, time_elapsed, inst->number_feasible_truck_legs, number_unique_truck_legs);
    if (fclose(csv))
        print_error("generate_csv_record fclose() error");




// Free and close CPLEX model
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);

    return 0;


}


static int CPXPUBLIC callback_driver(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle)
{
    if (contextid == CPX_CALLBACKCONTEXT_CANDIDATE)
        return callback_candidate2(context, contextid, userhandle);
    if (contextid == CPX_CALLBACKCONTEXT_RELAXATION)
        return callback_relaxation(context, contextid, userhandle);
    print_error("contextid unknownn in my_callback");
    return 1;
}


// static int CPXPUBLIC callback_candidate(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle)
// {
//     printf("*** Inside callback_candidate ***\n");

//     instance *inst = (instance *)userhandle;
//     double *xstar = (double *)malloc(inst->cols * sizeof(double));
//     double objval = CPX_INFBOUND;

//     // get the candidate solution
//     if (CPXcallbackgetcandidatepoint(context, xstar, 0, inst->cols - 1, &objval))
//         print_error("CPXcallbackgetcandidatepoint error");


//     int truck_succ[inst->dimension];
//     int drone_succ[inst->dimension];
//     gather_solution_leg_formulation(inst, xstar, truck_succ, drone_succ);
//     save_and_plot_solution_succ(inst, truck_succ, drone_succ, objval);



//     int *comp = NULL;
//     int *comp_cust = NULL;
//     int *succ = NULL;
//     int ncomp = 0; // number of connected components
//     int *length_comp = NULL;

//     // Retrieve the connected components of the current solution
//     findConnectedComponents(xstar, inst, &succ, &comp, &comp_cust, &ncomp, &length_comp);

//     if (ncomp == 1) {
//         if (objval < inst->z_best)
//             inst->z_best = objval;
//     }


//     printf("Print connected components...\n");
//     for (int i = 0; i < inst->dimension; i++) {
//         printf("comp[%d] = %d, comp_cust[%d] = %d\n", i, comp[i], i, comp_cust[i]);
//     }

//     printf("\n");
//     for (int C = 0; C < ncomp; C++) {
//         printf("length comp %d = %d\n", C, length_comp[C]);
//     }

//     // char c;
//     // scanf("%c", &c);

//     if (ncomp > 1 /*&& objval < inst->z_best*/)
//     {
//         // add one cut for each connected component
//         for (int mycomp = 0; mycomp < ncomp; mycomp++)
//         {
//             int nnz = 0;
//             int izero = 0;
//             char sense = 'L';
//             double rhs = length_comp[mycomp] - 1.0; // |S|-1
//             if (mycomp == comp[0])
//                 rhs = length_comp[mycomp] - 2.0;
//             int *index = (int *)calloc(inst->cols, sizeof(int));
//             double *value = (double *)calloc(inst->cols, sizeof(double));

//             for (int i = 0; i < inst->dimension - 1; i++)
//             {
//                 if (comp[i] != mycomp)
//                     continue;
//                 for (int k = 1; k < inst->dimension; k++)
//                 {
//                     if (k == i) continue;
//                     if (comp[k] != mycomp)
//                         continue;
//                     index[nnz] = x_pos_leg(i, k, inst);
//                     value[nnz++] = 1.0;
//                     for (int j = 1; j < inst->dimension - 1; j++) {
//                         if (j == i || j == k) continue;
//                         index[nnz] = z_pos_leg(i, j, k, inst);
//                         if (comp[j] == mycomp)
//                             value[nnz++] = 2.0;
//                         else
//                             value[nnz++] = 1.0;
//                     }
//                 }
//             }
//             // reject the solution and adds one cut
//             if (CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense, &izero, index, value))
//                 print_error("CPXcallbackrejectcandidate() error");
//             free(index);
//             free(value);
//         }

//         // add one cut for each connected component
//         for (int mycomp = 0; mycomp < ncomp; mycomp++)
//         {
//             int nnz = 0;
//             int izero = 0;
//             char sense = 'L';
//             double rhs = length_comp[mycomp] - 1.0; // |S|-1
//             if (mycomp == comp[0])
//                 rhs = length_comp[mycomp] - 2.0;
//             int *index = (int *)calloc(inst->cols, sizeof(int));
//             double *value = (double *)calloc(inst->cols, sizeof(double));

//             for (int i = 1; i < inst->dimension-1;i++){
//                 if (comp_cust[i] == mycomp)
//                     rhs += 1.0;
//             }

//             for (int i = 0; i < inst->dimension - 1; i++)
//             {
//                 if (comp[i] != mycomp && comp_cust[i] != mycomp)
//                     continue;
//                 for (int k = 1; k < inst->dimension; k++)
//                 {
//                     if (k == i) continue;
//                     if (comp[k] != mycomp && comp_cust[k] != mycomp)
//                         continue;
//                     index[nnz] = x_pos_leg(i, k, inst);
//                     value[nnz++] = 1.0;
//                     for (int j = 1; j < inst->dimension - 1; j++) {
//                         if (j == i || j == k) continue;
//                         index[nnz] = z_pos_leg(i, j, k, inst);
//                         if (comp[j] == mycomp || comp_cust[j] == mycomp)
//                             value[nnz++] = 2.0;
//                         else
//                             value[nnz++] = 1.0;
//                     }
//                 }
//             }
//             // reject the solution and adds one cut
//             if (CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense, &izero, index, value))
//                 print_error("CPXcallbackrejectcandidate() error2");
//             free(index);
//             free(value);
//         }

//     }
//     if (ncomp == 1) {
//         printf("ncomp = 1, obj = %.2f\n", objval);
//         // char c;
//         // scanf("%c",&c);
//     }
//     // else if (inst->param.opt)
//     // {
//     //     // the candidate solution has not connected components but could have crossings... let's apply 2-opt
//     //     double delta = two_opt_v2(inst, succ, 0);
//     //     if (delta < 0)
//     //     {
//     //         objval += delta;
//     //         // succ -> xstar
//     //         int nnz = 0;
//     //         int izero = 0;
//     //         int *index = (int *)calloc(inst->cols, sizeof(int));
//     //         double *xstar_succ = (double *)calloc(inst->cols, sizeof(double));

//     //         for (int i = 0; i < inst->dimension; i++)
//     //         {
//     //             for (int j = i + 1; j < inst->dimension; j++)
//     //             {
//     //                 index[nnz] = xpos(i, j, inst);
//     //                 if (j == succ[i] || i == succ[j])
//     //                 {
//     //                     xstar_succ[nnz++] = 1.0;
//     //                 }
//     //                 else
//     //                     xstar_succ[nnz++] = 0.0;
//     //             }
//     //         }
//     //         // sanity check
//     //         if (nnz != inst->cols)
//     //             print_error("Error in applying 2-opt in callback_candidate");

//     //         if (CPXcallbackpostheursoln(context, nnz, index, xstar_succ, objval, CPXCALLBACKSOLUTION_CHECKFEAS))
//     //             print_error("CPXcallbackpostheursoln() error");
//     //         if (inst->param.verbose >= DEBUG)
//     //             printf("[callback_candidate] Posted a new solution to CPLEX with incumbent: %f\n", objval);
//     //         free(xstar_succ);
//     //         free(index);
//     //     }
//     // }
//     free(comp);
//     free(succ);
//     free(length_comp);
//     free(xstar);
//     return 0;
// }


static int CPXPUBLIC callback_candidate2(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle)
{
    printf("*** Inside callback_candidate2 ***\n");

    instance *inst = (instance *)userhandle;
    double *xstar = (double *)malloc(inst->cols * sizeof(double));
    double objval = CPX_INFBOUND;

    // get the candidate solution
    if (CPXcallbackgetcandidatepoint(context, xstar, 0, inst->cols - 1, &objval))
        print_error("CPXcallbackgetcandidatepoint error");


    int truck_succ[inst->dimension];
    int drone_succ[inst->dimension];
    gather_solution_leg_formulation(inst, xstar, truck_succ, drone_succ);
    save_and_plot_solution_succ(inst, truck_succ, drone_succ, objval);

    // for (int i = 0; i < inst->dimension; i++)
    // {
    //     for (int j = 1; j < inst->dimension; j++)
    //     {
    //         if (xstar[x_pos_leg(i, j, inst)] > 0.001) // just save the selected (also partial) edges
    //         {
    //             // elist[loader++] = i;
    //             // elist[loader++] = j;
    //             // x[ecount++] = xstar[xpos(i, j, inst)];
    //             printf("x(%d,%d) = %f\n", i, j, xstar[x_pos_leg(i, j, inst)]);
    //         }
    //         for (int k = 1; k < inst->dimension; k++) {
    //             if (xstar[z_pos_leg(i, j, k, inst)] > 0.001) // just save the selected (also partial) edges
    //             {
    //                 printf("z(%d,%d,%d) = %f\n", i, j, k, xstar[z_pos_leg(i, j, k, inst)]);
    //                 // z[k] += xstar[z_pos_leg(i, j, k, inst)];
    //             }
    //         }

    //     }
    // }

    // for (int i = 0; i < inst->dimension; i++)
    // {
    //     for (int k = 1; k < inst->dimension; k++) {
    //         for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
    //             int l = inst->feasible_truck_legs2[i][k].legs[leg_id].ID;
    //             if (xstar[y_pos_leg(l, inst)] > 0.001) // just save the selected (also partial) edges
    //             {
    //                 printf("y(%d)[%d,%d] = %f {", l, i, k, xstar[y_pos_leg(l, inst)]);
    //                 int* truck_leg = inst->feasible_truck_legs2[i][k].legs[leg_id].leg;
    //                 for (int cust = 0; cust < inst->feasible_truck_legs2[i][k].legs[leg_id].length; cust++) {
    //                     printf("%d,", truck_leg[cust]);
    //                 }
    //                 printf("}\n");
    //             }
    //         }
    //     }
    // }

    int *comp = NULL;
    int *comp_cust = NULL;
    int *succ = NULL;
    int ncomp = 0; // number of connected components
    int *length_comp = NULL;

    // Retrieve the connected components of the current solution
    findConnectedComponents(xstar, inst, &succ, &comp, &ncomp, &length_comp);

    // if (ncomp == 1) {
    //     if (objval < inst->z_best)
    //         inst->z_best = objval;
    // }

    if (ncomp == 1) {
        // printf("length comp = %d\n", length_comp[0]);
        // for (int i = 0; i < inst->dimension; i++) {
        //     printf("comp[%d] = %d\n", i, comp[i]);
        // }
        if (length_comp[0] == inst->dimension) {
            if (objval < inst->z_best)
                inst->z_best = objval;
        }
        else {
            // printf("*** length_comp < inst->dimension\n");
            // add one cut for the main component from depot to depot
            {
                int mycomp = 0;
                int nnz = 0;
                int izero = 0;
                char sense = 'G';
                double rhs = 1.0;
                int *index = (int *)calloc(inst->cols, sizeof(int));
                double *value = (double *)calloc(inst->cols, sizeof(double));
                for (int i = 0; i < inst->dimension - 1; i++)
                {
                    if (comp[i] != mycomp)
                        continue;
                    // outgoing arc from S to V\S through a combined arc
                    for (int k = 1; k < inst->dimension; k++)
                    {
                        if (k == i) continue;
                        if (comp[k] == mycomp)
                            continue;
                        index[nnz] = x_pos_leg(i, k, inst);
                        value[nnz++] = 1.0;
                    }
                    // outgoing arc from S to V\S through an operation z_ijk, with i in S, j or k in V\S
                    for (int k = 1; k < inst->dimension; k++)
                    {
                        if (k == i) continue;
                        for (int j = 1; j < inst->dimension - 1; j++) {
                            if (j == i || j == k) continue;
                            if (comp[j] == mycomp && comp[k] == mycomp)
                                continue;
                            index[nnz] = z_pos_leg(i, j, k, inst);
                            value[nnz++] = 1.0;
                        }
                    }
                    // truck leg y_l with l in L_ik, with i,k in S, such that C(l)\S != empty set
                    for (int k = 1; k < inst->dimension; k++) {
                        if (comp[k] != mycomp) continue;
                        for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                            int l = inst->feasible_truck_legs2[i][k].legs[leg_id].ID;
                            int flag_leg_constraint = 0;
                            const int* truck_leg = inst->feasible_truck_legs2[i][k].legs[leg_id].leg;
                            for (int truck_cust_id = 1; truck_cust_id < inst->feasible_truck_legs2[i][k].legs[leg_id].length - 1; truck_cust_id++) {
                                if (comp[truck_leg[truck_cust_id]] != mycomp) {
                                    flag_leg_constraint = 1;
                                    break;
                                }
                            }
                            if (flag_leg_constraint) {
                                index[nnz] = y_pos_leg(l, inst);
                                value[nnz++] = 1.0;
                            }
                        }
                    }
                }
                // reject the solution and adds one cut
                if (CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense, &izero, index, value))
                    print_error("CPXcallbackrejectcandidate() error");
                free(index);
                free(value);
            }

        }
    }


    // printf("Print connected components...\n");
    // for (int i = 0; i < inst->dimension; i++) {
    //     printf("comp[%d] = %d\n", i, comp[i]);
    // }

    // printf("\n");
    // for (int C = 0; C < ncomp; C++) {
    //     printf("length comp %d = %d\n", C, length_comp[C]);
    // }

    // char c;
    // scanf("%c", &c);

    if (ncomp > 1)
    {
        // add one cut for the main component from depot to depot
        {
            int mycomp = 0;
            int nnz = 0;
            int izero = 0;
            char sense = 'G';
            double rhs = 1.0;
            int *index = (int *)calloc(inst->cols, sizeof(int));
            double *value = (double *)calloc(inst->cols, sizeof(double));
            for (int i = 0; i < inst->dimension - 1; i++)
            {
                if (comp[i] != mycomp)
                    continue;
                // outgoing arc from S to V\S through a combined arc
                for (int k = 1; k < inst->dimension; k++)
                {
                    if (k == i) continue;
                    if (comp[k] == mycomp)
                        continue;
                    index[nnz] = x_pos_leg(i, k, inst);
                    value[nnz++] = 1.0;
                }
                // outgoing arc from S to V\S through an operation z_ijk, with i in S, j or k in V\S
                for (int k = 1; k < inst->dimension; k++)
                {
                    if (k == i) continue;
                    for (int j = 1; j < inst->dimension - 1; j++) {
                        if (j == i || j == k) continue;
                        if (comp[j] == mycomp && comp[k] == mycomp)
                            continue;
                        index[nnz] = z_pos_leg(i, j, k, inst);
                        value[nnz++] = 1.0;
                    }
                }
                // truck leg y_l with l in L_ik, with i,k in S, such that C(l)\S != empty set
                for (int k = 1; k < inst->dimension; k++) {
                    if (comp[k] != mycomp) continue;
                    for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                        int l = inst->feasible_truck_legs2[i][k].legs[leg_id].ID;
                        int flag_leg_constraint = 0;
                        const int* truck_leg = inst->feasible_truck_legs2[i][k].legs[leg_id].leg;
                        for (int truck_cust_id = 1; truck_cust_id < inst->feasible_truck_legs2[i][k].legs[leg_id].length - 1; truck_cust_id++) {
                            if (comp[truck_leg[truck_cust_id]] != mycomp) {
                                flag_leg_constraint = 1;
                                break;
                            }
                        }
                        if (flag_leg_constraint) {
                            index[nnz] = y_pos_leg(l, inst);
                            value[nnz++] = 1.0;
                        }
                    }
                }
            }
            // reject the solution and adds one cut
            if (CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense, &izero, index, value))
                print_error("CPXcallbackrejectcandidate() error");
            free(index);
            free(value);
        }


        for (int mycomp = 1; mycomp < ncomp; mycomp++)
        {
            int nnz = 0;
            int izero = 0;
            char sense = 'G';
            double rhs = 1.0;
            int *index = (int *)calloc(inst->cols, sizeof(int));
            double *value = (double *)calloc(inst->cols, sizeof(double));
            for (int i = 0; i < inst->dimension - 1; i++)
            {
                if (comp[i] == mycomp)
                    continue;
                // arc from V\S to S through a combined arc
                for (int k = 1; k < inst->dimension; k++)
                {
                    if (k == i) continue;
                    if (comp[k] != mycomp)
                        continue;
                    index[nnz] = x_pos_leg(i, k, inst);
                    value[nnz++] = 1.0;
                }
                // arc from V\S to S through an operation z_ijk, with i in V\S, j or k in S
                for (int k = 1; k < inst->dimension; k++)
                {
                    if (k == i) continue;
                    for (int j = 1; j < inst->dimension - 1; j++) {
                        if (j == i || j == k) continue;
                        if (comp[j] != mycomp && comp[k] != mycomp)
                            continue;
                        index[nnz] = z_pos_leg(i, j, k, inst);
                        value[nnz++] = 1.0;
                    }
                }
                // truck leg y_l with l in L_ik, with i,k in V\S, such that C(l) intersection (S) != empty set
                for (int k = 1; k < inst->dimension; k++) {
                    if (comp[k] == mycomp) continue;
                    for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                        int l = inst->feasible_truck_legs2[i][k].legs[leg_id].ID;
                        int flag_leg_constraint = 0;
                        const int* truck_leg = inst->feasible_truck_legs2[i][k].legs[leg_id].leg;
                        for (int truck_cust_id = 1; truck_cust_id < inst->feasible_truck_legs2[i][k].legs[leg_id].length - 1; truck_cust_id++) {
                            if (comp[truck_leg[truck_cust_id]] == mycomp) {
                                flag_leg_constraint = 1;
                                break;
                            }
                        }
                        if (flag_leg_constraint) {
                            index[nnz] = y_pos_leg(l, inst);
                            value[nnz++] = 1.0;
                        }
                    }
                }
            }
            // reject the solution and adds one cut
            if (CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense, &izero, index, value))
                print_error("CPXcallbackrejectcandidate() error");
            free(index);
            free(value);
        }


        /*
        // add one cut for each connected component
        for (int mycomp = 1; mycomp < ncomp; mycomp++)
        {
            int nnz = 0;
            int izero = 0;
            char sense = 'G';
            // double rhs = 1.0;
            double rhs = length_comp[mycomp];
            int *index = (int *)calloc(inst->cols, sizeof(int));
            double *value = (double *)calloc(inst->cols, sizeof(double));

            for (int i = 0; i < inst->dimension - 1; i++)
            {
                if (comp[i] != mycomp)
                    continue;
                // a) outgoing arc from S to V\S through a combined arc
                for (int k = 1; k < inst->dimension; k++)
                {
                    if (k == i) continue;
                    if (comp[k] == mycomp)
                        continue;
                    index[nnz] = x_pos_leg(i, k, inst);
                    // value[nnz++] = 1.0;
                    value[nnz++] = length_comp[mycomp];
                }
                // b) outgoing arc from S to V\S through an operation z_ijk, with i in S, k in V\S (j in V)
                for (int k = 1; k < inst->dimension; k++)
                {
                    if (k == i) continue;
                    if (comp[k] == mycomp) continue;
                    for (int j = 1; j < inst->dimension - 1; j++) {
                        if (j == i || j == k) continue;
                        index[nnz] = z_pos_leg(i, j, k, inst);
                        // value[nnz++] = 1.0;
                        value[nnz++] = length_comp[mycomp];
                    }
                }
                // c) outgoing arc from S to V\S through an operation z_ijk, with i in S, j in V\S, k in S
                // for (int j = 1; j < inst->dimension - 1; j++) {
                //     if (j == i) continue;
                //     if (comp[j] == mycomp) continue;
                //     for (int k = 1; k < inst->dimension; k++)
                //     {
                //         if (k == i || k == j) continue;
                //         if (comp[k] != mycomp) continue;
                //         index[nnz] = z_pos_leg(i, j, k, inst);
                //         value[nnz++] = 1.0;
                //     }
                // }
                // d) truck leg y_l with l in L_ik, with i,k in S, such that C(l)\S != empty set
                // for (int k = 1; k < inst->dimension; k++) {
                //     if (comp[k] != mycomp) continue;
                //     if (inst->feasible_truck_legs_ik_number[i][k] > 0) {
                //         for (int l = inst->feasible_truck_legs_ik_begin[i][k]; l < inst->feasible_truck_legs_ik_begin[i][k] + inst->feasible_truck_legs_ik_number[i][k]; l++) {
                //             int flag_leg_constraint = 0;
                //             const int* truck_leg = inst->feasible_truck_legs[l];
                //             for (int truck_cust_id = 2; truck_cust_id < truck_leg[0]; truck_cust_id++) {
                //                 if (comp[truck_leg[truck_cust_id]] != mycomp) {
                //                     flag_leg_constraint = 1;
                //                     break;
                //                 }
                //             }
                //             if (flag_leg_constraint) {
                //                 index[nnz] = y_pos_leg(l, inst);
                //                 value[nnz++] = 1.0;
                //             }
                //         }
                //     }
                // }
            }
            // e) ingoing drone leg z_ijk, with j in S, i,k in V\S
            for (int i = 0; i < inst->dimension - 1; i++) {
                if (comp[i] == mycomp) continue;
                for (int k = 1; k < inst->dimension; k++) {
                    if (k == i) continue;
                    if (comp[k] == mycomp) continue;
                    for (int j = 1; j < inst->dimension - 1; j++) {
                        if (j == i || j == k) continue;
                        if (comp[j] != mycomp) continue;
                        index[nnz] = z_pos_leg(i, j, k, inst);
                        value[nnz++] = 1.0;
                    }
                }
            }
            // f) truck leg l in L_ik with i,k in V\S, such that C(l) intersection (S) != empty set
            for (int i = 0; i < inst->dimension - 1; i++) {
                if (comp[i] == mycomp) continue;
                for (int k = 1; k < inst->dimension; k++) {
                    if (k == i) continue;
                    if (comp[k] == mycomp) continue;
                    for (int l = inst->feasible_truck_legs_ik_begin[i][k]; l < inst->feasible_truck_legs_ik_begin[i][k] + inst->feasible_truck_legs_ik_number[i][k]; l++) {
                        int flag_leg_constraint = 0;
                        const int* truck_leg = inst->feasible_truck_legs[l];
                        for (int truck_cust_id = 2; truck_cust_id < truck_leg[0]; truck_cust_id++) {
                            if (comp[truck_leg[truck_cust_id]] == mycomp) {
                                flag_leg_constraint = 1;
                                break;
                            }
                        }
                        if (flag_leg_constraint) {
                            index[nnz] = y_pos_leg(l, inst);
                            value[nnz++] = 1.0;
                        }
                    }
                }
            }

            // reject the solution and adds one cut
            if (CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense, &izero, index, value))
                print_error("CPXcallbackrejectcandidate() error");
            free(index);
            free(value);
        }
        */


    }
    // if (ncomp == 1) {
    //     printf("ncomp = 1, obj = %.2f\n", objval);
    //     // char c;
    //     // scanf("%c",&c);
    // }

    free(comp);
    free(succ);
    free(length_comp);
    free(xstar);
    return 0;
}


static int CPXPUBLIC callback_candidate3(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle)
{
    printf("*** Inside callback_candidate3 ***\n");

    instance *inst = (instance *)userhandle;
    double *xstar = (double *)malloc(inst->cols * sizeof(double));
    double objval = CPX_INFBOUND;

    // get the candidate solution
    if (CPXcallbackgetcandidatepoint(context, xstar, 0, inst->cols - 1, &objval))
        print_error("CPXcallbackgetcandidatepoint error");


    int truck_succ[inst->dimension];
    int drone_succ[inst->dimension];
    gather_solution_leg_formulation(inst, xstar, truck_succ, drone_succ);
    save_and_plot_solution_succ(inst, truck_succ, drone_succ, objval);



    int *comp = NULL;
    int *comp_cust = NULL;
    int *succ = NULL;
    int ncomp = 0; // number of connected components
    int *length_comp = NULL;

    // Retrieve the connected components of the current solution
    findConnectedComponents(xstar, inst, &succ, &comp, &ncomp, &length_comp);

    if (ncomp == 1) {
        if (objval < inst->z_best)
            inst->z_best = objval;
    }


    printf("Print connected components...\n");
    for (int i = 0; i < inst->dimension; i++) {
        printf("comp[%d] = %d\n", i, comp[i]);
    }

    printf("\n");
    for (int C = 0; C < ncomp; C++) {
        printf("length comp %d = %d\n", C, length_comp[C]);
    }

    // char c;
    // scanf("%c", &c);

    if (ncomp > 1 /*&& objval < inst->z_best*/)
    {
        for (int mycomp = 0; mycomp < ncomp; mycomp++)
        {
            int nnz = 0;
            int izero = 0;
            char sense = 'L';
            double rhs = length_comp[mycomp] - 1.0;
            if (mycomp == 0)
                rhs = length_comp[mycomp] - 2.0;
            int *index = (int *)calloc(inst->cols, sizeof(int));
            double *value = (double *)calloc(inst->cols, sizeof(double));
            for (int i = 0; i < inst->dimension - 1; i++)
            {
                if (comp[i] != mycomp)
                    continue;
                // arc from S to S through a combined arc
                for (int k = 1; k < inst->dimension; k++)
                {
                    if (k == i) continue;
                    if (comp[k] != mycomp)
                        continue;
                    index[nnz] = x_pos_leg(i, k, inst);
                    value[nnz++] = 1.0;
                }
                // arc from S to S through an operation z_ijk, with i,k in S, j in V
                for (int k = 1; k < inst->dimension; k++)
                {
                    if (k == i) continue;
                    for (int j = 1; j < inst->dimension - 1; j++) {
                        if (j == i || j == k) continue;
                        if (comp[k] != mycomp)
                            continue;
                        index[nnz] = z_pos_leg(i, j, k, inst);
                        if (comp[j] == mycomp)
                            value[nnz++] = 2.0;
                        else
                            value[nnz++] = 1.0;
                    }
                }
                // truck leg y_l with l in L_ik, with i,k in S, such that C(l) intersection (S) != empty set
                // for (int k = 1; k < inst->dimension; k++) {
                //     if (comp[k] != mycomp) continue;
                //     for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                //         int l = inst->feasible_truck_legs2[i][k].legs[leg_id].ID;
                //         const int* truck_leg = inst->feasible_truck_legs2[i][k].legs[leg_id].leg;
                //         int n_cust = 0;
                //         for (int truck_cust_id = 2; truck_cust_id < truck_leg[0]; truck_cust_id++) {
                //             if (comp[truck_leg[truck_cust_id]] == mycomp) {
                //                 n_cust++;
                //             }
                //         }
                //         if (n_cust > 0) {
                //             index[nnz] = y_pos_leg(l, inst);
                //             value[nnz++] = n_cust;
                //         }
                //     }
                // }
            }
            // for (int i = 0; i < inst->dimension - 1; i++) {
            //     if (comp[i] == mycomp)
            //         continue;
            //     for (int k = 1; k < inst->dimension; k++) {
            //         if (k == i) continue;
            //         if (comp[k] == mycomp)
            //             continue;
            //         for (int j = 1; j < inst->dimension - 1; j++) {
            //             if (j == i || j == k) continue;
            //             if (comp[j] != mycomp)
            //                 continue;
            //             index[nnz] = z_pos_leg(i, j, k, inst);
            //             value[nnz++] = 1.0;
            //         }
            //         if (inst->feasible_truck_legs_ik_number[i][k] > 0) {
            //             for (int l = inst->feasible_truck_legs_ik_begin[i][k]; l < inst->feasible_truck_legs_ik_begin[i][k] + inst->feasible_truck_legs_ik_number[i][k]; l++) {
            //                 int n_cust = 0;
            //                 const int* truck_leg = inst->feasible_truck_legs[l];
            //                 for (int truck_cust_id = 2; truck_cust_id < truck_leg[0]; truck_cust_id++) {
            //                     if (comp[truck_leg[truck_cust_id]] == mycomp) {
            //                         n_cust++;
            //                     }
            //                 }
            //                 if (n_cust > 0) {
            //                     index[nnz] = y_pos_leg(l, inst);
            //                     value[nnz++] = n_cust;
            //                 }
            //             }
            //         }
            //     }
            // }
            // for (int i = 0; i < inst->dimension - 1; i++)
            // {
            //     if (comp[i] == mycomp)
            //         continue;
            //     for (int k = 1; k < inst->dimension; k++) {
            //         if (k == i) continue;
            //         if (comp[k] != mycomp)
            //             continue;
            //         index[nnz] = x_pos_leg(i, k, inst);
            //         value[nnz++] = -1.0;
            //         for (int j = 1; j < inst->dimension - 1; j++) {
            //             if (j == i || j == k) continue;
            //             if (comp[j] == mycomp) continue;
            //         }
            //         if (inst->feasible_truck_legs_ik_number[i][k] > 0) {
            //             for (int l = inst->feasible_truck_legs_ik_begin[i][k]; l < inst->feasible_truck_legs_ik_begin[i][k] + inst->feasible_truck_legs_ik_number[i][k]; l++) {
            //                 int n_cust = 0;
            //                 const int* truck_leg = inst->feasible_truck_legs[l];
            //                 for (int truck_cust_id = 2; truck_cust_id < truck_leg[0]; truck_cust_id++) {
            //                     if (comp[truck_leg[truck_cust_id]] == mycomp) {
            //                         n_cust++;
            //                     }
            //                 }
            //                 if (n_cust > 0) {
            //                     index[nnz] = y_pos_leg(l, inst);
            //                     value[nnz++] = -n_cust;
            //                 }
            //             }
            //         }
            //     }
            // }
            // reject the solution and adds one cut
            if (CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense, &izero, index, value))
                print_error("CPXcallbackrejectcandidate() error");
            free(index);
            free(value);
        }


    }
    if (ncomp == 1) {
        printf("ncomp = 1, obj = %.2f\n", objval);
        // char c;
        // scanf("%c",&c);
    }

    free(comp);
    free(succ);
    free(length_comp);
    free(xstar);
    return 0;
}




void findConnectedComponents(const double * xstar, instance * inst, int **succ, int **comp, int *ncomp, int **length_comp)
{
    assert(*comp == NULL);
    assert(*succ == NULL);
    assert(*length_comp == NULL);
    *comp = (int *)calloc(inst->dimension, sizeof(int));
    // *comp_cust = (int *)calloc(inst->dimension, sizeof(int));
    // *succ = (int *)calloc(inst->dimension, sizeof(int));
    // build succ[] and comp[] wrt xstar()...

    // initialization
    *ncomp = 0;
    for (int i = 0; i < inst->dimension; i++)
    {
        // (*succ)[i] = -1;
        (*comp)[i] = -1;
    }

    for (int start = 0; start < inst->dimension; start++)
    {
        if ((*comp)[start] != -1)
            continue; // node "start" was already visited, just skip it

        // check that start is not a truck/drone only customer
        int flag_truck_drone_customer = 1;
        for (int k = 1; k < inst->dimension; k++) {
            if ((*comp)[k] != -1 || k == start) continue;
            if (xstar[x_pos_leg(start, k, inst)] > 0.9) {
                flag_truck_drone_customer = 0;
                break;
            }
        }
        if (flag_truck_drone_customer == 1) {
            for (int j = 1; j < inst->dimension - 1; j++) {
                if ((*comp)[j] != -1 || j == start) continue;
                for (int k = 1; k < inst->dimension; k++) {
                    if ((*comp)[k] != -1 || k == start || k == j) continue;
                    if (xstar[z_pos_leg(start, j, k, inst)] > 0.9) {
                        flag_truck_drone_customer = 0;
                        break;
                    }
                }
            }
        }
        if (flag_truck_drone_customer == 1)
            continue;
        assert((*comp)[start] == -1);

        // a new component is found
        (*ncomp)++;
        (*length_comp) = (int *)realloc(*length_comp, (*ncomp) * sizeof(int));
        int i = start;
        int length = 1;
        int done = 0;
        while (!done && i != inst->dimension - 1) // go and visit the current component
        {
            (*comp)[i] = (*ncomp) - 1;
            done = 1;
            // look for a combined arc that starts from i
            for (int j = 0; j < inst->dimension; j++)
            {
                if (i == j)
                    continue;
                // if ((*comp)[j] == -1 && xstar[x_pos_leg(i, j, inst)] > 0.9) // the edge [i,j] is selected in xstar and j was not visited before
                if (xstar[x_pos_leg(i, j, inst)] > 0.9) // the edge [i,j] is selected in xstar and j was not visited before
                {
                    if ((*comp)[j] == (*comp)[i] || j == inst->dimension - 1)
                    {
                        if (j == inst->dimension - 1) {
                            (*comp)[j] = (*comp)[i];
                            length++;
                        }
                        (*length_comp)[(*ncomp) - 1] = length; // save length of the cycle
                        done = 1;
                        goto END_OF_WHILE;
                    }
                    (*comp)[j] = (*comp)[i];
                    length++;
                    i = j;
                    done = 0;
                    break;
                }
            }
            if (done == 1) // look for a drone leg that starts from i
            {
                for (int j = 1; j < inst->dimension - 1; j++) {
                    if (j == i) continue;
                    for (int k = 1; k < inst->dimension; k++) {
                        if (k == i || k == j) continue;
                        // if ((*comp)[j] == -1 && (*comp)[k] == -1 && xstar[z_pos_leg(i, j, k, inst)] > 0.9) // the edge [i,j] is selected in xstar and j was not visited before
                        if (xstar[z_pos_leg(i, j, k, inst)] > 0.9) // the edge [i,j] is selected in xstar and j was not visited before
                        {
                            // look for the associated truck leg
                            for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                                int l = inst->feasible_truck_legs2[i][k].legs[leg_id].ID;
                                if (xstar[y_pos_leg(l, inst)] > 0.9) {
                                    const int* truck_leg = inst->feasible_truck_legs2[i][k].legs[leg_id].leg;
                                    for (int truck_cust_id = 0; truck_cust_id < inst->feasible_truck_legs2[i][k].legs[leg_id].length - 1; truck_cust_id++) {
                                        (*comp)[truck_leg[truck_cust_id]] = (*comp)[i];
                                        if (truck_cust_id > 0)
                                            length++;
                                    }
                                    break;
                                }
                            }

                            (*comp)[j] = (*comp)[i];
                            length++;
                            if ((*comp)[k] == (*comp)[i] || k == inst->dimension - 1)
                            {
                                if (k == inst->dimension - 1) {
                                    (*comp)[k] = (*comp)[i];
                                    length++;
                                }
                                (*length_comp)[(*ncomp) - 1] = length; // save length of the cycle
                                done = 1;
                                goto END_OF_WHILE;
                            }
                            // (*succ)[i] = j;
                            // (*succ)[j] = k;
                            (*comp)[k] = (*comp)[i];
                            length++;
                            i = k;
                            done = 0;
                            break;
                        }
                    }
                    if (done == 0)
                        break;
                }
            }
        }
        // if (i != inst->dimension - 1) {
        //     (*succ)[i] = start;                       // last arc to close the cycle
        //     (*length_comp)[(*ncomp) - 1] = length; // save length of the cycle
        // }
        // else {
        //     (*length_comp)[(*ncomp) - 1] = length - 1; // save length of the cycle
        // }
END_OF_WHILE:;
        // go to the next component...
    }
}



static int CPXPUBLIC callback_relaxation(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle)
{
    printf("*** Inside callback_relaxation\n");
    int node_depth;
    CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEDEPTH, &node_depth);
    // if (!node_depth)
    //     return 0;
    instance *inst = (instance *)userhandle;
    double *xstar = (double *)malloc(inst->cols * sizeof(double));
    double objval = CPX_INFBOUND;
    double const eps = 0.1;

    if (CPXcallbackgetrelaxationpoint(context, xstar, 0, inst->cols - 1, &objval))
        print_error("CPXcallbackgetrelaxationpoint error");

    double x_val[inst->dimension];
    double z[inst->dimension];
    double y[inst->dimension];
    for (int i = 0; i < inst->dimension; i++) {
        x_val[i] = 0.0;
        z[i] = 0.0;
        y[i] = 0.0;
    }

    int ncomp;
    int *comp = (int *)calloc(inst->dimension, sizeof(int));
    int *length_comp = (int *)calloc(inst->dimension, sizeof(int));
    // list of edges in "node format" [ i_1, j_1 , i_2, j_2, i_3, j_3, ...]
    int *elist = malloc(2 * inst->cols * sizeof(int));

    int loader = 0;
    int ecount = 0; // edge count
    double *x = malloc(inst->cols * sizeof(double));

    double sum_flow = 0.0;
    for (int i = 0; i < inst->dimension; i++)
    {
        for (int j = 1; j < inst->dimension; j++)
        {
            if (xstar[x_pos_leg(i, j, inst)] > 0.001) // just save the selected (also partial) edges
            {
                // elist[loader++] = i;
                // elist[loader++] = j;
                // x[ecount++] = xstar[xpos(i, j, inst)];
                printf("x(%d,%d) = %f\n", i, j, xstar[x_pos_leg(i, j, inst)]);
                sum_flow += xstar[x_pos_leg(i, j, inst)];
                x_val[j] += xstar[x_pos_leg(i, j, inst)];
            }
            for (int k = 1; k < inst->dimension; k++) {
                if (xstar[z_pos_leg(i, j, k, inst)] > 0.001) // just save the selected (also partial) edges
                {
                    printf("z(%d,%d,%d) = %f\n", i, j, k, xstar[z_pos_leg(i, j, k, inst)]);
                    sum_flow += 2 * xstar[z_pos_leg(i, j, k, inst)];
                    z[j] += xstar[z_pos_leg(i, j, k, inst)];
                    // z[k] += xstar[z_pos_leg(i, j, k, inst)];
                }
            }

        }
    }

    for (int i = 0; i < inst->dimension; i++)
    {
        for (int k = 1; k < inst->dimension; k++) {
            for (int leg_id = 0; leg_id < inst->feasible_truck_legs2[i][k].count; leg_id++) {
                int l = inst->feasible_truck_legs2[i][k].legs[leg_id].ID;
                if (xstar[y_pos_leg(l, inst)] > 0.001) // just save the selected (also partial) edges
                {
                    printf("y(%d)[%d,%d] = %f {", l, i, k, xstar[y_pos_leg(l, inst)]);
                    int* truck_leg = inst->feasible_truck_legs2[i][k].legs[leg_id].leg;
                    for (int cust = 0; cust < inst->feasible_truck_legs2[i][k].legs[leg_id].length; cust++) {
                        printf("%d,", truck_leg[cust]);
                        if (cust > 0 )
                            y[truck_leg[cust]] += xstar[y_pos_leg(l, inst)];
                    }
                    printf("}\n");
                    sum_flow += (inst->feasible_truck_legs2[i][k].legs[leg_id].length - 2) * xstar[y_pos_leg(l, inst)];
                }
            }
        }
    }

    // for (int j = 1; j < inst->dimension - 1; j++)
    // {
    //     // if (xstar[b_pos_leg(j, inst)] > 0.001) // just save the selected (also partial) edges
    //     {
    //         printf("b(%d) = %f\n", j, xstar[b_pos_leg(j, inst)]);
    //     }
    // }
    printf("\nsum_flow = %f\n", sum_flow);

    for (int i = 0; i < inst->dimension; i++) {
        printf("x_val[%d] = %.2f, z[%d] = %.2f, y[%d] = %.2f\n", i, x_val[i] , i, z[i], i, y[i]);
        // if (x_val[i] < 0.9999 && y[i] < 0.9999)
        //     printf("*** ACHTUNG!!! *** \n");
    }
    char c;
    scanf("%c\n", &c);

    free(comp);
    free(length_comp);
    free(xstar);
    // print_error("done\n");
    return 0;
}
