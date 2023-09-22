double optimal_solver(instance *inst, instance_TSP *inst_tsp, int iter)
{
	printf("Computing the optimal FSTSP-VDS solution...\n");
    // Open CPLEX model
	int error;
	CPXENVptr env = CPXopenCPLEX(&error);
	if (error)
		print_error("Error in CPXopenCPLEX() (FSTSP-VDS)\n");
	CPXLPptr lp;
	lp = CPXcreateprob(env, &error, "FSTSP-VDS");

	if (error)
		print_error("Error in CPXopenCPLEX() (FSTSP-VDS)\n");
    // get timestamp
	inst->param.ticks ? CPXgetdettime(env, &inst->timestamp_start) : getTimeStamp(&inst->timestamp_start);

    // path for LP and log files
	char lp_path[1000];
    //sprintf(lp_path, "../output/%s/seed_%d/run_%d/model_%d.lp", inst->instance_name, inst->param.seed, inst->param.run, iter);
	sprintf(lp_path, "LP_%d.lp", iter);
	char log_path[1000];
    // sprintf(log_path, "../output/%s/seed_%d/run_%d/log_%d.txt", inst->instance_name, inst->param.seed, inst->param.run, iter);
	sprintf(log_path, "log_%d.txt", iter);

    // build model
	if (inst_tsp == NULL)
		build_model(env, lp, inst, 0);
	else
		build_model(env, lp, inst, inst_tsp->z_best);

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
    	print_error("CPXPARAM_Parallel error");

    // if (CPXsetintparam(env, CPX_PARAM_THREADS, 1)) // Set one thread
    //     print_error("CPX_PARAM_THREADS error");

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
    	print_error("Error in CPXsetlogfilename() (FSTSP-VDS)\n");

    // warm start
    if (inst_tsp != NULL){
    	int nzcnt = (inst->dimension-1);
    	int beg = 0;
    	int varindices[nzcnt];
    	double values[nzcnt];
    	int effortlevel = CPX_MIPSTART_REPAIR;

    	for (int i = 0; i < inst->dimension-1; i++){
    		printf("tsp_succ[%d] = %d\n",i,inst_tsp->succ[i]);
    		int succ = inst_tsp->succ[i];
    		if (succ == 0)
    			succ = inst->dimension-1;
    		varindices[i] = xTruck_pos(i,succ, inst);
    		values[i] = 1.0;
    	}

    	int status = CPXaddmipstarts(env, lp, 1, nzcnt, &beg, varindices, values, &effortlevel, NULL);
    	if (status)
    		print_error("Error in CPXaddmipstarts\n");
    }	

    int flag_restart = 0; // restart if in the optimal solution there is an infeasible drone leg
    do
    {
    	if (CPXmipopt(env, lp))
    		print_error("CPXmipopt() error");
        if (CPXwriteprob(env, lp, lp_path, NULL)) // write LP model
        	print_error("Error in CPXwriteprob() (FSTSP-VDS)\n");

        // solution status of the problem
        int lpstat = CPXgetstat(env, lp);
        printf("CPLEX status: %d\n", lpstat);
        if (lpstat == 103)
        	print_error("Infeasible problem.");
        if (lpstat == 108)
        	print_error("Time limit exceeded; no integer solution found.");

        // Use the optimal solution found by CPLEX
        if (CPXgetx(env, lp, inst->best_sol, 0, inst->cols - 1))
        	print_error("CPXgetx() error");

        CPXgetobjval(env, lp, &inst->z_best);      // Best objective value
        CPXgetbestobjval(env, lp, &inst->best_lb); // Best lower bound

        //printf("\nSOLUTION -----------------------------------------------\n");
        //printf("\nRUNNING : %s\n", optimal_model_full_name[inst->model_type]);

        // directed graph
        gather_solution(inst, inst->best_sol);
        printf("*** inside optimal_solver() ***\n");

        printf("\nObjective value: %lf\n", inst->z_best);
        printf("Lower bound: %lf\n", inst->best_lb);

        // get timestamp
        inst->param.ticks ? CPXgetdettime(env, &inst->timestamp_finish) : getTimeStamp(&inst->timestamp_finish);

        // Plot optimal solution
        save_and_plot_solution(inst, 0);
        printf("DIMENSION: %d\n", inst->dimension);

        printf("\nTRUCK EDGES:\n");
        printf("  [i]    [j]   [departure time]     [arrival time]     [waiting time at node j]\n");
        for (int i = 0; i < inst->dimension - 1; i++)
        {
        	printf("  %2d  -> %2d | %16.6f   %16.6f        %16.6f \n", inst->truck_seq[i], inst->truck_seq[i + 1], inst->best_sol[a_pos(inst->truck_seq[i], inst)], inst->best_sol[a_pos(inst->truck_seq[i], inst)] + inst->truck_times[inst->truck_seq[i]][inst->truck_seq[i + 1]], fabs(inst->best_sol[a_pos(inst->truck_seq[i + 1], inst)] - inst->best_sol[a_pos(inst->truck_seq[i], inst)] - inst->truck_times[inst->truck_seq[i]][inst->truck_seq[i + 1]]));
        	if (inst->truck_seq[i + 1] == inst->dimension - 1)
        		break;
        }

        printf("\nDRONE EDGES:\n");
        printf("  [i]    [j]   [departure time]     [arrival time] \n");
        for (int i = 0; i < inst->dimension - 1; i++)
        {
        	printf("  %2d  -> %2d | %16.6f   %16.6f        %16.6f \n", inst->drone_seq[i], inst->drone_seq[i + 1], inst->best_sol[a_pos(inst->drone_seq[i], inst)], inst->best_sol[a_pos(inst->drone_seq[i], inst)] + inst->truck_times[inst->drone_seq[i]][inst->drone_seq[i + 1]], fabs(inst->best_sol[a_pos(inst->drone_seq[i + 1], inst)] - inst->best_sol[a_pos(inst->drone_seq[i], inst)] - inst->truck_times[inst->drone_seq[i]][inst->drone_seq[i + 1]]));
        	if (inst->drone_seq[i + 1] == inst->dimension - 1)
        		break;
        }


        {
        	printf("\nyT:");
        	for (int i = 0; i < inst->dimension; i++)
        	{
                if (inst->best_sol[yT_pos(i, inst)] > 0.5) // succ[i] != node ??
                {
                	printf(" %d", i);
                }
            }
        }

        {
        	printf("\nyD:");
        	for (int i = 0; i < inst->dimension; i++)
        	{
                if (inst->best_sol[yD_pos(i, inst)] > 0.5) // succ[i] != node ??
                {
                	printf(" %d", i);
                }
            }
        }

        {
        	printf("\n");
        	for (int i = 0; i < inst->dimension; i++)
        	{
                if (inst->best_sol[yC_pos(i, inst)] > 0.5) // succ[i] != node ??
                {
                	printf("yC: %d (alpha = %.1f, beta = %.1f)\n", i, inst->best_sol[alpha_pos(i, inst)], inst->best_sol[beta_pos(i, inst)]);
                }
            }
        }

        printf("\n\nDRONE LEGS:\n");
        printf("  [i]    [j]    [k]    [travel time]         [min]       [min feasible]        [max feasible]     [max feasible] \n");

        flag_restart = 0;

        for (int j = 1; j < inst->dimension - 1; j++)
        {
        	if (inst->best_sol[yD_pos(j, inst) < 0.5])
        		continue;
        	int max_i = -1;
        	int max_k = -1;
        	double max = 0.0;
        	for (int i = 0; i < inst->dimension; i++)
        	{
        		for (int k = 0; k < inst->dimension; k++)
        		{
        			if (inst->best_sol[z_pos(i, j, k, inst)] > 0.5)
        			{
        				max_i = i;
        				max_k = k;
        				max = inst->best_sol[z_pos(i, j, k, inst)];
        			}
        		}
        	}
        	if (max_i == -1)
        		continue;
        	printf("  %2d --> %2d --> %2d | %12.6f ", max_i, j, max_k, inst->best_sol[a_pos(max_k, inst)] - inst->best_sol[a_pos(max_i, inst)]);
        	printf("  %15.6f     %15.6f       %15.6f       %15.6f\n", inst->min_time_drone[max_i][j][max_k], inst->min_feas_time_drone[max_i][j][max_k], inst->max_feas_time_drone[max_i][j][max_k], inst->max_time_drone[max_i][j][max_k]);
        	double time_ijk = inst->best_sol[a_pos(max_k, inst)] - inst->best_sol[a_pos(max_i, inst)];
        	if (time_ijk < inst->min_feas_time_drone[max_i][j][max_k] - 1e-4 || time_ijk > inst->max_feas_time_drone[max_i][j][max_k] + 1e-4)
        	{
        		printf("*** drone leg infeasible *** \n");
        		flag_restart++;

                // remenaing battery capacity for cruising
        		double B = UAV_BATTERY_CAPACITY;
        		B -= vertical_energy(TAKEOFF_SPEED, CRUISE_ALTITUDE, inst->nodes[j].weight);
        		B -= vertical_energy(LANDING_SPEED, CRUISE_ALTITUDE, inst->nodes[j].weight);
        		B -= vertical_energy(TAKEOFF_SPEED, CRUISE_ALTITUDE, 0.0);
        		B -= vertical_energy(LANDING_SPEED, CRUISE_ALTITUDE, 0.0);
        		printf("\nB: %.5f\n", B);

                // check if time_ijk is feasible
        		if (compute_min_energy_time(inst->drone_dists[max_i][j], inst->drone_dists[j][max_k], inst->nodes[j].weight, time_ijk - extra_time, 
        			2 * inst->drone_dists[max_i][j] / (inst->param.uav_min_speed + inst->param.uav_max_speed), 
        			2 * inst->drone_dists[j][max_k] / (inst->param.uav_min_speed + inst->param.uav_max_speed), 
        			inst->param.uav_min_speed, inst->param.uav_max_speed, B) <= B )
        		{
                    // OK, JUST UPTADE THE MIN/MAX TIMES
        			if (time_ijk < inst->min_feas_time_drone[max_i][j][max_k])
        			{
        				inst->min_feas_time_drone[max_i][j][max_k] = time_ijk;
        				inst->min_time_drone[max_i][j][max_k] = time_ijk;
        			}
        			else
        			{
        				inst->max_feas_time_drone[max_i][j][max_k] = time_ijk;
        				inst->max_time_drone[max_i][j][max_k] = time_ijk;
        			}
        			continue;
        		}
                // time_ijk is not feasible

                // FIND THE MINIMUM/MAXIMUM FEASIBLE USING BISECTION

        		struct timespec timestamp;
        		if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
        			print_error("Error clock_gettime");
        		double timestamp_start = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);

                if (time_ijk < inst->min_feas_time_drone[max_i][j][max_k]) // fix minimum
                {
                	inst->min_time_drone[max_i][j][max_k] = time_ijk;
                	inst->min_feas_time_drone[max_i][j][max_k] = bisection(inst->drone_dists[max_i][j], inst->drone_dists[j][max_k], inst->nodes[j].weight, B, 
                		inst->min_time_drone[max_i][j][max_k] - extra_time, inst->min_feas_time_drone[max_i][j][max_k] - extra_time, 0,
                		inst->param.uav_min_speed, inst->param.uav_max_speed);
                	inst->min_feas_time_drone[max_i][j][max_k] += extra_time;
                	inst->min_time_drone[max_i][j][max_k] = inst->min_feas_time_drone[max_i][j][max_k];
                }
                else // fix maximum
                {
                	inst->max_time_drone[max_i][j][max_k] = time_ijk;
                	inst->max_feas_time_drone[max_i][j][max_k] = bisection(inst->drone_dists[max_i][j], inst->drone_dists[j][max_k], inst->nodes[j].weight, B, 
                		inst->max_time_drone[max_i][j][max_k] - extra_time, inst->max_feas_time_drone[max_i][j][max_k] - extra_time, 1,
                		inst->param.uav_min_speed, inst->param.uav_max_speed);
                	inst->max_feas_time_drone[max_i][j][max_k] += extra_time;
                	inst->max_time_drone[max_i][j][max_k] = inst->min_feas_time_drone[max_i][j][max_k];
                }

                if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
                	print_error("Error clock_gettime");
                double timestamp_finish = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);

                double time_elapsed = timestamp_finish - timestamp_start;
                printf("Elapsed time to refine the time of the drone leg [%d][%d][%d]: %f seconds\n", max_i, j, max_k, time_elapsed);

                // add a constraint
                char **rname = (char **)calloc(1, sizeof(char *)); // array of strings to store the row names
                rname[0] = (char *)calloc(100, sizeof(char));

                if (time_ijk < inst->min_feas_time_drone[max_i][j][max_k]) // fix minimum
                {
                    int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
                    double rhs = inst->min_feas_time_drone[max_i][j][max_k] - inst_tsp->z_best;
                    char sense = 'G';
                    sprintf(rname[0], "min_refined(%d,%d,%d)", max_i, j, max_k);

                    if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                    	print_error("wrong CPXnewrows [min_refined]");
                    if (CPXchgcoef(env, lp, row, a_pos(max_k, inst), 1.0))
                    	print_error("wrong CPXchgcoef [min_refined]");
                    if (CPXchgcoef(env, lp, row, a_pos(max_i, inst), -1.0))
                    	print_error("wrong CPXchgcoef [min_refined]");
                    if (CPXchgcoef(env, lp, row, z_pos(max_i, j, max_k, inst), -inst_tsp->z_best))
                    	print_error("wrong CPXchgcoef [min_refined]");
                }
                else // fix maximum
                {
                    int row = CPXgetnumrows(env, lp); // get the number of rows inside the model
                    double rhs = inst->max_feas_time_drone[max_i][j][max_k] + inst_tsp->z_best;
                    char sense = 'G'; // ">="
                    sprintf(rname[0], "max_refined(%d,%d,%d)", max_i, j, max_k);
                    if (CPXnewrows(env, lp, 1, &rhs, &sense, NULL, rname))
                    	print_error("wrong CPXnewrows [max_refined]");

                    if (CPXchgcoef(env, lp, row, a_pos(max_k, inst), 1.0))
                    	print_error("wrong CPXchgcoef [drone_max_time_constraint]");
                    if (CPXchgcoef(env, lp, row, a_pos(max_i, inst), -1.0))
                    	print_error("wrong CPXchgcoef [drone_max_time_constraint]");
                    if (CPXchgcoef(env, lp, row, z_pos(max_i, j, max_k, inst), inst_tsp->z_best))
                    	print_error("wrong CPXchgcoef [drone_max_time_constraint]");
                }
                free(rname[0]);
                free(rname);
            }
        }

        // printf("\nArrival Times:\n");
        // for (int i = 0; i < inst->dimension; i++)
        // {
        //     printf(" %d = %f \n", i, inst->best_sol[a_pos(i, inst)]);
        // }
    } while (flag_restart);

    // printf("\nu:\n");
    // for (int i = 0; i < inst->dimension; i++)
    // {
    //     printf(" %d = %f \n", i, inst->best_sol[u_pos(i, inst)]);
    // }

    // Free and close CPLEX model
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);
    return 0;
}
