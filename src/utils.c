#include "../include/utils.h"
#include "../include/min_max_opt.h"

#include <sys/stat.h>
#include <assert.h>



void parse_command_line(int argc, char **argv, instance *inst)
{

	if (argc < 2)
	{
		fprintf(stderr, "Usage: %s -h for help\n", argv[0]);
		exit(1);
	}
	else if (argc == 2 && strcmp(argv[1], "-h") == 0)
	{
		print_help();
	}
	else
	{

		for (int i = 1; i < argc; i++)
		{
			if (strcmp(argv[i], "-f") == 0)
            {   // Input file
            	strcpy(inst->instance_path, argv[++i]);
            	if (inst->instance_path[strlen(inst->instance_path) - 1] == '/')
            		inst->instance_path[strlen(inst->instance_path) - 1] = '\0';
            	char *last = strrchr(inst->instance_path, '/');
            	if (last == NULL)
            		strcpy(inst->instance_name, inst->instance_path);
            	else
            		strcpy(inst->instance_name, last + 1);
            	continue;
            }

            if (strcmp(argv[i], "-t") == 0)
            {   // Time limit (seconds)
            	inst->time_limit = strtof(argv[++i], NULL);
            	continue;
            }

            if (strcmp(argv[i], "-m") == 0)
            {   // Model type
            	inst->model_type = strtol(argv[++i], NULL, 10);
            	continue;
            }

            if (strcmp(argv[i], "-seed") == 0)
            {   // seed
            	inst->param.seed = atoi(argv[++i]);
            	continue;
            }

            if (strcmp(argv[i], "-output_name") == 0)
            {   // name file score
            	strcpy(inst->param.output_name, argv[++i]);
            	continue;
            }

            if (strcmp(argv[i], "-iterations") == 0)
            {   // name file score
            	inst->param.iterations = atoi(argv[++i]);
            	printf("*** iterations: %d\n\n", inst->param.iterations);
            	if (inst->param.iterations < 0)
            		print_error("the iterations parameter cannot be a negative number\n");
            	continue;
            }

            if (strcmp(argv[i], "-v") == 0)
            {   // Verbosity
            	i++;
            	int v = strtol(argv[i], NULL, 10);
            	if (v > DEBUG)
            		inst->param.verbose = DEBUG;
            	else if (v < QUIET)
            		inst->param.verbose = QUIET;
            	else
            	{
            		switch (v)
            		{
            			case QUIET:
            			inst->param.verbose = QUIET;
            			break;
            			case NORMAL:
            			inst->param.verbose = NORMAL;
            			break;
            			case VERBOSE:
            			inst->param.verbose = VERBOSE;
            			break;
            			case NERD:
            			inst->param.verbose = NERD;
            			break;
            			case DEBUG:
            			inst->param.verbose = DEBUG;
            			break;
            			default:
            			inst->param.verbose = NORMAL;
            			break;
            		}
            	}
            	continue;
            }

            if (strcmp(argv[i], "-alpha") == 0)
            {   // Time limit (seconds)
            	inst->param.alpha = atoi(argv[++i]);
            	if (inst->param.alpha < 1)
            		print_error("the alpha parameter must be at least 1\n");
            	continue;
            }

            if (strcmp(argv[i], "--ticks") == 0)
            {   // use ticks instead of seconds
            	inst->param.ticks = 1;
            	continue;
            }
            if (strcmp(argv[i], "--interactive") == 0)
            {   // show plots
            	inst->param.interactive = 1;
            	continue;
            }

            if (strcmp(argv[i], "--saveplots") == 0)
            {   // save all plots
            	inst->param.saveplots = 1;
            	continue;
            }
            if (strcmp(argv[i], "--savefinalplot") == 0)
            {   // save plots
            	inst->param.savefinalplot = 1;
            	continue;
            }
            if (strcmp(argv[i], "--heur") == 0)
            {   // use the heuristic method instead of the exact one
            	inst->param.heur = 1;
            	continue;
            }
            if (strcmp(argv[i], "--poikonen") == 0)
            {   // use the parser for the poikonen instances
            	inst->param.poikonen = 1;
            	continue;
            }
            if (strcmp(argv[i], "--MHD") == 0)
            {   // use the parser for the poikonen instances
            	inst->param.MHD = 1;
            	continue;
            }
            if (strcmp(argv[i], "--ha") == 0)
            {   // use the parser for the ha instances
            	inst->param.ha = 1;
            	continue;
            }
            if (strcmp(argv[i], "--select_max_range_speeds") == 0)
            {   // use the parser for the ha instances
            	inst->param.select_max_range_speed = 1;
            	continue;
            }

            if (strcmp(argv[i], "--select_max_speed") == 0)
            {   // use the parser for the ha instances
            	inst->param.select_max_speed = 1;
            	continue;
            }
            if (strcmp(argv[i], "-UAV_min_speed") == 0)
            {   // use the parser for the ha instances
            	inst->param.uav_min_speed = atoi(argv[++i]);
            	if (inst->param.uav_min_speed < 0)
            		print_error("The minimum drone speed (in m/s) must be a positive value\n");
            	continue;
            }
            if (strcmp(argv[i], "-UAV_max_speed") == 0)
            {   // use the parser for the ha instances
            	inst->param.uav_max_speed = atoi(argv[++i]);
            	if (inst->param.uav_max_speed < 0)
            		print_error("The maximum drone speed (in m/s) must be a positive value\n");
            	continue;
            }
            else {
            	printf("Parameter '%s' not available\n", argv[i]);
            	print_error("Parameter not found");
            }
        }
        // if (( inst->param.poikonen == 1 || inst->param.ha == 1) && inst->param.heur == 0)
        // if (( inst->param.ha == 1) && inst->param.heur == 0)
        //     print_error("Error: Ha instances can be used only with the heuristic method (using the flag --heur)\n");

        print_command_line(inst);
    }
}

void parse_instance(instance *inst)
{
	parse_locations(inst);
    // allocate memory for the the drone/truck structures
	allocate_mem_arrays(inst);
	parse_truck_travel_data(inst);
	compute_drone_distances(inst);
    //if (!inst->param.heur)
	compute_min_max_drone_legs_times(inst);
    //else
    //    compute_opt_speeds(inst);
    // retrieve the maximum drone travel time for each (i,j,-)
	retrieve_max_ij_drone_leg_times(inst);
	compute_neighborhoods(inst);
    // count_all_feasible_truck_legs(inst);
    // count_all_feasible_truck_legs_no_recursion(inst);
}

void parse_instance_poikonen(instance *inst)
{
	parse_locations_poikonen(inst);
    // allocate memory for the the drone/truck structures
	allocate_mem_arrays(inst);
	compute_drone_distances(inst);
	compute_truck_travel_times_poikonen(inst);
	compute_drone_travel_times_poikonen(inst);
	if (inst->param.heur == 1)
		compute_neighborhoods(inst);
    // compute_truck_legs2(inst);
    // count_all_feasible_truck_legs_no_recursion(inst);
    // exit(1);
}

void parse_instance_ha(instance *inst)
{
	printf("Parsing ha instance...\n");
	double drone_speed, truck_speed, max_endurance;
	parse_locations_ha(inst, &truck_speed, &drone_speed, &max_endurance);
    // allocate memory for the the drone/truck structures
	allocate_mem_arrays(inst);
	compute_drone_distances(inst);
	compute_truck_travel_times_ha(inst, truck_speed);
	compute_drone_travel_times_ha(inst, drone_speed, max_endurance);
	compute_neighborhoods(inst);
}

// for instances in https://github.com/optimatorlab/mFSTSP-VDS/tree/master/Problems
// nodeID, nodeType, latDeg, lonDeg, altMeters, parcelWtLbs
// nodeType 0 represents the depot, nodeType 1 represents a customer.
// The depot is always always assigned to nodeID 0.
// Each node has a corresponding latitude and longitude (specified in degrees).
// The altitude is always 0. Customer nodes have a corresponding non-zero parcel weight (in [pounds]).
// There is no parcel associated with the depot.
void parse_locations(instance *inst)
{
	printf("\n\nParsing the locations...\n");
	FILE *fp = NULL;
	char filename[1000];
	strcpy(filename, inst->instance_path);
	strcat(filename, "/tbl_locations.csv");

	fp = fopen(filename, "r");
	if (fp == NULL)
		print_error("Could not open the file ");

    char line[1000]; // tmp string where to store a line

    // Read each line from the input file:
    // split the line using strtok and consider the generated tokens

    // read (ignore) the first line
    if (fgets(line, sizeof(line), fp) == NULL)
    	print_error("Error reading the first file of the input data.");

    inst->dimension = 1000; // initial instance dimension
    // allocate a temporary memory that will be reallocated when the number of nodes will be known or when the memory is full
    inst->nodes = (node_struct *)calloc(inst->dimension, sizeof(node_struct));

    int i = 0; // #nodes counter
    while (fgets(line, sizeof(line), fp) != NULL)
    {
        line[strcspn(line, "\n")] = 0; // removing trailing \n

        // double the memory of the inst->nodes array
        if (i == inst->dimension)
        {
        	inst->dimension *= 2;
        	inst->nodes = (node_struct *)realloc(inst->nodes, inst->dimension * sizeof(node_struct));
        }
        char delimiters[] = ",";

        inst->nodes[i].id = strtol(strtok(line, delimiters), NULL, 10);
        int nodeType = strtol(strtok(NULL, delimiters), NULL, 10); // nodeType
        if (!i && nodeType)
        	print_error("Error: node 0 must be the depot (nodeType = 0).");

        inst->nodes[i].x = strtod(strtok(NULL, delimiters), NULL);                   // latitude
        inst->nodes[i].y = strtod(strtok(NULL, delimiters), NULL);                   // longitude
        strtok(NULL, delimiters);                                                    // ignore altitude
        inst->nodes[i].weight = strtof(strtok(NULL, delimiters), NULL) * 0.45359237; // parcel weight (lbs -> kg)

        if (inst->param.verbose >= DEBUG)
        	printf("\tNODE %2d at coordinates (%11.6lf , %11.6lf) \t parcel weight = %7.2lf kg \n", inst->nodes[i].id, inst->nodes[i].x, inst->nodes[i].y, inst->nodes[i].weight);
        i++;
    }

    // adding an extra node that represent the returning depot
    inst->nodes[i].id = i;
    inst->nodes[i].x = inst->nodes[0].x;
    inst->nodes[i].y = inst->nodes[0].y;
    inst->nodes[i].weight = inst->nodes[0].weight;

    inst->dimension = ++i;                                                      // correct number of nodes in the instance
    inst->nodes = (node_struct *)realloc(inst->nodes, inst->dimension * sizeof(node_struct)); // realloc

    fclose(fp);

    if (inst->dimension < 3)
    	print_error("EXIT: the instance does not have any customer.");
}

// tbl_truck_travel_data_PG.csv contains the directed truck travel time and distance information from one node to another.
// All time and distance values were obtained by pgRouting, using OpenStreetMaps data.
// from location i, to location j, time [sec], distance [meters]
void parse_truck_travel_data(instance *inst)
{
	printf("\n\nParsing truck travel times and distances...\n");

	FILE *fp = NULL;
	char filename[1000];
	strcpy(filename, inst->instance_path);
	strcat(filename, "/tbl_truck_travel_data_PG.csv");

	fp = fopen(filename, "r");
	if (fp == NULL)
		print_error("Could not open the file");

    char line[1000]; // tmp string where to store a line
    // Read each line from the input file:
    // split the line using strtok and consider the generated tokens

    // read (ignore) the first line (header)
    if (fgets(line, sizeof(line), fp) == NULL)
    	print_error("Error reading the first file of the input data.");

    int nlines = 0; // #lines counter
    while (fgets(line, sizeof(line), fp) != NULL)
    {
        line[strcspn(line, "\n")] = 0; // removing trailing \n
        char delimiters[] = ",";

        int i = strtol(strtok(line, delimiters), NULL, 10);                                     // node i
        int j = strtol(strtok(NULL, delimiters), NULL, 10);                                     // node j
        inst->truck_times[i][j] = strtod(strtok(NULL, delimiters), NULL); 						// travel time
        if (j != 0)
        	inst->truck_times[i][j] += TRUCK_DELIVERY_TIME;
        // inst->truck_times[i][j] = strtod(strtok(NULL, delimiters), NULL);                    // travel time
        inst->truck_dists[i][j] = strtod(strtok(NULL, delimiters), NULL);                       // travel distance

        nlines++;
    }
    if (nlines != (inst->dimension - 1) * (inst->dimension - 1))
    	printf("WARNING: for some (i,j) truck travel times/distances are missing, by default these values are set to 0.\n");

    fclose(fp);

    // define the time/distance [customer -> returning depot] equal to time/distance [customer -> starting depot]
    // minus the truck delivery time (since the last node is not a customer but a depot)
    for (int i = 0; i < inst->dimension - 1; i++)
    {
    	inst->truck_times[i][inst->dimension - 1] = inst->truck_times[i][0];
    	inst->truck_dists[i][inst->dimension - 1] = inst->truck_dists[i][0];

    }

    if (inst->param.verbose >= DEBUG)
    {
    	for (int i = 0; i < inst->dimension; i++)
    	{
    		for (int j = 0; j < inst->dimension; j++)
    		{
    			printf("\ttruck_time(%3d,%3d) = %15.6lf \t\t dist(%3d,%3d) = %15.6lf \n", i, j, inst->truck_times[i][j], i, j, inst->truck_dists[i][j]);
    		}
    	}
    }
}

// allocate memory for the the drone/truck structures
void allocate_mem_arrays(instance *inst)
{
    // allocate the memory for the truck travel times/dists matrix
	allocate_mem_truck_times_and_dists(inst);
    // allocate the memory for the drone travel times matrix
	allocate_mem_drone_min_max_times(inst);
    // allocate the memory for the drone and truck sequences
	inst->truck_seq = (int *)calloc(inst->dimension, sizeof(int));
	inst->drone_seq = (int *)calloc(inst->dimension, sizeof(int));
    // allocate the memory for the neighbors arrays
	for (int i = 0; i < inst->dimension; i++) {
		inst->nodes[i].neighbors = (int *)calloc(NEIGHBORHOOD_SIZE, sizeof(int));
	}
}

void allocate_mem_truck_times_and_dists(instance *inst)
{
    // initialize the matrix of the truck travel times
	inst->truck_dists = (double **)calloc(inst->dimension, sizeof(double *));
	inst->truck_times = (double **)calloc(inst->dimension, sizeof(double *));
	for (int i = 0; i < inst->dimension; i++)
	{
		inst->truck_dists[i] = (double *)calloc(inst->dimension, sizeof(double));
		inst->truck_times[i] = (double *)calloc(inst->dimension, sizeof(double));
	}
    // truck times/dists can be accessed as follows:
    // truck_dists[i][j]
    // truck_times[i][j]
}

void allocate_mem_drone_min_max_times(instance *inst)
{
    // initialize the 3D matrices of the min/max drone's travel times
	inst->min_time_drone = (double ***)calloc(inst->dimension, sizeof(double *));
	inst->min_feas_time_drone = (double ***)calloc(inst->dimension, sizeof(double *));

	inst->max_time_drone = (double ***)calloc(inst->dimension, sizeof(double *));
	inst->max_feas_time_drone = (double ***)calloc(inst->dimension, sizeof(double *));

	for (int i = 0; i < inst->dimension; i++)
	{
		inst->min_time_drone[i] = (double **)calloc(inst->dimension, sizeof(double *));
		inst->min_feas_time_drone[i] = (double **)calloc(inst->dimension, sizeof(double *));

		inst->max_feas_time_drone[i] = (double **)calloc(inst->dimension, sizeof(double *));
		inst->max_time_drone[i] = (double **)calloc(inst->dimension, sizeof(double *));
	}

	for (int i = 0; i < inst->dimension; i++)
	{
		for (int j = 0; j < inst->dimension; j++)
		{
			inst->min_time_drone[i][j] = (double *)calloc(inst->dimension, sizeof(double));
			inst->min_feas_time_drone[i][j] = (double *)calloc(inst->dimension, sizeof(double));

			inst->max_feas_time_drone[i][j] = (double *)calloc(inst->dimension, sizeof(double));
			inst->max_time_drone[i][j] = (double *)calloc(inst->dimension, sizeof(double));
		}
	}
    // drone min/max times can be accessed as follows:
    // min_time_drone[i][j][k]
    // max_time_drone[i][j][k]

	inst->drone_dists = (double **)calloc(inst->dimension, sizeof(double *));
	for (int i = 0; i < inst->dimension; i++)
	{
		inst->drone_dists[i] = (double *)calloc(inst->dimension, sizeof(double));
	}
}

void compute_drone_distances(instance *inst)
{
	printf("Computing drone distances...\n");
	for (int i = 0; i < inst->dimension; i++)
	{
		for (int j = 0; j < inst->dimension; j++)
		{
            // if (i == j) {
            //     inst->drone_dists[i][j] = DBL_MAX;
            //     continue;
            // }
			if (inst->param.ha == 1)
				inst->drone_dists[i][j] = dist_EUC_2D(i, j, inst);
			else if (inst->param.poikonen == 1)
				inst->drone_dists[i][j] = dist_EUC_2D(i, j, inst);
			else
				inst->drone_dists[i][j] = dist(i, j, inst);
		}
	}
	printf("Drone distances computed.\n");
}

// tbl_truck_travel_data_PG.csv contains the directed truck travel time and distance information from one node to another.
// All time and distance values were obtained by pgRouting, using OpenStreetMaps data.
// from location i, to location j, time [sec], distance [meters]
void parse_min_max_drone_legs_times(instance *inst)
{
	printf("\n\nParsing the min/max drone legs travel times...\n");

	FILE *fp = NULL;
	char filename[1000];
	strcpy(filename, inst->instance_path);
	strcat(filename, "/min_max.csv");

	fp = fopen(filename, "r");
	if (fp == NULL)
		print_error("Could not open the file");

    char line[1000]; // tmp string where to store a line
    // Read each line from the input file:
    // split the line using strtok and consider the generated tokens

    // read (ignore) the first line (header)
    if (fgets(line, sizeof(line), fp) == NULL)
    	print_error("Error reading the first file of the input data.");

    int nlines = 0;     // #lines counter
    int flag_index = 0; // flag set to 1 if indices not valid are found
    while (fgets(line, sizeof(line), fp) != NULL)
    {
        line[strcspn(line, "\n")] = 0; // removing trailing \n
        char delimiters[] = ",";

        int i = strtol(strtok(line, delimiters), NULL, 10); // node i
        int j = strtol(strtok(NULL, delimiters), NULL, 10); // node j
        int k = strtol(strtok(NULL, delimiters), NULL, 10); // node k

        if (i < 0 || j < 0 || k < 0 || i >= inst->dimension || j >= inst->dimension || k >= inst->dimension)
        {
        	flag_index = 1;
        	strtok(NULL, delimiters);
        	strtok(NULL, delimiters);
        	strtok(NULL, delimiters);
        	continue;
        }

        inst->min_time_drone[i][j][k] = strtod(strtok(NULL, delimiters), NULL) + extra_time;      // min
        inst->min_feas_time_drone[i][j][k] = strtod(strtok(NULL, delimiters), NULL) + extra_time; //  min feasible

        inst->max_feas_time_drone[i][j][k] = strtod(strtok(NULL, delimiters), NULL) + extra_time; // max feasible
        inst->max_time_drone[i][j][k] = strtod(strtok(NULL, delimiters), NULL) + extra_time;      //  max

        if (inst->param.verbose >= DEBUG)
        	printf("\t[%3d,%3d,%3d] = \t %12.6lf \t %12.6lf \t %12.6lf \t %12.6lf \n", i, j, k, inst->min_time_drone[i][j][k], inst->min_feas_time_drone[i][j][k], inst->max_feas_time_drone[i][j][k], inst->max_time_drone[i][j][k]);
        nlines++;
    }

    if (flag_index)
    	printf("WARNING [parse_min_max_drone_legs_times]: some index is not valid.");

    fclose(fp);
}

void initialize_instance(instance *inst)
{
	struct timespec timestamp;
	if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
		print_error("Error clock_gettime");
    inst->model_type = 0;
    inst->time_limit = CPX_INFBOUND;
    inst->param.seed = 0;
    inst->param.run = 1;
    inst->param.verbose = NORMAL;
    inst->param.ticks = 0;
    inst->param.interactive = 0;
    inst->param.saveplots = 0;
    inst->param.heur = 0;
    inst->param.poikonen = 0;
    inst->param.ha = 0;
    inst->param.alpha = 1;
    inst->param.iterations = 2000;
    inst->param.neighborhood_size = NEIGHBORHOOD_SIZE;

    inst->param.uav_min_speed = UAV_MIN_SPEED;
    inst->param.uav_max_speed = UAV_MAX_SPEED;
    inst->param.select_max_range_speed = 0;
    inst->param.select_max_speed = 0;

    inst->dimension = -1;
    inst->nodes = NULL;
    inst->truck_seq = NULL;
    inst->drone_seq = NULL;
    inst->truck_dists = NULL;
    inst->truck_times = NULL;

    inst->z_best = DBL_MAX;

    inst->timestamp_start = 0.0;
    inst->timestamp_finish = 0.0;

    inst->timestamp_last_plot = 0.0;
    inst->plot_counter = 0;

    inst->number_feasible_truck_legs = 0;
    // inst->feasible_truck_legs = NULL;
    // inst->feasible_truck_legs_times = NULL;
    // inst->feasible_truck_legs_ik_begin = NULL;
    // inst->feasible_truck_legs_ik_number = NULL;
    // inst->reduced_truck_legs = NULL;

    // inst->reduced_feasible_truck_legs = NULL;
    // inst->reduced_number_feasible_truck_legs = 0;

    // new //
    // inst->feasible_truck_legs_ik = NULL;
    // inst->number_feasible_truck_legs_ik = NULL;

    inst->feasible_truck_legs2 = NULL;
    ////////

    strcpy(inst->instance_path, "NULL");
    strcpy(inst->instance_name, "NULL");
    strcpy(inst->param.output_name, "output");
    inst->gnuplotPipe = popen("gnuplot -persistent", "w");
}

void free_instance(instance *inst)
{
    // close gnuplot pipe
	if (pclose(inst->gnuplotPipe) == -1)
		print_error("pclose error");

	free(inst->nodes);
	free(inst->drone_seq);
	free(inst->truck_seq);
}

void print_command_line(instance *inst)
{
	printf("\nPARAMETERS ---------------------------------------------\n");
	printf("-f (file path) %s\n", inst->instance_path);
	if (inst->param.ticks == 1)
	{
		printf("-t (ticks limit) %.0f ticks\n", inst->time_limit);
		printf("--ticks (ACTIVE -> %d)\n", inst->param.ticks);
	}
	else
	{
		printf("-t (time limit) %.0f seconds\n", inst->time_limit);
	}

	if (inst->param.interactive == 1)
	{
		printf("--interactive (ACTIVE -> %d)\n", inst->param.interactive);
	}

	if (inst->param.saveplots == 1)
	{
		printf("--saveplots (ACTIVE -> %d)\n", inst->param.saveplots);
	}

	printf("-seed (seed) %d\n", inst->param.seed);
	printf("-v (verbosity) %d (%s)\n", inst->param.verbose, verbose_name[inst->param.verbose]);
	printf("--------------------------------------------------------\n\n");
}

void print_instance(instance *inst)
{
    // last node is the "fake" returning depot
	printf("\n\nINSTANCE -----------------------------------------------\n");
	printf("Name: %s\n", inst->instance_name);
	printf("Path: %s\n", inst->instance_path);
	printf("Dimension: %d (+ a returning depot)\n", inst->dimension - 1);

	if (inst->nodes != NULL)
	{
		printf("\nNode_id %*s Latitude %*s Longitude   Parcel_weight \n", 7, "", 5, "");
		for (int i = 0; i < inst->dimension - 1; i++)
			printf("%7d\t %15.6lf %15.6lf %12.2lf kg\n", inst->nodes[i].id, inst->nodes[i].x, inst->nodes[i].y, inst->nodes[i].weight);
	}
	printf("*%6d\t %15.6lf %15.6lf %12.2lf kg\n", inst->nodes[inst->dimension - 1].id, inst->nodes[inst->dimension - 1].x, inst->nodes[inst->dimension - 1].y, inst->nodes[inst->dimension - 1].weight);

	printf("--------------------------------------------------------\n\n");
}

void print_help()
{
	printf("\nHELP ---------------------------------------------------\n");
	printf("-f <path>       : used to pass the relative instance path \n");
	printf("-t <time>       : used to pass the total running time allowed in seconds\n");
	printf("--ticks         : used to set the way time is interpreted inside the solver (optional)\n");
	printf("--interactive   : used to plot all the solutions (by default none plot is displayed)\n");
	printf("--saveplots     : used to save all the solutions' plots (by default only the final solution's plot is saved)\n");
	printf("--math          : used to set the math-heuristic solver (optional)\n");
	printf("--heur          : used to set the heuristic solver (optional)\n");
	printf("--meta          : used to set the meta-heuristic solver (optional)\n");
	printf("--grasp <value> : used to set GRASP approach and possible choices (optional)\n");
	printf("-m <model>      : used to set the model type (based on the solver)\n");
	printf("-seed <seed>       : used to set the seed\n");
	printf("-v <value>      : used to set the verbosity, from QUIET (0) up to DEBUG (4)\n");
	printf("--------------------------------------------------------\n\n");
	exit(1);
}

void print_error(const char *err)
{
	fprintf(stderr, "\nERROR: %s \n\n", err);
	fflush(NULL);
	exit(1);
}

void print_error_status(const char *err, int e)
{
	printf("\n\n ERROR: exit with status %d. %s. \n\n", e, err);
	fflush(NULL);
	exit(1);
}

void print_message(const char *msg)
{
	fprintf(stdout, "\nMESSAGE: %s \n\n", msg);
	fflush(NULL);
}

void save_and_plot_solution(instance *inst, int iter)
{
	save_and_plot_solution_general(inst, inst->truck_seq, inst->drone_seq, iter);
}

void save_and_plot_solution_general(instance *inst, int *truck_seq, int *drone_seq, int iter)
{
	if (inst->param.saveplots || inst->param.interactive || iter == -1)
	{
		char title_gnuplot_command[200];
		if (iter >= 0)
			sprintf(title_gnuplot_command, "set title 'Best solution (iteration %d): %.2f'", iter, inst->z_best);
		else
			sprintf(title_gnuplot_command, "set title 'Final solution'");

        // write solution to file

		char data_points_filename[200];
		char truck_edges_filename[200];
		char drone_edges_filename[200];

		sprintf(data_points_filename, "data_temp/data_points_%d", inst->plot_counter);
		sprintf(truck_edges_filename, "data_temp/truck_edges_%d", inst->plot_counter);
		sprintf(drone_edges_filename, "data_temp/drone_edges_%d", inst->plot_counter);
		FILE *data_points = fopen(data_points_filename, "w");
		FILE *truck_edges = fopen(truck_edges_filename, "w");
		FILE *drone_edges = fopen(drone_edges_filename, "w");

        // write nodes position
		for (int i = 0; i < inst->dimension - 1; i++)
			fprintf(data_points, "%lf %lf %d \n", inst->nodes[i].x, inst->nodes[i].y, inst->nodes[i].id);

        // write truck edges
		for (int i = 0; i < inst->dimension - 1; i++)
		{
			if (truck_seq[i] == inst->dimension - 1)
				break;

			double delta_x = inst->nodes[truck_seq[i + 1]].x - inst->nodes[truck_seq[i]].x;
			double delta_y = inst->nodes[truck_seq[i + 1]].y - inst->nodes[truck_seq[i]].y;
			fprintf(truck_edges, "%lf %lf %lf %lf \n", inst->nodes[truck_seq[i]].x, inst->nodes[truck_seq[i]].y, delta_x, delta_y);
		}

        // write drone edges
		for (int i = 0; i < inst->dimension - 1; i++)
		{
			if (drone_seq[i] == inst->dimension - 1)
				break;

			double delta_x = inst->nodes[drone_seq[i + 1]].x - inst->nodes[drone_seq[i]].x;
			double delta_y = inst->nodes[drone_seq[i + 1]].y - inst->nodes[drone_seq[i]].y;
			fprintf(drone_edges, "%lf %lf %lf %lf \n", inst->nodes[drone_seq[i]].x, inst->nodes[drone_seq[i]].y, delta_x, delta_y);
		}

		fclose(data_points);
		fclose(truck_edges);
		fclose(drone_edges);

        if (inst->param.saveplots || inst->param.savefinalplot) // save plot
        {
            FILE *gnuplotPipe = popen("gnuplot", "w"); // local gnuplotPipe to avoid conflicts with the global gnuplotPipe
            char *out = (char *)calloc(1000, sizeof(char));
            char *plot_str = (char *)calloc(1000, sizeof(char));

            sprintf(out, "set output '../output/%s/seed_%d/run_%d/plot/sol_%d.jpg'", inst->instance_name, inst->param.seed, inst->param.run, iter);
            sprintf(plot_str, "plot '%s' with vectors head filled lc rgb 'blue' lw 1,\
            	'%s' with vectors head filled lc rgb 'red' dashtype '-_' lw 1,\
            	'%s' using 1:2:3 with labels offset (0,0) font 'Arial'",
            	truck_edges_filename, drone_edges_filename, data_points_filename, data_points_filename);


            char *commandsForGnuplot_drone[] = {title_gnuplot_command,
            	"set terminal jpeg size 1920,1080",
            	out,
            	"unset key",
                                                //"set autoscale",
            	"set ylabel 'Y'",
            	"set xlabel 'X'",
            	plot_str
            };

            //Send commands to gnuplot one by one.
            int commands = sizeof(commandsForGnuplot_drone) / sizeof(commandsForGnuplot_drone[0]);
            for (int i = 0; i < commands; i++)
            	fprintf(gnuplotPipe, "%s \n", commandsForGnuplot_drone[i]);

            pclose(gnuplotPipe); // execute the commands
            free(plot_str);
            free(out);
        }



        if (inst->param.interactive) // plot solution
        {
        	char plot_str[5000];

        	sprintf(plot_str, "plot '%s' with vectors head filled lc rgb 'blue' lw 1,\
        		'%s' with vectors head filled lc rgb 'red' dashtype '-_' lw 1,\
        		'%s' using 1:2:3 with labels offset (0,0) font 'Arial'",
        		truck_edges_filename, drone_edges_filename, data_points_filename, data_points_filename);

        	char *commandsForGnuplot_drone[] = {title_gnuplot_command,
                                                //"set term wxt noraise",
        		"set terminal qt noraise",
        		"unset key",
                                                //"set autoscale",
        		"set ylabel 'Y'",
        		"set xlabel 'X'",
        		plot_str
        	};

        	int commands = sizeof(commandsForGnuplot_drone) / sizeof(commandsForGnuplot_drone[0]);
        	for (int i = 0; i < commands; i++)
        		fprintf(inst->gnuplotPipe, "%s \n", commandsForGnuplot_drone[i]);
            fflush(inst->gnuplotPipe); // execute the commands
        }
    }
}

void save_and_plot_solution_succ(instance *inst, int *truck_succ, int *drone_succ, double objval)
{
	if (inst->param.saveplots || inst->param.interactive)
	{
		char title_gnuplot_command[200];
		sprintf(title_gnuplot_command, "set title 'Solution of cost: %.2f'", objval);

        // write solution to file

		char data_points_filename[200];
		char truck_edges_filename[200];
		char drone_edges_filename[200];

		sprintf(data_points_filename, "data_temp/data_points_%d", inst->plot_counter);
		sprintf(truck_edges_filename, "data_temp/truck_edges_%d", inst->plot_counter);
		sprintf(drone_edges_filename, "data_temp/drone_edges_%d", inst->plot_counter);
		FILE *data_points = fopen(data_points_filename, "w");
		FILE *truck_edges = fopen(truck_edges_filename, "w");
		FILE *drone_edges = fopen(drone_edges_filename, "w");

        // write nodes position
		for (int i = 0; i < inst->dimension - 1; i++)
			fprintf(data_points, "%lf %lf %d \n", inst->nodes[i].x, inst->nodes[i].y, inst->nodes[i].id);

        // write truck edges
		for (int i = 0; i < inst->dimension - 1; i++)
		{
			if (truck_succ[i] == -1) continue;
			double delta_x = inst->nodes[truck_succ[i]].x - inst->nodes[i].x;
			double delta_y = inst->nodes[truck_succ[i]].y - inst->nodes[i].y;
			fprintf(truck_edges, "%lf %lf %lf %lf \n", inst->nodes[i].x, inst->nodes[i].y, delta_x, delta_y);
		}

        // write drone edges
		for (int i = 0; i < inst->dimension - 1; i++)
		{
			if (drone_succ[i] == -1) continue;
			double delta_x = inst->nodes[drone_succ[i]].x - inst->nodes[i].x;
			double delta_y = inst->nodes[drone_succ[i]].y - inst->nodes[i].y;
			fprintf(drone_edges, "%lf %lf %lf %lf \n", inst->nodes[i].x, inst->nodes[i].y, delta_x, delta_y);
		}

		fclose(data_points);
		fclose(truck_edges);
		fclose(drone_edges);

        if (inst->param.saveplots || inst->param.savefinalplot) // save plot
        {
            FILE *gnuplotPipe = popen("gnuplot", "w"); // local gnuplotPipe to avoid conflicts with the global gnuplotPipe
            char *out = (char *)calloc(1000, sizeof(char));
            char *plot_str = (char *)calloc(1000, sizeof(char));

            sprintf(out, "set output '../output/%s/seed_%d/run_%d/plot/sol_tmp.jpg'", inst->instance_name, inst->param.seed, inst->param.run);
            sprintf(plot_str, "plot '%s' with vectors head filled lc rgb 'blue' lw 1,\
            	'%s' with vectors head filled lc rgb 'red' dashtype '-_' lw 1,\
            	'%s' using 1:2:3 with labels offset (0,0) font 'Arial'",
            	truck_edges_filename, drone_edges_filename, data_points_filename, data_points_filename);


            char *commandsForGnuplot_drone[] = {title_gnuplot_command,
            	"set terminal jpeg size 1920,1080",
            	out,
            	"unset key",
                                                //"set autoscale",
            	"set ylabel 'Y'",
            	"set xlabel 'X'",
            	plot_str
            };

            //Send commands to gnuplot one by one.
            int commands = sizeof(commandsForGnuplot_drone) / sizeof(commandsForGnuplot_drone[0]);
            for (int i = 0; i < commands; i++)
            	fprintf(gnuplotPipe, "%s \n", commandsForGnuplot_drone[i]);

            pclose(gnuplotPipe); // execute the commands
            free(plot_str);
            free(out);
        }



        if (inst->param.interactive) // plot solution
        {
        	char plot_str[5000];

        	sprintf(plot_str, "plot '%s' with vectors head filled lc rgb 'blue' lw 1,\
        		'%s' with vectors head filled lc rgb 'red' dashtype '-_' lw 1,\
        		'%s' using 1:2:3 with labels offset (0,0) font 'Arial'",
        		truck_edges_filename, drone_edges_filename, data_points_filename, data_points_filename);

        	char *commandsForGnuplot_drone[] = {title_gnuplot_command,
                                                //"set term wxt noraise",
        		"set terminal qt noraise",
        		"unset key",
                                                //"set autoscale",
        		"set ylabel 'Y'",
        		"set xlabel 'X'",
        		plot_str
        	};

        	int commands = sizeof(commandsForGnuplot_drone) / sizeof(commandsForGnuplot_drone[0]);
        	for (int i = 0; i < commands; i++)
        		fprintf(inst->gnuplotPipe, "%s \n", commandsForGnuplot_drone[i]);
            fflush(inst->gnuplotPipe); // execute the commands
        }
    }
}

/*void compute_min_max_drone_legs_times(instance *inst)
{
    FILE *fd = fopen("minmax_nlopt.csv", "w+");
    // retrieve all the weights
    double *weights = (double *)malloc(inst->dimension * sizeof(double));
    double *best_speed_weights = (double *)malloc(inst->dimension * sizeof(double));
    weights[0] = 0.0;
    best_speed_weights[0] = compute_max_range_speed(weights[0]);
    int n_weights = 1;

    for (int i = 0; i < inst->dimension; i++)
    {
        if (inst->nodes[i].weight < 0 || inst->nodes[i].weight > UAV_WEIGHT_LIMIT)
            continue;
        int flag = 0;
        for (int j = 0; j < n_weights; j++)
        {
            if (fabs(weights[j] - inst->nodes[i].weight) < 1e-6)
            {
                flag = 1;
                break;
            }
        }
        if (!flag)
            weights[n_weights++] = inst->nodes[i].weight;
    }
    weights = (double *)realloc(weights, n_weights * sizeof(double));
    best_speed_weights = (double *)realloc(best_speed_weights, n_weights * sizeof(double));

    // compute the max range speed for each weight (the speed with the smaller energy-per-meter consumption)

    for (int i = 0; i < n_weights; i++)
    {
        best_speed_weights[i] = compute_max_range_speed(weights[i]);
        printf("Max range speed for weight %.2f lb : %f\n", weights[i], best_speed_weights[i]);
    }

    for (int i = 0; i < inst->dimension; i++)
    {
        for (int j = 0; j < inst->dimension; j++)
        {
            for (int k = 0; k < inst->dimension; k++)
            {
                if (inst->nodes[j].weight > UAV_WEIGHT_LIMIT || inst->nodes[j].weight < 0)
                {
                    //fprintf(fd, "%d, %d, %d, 0, 0, 0, 0\n", i, j, k);
                    continue;
                }
                if (i == inst->dimension - 1 || j == 0 || j == inst->dimension - 1 || k == 0 || j == i || k == j || k == i)
                {
                    //fprintf(fd, "%d, %d, %d, 0, 0, 0, 0\n", i, j, k);
                    continue;
                }
                printf("i = %d, j = %d, k = %d \n", i, j, k);

                double B = UAV_BATTERY_CAPACITY;
                B -= vertical_energy(TAKEOFF_SPEED, CRUISE_ALTITUDE, inst->nodes[j].weight);
                B -= vertical_energy(LANDING_SPEED, CRUISE_ALTITUDE, inst->nodes[j].weight);
                B -= vertical_energy(TAKEOFF_SPEED, CRUISE_ALTITUDE, 0.0);
                B -= vertical_energy(LANDING_SPEED, CRUISE_ALTITUDE, 0.0);

                double d1, d2;
                d1 = inst->drone_dists[i][j]; // distance i -> j
                d2 = inst->drone_dists[j][k]; // distance j -> k

                // check if using the max range speeds the energy constraint is satisied
                double max_r_s1 = -1;
                for (int r = 0; r < n_weights; r++)
                {
                    if (fabs(weights[r] - inst->nodes[j].weight) < 1e-6)
                    {
                        max_r_s1 = best_speed_weights[r];
                        break;
                    }
                }
                if (max_r_s1 < 0)
                    print_error("Error in compute_min_max_drone_legs_times(), a weight has not been found in the array weights\n");
                if (cruise_energy(max_r_s1, inst->nodes[j].weight, d1) + cruise_energy(best_speed_weights[0], 0.0, d2) > B)
                    continue;
                // the drone leg i-->j-->k is feasible
                inst->min_feas_time_drone[i][j][k] = d1 / max_r_s1 + d2 / best_speed_weights[0];
                inst->max_feas_time_drone[i][j][k] = inst->min_feas_time_drone[i][j][k];

                // COMPUTE THE MINIMUM DRONE TRAVEL TIME for the path i-->j-->k
                double s1 = -1, s2 = -1;
                double s1_feas = -1, s2_feas = -1;
                //double min_feas_time = -1, min_time -1;
                //printf("\n[%d][%d][%d] B: %.12f\n", i, j, k, B);
                int code_min = compute_min_time(d1, d2, inst->nodes[j].weight, B, max_r_s1, best_speed_weights[0], &inst->min_time_drone[i][j][k], &inst->min_feas_time_drone[i][j][k], &s1, &s2, &s1_feas, &s2_feas);
                //printf("\t s1: %.2f s2: %.2f s1_feas: %.2f s2_feas: %.2f \n", s1, s2, s1_feas, s2_feas);
                // check
                double e_min;

                if (code_min < 0)
                {
                    // should not occur ...
                    // repair using the initial feasible solution
                    inst->min_feas_time_drone[i][j][k] = d1 / max_r_s1 + d2 / best_speed_weights[0];
                    // and maximum allowable time as infeasible solution
                    inst->min_time_drone[i][j][k] = d1 / UAV_MAX_SPEED + d2 / UAV_MAX_SPEED;
                    code_min = 5;
                }

                if (code_min == 5)
                {
                    // find an infeasible min time
                    // initialize infeasible min time
                    inst->min_time_drone[i][j][k] = d1 / UAV_MAX_SPEED + d2 / UAV_MAX_SPEED;
                    int it = 0; // while counter
                    double t = inst->min_feas_time_drone[i][j][k] - pow(2, it);
                    while (t > inst->min_time_drone[i][j][k])
                    {
                        double e = compute_min_energy_time(d1, d2, inst->nodes[j].weight, t, d1 / s1_feas, d2 / s2_feas);
                        //printf("*** [MIN] compute energy of time %f: %f ***\n", t, e);
                        e_min = e;
                        if (e > B)
                            break;
                        t = inst->min_feas_time_drone[i][j][k] - pow(2, it);
                        it++;
                    }
                    if (t > inst->min_time_drone[i][j][k])
                        inst->min_time_drone[i][j][k] = t;
                }
                else
                {
                    // use the returned solution for both the min and the min feasible times
                    inst->min_time_drone[i][j][k] = inst->min_feas_time_drone[i][j][k];
                    e_min = cruise_energy(s1, inst->nodes[j].weight, d1) + cruise_energy(s2, 0.0, d2);
                }

                //printf("min          : %.12f | %.12f \n", inst->min_time_drone[i][j][k], e_min);
                //printf("min feasibile: %.12f | %.12f \n", inst->min_feas_time_drone[i][j][k], cruise_energy(s1_feas, inst->nodes[j].weight, d1) + cruise_energy(s2_feas, 0.0, d2));

                // COMPUTE THE MAXIMUM DRONE TRAVEL TIME for the path i-->j-->k
                s1 = -1, s2 = -1;
                s1_feas = -1, s2_feas = -1;

                int code_max = compute_max_time(d1, d2, inst->nodes[j].weight, B, max_r_s1, best_speed_weights[0], &inst->max_time_drone[i][j][k], &inst->max_feas_time_drone[i][j][k], &s1, &s2, &s1_feas, &s2_feas);
                //printf("\t s1: %.2f s2: %.2f s1_feas: %.2f s2_feas: %.2f \n", s1, s2, s1_feas, s2_feas);

                if (code_max < 0)
                {
                    // it should not occur ...
                    // repair using the initial feasible solution
                    inst->max_feas_time_drone[i][j][k] = d1 / max_r_s1 + d2 / best_speed_weights[0];
                    // and maximum allowable time as infeasible solution
                    inst->max_time_drone[i][j][k] = d1 / UAV_MIN_SPEED + d2 / UAV_MIN_SPEED;
                    code_max = 5;
                }

                if (code_max == 5) // hitted max #iterations (solver ended prematurely)
                {
                    // find an infeasible max time
                    // initialize infeasible max time
                    inst->max_time_drone[i][j][k] = d1 / UAV_MIN_SPEED + d2 / UAV_MIN_SPEED;
                    //printf("*** finding a max infesible time...\n");
                    int it = 0; // while counter
                    double t = inst->max_feas_time_drone[i][j][k] + 0.5 * pow(2, it);
                    while (t < inst->max_time_drone[i][j][k])
                    {
                        double e = compute_min_energy_time(d1, d2, inst->nodes[j].weight, t, d1 / s1_feas, d2 / s2_feas);
                        //printf("*** [MAX] compute energy of time %f: %f ***\n", t, e);
                        if (e > B)
                            break;
                        t = inst->max_feas_time_drone[i][j][k] + 0.5 * pow(2, it);
                        it++;
                    }
                    if (t < inst->max_time_drone[i][j][k])
                        inst->max_time_drone[i][j][k] = t;
                }
                else // OK
                {
                    // use the returned solution for both the max and the max feasible times
                    inst->max_time_drone[i][j][k] = inst->max_feas_time_drone[i][j][k];
                }

                double h = B - cruise_energy(s1_feas, inst->nodes[j].weight, d1) - cruise_energy(s2_feas, 0.0, d2);
                //printf("max feasibile: %.12f | %.12f \n", inst->max_feas_time_drone[i][j][k], cruise_energy(s1_feas, inst->nodes[j].weight, d1) + cruise_energy(s2_feas, 0.0, d2) + h);
                h = B - cruise_energy(s1, inst->nodes[j].weight, d1) - cruise_energy(s2, 0.0, d2);
                //printf("max          : %.12f | %.12f \n", inst->max_time_drone[i][j][k], cruise_energy(s1, inst->nodes[j].weight, d1) + cruise_energy(s2, 0.0, d2) + h);

                // add extra time (launch/service/landing etc)
                inst->min_time_drone[i][j][k] += extra_time;
                inst->min_feas_time_drone[i][j][k] += extra_time;
                inst->max_feas_time_drone[i][j][k] += extra_time;
                inst->max_time_drone[i][j][k] += extra_time;
                // print to file
                fprintf(fd, "%d,%d,%d, %f, %f, %f, %f\n", i, j, k, inst->min_time_drone[i][j][k], inst->min_feas_time_drone[i][j][k], inst->max_feas_time_drone[i][j][k], inst->max_time_drone[i][j][k]);
                printf("\t Min time: %.3f seconds \n", inst->min_time_drone[i][j][k]);
                printf("\t Min feasible time: %.3f seconds \n", inst->min_feas_time_drone[i][j][k]);
                printf("\t Max feasible time: %.3f seconds \n", inst->max_feas_time_drone[i][j][k]);
                printf("\t Max time: %.3f seconds \n", inst->max_time_drone[i][j][k]);
            }
        }
    }
    fclose(fd);
}*/

void compute_min_max_drone_legs_times(instance *inst)
{
	FILE *fd = fopen("minmax_nlopt.csv", "w+");
    // retrieve all the weights
	double *weights = (double *)malloc(inst->dimension * sizeof(double));
	double *best_speed_weights = (double *)malloc(inst->dimension * sizeof(double));
	weights[0] = 0.0;
	int n_weights = 1;

	for (int i = 0; i < inst->dimension; i++)
	{
		if (inst->nodes[i].weight < 0 || inst->nodes[i].weight > UAV_WEIGHT_LIMIT)
			continue;
		int flag = 0;
		for (int j = 0; j < n_weights; j++)
		{
			if (fabs(weights[j] - inst->nodes[i].weight) < 1e-6)
			{
				flag = 1;
				break;
			}
		}
		if (!flag)
			weights[n_weights++] = inst->nodes[i].weight;
	}
	weights = (double *)realloc(weights, n_weights * sizeof(double));
	best_speed_weights = (double *)realloc(best_speed_weights, n_weights * sizeof(double));

    // compute the max range speed for each weight (the speed with the smaller energy-per-meter consumption)
	if (inst->param.select_max_speed == 0) {
		for (int i = 0; i < n_weights; i++)
		{
			best_speed_weights[i] = compute_max_range_speed(weights[i], inst->param.uav_min_speed, inst->param.uav_max_speed);
			printf("Max range speed for weight %.2f lb : %f\n", weights[i], best_speed_weights[i]);
		}
	}

	for (int i = 0; i < inst->dimension; i++)
	{
		for (int j = 0; j < inst->dimension; j++)
		{
			for (int k = 0; k < inst->dimension; k++)
			{
				if (inst->nodes[j].weight > UAV_WEIGHT_LIMIT || inst->nodes[j].weight < 0)
				{
                    //fprintf(fd, "%d, %d, %d, 0, 0, 0, 0\n", i, j, k);
					continue;
				}
				if (i == inst->dimension - 1 || j == 0 || j == inst->dimension - 1 || k == 0 || j == i || k == j || k == i)
				{
                    //fprintf(fd, "%d, %d, %d, 0, 0, 0, 0\n", i, j, k);
					continue;
				}
				printf("i = %d, j = %d, k = %d: \n", i, j, k);

				double B = UAV_BATTERY_CAPACITY;
				B -= vertical_energy(TAKEOFF_SPEED, CRUISE_ALTITUDE, inst->nodes[j].weight);
				B -= vertical_energy(LANDING_SPEED, CRUISE_ALTITUDE, inst->nodes[j].weight);
				B -= vertical_energy(TAKEOFF_SPEED, CRUISE_ALTITUDE, 0.0);
				B -= vertical_energy(LANDING_SPEED, CRUISE_ALTITUDE, 0.0);

				double d1, d2;
                d1 = inst->drone_dists[i][j]; // distance i -> j
                d2 = inst->drone_dists[j][k]; // distance j -> k

                double max_r_s1 = -1;

                if (inst->param.select_max_speed == 0) {
                    // check if using the max range speeds the energy constraint is satisfied
                	for (int r = 0; r < n_weights; r++)
                	{
                		if (fabs(weights[r] - inst->nodes[j].weight) < 1e-6)
                		{
                			max_r_s1 = best_speed_weights[r];
                			break;
                		}
                	}
                	if (max_r_s1 < 0)
                		print_error("Error in compute_min_max_drone_legs_times(), a weight has not been found in the array weights\n");
                	if (cruise_energy(max_r_s1, inst->nodes[j].weight, d1) + cruise_energy(best_speed_weights[0], 0.0, d2) > B)
                		continue;
                    // the drone leg i-->j-->k is feasible
                	inst->min_feas_time_drone[i][j][k] = d1 / max_r_s1 + d2 / best_speed_weights[0];
                	double h = B - cruise_energy(max_r_s1, inst->nodes[j].weight, d1) - cruise_energy(best_speed_weights[0], 0.0, d2);
                	inst->max_feas_time_drone[i][j][k] = inst->min_feas_time_drone[i][j][k] + h / hovering_power;

                } else {
                	if (cruise_energy(inst->param.uav_max_speed, inst->nodes[j].weight, d1) + cruise_energy(inst->param.uav_max_speed, 0.0, d2) > B) {
                		continue;
                	}
                	inst->min_feas_time_drone[i][j][k] = d1 / inst->param.uav_max_speed + d2 / inst->param.uav_max_speed;
                	double h = B - cruise_energy(inst->param.uav_max_speed, inst->nodes[j].weight, d1) - cruise_energy(inst->param.uav_max_speed, 0.0, d2);
                	inst->max_feas_time_drone[i][j][k] = inst->min_feas_time_drone[i][j][k] + h / hovering_power;
                }


                if (inst->param.select_max_range_speed == 0 && inst->param.select_max_speed == 0) {
                    // COMPUTE THE MINIMUM DRONE TRAVEL TIME for the path i-->j-->k
                	double s1 = -1, s2 = -1;
                	double s1_feas = -1, s2_feas = -1;
                    //double min_feas_time = -1, min_time -1;
                    //printf("\n[%d][%d][%d] B: %.12f\n", i, j, k, B);
                	int code_min = compute_min_time(d1, d2, inst->nodes[j].weight, B, max_r_s1, best_speed_weights[0],
                		inst->param.uav_min_speed, inst->param.uav_max_speed,
                		&inst->min_time_drone[i][j][k], &inst->min_feas_time_drone[i][j][k], &s1, &s2, &s1_feas, &s2_feas);
                    //printf("\t s1: %.2f s2: %.2f s1_feas: %.2f s2_feas: %.2f \n", s1, s2, s1_feas, s2_feas);
                    // check
                	double e_min;

                	if (code_min < 0)
                	{
                		printf("***code_min = %d\n", code_min);
                        // should not occur ...
                        // repair using the initial feasible solution
                		inst->min_feas_time_drone[i][j][k] = d1 / max_r_s1 + d2 / best_speed_weights[0];
                        // and maximum allowable time as infeasible solution
                		inst->min_time_drone[i][j][k] = d1 / inst->param.uav_max_speed + d2 / inst->param.uav_max_speed;
                		code_min = 5;
                	}

                	if (code_min == 5)
                	{
                		printf("***hitted max #iterations (solver ended prematurely)\n");
                        // find an infeasible min time
                        // initialize infeasible min time
                		inst->min_time_drone[i][j][k] = d1 / inst->param.uav_max_speed + d2 / inst->param.uav_max_speed;
                		if (inst->min_time_drone[i][j][k] < inst->min_feas_time_drone[i][j][k] - NLOPT_TOLERANCE){
	                        int it = 0; // while counter
	                        double t = inst->min_feas_time_drone[i][j][k] - NLOPT_TOLERANCE;
	                        printf("min feas speeds = [%f,%f]\n", s1_feas, s2_feas);
	                        printf("* min time = %f\n", inst->min_time_drone[i][j][k]);
	                        printf("* min feas time = %f\n", inst->min_feas_time_drone[i][j][k]);
	                        while (inst->min_feas_time_drone[i][j][k] - inst->min_time_drone[i][j][k] > NLOPT_TOLERANCE)
	                        {
	                        	double e = compute_min_energy_time(d1, d2, inst->nodes[j].weight, t, d1 / s1_feas, d2 / s2_feas,
	                        		inst->param.uav_min_speed, inst->param.uav_max_speed, B);
	                            //printf("*** [MIN] compute energy of time %f: %f ***\n", t, e);
	                        	if (e > B){
	                        		printf("* t = %f infeasible \n",t);
	                        		inst->min_time_drone[i][j][k] = t;
	                        	}
	                        	else{
	                        		printf("* t = %f feasible \n",t);
	                        		inst->min_feas_time_drone[i][j][k] = t;
	                        	}
	                        	it++;
	                        	double step = (inst->min_feas_time_drone[i][j][k] - inst->min_time_drone[i][j][k])/2;
	                        	if (step > NLOPT_TOLERANCE*pow(2,it))
	                        		step = NLOPT_TOLERANCE*pow(2,it);
	                        	t = inst->min_feas_time_drone[i][j][k] - step;

	                        }
	                        printf("*** min time = %f\n", inst->min_time_drone[i][j][k]);
	                        printf("*** min feas time = %f\n", inst->min_feas_time_drone[i][j][k]);
	                        // if (it > 3)
	                        // 	print_error("");
	                    }
	                }
	                else
	                {
                        // use the returned solution for both the min and the min feasible times
	                	inst->min_time_drone[i][j][k] = inst->min_feas_time_drone[i][j][k];
	                	e_min = cruise_energy(s1, inst->nodes[j].weight, d1) + cruise_energy(s2, 0.0, d2);
	                }

                    //printf("min          : %.12f | %.12f \n", inst->min_time_drone[i][j][k], e_min);
                    //printf("min feasibile: %.12f | %.12f \n", inst->min_feas_time_drone[i][j][k], cruise_energy(s1_feas, inst->nodes[j].weight, d1) + cruise_energy(s2_feas, 0.0, d2));

                    // COMPUTE THE MAXIMUM DRONE TRAVEL TIME for the path i-->j-->k
	                s1 = -1, s2 = -1;
	                s1_feas = -1, s2_feas = -1;

	                int code_max = compute_max_time(d1, d2, inst->nodes[j].weight, B, max_r_s1, best_speed_weights[0],
	                	inst->param.uav_min_speed, inst->param.uav_max_speed,
	                	&inst->max_time_drone[i][j][k], &inst->max_feas_time_drone[i][j][k], &s1, &s2, &s1_feas, &s2_feas);
                    //printf("\t s1: %.2f s2: %.2f s1_feas: %.2f s2_feas: %.2f \n", s1, s2, s1_feas, s2_feas);

	                if (code_max < 0)
	                {
	                	printf("***code_max = %d\n", code_max);
                        // it should not occur ...
                        // repair using the initial feasible solution
	                	inst->max_feas_time_drone[i][j][k] = d1 / max_r_s1 + d2 / best_speed_weights[0];
                        // and maximum allowable time as infeasible solution
	                	inst->max_time_drone[i][j][k] = d1 / inst->param.uav_min_speed + d2 / inst->param.uav_min_speed;
	                	code_max = 5;
	                }

                    if (code_max == 5) // hitted max #iterations (solver ended prematurely)
                    {
                    	printf("***hitted max #iterations (solver ended prematurely)\n");
                        // find an infeasible max time
                        // initialize infeasible max time
                    	inst->max_time_drone[i][j][k] = d1 / inst->param.uav_min_speed + d2 / inst->param.uav_min_speed;
                    	if (inst->max_time_drone[i][j][k] > inst->max_feas_time_drone[i][j][k] + NLOPT_TOLERANCE){
                    		printf("max feas speeds = [%f,%f]\n", s1_feas, s2_feas);
                    		printf("* max time = %f\n", inst->max_time_drone[i][j][k]);
                    		printf("* max feas time = %f\n", inst->max_feas_time_drone[i][j][k]);
	                        //printf("*** finding a max infesible time...\n");
	                        int it = 1; // while counter
	                        double t = inst->max_feas_time_drone[i][j][k] + NLOPT_TOLERANCE;
	                        while (inst->max_time_drone[i][j][k] - inst->max_feas_time_drone[i][j][k] > NLOPT_TOLERANCE)
	                        {
	                        	double e = compute_min_energy_time(d1, d2, inst->nodes[j].weight, t, d1 / s1_feas, d2 / s2_feas,
	                        		inst->param.uav_min_speed, inst->param.uav_max_speed, B);
	                            //printf("*** [MAX] compute energy of time %f: %f ***\n", t, e);
	                        	if (e > B){
	                        		printf("* t = %f infeasible \n",t);
	                        		inst->max_time_drone[i][j][k] = t;
	                        	}
	                        	else{
	                        		printf("* t = %f feasible \n",t);
	                        		inst->max_feas_time_drone[i][j][k] = t;

	                        	}
	                        	double step = (inst->max_time_drone[i][j][k] - inst->max_feas_time_drone[i][j][k])/2;
	                        	it++;
	                        	if (step > NLOPT_TOLERANCE*pow(2,it))
	                        		step = NLOPT_TOLERANCE*pow(2,it);
	                        	t = inst->max_feas_time_drone[i][j][k] + step;
	                        }
	                        printf("*** max time = %f\n", inst->max_time_drone[i][j][k]);
	                        printf("*** max feas time = %f\n", inst->max_feas_time_drone[i][j][k]);
	                        // if (it > 3)
	                        // 	print_error("");
	                    }
	                }
                    else // OK
                    {
                        // use the returned solution for both the max and the max feasible times
                    	inst->max_time_drone[i][j][k] = inst->max_feas_time_drone[i][j][k];
                    }
                    if (inst->max_time_drone[i][j][k] - inst->max_feas_time_drone[i][j][k] > 1e-4)
                    	print_error("An optimality tolerance greater of 1e-4 has been found. EXIT");
                    if (inst->min_feas_time_drone[i][j][k] - inst->min_time_drone[i][j][k] > 1e-4)
                    	print_error("An optimality tolerance greater of 1e-4 has been found. EXIT");

                }
                else {
                	inst->min_time_drone[i][j][k] = inst->min_feas_time_drone[i][j][k];
                	inst->max_time_drone[i][j][k] = inst->max_feas_time_drone[i][j][k];
                }

                // add extra time (launch/service/landing etc)
                inst->min_time_drone[i][j][k] += extra_time;
                inst->min_feas_time_drone[i][j][k] += extra_time;
                inst->max_feas_time_drone[i][j][k] += extra_time;
                inst->max_time_drone[i][j][k] += extra_time;
                // print to file
                fprintf(fd, "%d,%d,%d, %f, %f, %f, %f\n", i, j, k, inst->min_time_drone[i][j][k], inst->min_feas_time_drone[i][j][k], inst->max_feas_time_drone[i][j][k], inst->max_time_drone[i][j][k]);
                printf("\t Min time: %.3f seconds \n", inst->min_time_drone[i][j][k]);
                printf("\t Min feasible time: %.3f seconds \n", inst->min_feas_time_drone[i][j][k]);
                printf("\t Max feasible time: %.3f seconds \n", inst->max_feas_time_drone[i][j][k]);
                printf("\t Max time: %.3f seconds \n", inst->max_time_drone[i][j][k]);
            }
        }
    }
    fclose(fd);
}

int generate_csv_record(char *path, char* output_name, char *instance_name, int seed, int model, int run, double z_best, double time_elapsed, double time_incumbent, int ticks)
{
	FILE *csv;
	char output_path[1000];
	sprintf(output_path, "%s/%s.csv", path, output_name);
	int flag_new_file = 0;
	if (!IsPathExist(output_path))
		flag_new_file = 1;
	csv = fopen(output_path, "a");
	if (csv == NULL)
		print_error("generate_csv_record fopen() error");
    // if (flag_new_file)
    // {
    //     if (!ticks)
    //         fprintf(csv, "# name instance, model, seed, run, incumbent, time elapsed [s], time incumbent [s]\n");
    //     else
    //         fprintf(csv, "# name instance, model, seed, run, incumbent, time elapsed [ticks]\n");
    // }
    // fprintf(csv, "%s, %d, %d, %d, %f, %f, %f\n", instance_name, model, seed, run, z_best, time_elapsed, time_incumbent);

	fprintf(csv, "%s, %d, %f, %f\n", instance_name, model, z_best, time_elapsed);

	if (fclose(csv))
		print_error("generate_csv_record fclose() error");

	return 0;
}

void createInstanceFolders(instance *inst)
{
	if (!IsPathExist("../output"))
	{
		if (mkdir("../output", 0777))
			print_error("error creating output folder");
	}

	char instance_folder_path[1000];
	sprintf(instance_folder_path, "../output/%s", inst->instance_name);
	if (!IsPathExist(instance_folder_path))
	{
		if (mkdir(instance_folder_path, 0777))
			print_error("error creating instance folder");
	}
	char seed_path[1050];
	sprintf(seed_path, "%s/seed_%d", instance_folder_path, inst->param.seed);
	if (!IsPathExist(seed_path))
	{
		if (mkdir(seed_path, 0777))
			print_error("error creating seed folder");
	}
	setRunNumber(inst);
	char run_path[1100];
	sprintf(run_path, "%s/run_%d", seed_path, inst->param.run);
	if (mkdir(run_path, 0777))
		print_error("error creating run folder");
	char plot_path[1110];
	sprintf(plot_path, "%s/plot", run_path);
	if (mkdir(plot_path, 0777))
		print_error("error creating plot folder");

	if (!IsPathExist("data_temp"))
		if (mkdir("data_temp", 0777))
			print_error("error creating temp folder");


	}

	int IsPathExist(const char *s)
	{
		struct stat buffer;
		return !stat(s, &buffer);
	}

	void getTimeStamp(double *ts)
	{
		struct timespec timestamp;
		if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
			print_error("Error clock_gettime");
		*ts = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);
	}

// comparison function for qsort
	int compare(const void *a, const void *b)
	{
		if (*(double *)a > *(double *)b)
			return 1;
		if (*(double *)a < * (double *)b)
			return -1;
		return 0;
	}

	void compute_opt_speeds(instance *inst)
	{
		double granularity = 0.5;
		int nspeeds = (int)((inst->param.uav_max_speed - inst->param.uav_min_speed) / granularity) + 1;

    // speeds
		double speeds[nspeeds];

    // initialize speeds
    // initialize times w.r.t. to each speed
		for (int i = 0; i < nspeeds; i++)
		{
			speeds[nspeeds - 1 - i] = inst->param.uav_max_speed - i * granularity;
			printf("speed[%d] = %f\n", nspeeds - 1 - i, speeds[nspeeds - 1 - i]);
		}
		double min_speed = speeds[0];

		double *cruise_power_0 = (double *)calloc(nspeeds, sizeof(double));
		cruise_power_consumption(0.0, nspeeds, speeds, cruise_power_0);

    // double takeoff_power_0 = vertical_power_consumption(0, UAV_ASCENDING_SPEED);
    // double landing_power_0 = vertical_power_consumption(0, UAV_DESCENDING_SPEED);

		for (int i = 0; i < inst->dimension - 1; i++)
		{
			for (int j = 1; j < inst->dimension - 1; j++)
			{
				if (j == i)
					continue;
				for (int k = 1; k < inst->dimension; k++)
				{
					if (k == j || k == i)
						continue;
					double w = inst->nodes[j].weight;
					if (w > UAV_WEIGHT_LIMIT)
						continue;
                //double d1 = dist(i, j, inst);
                //double d2 = dist(j, k, inst);
					double d1 = inst->drone_dists[i][j];
					double d2 = inst->drone_dists[j][k];
                //printf("d1 = %f, d2 = %f\n", d1, d2);

					int opt_min_idx_s1, opt_min_idx_s2;
					int opt_max_idx_s1, opt_max_idx_s2;

                // compute the energy for each couple of speed (s1,s2)
					double *cruise_power_w = (double *)calloc(nspeeds, sizeof(double));
					cruise_power_consumption(w, nspeeds, speeds, cruise_power_w);

                // travel times
					double T1[nspeeds];
					double T2[nspeeds];
					for (int i = 0; i < nspeeds; i++)
					{
						T1[i] = d1 * (1 / speeds[i]);
						T2[i] = d2 * (1 / speeds[i]);
					}

                // remeaning battery capacity for the horizontal flights [J]
					double newB = UAV_BATTERY_CAPACITY;
					newB -= vertical_energy(TAKEOFF_SPEED, CRUISE_ALTITUDE, inst->nodes[j].weight);
					newB -= vertical_energy(LANDING_SPEED, CRUISE_ALTITUDE, inst->nodes[j].weight);
					newB -= vertical_energy(TAKEOFF_SPEED, CRUISE_ALTITUDE, 0.0);
					newB -= vertical_energy(LANDING_SPEED, CRUISE_ALTITUDE, 0.0);
                //printf("newB = %f\n", newB);
                // compute the optimal feasible couple of speed and the corresponding travel time
					printf("i = %d, j = %d, k = %d \n", i, j, k);
					double min_tt = min_time(newB, cruise_power_w, cruise_power_0, min_speed, granularity, nspeeds, speeds, T1, T2, &opt_min_idx_s1, &opt_min_idx_s2);
					if (min_tt < 0)
					{
						printf("No feasible speeds have been found. \n\n");
					}
					else
					{
                    // inst->min_time_drone[i][j][k] = min_tt + 2 * (UAV_CRUISE_ALTITUDE / UAV_ASCENDING_SPEED + UAV_ROTATION_TIME) + 2 * (UAV_CRUISE_ALTITUDE / UAV_DESCENDING_SPEED) + UAV_DELIVERY_TIME;
						inst->min_time_drone[i][j][k] = min_tt + extra_time;
						printf("\t Min time: %.3f seconds \n", inst->min_time_drone[i][j][k]);
                    //printf("\t [Min] Optimal speeds: (%.1f m/s, %.1f m/s) \n", speeds[opt_min_idx_s1], speeds[opt_min_idx_s2]);
                    //printf("\t [Min] Energy = %f kJ \n\n", (T1[opt_min_idx_s1] * cruise_power_w[opt_min_idx_s1] + T2[opt_min_idx_s2] * cruise_power_0[opt_min_idx_s2]) / 1000);

						double max_tt = max_time(newB, cruise_power_w, cruise_power_0, inst->param.uav_max_speed, granularity, nspeeds, speeds, T1, T2, &opt_max_idx_s1, &opt_max_idx_s2);
						if (max_tt < 0)
						{
							printf("No feasible speeds have been found. \n\n");
						}
						else
						{
							inst->max_time_drone[i][j][k] = max_tt + 2 * (UAV_CRUISE_ALTITUDE / UAV_ASCENDING_SPEED + UAV_ROTATION_TIME) + 2 * (UAV_CRUISE_ALTITUDE / UAV_DESCENDING_SPEED) + UAV_DELIVERY_TIME;
							inst->max_time_drone[i][j][k] = max_tt + extra_time;
							printf("\t Max time: %.3f seconds \n", inst->max_time_drone[i][j][k]);
                        //printf("\t [Max] Optimal speeds: (%.1f m/s, %.1f m/s) \n", speeds[opt_max_idx_s1], speeds[opt_max_idx_s2]);
                        //printf("\t [Max] Energy (to go from i to k) = %f kJ \n\n", (T1[opt_max_idx_s1] * cruise_power_w[opt_max_idx_s2] + T2[opt_min_idx_s2] * cruise_power_0[opt_min_idx_s2]) / 1000);
						}
					}
                // free cruise_power_w array
					free(cruise_power_w);
				}
			}
		}

    // struct timespec timestamp;
    // if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
    //     print_error("Error clock_gettime");
    // double timestamp_start = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);

    // if (clock_gettime(CLOCK_REALTIME, &timestamp) == -1)
    //     print_error("Error clock_gettime");
    // double timestamp_finish = timestamp.tv_sec + timestamp.tv_nsec * pow(10, -9);

    // double time_elapsed = timestamp_finish - timestamp_start;
    // printf("Elapsed time: %f seconds\n", time_elapsed);

    // free cruise_power_0 array
		free(cruise_power_0);
	}

// compute the cruise power consumption to travel one second at speed s[i] with payload w
	void cruise_power_consumption(double w, int nspeeds, double *speeds, double *P)
	{
    // double c1 = C1;
    // double c2 = C2;
    // double c4 = C4;
    // double c5 = C5;
    // double g = G;

    // double gforce_1 = (W + w) * g;
    // double gforce_2 = W * g;
    // double c12 = c1 + c2;
    // double alpha_rad = 10.0 / 180 * M_PI;
    // float c5_f = c5 * cos(alpha_rad) * cos(alpha_rad);
    // double c4_f = c4 * c4;

		for (int i = 0; i < nspeeds; i++)
		{
        //P[i] = c12 * pow(pow((gforce_1 - c5_f * speeds[i] * speeds[i]), 2) + c4_f * pow(speeds[i], 4), 0.75) + c4 * pow(speeds[i], 3);
			P[i] = cruise_power(speeds[i], w);
		}
	}

// energy_grid = matrix of energy for all speed combinatation
// B = battery capacity (e.g., 500 KJ)
	double min_time(double B, double *power_w, double *power_0, double min_speed, double granularity, int nspeeds, double *speeds, double *T1, double *T2, int *opt_idx_s1, int *opt_idx_s2)
	{
		int opt_i = -1, opt_j = -1;
		double min_time = DBL_MAX;
    //int min_speed_idx = (min_speed - 1.0) / granularity;
		int min_speed_idx = 0;

		for (int i = min_speed_idx; i < nspeeds; i++)
		{
			for (int j = min_speed_idx; j < nspeeds; j++)
			{
				if (T1[i] * power_w[i] + T2[j] * power_0[j] > B)
					continue;
				if (T1[i] + T2[j] < min_time)
				{
					min_time = T1[i] + T2[j];
					opt_i = i;
					opt_j = j;
				}
			}
		}
		if (opt_i == -1)
		{
			return -1.0;
		}

		*opt_idx_s1 = opt_i;
		*opt_idx_s2 = opt_j;
		return min_time;
	}

	double max_time(double B, double *power_w, double *power_0, double max_speed, double granularity, int nspeeds, double *speeds, double *T1, double *T2, int *opt_idx_s1, int *opt_idx_s2)
	{
		double c1 = 2.8037;
		double c2 = 0.3177;
		double g = 9.8;

		int opt_i = -1, opt_j = -1;
		double max_time = 0.0;

		double p_h = (c1 + c2) * pow((W * g), 1.5);

    //int max_speed_idx = (max_speed - 1.0) / granularity;
		int max_speed_idx = nspeeds;

		for (int i = 0; i < max_speed_idx; i++)
		{
			for (int j = 0; j < max_speed_idx; j++)
			{
				double e_tmp = T1[i] * power_w[i] + T2[j] * power_0[j];
				if (T1[i] * power_w[i] + T2[j] * power_0[j] > B)
					continue;
				double t_hover = (B - e_tmp) / p_h;
				if (T1[i] + T2[j] + t_hover > max_time)
				{
					max_time = T1[i] + T2[j] + t_hover;
					opt_i = i;
					opt_j = j;
				}
			}
		}
		if (opt_i == -1)
		{
			return -1.0;
		}

		*opt_idx_s1 = opt_i;
		*opt_idx_s2 = opt_j;
		return max_time;
	}

	void setRunNumber(instance *inst)
	{
		char path[1000];
		sprintf(path, "../output/%s/seed_%d", inst->instance_name, inst->param.seed);
		inst->param.run = 1 + countDir(path);
	}

	int countDir(char *dir_path)
	{
		struct dirent *dp;
		DIR *fd;

		if ((fd = opendir(dir_path)) == NULL)
		{
			print_error("Error in opendir\n");
			return 0;
		}
		int count = 0;
		while ((dp = readdir(fd)) != NULL)
		{
			if (!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, ".."))
            continue; /* skip self and parent */
				count++;
		}
		closedir(fd);
		return count;
	}

// for instances in http://mario.ruthmair.at/wp-content/uploads/2020/04/socnet-instances-v2.zip

// /*Number of Nodes*/
// [nnodes]
// /*The Depot*/
// [x_coor_depot] [y_coor_depot depot[]
// /*The Locations (x_coor y_coor name)*/

// Each node has a corresponding latitude and longitude (specified in degrees).
// The altitude is always 0. Customer nodes have a corresponding non-zero parcel weight (in [pounds]).
// There is no parcel associated with the depot.
	void parse_locations_poikonen(instance *inst)
	{
		printf("\n\nParsing the locations...\n");
		FILE *fp = NULL;
		char filename[1000];

		fp = fopen(inst->instance_path, "r");
		if (fp == NULL)
			print_error("Could not open the file ");

    char line[1000]; // tmp string where to store a line

    // Read each line from the input file:
    // split the line using strtok and consider the generated tokens

    char delimiters[] = " ";

    // read (ignore) the first line
    if (fgets(line, sizeof(line), fp) == NULL)
    	print_error("Error reading the first line of the input data.");
    // read #nodes
    if (fgets(line, sizeof(line), fp) == NULL)
    	print_error("Error reading the second line of the input data.");
    inst->dimension = strtol(strtok(line, delimiters), NULL, 10) + 1;
    // allocate memory for the nodes array
    inst->nodes = (node_struct *)calloc(inst->dimension, sizeof(node_struct));
    // read (ignore) the third line
    if (fgets(line, sizeof(line), fp) == NULL)
    	print_error("Error reading the third line of the input data.");
    // parse x,y coordinates of the depot
    if (fgets(line, sizeof(line), fp) == NULL)
    	print_error("Error reading the third line of the input data.");
    inst->nodes[0].x = strtod(strtok(line, delimiters), NULL); // latitude
    inst->nodes[0].y = strtod(strtok(NULL, delimiters), NULL); // longitude
    inst->nodes[0].id = 0;
    inst->nodes[0].weight = 0;
    if (inst->param.verbose >= DEBUG)
    	printf("\tNODE 0 (DEPOT) at coordinates (%11.6lf , %11.6lf)\n", inst->nodes[0].x, inst->nodes[0].y);
    // read (ignore) the fifth line
    if (fgets(line, sizeof(line), fp) == NULL)
    	print_error("Error reading the fifth line of the input data.");
    for (int i = 1; i < inst->dimension - 1; i++)
    {
    	if (fgets(line, sizeof(line), fp) == NULL)
    		print_error("Error reading a line of the input data.");
        line[strcspn(line, "\n")] = 0; // removing trailing \n

        inst->nodes[i].x = strtod(strtok(line, delimiters), NULL); // latitude
        inst->nodes[i].y = strtod(strtok(NULL, delimiters), NULL); // longitude
        inst->nodes[i].id = i;
        inst->nodes[i].weight = 0;
        if (inst->param.MHD == 1) {
        	if ((i - 1) % 5 == 0)
        		inst->nodes[i].truck_only = 1;
        }
        if (inst->param.verbose >= DEBUG)
        	printf("\tNODE %2d at coordinates (%11.6lf , %11.6lf)\n", inst->nodes[i].id, inst->nodes[i].x, inst->nodes[i].y);
    }

    // define final depot that represent the returning depot
    inst->nodes[inst->dimension - 1].id = inst->dimension - 1;
    inst->nodes[inst->dimension - 1].x = inst->nodes[0].x;
    inst->nodes[inst->dimension - 1].y = inst->nodes[0].y;
    inst->nodes[inst->dimension - 1].weight = inst->nodes[0].weight;
    if (inst->param.verbose >= DEBUG)
    	printf("\tNODE %d (FINAL DEPOT) at coordinates (%11.6lf , %11.6lf)\n", inst->dimension - 1, inst->nodes[inst->dimension - 1].x, inst->nodes[inst->dimension - 1].y);
    fclose(fp);

    if (inst->dimension < 3)
    	print_error("EXIT: the instance does not have any customer.");
}

void compute_truck_travel_times_poikonen(instance *inst)
{
	printf("\n\nComputing truck travel times...\n");

	for (int i = 0; i < inst->dimension; i++)
	{
		for (int j = 0; j < inst->dimension; j++)
		{
			if (j == i) {
				inst->truck_times[i][j] = DBL_MAX;
				continue;
			}
			inst->truck_times[i][j] = (int)(fabs(inst->nodes[i].x - inst->nodes[j].x) + fabs(inst->nodes[i].y - inst->nodes[j].y));
		}
	}

	if (inst->param.verbose >= DEBUG)
	{
		for (int i = 0; i < inst->dimension; i++)
		{
			for (int j = 0; j < inst->dimension; j++)
			{
				if (inst->truck_times[i][j] == DBL_MAX) continue;
				printf("\ttruck_time(%3d,%3d) = %15.6lf \n", i, j, inst->truck_times[i][j]);
			}
		}
	}
	printf("Truck travel times computed.\n");
}

void compute_drone_travel_times_poikonen(instance *inst)
{
	printf("\n\nComputing drone travel times...\n");

    // define maximum drone travel time = max truck time * n
	int max_drone_tt = 0;
	for (int i = 0; i < inst->dimension; i++)
	{
		for (int j = 0; j < inst->dimension; j++)
		{
			if (j == i)
				continue;
			if (inst->truck_times[i][j] == DBL_MAX)
				continue;
			if (inst->truck_times[i][j] > max_drone_tt)
				max_drone_tt = (int)inst->truck_times[i][j];
		}
	}
	max_drone_tt *= inst->dimension;

	for (int i = 0; i < inst->dimension - 1; i++)
	{
		for (int j = 1; j < inst->dimension - 1; j++)
		{

			for (int k = 1; k < inst->dimension; k++)
			{
				if (j == i) {
					inst->max_feas_time_drone[i][j][k] = DBL_MAX;
					inst->max_time_drone[i][j][k] = DBL_MAX;
					continue;
				}
				if (inst->nodes[j].truck_only == 1) {
					inst->max_feas_time_drone[i][j][k] = DBL_MAX;
					inst->max_time_drone[i][j][k] = DBL_MAX;
					continue;
				}
				if (k == i || k == j) {
					inst->max_feas_time_drone[i][j][k] = DBL_MAX;
					inst->max_time_drone[i][j][k] = DBL_MAX;
					continue;
				}

                /*
                if (inst->param.alpha == 3)
                                {
                                    inst->min_feas_time_drone[i][j][k] = (int)(dist_EUC_2D(i, j, inst) * 0.3330); //0.330
                                    inst->min_feas_time_drone[i][j][k] += (int)(dist_EUC_2D(j, k, inst) * 0.3330); // 0.330
                                    // inst->min_feas_time_drone[i][j][k] = (int)(pow(pow(inst->nodes[i].x - inst->nodes[j].x, 2) + pow(inst->nodes[i].y - inst->nodes[j].y, 2), 0.5) * 0.3330);
                                    // inst->min_feas_time_drone[i][j][k] += (int)(pow(pow(inst->nodes[j].x - inst->nodes[k].x, 2) + pow(inst->nodes[j].y - inst->nodes[k].y, 2), 0.5) * 0.3330);
                                }
                                else
                                {
                                    inst->min_feas_time_drone[i][j][k] = (int)(dist_EUC_2D(i, j, inst) / inst->param.alpha);
                                    inst->min_feas_time_drone[i][j][k] += (int)(dist_EUC_2D(j, k, inst) / inst->param.alpha);
                                    // inst->min_feas_time_drone[i][j][k] = (int)(pow(pow(inst->nodes[i].x - inst->nodes[j].x, 2) + pow(inst->nodes[i].y - inst->nodes[j].y, 2), 0.5) / (double)inst->param.alpha);
                                    // inst->min_feas_time_drone[i][j][k] += (int)(pow(pow(inst->nodes[j].x - inst->nodes[k].x, 2) + pow(inst->nodes[j].y - inst->nodes[k].y, 2), 0.5) / (double)inst->param.alpha);
                                }
                */
				double alphaDrone1 = 1.0;
				double alphaDrone2 = 0.5;
				double alphaDrone3 = 0.333;
				double PREC = 1.0;
				switch (inst->param.alpha) {
					case 1:
					inst->min_feas_time_drone[i][j][k] =  (int)(euclDist(inst->nodes[i].x, inst->nodes[j].x, inst->nodes[i].y, inst->nodes[j].y) * alphaDrone1 * PREC) / PREC;
					inst->min_feas_time_drone[i][j][k] +=  (int)(euclDist(inst->nodes[j].x, inst->nodes[k].x, inst->nodes[j].y, inst->nodes[k].y) * alphaDrone1 * PREC) / PREC;
					break;
					case 2:
					inst->min_feas_time_drone[i][j][k] =  (int)(euclDist(inst->nodes[i].x, inst->nodes[j].x, inst->nodes[i].y, inst->nodes[j].y) * alphaDrone2 * PREC) / PREC;
					inst->min_feas_time_drone[i][j][k] +=  (int)(euclDist(inst->nodes[j].x, inst->nodes[k].x, inst->nodes[j].y, inst->nodes[k].y) * alphaDrone2 * PREC) / PREC;
					break;
					case 3:
					inst->min_feas_time_drone[i][j][k] =  (int)(euclDist(inst->nodes[i].x, inst->nodes[j].x, inst->nodes[i].y, inst->nodes[j].y) * alphaDrone3 * PREC) / PREC;
					inst->min_feas_time_drone[i][j][k] +=  (int)(euclDist(inst->nodes[j].x, inst->nodes[k].x, inst->nodes[j].y, inst->nodes[k].y) * alphaDrone3 * PREC) / PREC;
					break;
				}

				inst->max_feas_time_drone[i][j][k] = max_drone_tt;
				inst->max_time_drone[i][j][k] = max_drone_tt;
				if (inst->param.MHD == 1) {
					if (inst->min_feas_time_drone[i][j][k] > POIKONEN_UAV_ENDURANCE + 1e-6) {
						inst->min_feas_time_drone[i][j][k] = DBL_MAX;
						inst->min_time_drone[i][j][k] = DBL_MAX;
						inst->max_feas_time_drone[i][j][k] = DBL_MAX;
						inst->max_time_drone[i][j][k] = DBL_MAX;
					}
					else {
						inst->min_time_drone[i][j][k] = inst->min_feas_time_drone[i][j][k];
						inst->max_feas_time_drone[i][j][k] = POIKONEN_UAV_ENDURANCE;
						inst->max_time_drone[i][j][k] = POIKONEN_UAV_ENDURANCE;
					}
				}
			}
		}
	}

    /*    if (inst->param.verbose >= DEBUG)
        {
            for (int i = 0; i < inst->dimension; i++)
            {
                for (int j = 0; j < inst->dimension; j++)
                {
                    if (j == i)
                        continue;
                    for (int k = 0; k < inst->dimension; k++)
                    {
                        printf("\tmin drone_leg_time(%3d,%3d,%3d) = %15.6lf\n", i, j, k, inst->min_feas_time_drone[i][j][k]);
                        if ( inst->param.MHD ) {
                            printf("\tmax drone_leg_time(%3d,%3d,%3d) = %15.6lf\n", i, j, k, inst->max_feas_time_drone[i][j][k]);
                        }
                    }
                }
            }
            printf("\tmax drone_leg_time = %d\n", max_drone_tt);
        }*/
	printf("Drone travel times computed.\n");
}


// Ha et al. instances, paper: "On the min-cost Traveling Salesman Problem with Drone" https://doi.org/10.1016/j.trc.2017.11.015s
// The instances can be found at http://orlab.com.vn/files/tspd_instances.zip

// (last column after (x,y) coordinates: 1: truck-only customer)
// The authors define the truck/drone speeds in km/h
// LAUNCH_TIME, RETRIEVE_TIME and ENDURANCE are expressed in hours...
// by default we assume the launch and retrieval times to be 60 seconds as assumed by Ha et al.

// Example instance mbA101.txt:
/*
CUSTOMER_SIZE: 10
AREA_SIZE: 10.000000
DEPOT_LOCATION: BOTTOM_LEFT
DRONE_SPEED: 40.000000
TRUCK_SPEED: 40.000000
TRUCK_COST: 25.000000
ENDURANCE: 0.333333
WAITING_TIME: 99999.0
LAUNCH_TIME: 0.016666667
RETRIEVE_TIME: 0.016666667

NAME: mbA101
TYPE: TSP
COMMENT: TSP-D instance
DIMENSION: 11
EDGE_WEIGHT_TYPE : MAN_2D
NODE_COORD_TYPE : TWOD_COORDS
NODE_COORD_SECTION
0 0.0 0.0 0
1 9.191654 4.842804 0
2 1.469186 2.544484 0
3 3.917438 6.771700 0
4 2.989492 0.948969 0
5 1.850169 6.584529 0
6 5.546938 6.344087 1
7 7.233497 3.986597 0
8 8.239579 9.105439 0
9 1.269210 3.537688 0
10 0.047252 4.966567 0
EOF
*/


// /*Number of Nodes*/
// [nnodes]
// /*The Depot*/
// [x_coor_depot] [y_coor_depot depot[]
// /*The Locations (x_coor y_coor name)*/

// Each node has a corresponding latitude and longitude (specified in degrees).
// The altitude is always 0. Customer nodes have a corresponding non-zero parcel weight (in [pounds]).
// There is no parcel associated with the depot.
void parse_locations_ha(instance *inst, double * truck_speed, double *drone_speed, double * max_endurance)
{
	printf("\n\nParsing the locations...\n");
	FILE *fp = NULL;
	char filename[1000];

	fp = fopen(inst->instance_path, "r");
	if (fp == NULL)
		print_error("Could not open the file ");

	char delimiters[] = " ";

    char line[1000]; // tmp string where to store a line

    // Read each line from the input file:
    // split the line using strtok and consider the generated tokens

    while (fgets(line, sizeof(line), fp) != NULL) {
    	char *tmp_token;
    	tmp_token = strtok(line, delimiters);

    	if (strcmp(tmp_token, "EOF") == 0) {
    		break;
    	}

    	else if (strcmp(tmp_token, "0") == 0) {
    		inst->nodes[0].id = 0;
            inst->nodes[0].x = strtod(strtok(NULL, delimiters), NULL) * 1000; // x coordinate
            inst->nodes[0].y = strtod(strtok(NULL, delimiters), NULL) * 1000; // y coordinate
            inst->nodes[0].truck_only = 0;
            break;
        }
        // CUSTOMER_SIZE
        else if (strcmp(tmp_token, "CUSTOMER_SIZE:") == 0)
        {
            inst->dimension = strtol(strtok(NULL, delimiters), NULL, 10) + 2; // starting and ending depot
            // allocate memory for the nodes array
            inst->nodes = (node_struct *)calloc(inst->dimension, sizeof(node_struct));
            if (inst->param.verbose >= DEBUG)
            	printf("CUSTOMER_SIZE: %d\n", inst->dimension - 2);
            continue;
        }
        // TRUCK_SPEED
        else if (strcmp(tmp_token, "TRUCK_SPEED:") == 0)
        {
        	*truck_speed = strtod(strtok(NULL, delimiters), NULL) / 3.60f;
        	if (inst->param.verbose >= DEBUG)
        		printf("TRUCK_SPEED: %f\n", truck_speed);
        	continue;
        }
        // DRONE_SPEED
        else if (strcmp(tmp_token, "DRONE_SPEED:") == 0)
        {
        	*drone_speed = strtod(strtok(NULL, delimiters), NULL) / 3.60f;
        	if (inst->param.verbose >= DEBUG)
        		printf("DRONE_SPEED: %f\n", drone_speed);
        	continue;
        }
        // ENDURANCE
        else if (strcmp(tmp_token, "ENDURANCE:") == 0)
        {
        	*max_endurance = 3600.0f * strtod(strtok(NULL, delimiters), NULL);
        	if (inst->param.verbose >= DEBUG)
        		printf("ENDURANCE: %f\n", max_endurance);
        	*max_endurance = 1200.0f;
        	continue;
        }
        /*        else if (strcmp(tmp_token, "LAUNCH_TIME:") == 0)
                {
                    *launch_time = 3600.0f * strtod(strtok(NULL, delimiters), NULL);
                    if (inst->param.verbose >= DEBUG)
                        printf("LAUNCH_TIME: %f\n", launch_time);
                    continue;
                }
                else if (strcmp(tmp_token, "RETRIEVE_TIME:") == 0)
                {
                    *retrieve_time = 3600.0f * strtod(strtok(NULL, delimiters), NULL);
                    if (inst->param.verbose >= DEBUG)
                        printf("RETRIEVE_TIME: %f\n", retrieve_time);
                    continue;
                }
        */        
        else {
        	continue;
        }
    }

    if (inst->dimension == 0)
    	print_error("Unable to parse the input file.");

    // PARSE CUSTOMERS' LOCATION
    if (inst->param.verbose >= DEBUG)
    	printf("\tDEPOT %d at coordinates (%11.6lf , %11.6lf), truck-only: %d\n", inst->nodes[0].id, inst->nodes[0].x, inst->nodes[0].y, inst->nodes[0].truck_only);

    for (int i = 1; i < inst->dimension - 1; i++) {
        // read a customer line
    	if (fgets(line, sizeof(line), fp) == NULL)
    		print_error("Error reading a customer line of the input data.");
        inst->nodes[i].id = strtol(strtok(line, delimiters), NULL, 10); // customer id
        inst->nodes[i].x = strtod(strtok(NULL, delimiters), NULL) * 1000; // x coordinate
        inst->nodes[i].y = strtod(strtok(NULL, delimiters), NULL) * 1000; // y coordinate
        inst->nodes[i].truck_only = strtol(strtok(NULL, delimiters), NULL, 10); // truck_only
        if (inst->param.verbose >= DEBUG)
        	printf("\tNODE %2d at coordinates (%11.6lf , %11.6lf), truck-only: %d\n", inst->nodes[i].id, inst->nodes[i].x, inst->nodes[i].y, inst->nodes[i].truck_only);
    }
    // define the final depot equal to the starting depot
    inst->nodes[inst->dimension - 1].id = inst->dimension - 1;
    inst->nodes[inst->dimension - 1].x = inst->nodes[0].x;
    inst->nodes[inst->dimension - 1].y = inst->nodes[0].y;
    inst->nodes[inst->dimension - 1].truck_only = inst->nodes[0].truck_only;

    if (inst->param.verbose >= DEBUG)
    	printf("\tENDING DEPOT %2d at coordinates (%11.6lf , %11.6lf), truck-only: %d\n", inst->nodes[inst->dimension - 1].id, inst->nodes[inst->dimension - 1].x, inst->nodes[inst->dimension - 1].y, inst->nodes[inst->dimension - 1].truck_only);


    fclose(fp);

    if (inst->dimension < 3)
    	print_error("EXIT: the instance does not have any customer.");
}

void compute_truck_travel_times_ha(instance *inst, double truck_speed) {
	printf("\n\nComputing truck travel times...\n");

	for (int i = 0; i < inst->dimension; i++)
	{
		for (int j = 0; j < inst->dimension; j++)
		{
			if (j == i)
				continue;
			inst->truck_times[i][j] = dist_MAN_2D(i, j, inst) / truck_speed;
		}
	}

	if (inst->param.verbose >= DEBUG)
	{
		for (int i = 0; i < inst->dimension; i++)
		{
			for (int j = 0; j < inst->dimension; j++)
			{
				printf("\ttruck_time(%3d,%3d) = %15.6lf \n", i, j, inst->truck_times[i][j]);
			}
		}
	}
}


void compute_drone_travel_times_ha(instance *inst, double drone_speed, double max_endurance) {
	printf("\n\nComputing drone travel times...\n");

	for (int i = 0; i < inst->dimension - 1; i++)
	{
		for (int j = 1; j < inst->dimension - 1; j++)
		{
			if (j == i)
				continue;
			if (inst->nodes[j].truck_only == 1)
				continue;
			for (int k = 1; k < inst->dimension; k++)
			{
				if (k == i || k == j)
					continue;

				inst->min_feas_time_drone[i][j][k] = ( dist_EUC_2D(i, j, inst) + dist_EUC_2D(j, k, inst)) / drone_speed;
                /*if (i > 0 && inst->min_time_drone[i][j][k] > max_endurance - 60.0) { // infeasible drone leg
                    inst->min_time_drone[i][j][k] = 0.0f;
                    inst->max_time_drone[i][j][k] = 0.0f;
                }
                else if (i == 0 && inst->min_time_drone[i][j][k] > max_endurance) {
                    inst->min_time_drone[i][j][k] = 0.0f;
                    inst->max_time_drone[i][j][k] = 0.0f;
                }
                else { // feasible drone leg
                    inst->max_time_drone[i][j][k] = max_endurance;
                    if (i > 0)
                        inst->max_time_drone[i][j][k] = max_endurance - 60.0;
                    if (k == inst->dimension - 1) // the drone does not have to wait the truck when returing to the depot
                        inst->max_time_drone[i][j][k] = 1e9;
                }*/
				if (inst->min_feas_time_drone[i][j][k] > max_endurance) {
					inst->min_feas_time_drone[i][j][k] = 0.0f;
					inst->max_feas_time_drone[i][j][k] = 0.0f;
				}
				else {
					inst->max_feas_time_drone[i][j][k] = max_endurance;
                    if (k == inst->dimension - 1) // the drone does not have to wait the truck when returing to the depot
                    	inst->max_feas_time_drone[i][j][k] = 1e9;
                }

            }
        }
    }

    if (inst->param.verbose >= DEBUG)
    {
    	for (int i = 0; i < inst->dimension; i++)
    	{
    		for (int j = 0; j < inst->dimension; j++)
    		{
    			if (j == i)
    				continue;
    			for (int k = 0; k < inst->dimension; k++)
    			{
    				printf("\tmin drone_leg_time(%3d,%3d,%3d) = %15.6lf\n", i, j, k, inst->min_feas_time_drone[i][j][k]);
    				printf("\tmax drone_leg_time(%3d,%3d,%3d) = %15.6lf\n", i, j, k, inst->max_feas_time_drone[i][j][k]);
    			}
    		}
    	}
    }
}

// compute for each node the K closest nodes wrt truck traveling time
void compute_neighborhoods(instance *inst) {
	printf("\nComputing for each node the K closest nodes...\n");
	if (inst->param.neighborhood_size > inst->dimension - 3) {
		inst->param.neighborhood_size = inst->dimension - 3;
		for (int i = 0; i < inst->dimension; i++)
			inst->nodes[i].neighbors = (int *)realloc(inst->nodes[i].neighbors, inst->param.neighborhood_size * sizeof(int));
	}

	for (int i = 1; i < inst->dimension - 1; i++)
		compute_k_closest_nodes(inst, i, inst->param.neighborhood_size);
	printf("Done.\n");
}

// compute the K closest nodes to node [node] wrt truck traveling time
void compute_k_closest_nodes(instance *inst, int node, int k) {
	if (k > inst->dimension - 3)
		print_error("[compute_k_closest_nodes]: k > inst->dimension-3 ");
    // initialize
	for (int i = 0; i < inst->param.neighborhood_size; i++) {
		inst->nodes[node].neighbors[i] = -1;
	}

	for (int i = 1; i < inst->dimension - 1; i++) {
		if (i == node)
			continue;
		int last_node = inst->nodes[node].neighbors[inst->param.neighborhood_size - 1];
		if (last_node == -1 || inst->truck_times[node][last_node] > inst->truck_times[node][i]) {
            // put node [i] in the list
			int k = 1;
			for (k; k < inst->param.neighborhood_size; k++) {
				int next_node = inst->nodes[node].neighbors[inst->param.neighborhood_size - 1 - k];
				if (next_node == -1)
					continue;
				if (inst->truck_times[node][next_node] < inst->truck_times[node][i]) {
					break;
				}
			}
			k--;
            // insert node [i] in position inst->param.neighborhood_size-1-k
			memmove(inst->nodes[node].neighbors + inst->param.neighborhood_size - k, inst->nodes[node].neighbors + inst->param.neighborhood_size - 1 - k, k * sizeof(int));
			inst->nodes[node].neighbors[inst->param.neighborhood_size - 1 - k] = i;
		}
	}
}



// int count_all_feasible_truck_legs(instance *inst) {
//     int count = 0;
//     int count_unique = 0;
//     for (int i = 0; i < inst->dimension - 1; i++) {
//         for (int j = 0; j < inst->dimension - 1; j++) {
//             if (j == i)
//                 continue;
//             if (inst->nodes[j].truck_only)
//                 continue;
//             for (int k = 1; k < inst->dimension; k++) {
//                 if (k == j || k == i)
//                     continue;
//                 if (i == 0 && k == inst->dimension - 1)
//                     continue;
//                 if (inst->max_feas_time_drone[i][j][k] <= 1e-4 || inst->truck_times[i][k] <= 1e-4)
//                     continue;
//                 int * R = malloc(sizeof(int) * inst->dimension - 3);
//                 int size_R = 0;
//                 for (int c = 1; c < inst->dimension - 1; c++) {
//                     if (c == i || c == j || c == k)
//                         continue;
//                     R[size_R++] = c;
//                 }
//                 int *unique_truck_legs = (int*)calloc(pow(2, inst->dimension), sizeof(int));
//                 int *positive_idx = (int*)calloc(pow(2, inst->dimension), sizeof(int));
//                 int *size_pos_idx;
//                 *size_pos_idx = 0;
//                 // printf("inst->truck_times[%d][%d]: %f\n", i, k, inst->truck_times[i][k]);
//                 int count_ijk = count_truck_legs(inst, R, size_R, i, inst->truck_times[i][k], i, j, k, unique_truck_legs, positive_idx, size_pos_idx);
//                 count += count_ijk;
//                 // printf("DRONE LEG [%d][%d][%d]: %d feasible truck legs found\n", i, j, k, count);
//                 // count feasible
//                 count_unique += (*size_pos_idx);
//                 free(unique_truck_legs);
//             }
//         }
//     }
//     printf("Feasible truck legs: %d\n", count);
//     printf("Unique truck legs: %d\n", count_unique);

//     return count;
// }


// int count_truck_legs(instance *inst, int *R, int size_R, int last_truck_leg_cust, double truck_leg_cost, int i, int j, int k, int *unique_truck_legs,
//                      int* positive_idx, int* size_pos_idx) {
//     // R: remeaning customers not yet explored
//     // printf("DRONE LEG: %d,%d,%d, max_time: %f\n", i, j, k, inst->max_feas_time_drone[i][j][k]);
//     // printf("R: ");
//     // for (int i = 0; i < size_R; i++) {
//     //     printf("%d, ", R[i]);
//     // }
//     // printf("truck leg time: %f", truck_leg_cost);
//     // printf("\n");


//     // printf("last_truck_leg_cust: %d\n", last_truck_leg_cust);

//     if (truck_leg_cost < 1e-4) {
//         print_error("ACHTUNG! truck leg cost < 1e-4\n");
//     }

//     if (truck_leg_cost > inst->max_feas_time_drone[i][j][k]) {
//         // printf("\t infeasible\n");
//         return 0;
//     }

//     int val = 0;
//     for ( int r = 0; r < size_R; r++) {
//         val += pow(2, R[r]);
//     }
//     unique_truck_legs[val]++;
//     if (unique_truck_legs[val] == 1) {
//         positive_idx[*size_pos_idx] = val;
//         (*size_pos_idx) ++;
//     }
//     // printf("\t feasible\n");

//     if (size_R == 0) {
//         return 1;
//     }


//     // char c;
//     // scanf("%c", &c);




//     // printf("Extend truck leg: %d, %d\n", last_truck_leg_cust, k);

//     if (size_R - 1 == 0) {
//         // printf("\t substitute %d -> %d with %d -> %d -> %d\n", last_truck_leg_cust, k, last_truck_leg_cust, R[0], k);
//         double new_truck_leg_cost = truck_leg_cost
//                                     - inst->truck_times[last_truck_leg_cust][k]
//                                     + inst->truck_times[last_truck_leg_cust][R[0]]
//                                     + TRUCK_DELIVERY_TIME
//                                     + inst->truck_times[R[0]][k];
//         return 1 + count_truck_legs(inst, NULL, 0, R[0], new_truck_leg_cost, i, j, k, unique_truck_legs, positive_idx, size_pos_idx);
//     }

//     int count = 1;

//     for (int r = 0; r < size_R; r++) {
//         // printf("\t substitute %d -> %d with %d -> %d -> %d\n", last_truck_leg_cust, k, last_truck_leg_cust, R[r], k);
//         double new_truck_leg_cost = truck_leg_cost
//                                     - inst->truck_times[last_truck_leg_cust][k]
//                                     + inst->truck_times[last_truck_leg_cust][R[r]]
//                                     + TRUCK_DELIVERY_TIME
//                                     + inst->truck_times[R[r]][k];

//         int * new_R = malloc(sizeof(int) * (size_R - 1));
//         memcpy(new_R, R, sizeof(int) * (r));
//         memcpy(new_R + r, R + r + 1, sizeof(int) * ((size_R - 1 ) - (r + 1) + 1));
//         count += count_truck_legs(inst, new_R, size_R - 1, R[r], new_truck_leg_cost, i, j, k, unique_truck_legs, positive_idx, size_pos_idx);
//     }
//     return count;
// }


// int count_all_feasible_truck_legs_no_recursion(instance *inst) {
//     printf("Computing all feasible truck legs...\n");
//     assert(inst->number_feasible_truck_legs == 0);
//     assert(inst->feasible_truck_legs == NULL);
//     assert(inst->feasible_truck_legs_times == NULL);
//     // assert(inst->feasible_truck_legs_ik_indices == NULL);

//     assert(inst->feasible_truck_legs_ik_begin == NULL);
//     assert(inst->feasible_truck_legs_ik_number == NULL);

//     // int*** feasible_truck_legs_ik_indices = (int***)malloc(sizeof(int**)*inst->dimension);
//     // inst->feasible_truck_legs_ik_indices = (int***)malloc(sizeof(int**)*inst->dimension);
//     inst->feasible_truck_legs_ik_begin = (int**)malloc(sizeof(int*)*inst->dimension);
//     inst->feasible_truck_legs_ik_number = (int**)malloc(sizeof(int*)*inst->dimension);
//     // int** count_ik = (int**)malloc(sizeof(int*)*inst->dimension);
//     for (int i = 0; i < inst->dimension; i++) {
//         // inst->feasible_truck_legs_ik_indices[i] = (int**)malloc(sizeof(int*)*inst->dimension);
//         inst->feasible_truck_legs_ik_begin[i] = (int*)malloc(sizeof(int) * inst->dimension);
//         inst->feasible_truck_legs_ik_number[i] = (int*)malloc(sizeof(int) * inst->dimension);
//         // count_ik[i] = (int*)calloc(inst->dimension, sizeof(int));
//         for (int k = 0; k < inst->dimension; k++) {
//             // inst->feasible_truck_legs_ik_indices[i][k] = NULL;
//             inst->feasible_truck_legs_ik_begin[i][k] = INT_MAX;
//             inst->feasible_truck_legs_ik_number[i][k] = 0;
//         }
//     }

//     for (int i = 0; i < inst->dimension - 1; i++) {
//         for (int k = 1; k < inst->dimension; k++) {
//             if (k == i)
//                 continue;

//             const int count_partial = count_truck_legs_no_recursion_save(inst, i, k, &(inst->feasible_truck_legs), &(inst->feasible_truck_legs_times), inst->number_feasible_truck_legs);
//             // count += count_partial;
//             // count_ik[i][k] = count_partial;
//             if (count_partial > 0) {
//                 printf("count_partial i = %d, k = %d = %d\n", i, k, count_partial);
//                 inst->feasible_truck_legs_ik_begin[i][k] = inst->number_feasible_truck_legs;
//                 inst->feasible_truck_legs_ik_number[i][k] = count_partial;
//                 inst->number_feasible_truck_legs += count_partial;
//             }


//             for (int c = 0; c < count_partial; c++) {
//                 printf(" (%d)", inst->feasible_truck_legs[inst->number_feasible_truck_legs - count_partial + c][0]);
//                 int v = 1;
//                 while (1) {
//                     printf(" %d", inst->feasible_truck_legs[inst->number_feasible_truck_legs - count_partial + c][v]);
//                     if (inst->feasible_truck_legs[inst->number_feasible_truck_legs - count_partial + c][v] == k)
//                         break;
//                     v++;
//                 }
//                 printf("\n");
//             }
//         }
//     }
//     printf("Feasible truck legs: %d\n", inst->number_feasible_truck_legs);
//     // remove_non_optimal_truck_legs(inst, inst->feasible_truck_legs, inst->feasible_truck_legs_times, inst->number_feasible_truck_legs);

//     return inst->number_feasible_truck_legs;
// }


// int count_truck_legs_no_recursion_save(instance *inst, const int i, const int k, int*** feasible_truck_legs, double** feasible_truck_legs_times, const int init_truck_legs_count) {
//     // char c;
//     // scanf("%c", &c);
//     if (i == k) return 0;
//     if (i == inst->dimension - 1 || k == 0) return 0;
//     const int flag_unique = 1;

//     int truck_legs_count_ik = 0;
//     int truck_legs_count_overall = init_truck_legs_count;



//     const int begin_truck_legs_ik = init_truck_legs_count;
//     // printf("\n****** i = %d, k = %d, begin_truck_legs_ik = %d\n", i, k, begin_truck_legs_ik);

//     for (int j = 1; j < inst->dimension - 1; j++) {

//         if (inst->nodes[j].truck_only) continue;
//         if (inst->max_feas_time_drone[i][j][k] == DBL_MAX) continue;

//         int *prev_length_idx = NULL;
//         int *curr_length_idx = NULL;


//         int count_truck_legs_with_length[inst->dimension];
//         for (int length = 0; length < inst->dimension; length++)
//             count_truck_legs_with_length[length] = 0;

//         int truck_leg_length = 2;

//         {
//             double truck_leg_time = inst->truck_times[i][k];
//             if (truck_leg_time == DBL_MAX)
//                 continue;
//             if (truck_leg_time > inst->max_feas_time_drone[i][j][k])
//                 continue;
//             // CHECK IF CURRENT TRUCK LEG IS NOT ALREADY IN feasible_truck_legs
//             int current_truck_leg[truck_leg_length + 1];
//             current_truck_leg[0] = truck_leg_length;
//             current_truck_leg[1] = i;
//             current_truck_leg[2] = k;

//             int found = 0;
//             int id_unique = -1;
//             if (flag_unique) {
//                 for (int old_id = begin_truck_legs_ik; old_id < truck_legs_count_overall; old_id++) {
//                     if ((*feasible_truck_legs)[old_id][0] != truck_leg_length)
//                         continue;
//                     int compare_value = memcmp(current_truck_leg, (*feasible_truck_legs)[old_id], (truck_leg_length + 1) * sizeof(int));
//                     if (compare_value == 0) {
//                         found = 1;
//                         id_unique = old_id;
//                         break;
//                     }
//                 }
//             }

//             int id_to_store = truck_legs_count_overall;

//             if (!found) {
//                 // insert truck leg and truck leg time
//                 *feasible_truck_legs = realloc(*feasible_truck_legs, (truck_legs_count_overall + 1) * sizeof(int*));
//                 (*feasible_truck_legs)[truck_legs_count_overall] = (int*)malloc((truck_leg_length + 1) * sizeof(int));
//                 memmove((*feasible_truck_legs)[truck_legs_count_overall], current_truck_leg, (truck_leg_length + 1)*sizeof(int));
//                 *feasible_truck_legs_times = realloc(*feasible_truck_legs_times, (truck_legs_count_overall + 1) * sizeof(double));
//                 (*feasible_truck_legs_times)[truck_legs_count_overall] = truck_leg_time;
//                 truck_legs_count_overall++;
//                 truck_legs_count_ik++;
//             }
//             else {
//                 if (flag_unique) {
//                     assert(id_unique != -1);
//                     id_to_store = id_unique;
//                 }
//             }

//             curr_length_idx = (int*)realloc(curr_length_idx, (count_truck_legs_with_length[truck_leg_length] + 1) * sizeof(int) );
//             curr_length_idx[count_truck_legs_with_length[truck_leg_length]] = id_to_store;

//             assert(curr_length_idx[count_truck_legs_with_length[truck_leg_length]] >= begin_truck_legs_ik);
//             // printf("*** id truck leg = %d\n", curr_length_idx[count_truck_legs_with_length[truck_leg_length]]);
//             // printf("j = %d : ", j);
//             // for (int v = 1; v < truck_leg_length + 1; v++) {
//             //     printf("%d ", (*feasible_truck_legs)[id_to_store][v]);
//             // }
//             // printf("\n");

//             count_truck_legs_with_length[truck_leg_length]++;

//         }


//         truck_leg_length++;
//         // int last_truck_leg_index = truck_legs_count_overall;


//         for (; truck_leg_length <= inst->dimension - 1; truck_leg_length++) {
//             free(prev_length_idx);
//             prev_length_idx = curr_length_idx;
//             curr_length_idx = NULL;

//             if (count_truck_legs_with_length[truck_leg_length - 1] == 0)
//                 break;

//             // EXTEND TRUCK LEGS with length truck_length-1
//             assert(count_truck_legs_with_length[truck_leg_length] == 0);
//             const int offset = truck_legs_count_overall - count_truck_legs_with_length[truck_leg_length - 1];

//             // for (int prev_leg_id = offset; prev_leg_id < last_truck_leg_index; prev_leg_id++) {
//             // printf("truck_leg_length = %d\n", truck_leg_length);
//             // printf("prev_length_idx: ");
//             // for (int id_leg = 0; id_leg < count_truck_legs_with_length[truck_leg_length - 1]; id_leg++) {
//             // printf(" %d ", prev_length_idx[id_leg]);
//             // }
//             // printf("\n");

//             for (int id_leg = 0; id_leg < count_truck_legs_with_length[truck_leg_length - 1]; id_leg++) {
//                 int prev_leg_id = prev_length_idx[id_leg];
//                 // if (prev_leg_id < begin_truck_legs_ik) {
//                 //     printf("prev_leg_id = %d, begin_truck_legs_ik = %d\n", prev_leg_id, begin_truck_legs_ik);
//                 //     exit(1);
//                 // }
//                 assert(prev_leg_id >= begin_truck_legs_ik);
//                 assert(prev_leg_id < truck_legs_count_overall);
//                 assert((*feasible_truck_legs)[prev_leg_id][0] == truck_leg_length - 1);
//                 assert((*feasible_truck_legs)[prev_leg_id][1] == i);
//                 assert((*feasible_truck_legs)[prev_leg_id][truck_leg_length - 1] == k);
//                 // if ((*feasible_truck_legs)[prev_leg_id][1] != i || (*feasible_truck_legs)[prev_leg_id][truck_leg_length - 1] != k) {
//                 //     printf("i = %d, k = %d : ", i, k);
//                 //     for (int v = 1; v < truck_leg_length; v++) {
//                 //         printf("%d ", (*feasible_truck_legs)[prev_leg_id][v]);
//                 //     }
//                 //     printf("\n");
//                 // }

//                 // access prev_leg and try to append a new customer
//                 int used_customers[inst->dimension];
//                 for (int cust = 0; cust < inst->dimension; cust++)
//                     used_customers[cust] = 0;
//                 for (int cust_id = 1; cust_id < truck_leg_length; cust_id++) {
//                     used_customers[ (*feasible_truck_legs)[prev_leg_id][cust_id] ] = 1;
//                 }
//                 used_customers[j] = 1;

//                 for (int cust = 1; cust < inst->dimension - 1; cust++) {
//                     if (used_customers[cust]) continue;
//                     int last_truck_customer = (*feasible_truck_legs)[prev_leg_id][truck_leg_length - 2];
//                     if (inst->truck_times[last_truck_customer][cust] == DBL_MAX || inst->truck_times[cust][k] == DBL_MAX)
//                         continue;
//                     double new_truck_leg_time = (*feasible_truck_legs_times)[prev_leg_id]
//                                                 - inst->truck_times[last_truck_customer][k]
//                                                 + inst->truck_times[last_truck_customer][cust]
//                                                 + inst->truck_times[cust][k];
//                     // double true_truck_leg_time = 0.0f;
//                     // for (int cust_id = 1; cust_id < (*feasible_truck_legs)[prev_leg_id][0]; cust_id++) {
//                     //     int current_customer = (*feasible_truck_legs)[prev_leg_id][cust_id];
//                     //     int next_customer = (*feasible_truck_legs)[prev_leg_id][cust_id + 1];
//                     //     true_truck_leg_time += inst->truck_times[current_customer][next_customer];
//                     // }
//                     // true_truck_leg_time -= inst->truck_times[last_truck_customer][k];
//                     // true_truck_leg_time += inst->truck_times[last_truck_customer][cust];
//                     // true_truck_leg_time += inst->truck_times[cust][k];

//                     // assert(fabs(true_truck_leg_time - new_truck_leg_time) < 1e-4);

//                     if (new_truck_leg_time > inst->max_feas_time_drone[i][j][k])
//                         continue;
//                     // CHECK IF CURRENT TRUCK LEG IS NOT ALREADY IN feasible_truck_legs
//                     int current_truck_leg[truck_leg_length + 1];
//                     current_truck_leg[0] = truck_leg_length;
//                     memcpy(current_truck_leg + 1, (*feasible_truck_legs)[prev_leg_id] + 1, (truck_leg_length - 2)*sizeof(int));
//                     current_truck_leg[truck_leg_length - 1] = cust;
//                     current_truck_leg[truck_leg_length] = k;


//                     int found = 0;
//                     int id_unique = -1;
//                     if (flag_unique) {
//                         for (int old_id = begin_truck_legs_ik; old_id < truck_legs_count_overall; old_id++) {
//                             if ((*feasible_truck_legs)[old_id][0] != truck_leg_length)
//                                 continue;
//                             int compare_value = memcmp(current_truck_leg, (*feasible_truck_legs)[old_id], (truck_leg_length + 1) * sizeof(int));
//                             if (compare_value == 0) {
//                                 found = 1;
//                                 id_unique = old_id;
//                                 assert(old_id >= begin_truck_legs_ik);
//                                 break;
//                             }
//                         }
//                     }
//                     /*
//                     if (!found) {
//                         // insert truck leg and truck leg time
//                         *feasible_truck_legs = realloc(*feasible_truck_legs, (truck_legs_count_overall + 1) * sizeof(int*));
//                         (*feasible_truck_legs)[truck_legs_count_overall] = (int*)malloc((truck_leg_length + 1) * sizeof(int));
//                         memmove((*feasible_truck_legs)[truck_legs_count_overall], current_truck_leg, (truck_leg_length + 1)*sizeof(int));
//                         *feasible_truck_legs_times = realloc(*feasible_truck_legs_times, (truck_legs_count_overall + 1) * sizeof(double));
//                         (*feasible_truck_legs_times)[truck_legs_count_overall] = new_truck_leg_time;
//                         count_truck_legs_with_length[truck_leg_length]++;
//                         truck_legs_count_overall++;
//                         truck_legs_count_ik++;
//                     }
//                     */
//                     int id_to_store = truck_legs_count_overall;

//                     if (!found) {
//                         // insert truck leg and truck leg time
//                         *feasible_truck_legs = realloc(*feasible_truck_legs, (truck_legs_count_overall + 1) * sizeof(int*));
//                         (*feasible_truck_legs)[truck_legs_count_overall] = (int*)malloc((truck_leg_length + 1) * sizeof(int));
//                         memmove((*feasible_truck_legs)[truck_legs_count_overall], current_truck_leg, (truck_leg_length + 1)*sizeof(int));
//                         *feasible_truck_legs_times = realloc(*feasible_truck_legs_times, (truck_legs_count_overall + 1) * sizeof(double));
//                         (*feasible_truck_legs_times)[truck_legs_count_overall] = new_truck_leg_time;
//                         truck_legs_count_overall++;
//                         truck_legs_count_ik++;
//                     }
//                     else {
//                         if (flag_unique) {
//                             id_to_store = id_unique;
//                         }
//                     }

//                     curr_length_idx = (int*)realloc(curr_length_idx, (count_truck_legs_with_length[truck_leg_length] + 1) * sizeof(int) );
//                     curr_length_idx[count_truck_legs_with_length[truck_leg_length]] = id_to_store;
//                     assert(curr_length_idx[count_truck_legs_with_length[truck_leg_length]] >= begin_truck_legs_ik);

//                     count_truck_legs_with_length[truck_leg_length]++;
//                 }

//             }
//             // last_truck_leg_index = truck_legs_count_overall;

//             // int n_free = inst->dimension - 3;
//             // if (i > 0) n_free--;
//             // if (k < inst->dimension - 1) n_free--;
//             // int check_count = n_free;
//             // for (int n = 1; n < truck_leg_length - 2; n++) {
//             //     check_count *= n_free - n;
//             // }
//             // printf("count_truck_legs_with_length[%d] = %d vs check_count = %d\n", truck_leg_length, count_truck_legs_with_length[truck_leg_length], check_count);
//             // // char c;
//             // // scanf("%c", &c);
//             // if (count_truck_legs_with_length[truck_leg_length] != check_count) {
//             //     printf("count_truck_legs_with_length[truck_leg_length] != check_count\n");
//             //     exit(1);
//             // }
//         }
//         free(prev_length_idx);
//         free(curr_length_idx);
//     }
//     return truck_legs_count_ik;
// }

// void remove_non_optimal_truck_legs(instance *inst, int** feasible_truck_legs, const double* feasible_truck_legs_times, const int truck_legs_count) {

//     inst->reduced_truck_legs = (int*)calloc(inst->number_feasible_truck_legs, sizeof(int));

//     int** copy_feasible_truck_legs = (int**)malloc(inst->number_feasible_truck_legs * sizeof(int*));
//     for (int l = 0; l < inst->number_feasible_truck_legs; l++) {
//         copy_feasible_truck_legs[l] = (int*)malloc((feasible_truck_legs[l][0] + 2) * sizeof(int));
//         copy_feasible_truck_legs[l][0] = l;
//         memcpy(copy_feasible_truck_legs[l] + 1, feasible_truck_legs[l], (feasible_truck_legs[l][0] + 1)*sizeof(int));
//     }

//     for (int l = 0; l < inst->number_feasible_truck_legs; l++) {
//         printf("TRUCK LEG %d: ", l);
//         for (int i = 0; i < 1 + feasible_truck_legs[l][0]; i++)
//             printf("%d ", feasible_truck_legs[l][i]);
//         // printf("\nTRUCK LEG %d: ", l);
//         // for (int i = 1; i < 1 + feasible_truck_legs[l][0]; i++)
//         //     printf("%d ", feasible_truck_legs[l][i]);
//         // printf("\n");
//         printf("\nTRUCK LEG COPY %d: ", l);
//         for (int i = 0; i < 2 + feasible_truck_legs[l][0]; i++)
//             printf("%d ", copy_feasible_truck_legs[l][i]);
//         printf("\n");
//         qsort(copy_feasible_truck_legs[l] + 3, copy_feasible_truck_legs[l][1] - 2, sizeof(int), compare_int_asc);
//         printf("TRUCK LEG COPY SORTED %d: ", l);
//         for (int i = 0; i < 2 + feasible_truck_legs[l][0]; i++)
//             printf("%d ", copy_feasible_truck_legs[l][i]);
//         printf("\n");
//     }
//     printf("*** TRUCK LEGS SORTED ****\n");
//     qsort(copy_feasible_truck_legs, inst->number_feasible_truck_legs, sizeof(int*), compare_feasible_truck_legs);
//     for (int l = 0; l < inst->number_feasible_truck_legs; l++) {
//         printf("TRUCK LEG %d: ", l);
//         printf("cost = %.2f, ", inst->feasible_truck_legs_times[copy_feasible_truck_legs[l][0]]);
//         for (int i = 0; i < 2 + copy_feasible_truck_legs[l][1]; i++) {
//             printf("%d ", copy_feasible_truck_legs[l][i]);
//         }
//         printf("\n");
//     }

//     // int* feasible_optimal_id_legs = NULL;
//     assert(inst->reduced_feasible_truck_legs == NULL);
//     int N = 0;
//     int best_id = copy_feasible_truck_legs[0][0];
//     int best_cost = inst->feasible_truck_legs_times[copy_feasible_truck_legs[0][0]];
//     for (int l = 1; l < inst->number_feasible_truck_legs; l++) {
//         if (copy_feasible_truck_legs[l - 1][1] != copy_feasible_truck_legs[l][1] || memcmp(copy_feasible_truck_legs[l - 1] + 2, copy_feasible_truck_legs[l] + 2, copy_feasible_truck_legs[l][1]*sizeof(int)) != 0) {
//             inst->reduced_feasible_truck_legs = (int*)realloc(inst->reduced_feasible_truck_legs, (N + 1) * sizeof(int));
//             if (inst->reduced_feasible_truck_legs == NULL) {
//                 print_error("Realloc inst->reduced_feasible_truck_legs failed\n");
//             }
//             inst->reduced_feasible_truck_legs[N++] = best_id;
//             inst->reduced_truck_legs[best_id] = 1;
//             best_id = copy_feasible_truck_legs[l][0];
//             best_cost = inst->feasible_truck_legs_times[copy_feasible_truck_legs[l][0]];
//         }
//         else {
//             double cost = inst->feasible_truck_legs_times[copy_feasible_truck_legs[l][0]];
//             if (cost < best_cost - 1e-6) {
//                 best_cost = cost;
//                 best_id = copy_feasible_truck_legs[l][0];
//             }
//         }
//     }
//     inst->reduced_feasible_truck_legs = (int*)realloc(inst->reduced_feasible_truck_legs, (N + 1) * sizeof(int));
//     if (inst->reduced_feasible_truck_legs == NULL) {
//         print_error("Realloc inst->reduced_feasible_truck_legs failed\n");
//     }
//     inst->reduced_feasible_truck_legs[N++] = best_id;
//     inst->reduced_truck_legs[best_id] = 1;

//     // qsort(inst->reduced_feasible_truck_legs, N, sizeof(int), compare_int_asc);

//     printf("\nReduced IDs (N = %d):\n", N);
//     for (int n = 0; n < N; n++) {
//         printf("%d\n", inst->reduced_feasible_truck_legs[n]);
//     }
//     inst->reduced_number_feasible_truck_legs = N;
//     print_error("ok");

// }



// int count_truck_legs_no_recursion(instance * inst, int i, int j, int k) {
//     // char c;
//     // scanf("%c", &c);
//     if (inst->nodes[j].truck_only) return 0;
//     if (i == j || i == k || j == k) return 0;
//     if (inst->max_feas_time_drone[i][j][k] < 0 ) return 0;

//     int count = 0; // TO COUNT THE NUMBER OF UNIQUE TRUCK LEGS, JUST SORT THE TRUCK LEGS FOUND AND REMOVE DUPLICATES!
//     int count_unique = 0;

//     int count_l[inst->dimension];

//     int *** truck_legs_customers = (int***)malloc(sizeof(int*)*inst->dimension);

//     double ** truck_leg_times = (double**)malloc(sizeof(int*)*inst->dimension);

//     // initialize truck_legs_customers for truck legs with only 1 customer
//     truck_legs_customers[1] = (int**)malloc(sizeof(int*) * (inst->dimension - 2));
//     truck_leg_times[1] = (double*)malloc(sizeof(double) * (inst->dimension - 2));

//     // printf("l = 1 (%d,%d,%d):\n", i, j, k);
//     count_l[1] = 0;
//     for (int c = 1; c < inst->dimension - 1; c++) {
//         if (c == i || c == j || c == k)
//             continue;
//         double truck_leg_time = inst->truck_times[i][c] + inst->truck_times[c][k];
//         if (truck_leg_time > inst->max_feas_time_drone[i][j][k])
//             continue;
//         truck_legs_customers[1][count_l[1]] = (int *) malloc(sizeof(int));
//         truck_legs_customers[1][count_l[1]][0] = c;
//         truck_leg_times[1][count_l[1]] = truck_leg_time;
//         count_l[1]++;
//         // printf("\t{%d}\n", truck_legs_customers[1][count_l[1] - 1][0]);
//     }
//     count += count_l[1];


//     // for (int prev_leg_id = 0; prev_leg_id < count_l[1]; prev_leg_id++) {
//     //     printf("\t{");
//     //     printf("%d,", truck_legs_customers[1][prev_leg_id][0]);
//     // }


//     for (int l = 2; l < inst->dimension - 1; l++) {
//         if (count_l[l - 1] == 0)
//             break;
//         // printf("\nl = %d (%d,%d,%d):\n", l, i, j, k);

//         truck_legs_customers[l] = (int**)malloc(sizeof(int*) *  count_l[l - 1] * (inst->dimension - 2 - (l - 1)) );
//         truck_leg_times[l] = (double*)malloc(sizeof(double) * count_l[l - 1] * (inst->dimension - 2 - (l - 1)) );

//         count_l[l] = 0;
//         for (int prev_leg_id = 0; prev_leg_id < count_l[l - 1]; prev_leg_id++) {
//             // printf("\n\t{");
//             // for (int w = 0; w < l - 1; w++) {
//             //     printf("%d,", truck_legs_customers[l - 1][prev_leg_id][w]);
//             // }
//             // printf("} -> ");
//             // access prev_leg and try to append a new customer
//             int used_customers[inst->dimension];
//             for (int w = 0; w < inst->dimension; w++) used_customers[w] = 0;
//             used_customers[i] = 1;
//             used_customers[j] = 1;
//             used_customers[k] = 1;
//             for (int r = 0; r < l - 1; r++) {
//                 used_customers[ truck_legs_customers[l - 1][prev_leg_id][r] ] = 1;
//             }
//             // printf("Used customers: ");
//             // for (int w = 0; w < inst->dimension; w++) {
//             //     if (used_customers[ w ])
//             //         printf("*%d,", w);
//             // }
//             // printf("\n");

//             for (int c = 1; c < inst->dimension - 1; c++) {
//                 if (used_customers[c]) continue;
//                 double new_truck_leg_time = truck_leg_times[l - 1][prev_leg_id]
//                                             - inst->truck_times[truck_legs_customers[l - 1][prev_leg_id][l - 2]][k]
//                                             + inst->truck_times[truck_legs_customers[l - 1][prev_leg_id][l - 2]][c]
//                                             + inst->truck_times[c][k];
//                 if (new_truck_leg_time > inst->max_feas_time_drone[i][j][k])
//                     continue;
//                 truck_legs_customers[l][count_l[l]] = (int*)malloc(sizeof(int) * l);
//                 memcpy(truck_legs_customers[l][count_l[l]], truck_legs_customers[l - 1][prev_leg_id], sizeof(int) * (l - 1));
//                 truck_legs_customers[l][count_l[l]][l - 1] = c;
//                 truck_leg_times[l][count_l[l]] = new_truck_leg_time;
//                 // printf("\n\t\t{");
//                 // for (int w = 0; w < l; w++) {
//                 //     printf("%d,", truck_legs_customers[l][count_l[l]][w]);
//                 // }
//                 // printf("}");

//                 count_l[l]++;
//             }

//         }
//         count += count_l[l];

//         // for (int prev_leg_id = 0; prev_leg_id < count_l[l]; prev_leg_id++) {
//         //     printf("truck_leg[%d][%d] {", l, prev_leg_id);
//         //     for (int w = 0; w < l; w++)
//         //         printf("%d,", truck_legs_customers[l][prev_leg_id][w]);
//         //     printf("}\n");
//         // }
//         // char c;
//         // scanf("%c",&c);
//     }

//     // for (int l = 1; l < inst->dimension - 1; l++) {
//     //     if (count_l[l] == 0 ) break;
//     //     qsort(truck_legs_customers[l], count_l[l], sizeof(int*), compare_truck_legs);
//     //     int count_wo_duplicated = 1;
//     //     for (int w = 1; w < count_l[l]; w++){
//     //         if (memcmp(truck_legs_customers[l][w], truck_legs_customers[l][w-1], l) != 0)
//     //             count_wo_duplicated++;
//     //     }
//     //     count_unique += count_wo_duplicated;
//     // }
//     // for (int l = 2; l < inst->dimension - 1; l++) {
//     //     if (count_l[l] == 0 ) break;
//     //     for (int leg_id = 0; leg_id < count_l[l]; leg_id++) {
//     //         qsort(truck_legs_customers[l][leg_id], l, sizeof(int), compare_int);
//     //     }
//     // }

//     return count;
//     // return count_unique;
// }



// int compute_truck_legs(instance* inst) {
//     assert(inst->feasible_truck_legs_ik == NULL);

//     inst->number_feasible_truck_legs_ik = (int**)malloc(sizeof(int) * inst->dimension);
//     inst->feasible_truck_legs_ik = (int****)malloc(sizeof(int***) * inst->dimension);

//     for (int i = 0; i < inst->dimension; i++) {
//         inst->number_feasible_truck_legs_ik[i] = (int*)malloc(sizeof(int) * inst->dimension);
//         inst->feasible_truck_legs_ik[i] = (int***)malloc(sizeof(int**) * inst->dimension);
//         for (int k = 0; k < inst->dimension; k++) {
//             inst->number_feasible_truck_legs_ik[i][k] = 0;
//             inst->feasible_truck_legs_ik[i][k] = NULL;
//         }
//     }

//     int count = 0;
//     for (int i = 0; i < inst->dimension - 1; i++) {
//         printf("*** i = %d\n", i);
//         // truck leg of length 2
//         int** truck_legs_prev_l_idx = NULL;
//         int count_prev_l = 0;
//         for (int w = 1; w < inst->dimension; w++) {
//             printf("*** w = %d\n", w);
//             if (inst->truck_times[i][w] == DBL_MAX || inst->truck_times[i][w] > POIKONEN_UAV_ENDURANCE)
//                 continue;
//             // check if it exists a feasible drone leg for this feasible truck leg
//             int flag_j_found = 0;
//             for (int j = 1; j < inst->dimension - 1; j++) {
//                 if (inst->nodes[j].truck_only || j == i || j == w)
//                     continue;
//                 if (inst->max_feas_time_drone[i][j][w] == DBL_MAX) continue;
//                 if (inst->max_feas_time_drone[i][j][w] > inst->truck_times[i][w] - 1e-6) {
//                     flag_j_found = 1;
//                     break;
//                 }
//             }
//             if (!flag_j_found)
//                 continue;

//             int* truck_leg = (int*)malloc(sizeof(int) * 4);
//             truck_leg[0] = 2;
//             truck_leg[1] = inst->truck_times[i][w];
//             truck_leg[2] = i;
//             truck_leg[3] = w;
//             printf("here ok\n");
//             if (inst->number_feasible_truck_legs_ik[i][w] == 0)
//                 assert(inst->feasible_truck_legs_ik[i][w] == NULL);
//             inst->feasible_truck_legs_ik[i][w] = (int**)realloc(inst->feasible_truck_legs_ik[i][w], sizeof(int*) * (inst->number_feasible_truck_legs_ik[i][w] + 1));
//             assert(inst->feasible_truck_legs_ik[i][w] != NULL);
//             inst->feasible_truck_legs_ik[i][w][inst->number_feasible_truck_legs_ik[i][w]] = truck_leg;
//             printf("here ok2\n");

//             truck_legs_prev_l_idx = (int**)realloc(truck_legs_prev_l_idx, sizeof(int*) * (count_prev_l + 1));
//             truck_legs_prev_l_idx[count_prev_l] = truck_leg;

//             inst->number_feasible_truck_legs_ik[i][w]++;
//             count_prev_l++;

//             printf("New truck leg found: [%d]{%d} %d,%d\n",
//                    inst->feasible_truck_legs_ik[i][w][inst->number_feasible_truck_legs_ik[i][w] - 1][0],
//                    inst->feasible_truck_legs_ik[i][w][inst->number_feasible_truck_legs_ik[i][w] - 1][1],
//                    inst->feasible_truck_legs_ik[i][w][inst->number_feasible_truck_legs_ik[i][w] - 1][2],
//                    inst->feasible_truck_legs_ik[i][w][inst->number_feasible_truck_legs_ik[i][w] - 1][3]);
//             count++;
//             // char c;
//             // scanf("%c", &c);
//         }
//         continue;
//         // truck leg of length > 2
//         for (int L = 3; L < inst->dimension - 1; L++) {
//             if (count_prev_l == 0) break;
//             int count_l = 0;
//             int** truck_legs_l_idx = NULL;
//             for (int prev_id = 0; prev_id < count_prev_l; prev_id++) {
//                 int *prev_truck_leg = truck_legs_prev_l_idx[prev_id];
//                 int* used_customers = (int*)calloc(inst->dimension, sizeof(int));
//                 used_customers[0] = 1;
//                 for (int j = 0; j < L - 1; j++) {
//                     used_customers[prev_truck_leg[2 + j]] = 1;
//                 }
//                 // try to extend prev_truck_leg
//                 for (int w = 1; w < inst->dimension; w++) {
//                     assert(prev_truck_leg[0] == L - 1);
//                     if (used_customers[w]) continue;
//                     if (prev_truck_leg[L] == inst->dimension - 1) continue;
//                     int last_node = prev_truck_leg[L];
//                     if (inst->truck_times[last_node][w] == DBL_MAX) continue;
//                     double old_cost = prev_truck_leg[1];
//                     double new_time = prev_truck_leg[1] + inst->truck_times[last_node][w];
//                     if (new_time  > POIKONEN_UAV_ENDURANCE) continue;
//                     // check if it exists a feasible drone leg for this feasible truck leg
//                     int flag_j_found = 0;
//                     for (int j = 1; j < inst->dimension - 1; j++) {
//                         if (inst->nodes[j].truck_only || used_customers[j])
//                             continue;
//                         if (inst->max_feas_time_drone[i][j][w] == DBL_MAX) continue;
//                         if (inst->max_feas_time_drone[i][j][w] > new_time - 1e-6) {
//                             flag_j_found = 1;
//                             break;
//                         }
//                     }
//                     if (!flag_j_found)
//                         continue;

//                     int *new_truck_leg = (int*)malloc((L + 2) * sizeof(int));
//                     memcpy(new_truck_leg + 2, prev_truck_leg + 2, (L - 1)*sizeof(int));
//                     new_truck_leg[0] = L;
//                     new_truck_leg[1] = new_time;
//                     new_truck_leg[L + 1] = w;
//                     if (inst->number_feasible_truck_legs_ik[i][w] == 0)
//                         assert(inst->feasible_truck_legs_ik[i][w] == NULL);
//                     inst->feasible_truck_legs_ik[i][w] = (int**)realloc(inst->feasible_truck_legs_ik[i][w], sizeof(int*) * (inst->number_feasible_truck_legs_ik[i][w] + 1));
//                     assert(inst->feasible_truck_legs_ik[i][w] != NULL);

//                     inst->feasible_truck_legs_ik[i][w][inst->number_feasible_truck_legs_ik[i][w]] = new_truck_leg;

//                     truck_legs_l_idx = (int**)realloc(truck_legs_l_idx, sizeof(int*) * (count_l + 1));
//                     truck_legs_l_idx[count_l] = new_truck_leg;

//                     inst->number_feasible_truck_legs_ik[i][w]++;
//                     count_l++;
//                     printf("New truck leg found: [%d]{%d} ", new_truck_leg[0], new_truck_leg[1]);
//                     for (int l = 0; l < L; l++) {
//                         printf("%d,", new_truck_leg[2 + l]);
//                     }
//                     printf("\n");
//                     count++;
//                     // char c;
//                     // scanf("%c", &c);

//                 }
//             }
//             free(truck_legs_prev_l_idx);
//             truck_legs_prev_l_idx = truck_legs_l_idx;
//             count_prev_l = count_l;
//         }
//     }
//     printf("*** truck legs found = %d\n", count);
// }



int compute_truck_legs2(instance* inst) {
	assert(inst->feasible_truck_legs2 == NULL);

	inst->feasible_truck_legs2 = (feasibleTruckLegs_ik**)malloc(sizeof(feasibleTruckLegs_ik*) * inst->dimension);

	for (int i = 0; i < inst->dimension; i++) {
		inst->feasible_truck_legs2[i] = (feasibleTruckLegs_ik*)malloc(sizeof(feasibleTruckLegs_ik) * inst->dimension);
		for (int k = 0; k < inst->dimension; k++) {
			inst->feasible_truck_legs2[i][k].count = 0;
			inst->feasible_truck_legs2[i][k].legs = NULL;
		}
	}

	int tot_count = 0;
	for (int i = 0; i < inst->dimension - 1; i++) {
        // truck leg of length 2
		feasibleTruckLeg* truck_legs_prev_l_idx = NULL;
		int count_prev_l = 0;
		for (int w = 1; w < inst->dimension; w++) {
			if (inst->truck_times[i][w] == DBL_MAX)
				continue;
			if (inst->truck_times[i][w] > POIKONEN_UAV_ENDURANCE + 1e-6)
				continue;


			int* truck_leg = (int*)malloc(sizeof(int) * 2);
			truck_leg[0] = i;
			truck_leg[1] = w;
			if (inst->feasible_truck_legs2[i][w].count == 0)
				assert(inst->feasible_truck_legs2[i][w].legs == NULL);

			inst->feasible_truck_legs2[i][w].legs = (feasibleTruckLeg*)realloc(inst->feasible_truck_legs2[i][w].legs, sizeof(feasibleTruckLeg) * (inst->feasible_truck_legs2[i][w].count  + 1));
			assert(inst->feasible_truck_legs2[i][w].legs != NULL);
			inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].leg = truck_leg;
			inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].time = inst->truck_times[i][w];
			inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].length = 2;
			inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].flag_active = 0;

            // check if it exists a feasible drone leg for this feasible truck leg
			for (int j = 1; j < inst->dimension - 1; j++) {
				if (inst->nodes[j].truck_only || j == i || j == w)
					continue;
				if (inst->max_feas_time_drone[i][j][w] == DBL_MAX) continue;
				if (inst->max_feas_time_drone[i][j][w] > inst->truck_times[i][w] - 1e-6) {
					inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].flag_feasible_drone_leg_exist = 1;
					break;
				}
			}

			truck_legs_prev_l_idx = (feasibleTruckLeg*)realloc(truck_legs_prev_l_idx, sizeof(feasibleTruckLeg) * (count_prev_l + 1));
			truck_legs_prev_l_idx[count_prev_l].length = 2;
			truck_legs_prev_l_idx[count_prev_l].time = inst->truck_times[i][w];
			truck_legs_prev_l_idx[count_prev_l].leg = truck_leg;
			truck_legs_prev_l_idx[count_prev_l].flag_feasible_drone_leg_exist = 0;

			inst->feasible_truck_legs2[i][w].count++;
			count_prev_l++;
			tot_count++;
		}
        // truck leg of length > 2
		for (int L = 3; L < inst->dimension - 1; L++) {
			if (count_prev_l == 0) break;
			int count_l = 0;
			feasibleTruckLeg* truck_legs_l_idx = NULL;
			for (int prev_id = 0; prev_id < count_prev_l; prev_id++) {
				feasibleTruckLeg prev_truck_leg = truck_legs_prev_l_idx[prev_id];

				int* used_customers = (int*)calloc(inst->dimension, sizeof(int));
				used_customers[0] = 1;
				for (int j = 0; j < L - 1; j++) {
					used_customers[prev_truck_leg.leg[j]] = 1;
				}
                // try to extend prev_truck_leg
				for (int w = 1; w < inst->dimension; w++) {
					assert(prev_truck_leg.length == L - 1);
					assert(prev_truck_leg.leg[0] == i);
					if (used_customers[w]) continue;
					int last_node = prev_truck_leg.leg[L - 2];
					if (last_node == inst->dimension - 1) continue;
					if (inst->truck_times[last_node][w] == DBL_MAX) continue;
					double new_time = prev_truck_leg.time + inst->truck_times[last_node][w];
					if (new_time > POIKONEN_UAV_ENDURANCE + 1e-6) continue;


					int *new_truck_leg = (int*)malloc(L * sizeof(int));
					memcpy(new_truck_leg, prev_truck_leg.leg, (L - 1)*sizeof(int));
					new_truck_leg[L - 1] = w;

					inst->feasible_truck_legs2[i][w].legs = (feasibleTruckLeg*)realloc(inst->feasible_truck_legs2[i][w].legs, sizeof(feasibleTruckLeg) * (inst->feasible_truck_legs2[i][w].count  + 1));
					assert(inst->feasible_truck_legs2[i][w].legs != NULL);
					inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].time = new_time;
					inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].length = L;
					inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].leg = new_truck_leg;
					inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].flag_feasible_drone_leg_exist = 0;
					inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].flag_active = 0;

                    // check if it exists a feasible drone leg for this feasible truck leg
					int flag_j_found = 0;
					for (int j = 1; j < inst->dimension - 1; j++) {
						if (inst->nodes[j].truck_only || used_customers[j])
							continue;
						if (inst->max_feas_time_drone[i][j][w] == DBL_MAX) continue;
						if (inst->max_feas_time_drone[i][j][w] > new_time - 1e-6) {
							inst->feasible_truck_legs2[i][w].legs[inst->feasible_truck_legs2[i][w].count].flag_feasible_drone_leg_exist = 1;
							break;
						}
					}

					inst->feasible_truck_legs2[i][w].count++;

					truck_legs_l_idx = (feasibleTruckLeg*)realloc(truck_legs_l_idx, sizeof(feasibleTruckLeg) * (count_l + 1));
					truck_legs_l_idx[count_l].length = L;
					truck_legs_l_idx[count_l].time = new_time;
					truck_legs_l_idx[count_l].leg = new_truck_leg;

					count_l++;
					tot_count++;
				}
			}
			free(truck_legs_prev_l_idx);
			truck_legs_prev_l_idx = truck_legs_l_idx;
			count_prev_l = count_l;
		}
	}



	inst->number_feasible_truck_legs = tot_count;

    // set IDs
	int id = 0;
	for (int i = 0; i < inst->dimension - 1; i++) {
		for (int k = 1; k < inst->dimension; k++) {
			for (int l = 0; l < inst->feasible_truck_legs2[i][k].count; l++) {
				inst->feasible_truck_legs2[i][k].legs[l].ID = id++;
                // printf("[%d][%d] %d id = %d\n", i, k, l, inst->feasible_truck_legs2[i][k].legs[l].ID);
			}
		}
	}
	return tot_count;
}



int compare_int_asc(const void* a, const void* b)
{
	return (*(int*)a) - (*(int*)b);
}

int compare_int(const void* a, const void* b)
{
	return (*(int*)a) < (*(int*)b);
}

int compare_truck_legs(const void* a, const void* b)
{
    // printf("[%d,%d]\n", (*(int**)a)[0], (*(int**)b)[0]);
	return memcmp ( *(int**)a, *(int**)b, sizeof(*(int**)a));
}

int compare_feasible_truck_legs(const void* a, const void* b)
{
	const int* truck_leg_a = *(int**)a;
	const int* truck_leg_b = *(int**)b;
	if (truck_leg_a[1] != truck_leg_b[1]) {
		return truck_leg_a[1] - truck_leg_b[1];
	}
	return memcmp(truck_leg_a + 2, truck_leg_b + 2, truck_leg_a[1] * sizeof(int));
}

