#include "../include/min_max_opt.h"


// TAKEOFF/LANDING (vertical) Power consumption w.r.t. speed v [m/s] and paylaod weight w [kg]
double vertical_power(double speed, double weight)
{
	return K1 * (W + weight) * G * (speed / 2 + sqrt(pow(speed, 2) / 4 + (W + weight) * G / pow(K2, 2))) + C2 * pow((W + weight) * G, (3.0f / 2.0f));
}

// Energy to move the drone vertically for [delta_altitude] meters at a certain speed and with a certain parcel weight
double vertical_energy(double speed, double delta_altitude, double weight)
{
	return vertical_power(speed, weight) * delta_altitude / speed;
}

// CRUISE power consumption w.r.t. speed v [m/s] and paylaod weight w [kg]
double cruise_power(double speed, double weight)
{
	return c12 * (pow(pow((W + weight) * G, 2) - (2 * (W + weight) * G * q5) * pow(speed, 2) + c45 * pow(speed, 4), (3.0 / 4.0))) + C4 * pow(speed, 3);
}

// Energy consumption to travel d meters at the speed v[m / s] with a payload of weight w[kg] double cruise_energy(double speed, double weight, double dist)
double cruise_energy(double speed, double weight, double dist)
{
	return (dist / speed) * cruise_power(speed, weight);
}

// Energy consumption to travel d meters in t seconds with a payload weight
double cruise_energy_time(double dist, double time, double weight)
{
	return time * cruise_power(dist / time, weight);
}

// *** MIN TIME *** //

double obj_min_drone_leg_time(unsigned n, const double *x, double *grad, void *data)
{
	my_params *d = (my_params *)data;
	double B = d->B;
	double d1 = d->d1, d2 = d->d2;
	double w = d->w;
	if (grad)
	{
		grad[0] = -d1 * (1 / pow(x[0], 2));
		grad[1] = -d2 * (1 / pow(x[1], 2));
	}
	double obj = d1 * (1 / x[0]) + d2 * (1 / x[1]);
    //printf("f(%.10g,%.10g) = %0.10g\n", x[0], x[1], obj);
    //printf("E: %f\n", cruise_energy(x[0], w, d1) + cruise_energy(x[1], 0.0, d2));

	if (cruise_energy(x[0], w, d1) + cruise_energy(x[1], 0.0, d2) <= B)
	{
		if (obj < * (d->best_obj_feas))
		{
            // new best feasible solution found
			*(d->best_obj_feas) = obj;
			*(d->best_s1_feas) = x[0];
			*(d->best_s2_feas) = x[1];
		}
	}
	return obj;
}

double constraint_min_drone_leg_time(unsigned n, const double *x, double *grad, void *data)
{
	my_params *d = (my_params *)data;
	double B = d->B;
	double d1 = d->d1, d2 = d->d2;
	double w = d->w;
	return (cruise_energy(x[0], w, d1) + cruise_energy(x[1], 0.0, d2) - B);
}

int compute_min_time(double d1, double d2, double w, double B, double init_s1, double init_s2, double uav_min_speed, double uav_max_speed,
	double *min_t, double *min_feas_t, double *s1, double *s2, double *s1_feas, double *s2_feas)
{
    // data
    // feasible sol data
	double best_obj_feas = DBL_MAX;
	double best_s1_feas = -1;
	double best_s2_feas = -1;

	if (uav_min_speed < 0) {
		uav_min_speed = UAV_MIN_SPEED;
	}
	if (uav_max_speed < 0) {
		uav_max_speed = UAV_MAX_SPEED;
	}
	my_params data = {B, 0.0, d1, d2, w, &best_obj_feas, &best_s1_feas, &best_s2_feas};

	nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA, 2); /* algorithm and dimensionality */

    double lb[2] = {uav_min_speed, uav_min_speed}; /* lower bounds */
    double ub[2] = {uav_max_speed, uav_max_speed}; /* upper bounds */
	nlopt_set_lower_bounds(opt, lb);
	nlopt_set_upper_bounds(opt, ub);
    nlopt_set_maxeval(opt, 500); // max #iterations

    nlopt_set_min_objective(opt, obj_min_drone_leg_time, &data);

    nlopt_add_inequality_constraint(opt, constraint_min_drone_leg_time, &data, NLOPT_TOLERANCE);

    //nlopt_set_xtol_rel(opt, 1e-8);
    nlopt_set_ftol_abs(opt, NLOPT_TOLERANCE);

    // initial guess
    double x[2] = {init_s1, init_s2};
    double minf;
    int res_code = nlopt_optimize(opt, x, &minf);
    nlopt_destroy(opt);
    printf("**res_code [compute_min_time]: %d\n", res_code);
    // if (res_code < 0)
    // {
    // 	return res_code;
    // }

    // if(cruise_energy(x[0], w, d1) + cruise_energy(x[1], 0.0, d2) > B){
    //     // infeasible solution found
    // }

    // double e0 = cruise_energy(x[0], w, d1) + cruise_energy(x[1], 0.0, d2);
    // printf("\tfound minimum at f(%.10g,%.10g) = %0.10g | E = %f\n", x[0], x[1], minf, e0);

    // //if (e0 <= B)
    // //     printf("\t* feasible...\n");

    // double e = cruise_energy(*(data.best_s1_feas), w, d1) + cruise_energy(*(data.best_s2_feas), 0.0, d2);
    // printf("\tminimum feasible at f(%.10g,%.10g) = %0.10g | E = %f\n", *(data.best_s1_feas), *(data.best_s2_feas), *(data.best_obj_feas), e);

    // if (e > B)
    //     return -1;

    /*
    if (cruise_energy(x[0], w, d1) + cruise_energy(x[1], 0.0, d2) > B)
    {
        if (s1 == -1 || minf < data.best_obj)
        {
            data.best_obj = minf;
            data.best_s1 = x[0];
            data.best_s2 = x[1];
        }
    }
    */
    //printf("feasible min time found!\n");
    *min_feas_t = best_obj_feas;
    *s1_feas = best_s1_feas;
    *s2_feas = best_s2_feas;
    return res_code;
}

// *** //

// *** MAX TIME *** //
//int count = 0;

double obj_max_drone_leg_time(unsigned n, const double *x, double *grad, void *data)
{
	my_params *d = (my_params *)data;
	double B = d->B;
	double d1 = d->d1, d2 = d->d2;
	double w = d->w;

	double time = x[0] + x[1] + (B - cruise_energy_time(d1, x[0], w) - cruise_energy_time(d2, x[1], 0.0)) / hovering_power;

	if (cruise_energy_time(d1, x[0], w) + cruise_energy_time(d2, x[1], 0.0) <= B)
	{
		if (time > *(d->best_obj_feas))
		{
            // new best feasible solution found
			*(d->best_obj_feas) = time;
			*(d->best_s1_feas) = d1 / x[0];
			*(d->best_s2_feas) = d2 / x[1];
		}
	}

	double obj = -x[0] - x[1] - x[2];
    //printf("{%d} obj: %f\n E: %f\n", count, -obj, cruise_energy_time(d1, x[0], w) + cruise_energy_time(d2, x[1], 0.0) + (B - cruise_energy_time(d1, x[0], w) - cruise_energy_time(d2, x[1], 0.0)) / hovering_power);
	return obj;
}

double constraint1_max_drone_leg_time(unsigned n, const double *x, double *grad, void *data)
{
	my_params *d = (my_params *)data;
	double B = d->B;
	double d1 = d->d1, d2 = d->d2;
	double w = d->w;
	return (cruise_energy_time(d1, x[0], w) + cruise_energy_time(d2, x[1], 0.0) - B);
}

double constraint2_max_drone_leg_time(unsigned n, const double *x, double *grad, void *data)
{
	my_params *d = (my_params *)data;
	double B = d->B;
	double d1 = d->d1, d2 = d->d2;
	double w = d->w;
	return (x[2] * hovering_power - (B - cruise_energy_time(d1, x[0], w) - cruise_energy_time(d2, x[1], 0.0)));
}

int compute_max_time(double d1, double d2, double w, double B, double init_s1, double init_s2,
	double uav_min_speed, double uav_max_speed,
	double *max_t, double *max_feas_t, double *s1, double *s2, double *s1_feas, double *s2_feas)
{

    // data
    // feasible sol data
	double best_obj_feas = 0.0;
	double best_s1_feas = -1;
	double best_s2_feas = -1;

	if (uav_min_speed < 0) {
		uav_min_speed = UAV_MIN_SPEED;
	}
	if (uav_max_speed < 0) {
		uav_max_speed = UAV_MAX_SPEED;
	}

	my_params data = {B, 0.0, d1, d2, w, &best_obj_feas, &best_s1_feas, &best_s2_feas};

	nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA, 3); /* algorithm and dimensionality */

    double lb[3] = {d1 / uav_max_speed, d2 / uav_max_speed, 0.0};                /* lower bounds */
    double ub[3] = {d1 / uav_min_speed, d2 / uav_min_speed, B / hovering_power}; /* upper bounds */
	nlopt_set_lower_bounds(opt, lb);
	nlopt_set_upper_bounds(opt, ub);
    nlopt_set_maxeval(opt, 800); // max #iterations

    nlopt_set_min_objective(opt, obj_max_drone_leg_time, &data);
    nlopt_add_inequality_constraint(opt, constraint1_max_drone_leg_time, &data, NLOPT_TOLERANCE);
    nlopt_add_inequality_constraint(opt, constraint2_max_drone_leg_time, &data, NLOPT_TOLERANCE);

    nlopt_set_ftol_abs(opt, NLOPT_TOLERANCE);

    // initial guess
    double x[3] = {d1 / init_s1, d2 / init_s2, (B - cruise_energy(init_s1, w, d1) - cruise_energy(init_s2, 0.0, d2)) / hovering_power};
    //double x[3] = {d1 / init_s1, d2 / init_s2, 0.0};
    double e_init = cruise_energy_time(d1, x[0], w) + cruise_energy_time(d2, x[1], 0.0) + x[2] * hovering_power;
    //B += 1e-4;
    //printf("B = %.24f\n", B);
    //printf("e_init = %.24f\n", e_init);
    double minf;
    int res_code = nlopt_optimize(opt, x, &minf);
    //printf("\t res_code: %d\n", res_code);
    nlopt_destroy(opt);

    // if (res_code < 0)
    // {
    	printf("**res_code [compute_max_time] = %d\n", res_code);
    	// return res_code;
    // }

    // double e0 = cruise_energy_time(d1, x[0], w) + cruise_energy_time(d2, x[1], 0.0) + x[2] * hovering_power;
    // if (e0 <= B)
    //     printf("\t* feasible...\n");
    // printf("\tfound maximum at f(%.15g,%.15g) = %0.10g (hovering: %f) | %f\n", d1 / x[0], d2 / x[1], -minf, x[2], e0);
    // double e = cruise_energy(best_s1_feas, w, d1) + cruise_energy(best_s2_feas, 0.0, d2);
    // printf("\tmaximum feasible at f(%.15g,%.15g) = %0.10g (hovering: %f) | %f\n", best_s1_feas, best_s2_feas, best_obj_feas, (B - e) / hovering_power, e);
    // printf("\tcheck time: %f\n", d1 / best_s1_feas + d2 / best_s2_feas + (B - e) / hovering_power);


    //printf("found minimum after %d evaluations\n", count);
    // if (*(data.best_s1_feas) == -1)
    //     return -1;
    //double e = cruise_energy(*(data.best_s1_feas), w, d1) + cruise_energy(*(data.best_s2_feas), 0.0, d2);
    //printf("E_feas: %.15f\n", e);

    // if (e > B)
    //     return -1;
    // *max_t = -minf;
    // *s1 = d1 / x[0];
    // *s2 = d2 / x[1];
    // *max_feas_t = *(data.best_obj_feas);
    // *s1_feas = *(data.best_s1_feas);
    // *s2_feas = *(data.best_s2_feas);

    *max_feas_t = *(data.best_obj_feas);
    *s1_feas = *(data.best_s1_feas);
    *s2_feas = *(data.best_s2_feas);
    return res_code;
}

// *** //

// *** min energy-per-meter *** //

double obj_max_range_speed(unsigned n, const double *x, double *grad, void *data)
{
	double w = *((double *)data);
	return (cruise_power(x[0], w) / x[0]);
}

double compute_max_range_speed(double w, double uav_min_speed, double uav_max_speed)
{
	if (uav_min_speed < 0) {
		uav_min_speed = UAV_MIN_SPEED;
	}
	if (uav_max_speed < 0) {
		uav_max_speed = UAV_MAX_SPEED;
	}

	nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA, 1); /* algorithm and dimensionality */

    double lb[1] = {uav_min_speed}; /* lower bound */
    double ub[1] = {uav_max_speed}; /* upper bound */
	nlopt_set_lower_bounds(opt, lb);
	nlopt_set_upper_bounds(opt, ub);
    //nlopt_set_maxeval(opt, 200); // max #iterations

	nlopt_set_min_objective(opt, obj_max_range_speed, &w);

	nlopt_set_xtol_rel(opt, NLOPT_TOLERANCE);

    // initial guess
	double x[1] = {(uav_max_speed - uav_min_speed) / 2};
	double minf;

	int res_code = nlopt_optimize(opt, x, &minf);
	nlopt_destroy(opt);

	if (res_code < 0)
	{
		return res_code;
	}
	return x[0];
}

// *** //

double obj_min_energy_time(unsigned n, const double *x, double *grad, void *data)
{
	my_params *d = (my_params *)data;
	double d1 = d->d1, d2 = d->d2;
	double w = d->w;
	double time = d->TIME;
	return (cruise_energy_time(d1, x[0], w) + cruise_energy_time(d2, x[1], 0.0) + hovering_power * (time - x[0] - x[1]));
}

double constraint_min_energy_time(unsigned n, const double *x, double *grad, void *data)
{
	my_params *d = (my_params *)data;
	double time = d->TIME;
	return (x[0] + x[1] - time);
}

double compute_min_energy_time(double d1, double d2, double w, double time, double init_t1, double init_t2, double uav_min_speed, double uav_max_speed, double energy_battery)
{
	if (uav_min_speed < 0) {
		uav_min_speed = UAV_MIN_SPEED;
	}
	if (uav_max_speed < 0) {
		uav_max_speed = UAV_MAX_SPEED;
	}
    // data
	my_params data = {0.0, time, d1, d2, w, NULL, NULL, NULL};

	nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA, 2); /* algorithm and dimensionality */

    double lb[2] = {d1 / uav_max_speed, d2 / uav_max_speed}; /* lower bounds */
    double ub[2] = {d1 / uav_min_speed, d2 / uav_min_speed}; /* upper bounds */
	nlopt_set_lower_bounds(opt, lb);
	nlopt_set_upper_bounds(opt, ub);

	nlopt_set_min_objective(opt, obj_min_energy_time, &data);
	nlopt_add_inequality_constraint(opt, constraint_min_energy_time, &data, NLOPT_TOLERANCE);
	nlopt_set_ftol_abs(opt, NLOPT_TOLERANCE);
	nlopt_set_stopval(opt, energy_battery - NLOPT_TOLERANCE);

    // initial guess
	double x[2] = {init_t1, init_t2};
	double minf;

	int return_code = nlopt_optimize(opt, x, &minf);
	if (return_code < 0)
	{
		printf("NLOPT return_code: %d\n",return_code);
        // check if it is feasible anyway
		printf("min_energy_time: %f (s1: %f, s2: %f)\n", minf, d1 / x[0], d2 / x[1]);
		double true_energy = cruise_energy(d1 / x[0], w, d1) + cruise_energy(d2 / x[1], 0.0, d2) + (time - x[0] - x[1]) * hovering_power;
		printf("check energy: %f\n", true_energy);
        // return return_code;
		if (minf > true_energy + NLOPT_TOLERANCE)
		{
			print_error("Error!");
		}
	}

	nlopt_destroy(opt);
    //printf("min_energy_time: %f (s1: %f, s2: %f)\n", minf, d1 / x[0], d2 / x[1]);
    //printf("check energy: %f\n", cruise_energy(d1 / x[0], w, d1) + cruise_energy(d2 / x[1], 0.0, d2) + (time - x[0] - x[1]) * hovering_power);
	return minf;
}

double bisection(double d1, double d2, double w, double B, double lb_time, double ub_time, int min_max_flag, double uav_min_speed, double uav_max_speed)
{
	if (uav_min_speed < 0) {
		uav_min_speed = UAV_MIN_SPEED;
	}
	if (uav_max_speed < 0) {
		uav_max_speed = UAV_MAX_SPEED;
	}
	while (ub_time - lb_time > NLOPT_TOLERANCE)
	{
		printf("\nlb: %.6f , ub: %.6f\n", lb_time, ub_time);
		double t = (lb_time + ub_time) / 2;
		double e = compute_min_energy_time(d1, d2, w, t, 2 * d1 / (uav_min_speed + uav_max_speed), 2 * d2 / (uav_min_speed + uav_max_speed),
			uav_min_speed, uav_max_speed, B);
		printf("T: %.6f | E: %.6f\n", t + extra_time, e);
        if (min_max_flag == 0) // lb is not feasible, ub is feasible
        {
        	if (e <= B)
        		ub_time = t;
        	else
        		lb_time = t;
        }
        else // lb is feasibile, ub is not feasible
        {
        	if (e > B)
        		ub_time = t;
        	else
        		lb_time = t;
        }
    }
    if (min_max_flag == 0)
    	return ub_time;
    return lb_time;
}
