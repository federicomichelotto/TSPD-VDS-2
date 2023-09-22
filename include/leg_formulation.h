#ifndef LEG_FORMULATION_H
#define LEG_FORMULATION_H

int x_pos_leg(int i, int j, instance *inst);
int y_pos_leg(int l, instance *inst);
int z_pos_leg(int i, int j, int k, instance *inst);
int w_pos_leg(int k, instance *inst);
int u_pos_leg(int i, instance *inst);
void build_leg_based_model(CPXENVptr env, CPXLPptr lp, instance *inst, double min_cost_TSP);
double gather_solution_leg_formulation(instance *inst, const double *xstar, int* truck_succ, int* drone_succ);
double leg_formulation(instance * inst, instance_TSP * inst_tsp, int iter);
void findConnectedComponents(const double *xstar, instance *inst, int **succ, int **comp, int *ncomp, int **length_comp);



// Callback
static int CPXPUBLIC callback_driver(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);
static int CPXPUBLIC callback_candidate(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);
static int CPXPUBLIC callback_candidate2(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);
static int CPXPUBLIC callback_candidate3(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);
static int CPXPUBLIC callback_relaxation(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void *userhandle);

#endif //LEG_FORMULATION_H
