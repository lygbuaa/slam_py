#include <time.h>
#include <string.h>
#include <libgen.h>

#include <options/options.h>
#include "../csm/csm_all.h"

void say_hello(void);

void csm_py(const double** points_ref, const int num_ref, 
            const double** points_sens, const int num_sens,
            double* pose_old, double* pose_new);