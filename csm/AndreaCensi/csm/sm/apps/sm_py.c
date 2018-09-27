#include "sm_py.h"

void say_hello(void){
    printf("hello, sm_py!\n");
}

/*
void csm_py(const double* points_ref, const int num_ref, 
            const double* points_sens, const int num_sens,
            double* pose_old, double* pose_new)*/
void csm_py(const double** points_ref, const int num_ref, 
            const double** points_sens, const int num_sens,
            double* pose_old, double* pose_new)
{
    // printf("hello, csm_py called !\n");
    // sm_debug_write(1); //enable debug
    struct sm_params params;
	struct sm_result result;

    LDP laser_ref = ld_alloc_new(num_ref);
    laser_ref->min_theta = -2.27;
    laser_ref->max_theta = 2.27;
    for( int i = 0; i < num_ref; i++ ){
        laser_ref->points[i].p[0] = *((double*)points_ref + 2*i + 0);
        laser_ref->points[i].p[1] = *((double*)points_ref + 2*i + 1);
        laser_ref->valid[i] = 1;
    }
    LDP laser_sens = ld_alloc_new(num_sens);
    laser_sens->min_theta = -2.27;
    laser_sens->max_theta = 2.27;
    for( int i = 0; i < num_sens; i++ ){
        laser_sens->points[i].p[0] = *((double*)points_sens + 2*i + 0);
        laser_sens->points[i].p[1] = *((double*)points_sens + 2*i + 1);
        laser_sens->valid[i] = 1;
    }

    params.laser_ref  = laser_ref;
    params.laser_sens = laser_sens;
    for(int i =0; i < 3; i++){
        params.first_guess[i] = 0.0f;
        params.laser[i] = 0.0f;
    }
    params.max_angular_correction_deg = 45.0;
    params.max_linear_correction = 0.50;
    params.max_iterations = 10;
    params.epsilon_xy = 0.000001;
    params.epsilon_theta = 0.000001;
    params.max_correspondence_dist = 0.3;
    params.sigma = 0.010;
    params.use_corr_tricks = 0;
    params.restart = 0;
    params.restart_threshold_mean_error = 0.01;
    params.restart_dt = 1.0;
    params.restart_dtheta = 0.1;
    params.clustering_threshold = 0.25;
    params.orientation_neighbourhood = 20;
    params.use_point_to_line_distance = 1;
    params.do_alpha_test = 0;
    params.do_alpha_test_thresholdDeg = 20.0;
    params.outliers_maxPerc = 0.90;
    params.outliers_adaptive_order = 0.7;
    params.outliers_adaptive_mult = 2.0;
    params.do_visibility_test = 0;
    params.outliers_remove_doubles = 1;
    params.do_compute_covariance = 0;
    params.debug_verify_tricks = 0;
    params.use_ml_weights = 0;
    params.use_sigma_weights = 0;

    sm_icp_py(&params, &result);

    for(int i =0; i < 3; i++){
        pose_new[i] = result.x[i];
    }
    // printf("params.laser_ref->nrays:%d\n", params.laser_ref->nrays);
    // printf("params.laser_sens->nrays:%d\n", params.laser_sens->nrays);

    // for( int i = 0; i < num_ref; i++ ){
    //     pose_old[0] += *((double*)points_ref + 2*i + 0);
    //     pose_old[1] += *((double*)points_ref + 2*i + 1);
    // }
    // for( int i = 0; i < num_sens; i++ ){
    //     pose_new[0] += *((double*)points_sens + 2*i + 0);
    //     pose_new[1] += *((double*)points_sens + 2*i + 1);
    // }

    return;
}