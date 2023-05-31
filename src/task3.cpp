#include <eigen/Eigen/Dense>
#include "cox.hpp"
#include "global_vars.h"
#include "task1.h"
#include "task3.h"
#include "stop_vars.h"

void *Task3Thread(void *arg)
{
    run_cox_flag = DISABLED;
    while (global_stop_msg != ACTIVE)
    {
        if (run_cox_flag == ACTIVE)
        {
            CoxAlgo();
            finish_cox_flag = ACTIVE;
            run_cox_flag = DISABLED;
        }
    }
    return NULL;
}