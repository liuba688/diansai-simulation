/*********************************************************************************************************************
* MSPM0G3507 integrated contest controller
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "car_app.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    car_app_init();
    car_app_run();
    return 0;
}
