#include "control.h"


#include "depth.h"
#include "sensors.h"

#define DEPTH_BOUND = 1.0f


static void task_move_to_depth(void);
static void task_maintain_depth(void);
static void task_profile(void);




static Profiler_t profiler;



void task_move_to_park(void)
{

    /** Set the depth */
    PST_set_depth(profiler.park.depth);

    /** Monitor the movement */
    do
    {
        /* code */
    } while (/* condition */);
    

    /** Kill the task */
    vTaskDelete(NULL);
}
void task_move_to_depth(void)
{   
    /** Send Command to Move to Depth */
    PST_set_depth(profiler.start.depth)

    /** Loop forever */
    while(1);

}

void task_maintain_depth(void)
{
    /** @todo is there anything to this? Probably not */
}


void task_profile(void)
{
    
    /** Send Command to surface (well, float depth) */
    /** @todo Add function call */

    /** Loop forever */
    while(1);
}

void task_move_to_surface(void)
{
    /** Send command to move to surface, but not tx position */
    PST_set_depth()
}



int32_t CTRL_MoveToPark(float depth)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
    sDepth_Measurement_t depth = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;
    

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();


    /** Set the Depth we're moving to */
    CTRL_set_park_depth(depth);


    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, xDepthHandle);
    xTaskCreate((TaskFunction_t) task_move_to_depth, "move_to_depth", 128, NULL, 1, xPistonHandle);

    if( (xDepthHandle != NULL) && (xPistonHandle != NULL) )
    {
        /** Start the tasks */
        vTaskStartScheduler();

        bool atDepth = false;

        float depth_max = depth + DEPTH_BOUND;
        float depth_min = depth - DEPTH_BOUND;

        while(!atDepth)
        {
            DEPTH_Read(&depth);

            if(depth.Depth >= profiler.crush_limit)
            {
                /** Emergency blow !!! */

            } else if ( (depth.Depth >= depth_min) && (depth.Depth <= depth_max) )
            {   
                /** Wait for X # of seconds to validate at we're not drifting */

            }
        }

        vTaskDelete(xDepthHandle);
        vTaskDelete(xPistonHandle);
    } else {


    }

}

int32_t CTRL_MoveToStartDepth(float depth)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
    sDepth_Measurement_t depth = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;
    

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();


    /** Set the Depth we're moving to */
    CTRL_set_park_depth(depth);


    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, xDepthHandle);
    xTaskCreate((TaskFunction_t) task_move_to_depth, "move_to_depth", 128, NULL, 1, xPistonHandle);

    if( (xDepthHandle != NULL) && (xPistonHandle != NULL) )
    {
        /** Start the tasks */
        vTaskStartScheduler();

        bool atDepth = false;

        float depth_max = depth + DEPTH_BOUND;
        float depth_min = depth - DEPTH_BOUND;

        while(!atDepth)
        {
            DEPTH_Read(&depth);

            if(depth.Depth >= profiler.crush_limit)
            {
                /** Emergency blow !!! */

            } else if ( (depth.Depth >= depth_min) && (depth.Depth <= depth_max) )
            {   
                /** Wait for X # of seconds to validate at we're not drifting */

            }
        }

        vTaskDelete(xDepthHandle);
        vTaskDelete(xPistonHandle);
    } else {


    }
    
}


int32_t CTRL_MaintainDepth(float depth, uint32_t time_s)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
    sDepth_Measurement_t depth = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();

    /** Set a 1 Hz rate to the pressure sensor */
    SENS_set_depth_rate(1);

    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, xDepthHandle);
    xTaskCreate((TaskFunction_t) task_maintain_depth, "move_to_depth", 128, NULL, 1, xPistonHandle);


    uint32_t time = time_s;

    while(time--)
    {

    }

    vTaskDelete(xDepthHandle);
    vTaskDelete(xPistonHandle);
    
}


int32_t CTRL_Profile(float top_depth, float rise_rate, bool break_thru_lens)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
    sDepth_Measurement_t depth = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xTemperatureHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();

    /** Set a 1 Hz rate to the pressure sensor */
    SENS_set_depth_rate(4);

    /** Set the temperature setting */
    SENS_set_temperature_rate(4);

    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, xDepthHandle);
    xTaskCreate((TaskFunctin_t) task_temperature, "temperature", 128, NULL, 1, xTemperatureHandle);
    xTaskCreate((TaskFunction_t) task_profile, "move_to_depth", 128, NULL, 1, xPistonHandle);


    if( (xDepthHandle != NULL) && (xPistonHandle != NULL))
    {

        /** */
        bool completeFlag = false;
        while(!completeFlag)
        {



        }

    }

    

}

int32_t CTRL_MoveToSurface(uint32_t timeout)
{
    int32_t retVal = CTRL_ERROR_FAILURE;
    sDepth_Measurement_t depth = {0};
    TaskHandle_t xDepthHandle = NULL;
    TaskHandle_t xPistonHandle = NULL;
    

    /** In any move, GPS & Iridium should be off to save power */
    module_turn_off_gps_and_iridium();


    /** Set the Depth we're moving to */
    CTRL_set_park_depth(depth);


    /** Create the tasks */
    xTaskCreate((TaskFunction_t) task_depth, "depth", 128, NULL, 1, xDepthHandle);
    xTaskCreate((TaskFunction_t) task_move_to_depth, "move_to_depth", 128, NULL, 1, xPistonHandle);

    if( (xDepthHandle != NULL) && (xPistonHandle != NULL) )
    {
        /** Start the tasks */
        vTaskStartScheduler();

        bool atDepth = false;

        float depth_max = depth + DEPTH_BOUND;
        float depth_min = depth - DEPTH_BOUND;

        while(!atDepth)
        {
            DEPTH_Read(&depth);

            if(depth.Depth >= profiler.crush_limit)
            {
                /** Emergency blow !!! */

            } else if ( (depth.Depth >= depth_min) && (depth.Depth <= depth_max) )
            {   
                /** Wait for X # of seconds to validate at we're not drifting */

            }
        }

        vTaskDelete(xDepthHandle);
        vTaskDelete(xPistonHandle);
    } else {


    }
}














static void module_turn_off_gps_and_iridium(void)
{
    /** @todo Add functions */
}