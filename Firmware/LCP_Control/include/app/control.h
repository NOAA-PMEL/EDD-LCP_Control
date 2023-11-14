#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#define G_CONST                                 ( 9.80665 )


typedef enum eCTRL_Error_t{
    CTRL_ERROR_NONE  = 1,
    CTRL_ERROR_FAILURE = 0,
    CTRL_ERROR_TASK_FAILURE = -1,
    CTRL_ERROR_DEPTH_NOT_REACHED = -2,
    CTRL_ERROR_DEPTH_NOT_MAINTAINED = -3,


}CTRL_Errot_t;

//struct {
//    struct {
//        float previous;     /**< Previous volume setting (mL) */
//        float current;      /**< Current volume of system (mL) */
//    }volume;
//
//    struct{
//        float previous;     /**< Previous volume setting (N) */
//        float current;      /**< Current system buoyancy (N) */
//    }buoyancy;
//    struct {
//        struct {
//            float records[3];   /**< Previous 3 rate data record */
//            float average;      /**< Average of previous 3 rates */
//        }previous;
//
//        float current;          /**< Current rate */
//    }rate;
//}Actual;

//float module_calculate_buoyancy_from_ascent_rate(float rate);
//float module_calculate_volume_from_ascent_rate(float rate);
//float module_calculate_buoyancy_from_descent_rate(float rate);
//float module_calculate_volume_from_descent_rate(float rate);

float module_ctrl_set_buoyancy_from_rate(float rate, bool falling);

int32_t CTRL_MoveToPark(float depth);
int32_t CTRL_MoveToStartDepth(float depth);
int32_t CTRL_MaintainDepth(float depth, uint32_t time_s);
int32_t CTRL_Profile(float top_depth, float rise_rate, bool break_thru_lens);
int32_t CTRL_MoveToSurface(uint32_t timeout);
int32_t CTRL_MoveToTransmit(uint32_t timeout);

void CTRL_set_park_depth(float depth);
void CTRL_set_park_residence_time(uint32_t sec);
void CTRL_set_profile_depth(float depth);
void CTRL_set_profile_residence_time(uint32_t sec);
void CTRL_set_profile_rise_rate(float rate);
void CTRL_set_profile_break_thru(bool lens);
void CTRL_set_crush_limit(float depth);


#endif // CONTROL_H
