#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>


typedef enum eCTRL_Error_t{
    CTRL_ERROR_NONE  = 1,
    CTRL_ERROR_FAILURE = 0,
    CTRL_ERROR_TASK_FAILURE = -1,
    CTRL_ERROR_DEPTH_NOT_REACHED = -2,
    CTRL_ERROR_DEPTH_NOT_MAINTAINED = -3,


}CTRL_Errot_t;


typedef struct sProfiler_t{
    struct {
        float depth;
        uitn32_t residence_time;
    }park;
    struct {
        float depth;
        uint32_t residence_time;
    }start;
    struct {
        float top_depth;
        float rate;
        bool break_thru_lens;
    }profile;

    float crush_limit;

}Profiler_t;




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
