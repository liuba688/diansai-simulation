#include "h_mission.h"

#include <stddef.h>
#include <string.h>
#include "zdt_emm_v5.h"

#define MODE_RETRY_TICKS       (50U)
#define MODE_MAX_RETRIES       (3U)
#define START_TIMEOUT_TICKS    (30U)
#define HEARTBEAT_TICKS        (20U)  /* 200 ms; keep ample margin below camera timeout. */

uint8_t h_mission_needs_balance(car_task_t task)
{
    return CAR_TASK_3_ROUND_TRIP == task
        || CAR_TASK_4_AB_BALANCE == task
        || CAR_TASK_5_LAP_BALANCE == task;
}

static h_route_t route_for_task(car_task_t task)
{
    if(CAR_TASK_4_AB_BALANCE == task) return H_ROUTE_AB_150_CM;
    if(CAR_TASK_2_RACE == task || CAR_TASK_5_LAP_BALANCE == task
       || CAR_TASK_6_ARBITRARY == task) return H_ROUTE_RACE_LINE;
    return H_ROUTE_NONE;
}

static void send_command(uint8_t type, h_mission_t *m,
                         car_speed_tier_t speed, uint8_t flags,
                         uint32_t tick)
{
    vision_uart_send_command(type, (uint8_t)m->task, m->run_id,
        m->target_x10_mm, (uint8_t)speed, flags, tick * 10U);
    m->last_command_tick = tick;
}

void h_mission_init(h_mission_t *m)
{
    if(NULL == m) return;
    memset(m, 0, sizeof(*m));
    m->state = H_MISSION_MENU;
    ball_balance_init(&m->balance);
}

uint8_t h_mission_prepare(h_mission_t *m, car_task_t task,
                          car_speed_tier_t speed, int16_t target,
                          uint32_t tick)
{
    if(NULL == m || task < CAR_TASK_1_VIDEO || task > CAR_TASK_6_ARBITRARY)
        return 0U;
    if(CAR_TASK_6_ARBITRARY == task)
    {
        m->task = task; m->fault_code = 6U; m->state = H_MISSION_FAULT;
        return 0U;
    }
    m->task = task; m->route = route_for_task(task);
    m->run_id++; if(0U == m->run_id) m->run_id = 1U;
    m->target_x10_mm = target;
    m->fault_code = 0U; m->camera_ready = 0U; m->camera_started = 0U;
    m->last_event_valid = 0U; m->command_retries = 0U;
    m->state = H_MISSION_CONFIGURING; m->state_tick = tick;
    ball_balance_stop(&m->balance);
    if(h_mission_needs_balance(task)) zdt_emm_begin(tick);
    send_command(VISION_MSG_MODE_SELECT, m, speed, 0U, tick);
    m->command_retries = 1U;
    return 1U;
}

uint8_t h_mission_start(h_mission_t *m, car_speed_tier_t speed, uint32_t tick)
{
    ball_balance_mode_t mode = BALL_MODE_OFF;
    if(NULL == m || H_MISSION_READY != m->state) return 0U;
    m->start_tick = tick; m->state_tick = tick;
    m->state = H_MISSION_STARTING; m->camera_started = 0U;
    if(CAR_TASK_3_ROUND_TRIP == m->task) mode = BALL_MODE_TASK3_ROUND_TRIP;
    else if(CAR_TASK_4_AB_BALANCE == m->task || CAR_TASK_5_LAP_BALANCE == m->task)
        mode = BALL_MODE_CENTER_HOLD;
    ball_balance_start(&m->balance, mode, tick);
    zdt_emm_set_step_limits(CAR_TASK_3_ROUND_TRIP == m->task ? 48U : 24U, 48U);
    send_command(VISION_MSG_START, m, speed, 1U, tick);
    return 1U;
}

static uint8_t event_matches(const h_mission_t *m,
                             const vision_uart_snapshot_struct *v)
{
    return v->event.task_id == (uint8_t)m->task
        && v->event.run_id == m->run_id;
}

void h_mission_update(h_mission_t *m,
                      const vision_uart_snapshot_struct *v,
                      car_speed_tier_t speed, uint32_t tick)
{
    uint8_t new_event = 0U;
    uint8_t ball_valid = 0U;
    if(NULL == m || NULL == v) return;
    zdt_emm_update(tick);
    if(!m->last_event_valid || v->event_sequence != m->last_event_sequence)
    {
        m->last_event_valid = 1U; m->last_event_sequence = v->event_sequence;
        new_event = event_matches(m, v);
    }

    if(H_MISSION_CONFIGURING == m->state)
    {
        if(new_event && VISION_MSG_MODE_READY == v->event_type
           && 0U == v->event.result) m->camera_ready = 1U;
        if(m->camera_ready && (!h_mission_needs_balance(m->task) || zdt_emm_is_ready()))
        { m->state = H_MISSION_READY; m->state_tick = tick; }
        else if(tick - m->last_command_tick >= MODE_RETRY_TICKS)
        {
            if(m->command_retries < MODE_MAX_RETRIES)
            {
                send_command(VISION_MSG_MODE_SELECT, m, speed, 0U, tick);
                m->command_retries++;
            }
            else { m->fault_code = 10U; m->state = H_MISSION_FAULT; }
        }
    }
    else if(H_MISSION_STARTING == m->state)
    {
        if(new_event && VISION_MSG_STARTED == v->event_type && 0U == v->event.result)
        { m->camera_started = 1U; m->state = H_MISSION_RUNNING; m->state_tick = tick; }
        else if(tick - m->state_tick >= START_TIMEOUT_TICKS)
        { m->fault_code = 11U; m->state = H_MISSION_FAULT; }
    }
    else if(H_MISSION_RUNNING == m->state)
    {
        if(new_event && VISION_MSG_FAULT == v->event_type)
        { m->fault_code = v->event.fault_code ? v->event.fault_code : 12U; m->state = H_MISSION_FAULT; }
        if(new_event && VISION_MSG_TASK_COMPLETE == v->event_type
           && CAR_TASK_3_ROUND_TRIP != m->task)
        { m->stop_tick = tick; m->state = H_MISSION_FINISHED; }
        if(h_mission_needs_balance(m->task))
        {
            ball_valid = v->ball_fresh
                && v->ball.task_id == (uint8_t)m->task
                && v->ball.run_id == m->run_id
                && v->ball.measurement_age_ms <= 150U;
            ball_balance_update(&m->balance, &v->ball, ball_valid, tick);
            if(ball_balance_has_fault(&m->balance))
            { m->fault_code = (uint8_t)(20U + m->balance.fault_code); m->state = H_MISSION_FAULT; }
            else
            {
                zdt_emm_request_target(m->balance.motor_target_pulses, tick);
                if(CAR_TASK_3_ROUND_TRIP == m->task
                   && H_MISSION_RUNNING == m->state
                   && ball_balance_is_complete(&m->balance))
                {
                    m->stop_tick = tick;
                    ball_balance_stop(&m->balance);
                    if(!zdt_emm_return_home(tick)) zdt_emm_emergency_stop();
                    m->state = H_MISSION_FINISHED;
                }
            }
        }
        if(h_mission_needs_balance(m->task) && zdt_emm_has_fault())
        { m->fault_code = 30U; m->state = H_MISSION_FAULT; }
        if(!v->link_alive)
        { m->fault_code = 31U; m->state = H_MISSION_FAULT; }
    }

    if((H_MISSION_CONFIGURING == m->state
        || H_MISSION_READY == m->state
        || H_MISSION_STARTING == m->state)
       && h_mission_needs_balance(m->task) && zdt_emm_has_fault())
    {
        m->fault_code = 30U;
        m->state = H_MISSION_FAULT;
    }
    if(H_MISSION_FAULT == m->state)
    {
        zdt_emm_emergency_stop();
        ball_balance_stop(&m->balance);
    }
    if(m->state >= H_MISSION_CONFIGURING
       && tick - m->last_command_tick >= HEARTBEAT_TICKS)
        send_command(VISION_MSG_MCU_HEARTBEAT, m, speed, (uint8_t)m->state, tick);
}

void h_mission_stop(h_mission_t *m, uint8_t reason, uint32_t tick)
{
    if(NULL == m) return;
    send_command(VISION_MSG_STOP, m, car_menu_get_speed_tier(), reason, tick);
    ball_balance_stop(&m->balance);
    /* BACK returns to the single power-on level zero during normal operation. */
    if(!zdt_emm_return_home(tick)) zdt_emm_emergency_stop();
    m->stop_tick = tick; m->state = H_MISSION_MENU;
    m->task = CAR_TASK_NONE; m->route = H_ROUTE_NONE;
}

void h_mission_finish_chassis(h_mission_t *m, uint32_t tick)
{
    if(NULL == m) return;
    m->stop_tick = tick;
    if(CAR_TASK_2_RACE == m->task) m->state = H_MISSION_FINISHED;
    /* Task 4/5 keep the ball loop alive on the stopped car for judging. */
}

uint8_t h_mission_car_allowed(const h_mission_t *m)
{ return m && H_MISSION_RUNNING == m->state && H_ROUTE_NONE != m->route; }
uint8_t h_mission_is_ready(const h_mission_t *m)
{ return m && H_MISSION_READY == m->state; }
