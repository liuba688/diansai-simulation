#include "car_menu.h"

#include <stdio.h>
#include <string.h>

#define MENU_DEBOUNCE_TICKS (3U)
#define MENU_LONG_TICKS     (100U)

typedef enum { KEY_UP = 0, KEY_DOWN, KEY_OK, KEY_BACK, KEY_COUNT } key_id_t;

typedef struct
{
    uint8 raw;
    uint8 stable;
    uint8 debounce;
    uint16 held;
    uint8 short_event;
    uint8 long_event;
    uint8 long_consumed;
} key_state_t;

typedef struct { car_task_t task; const char *name; } menu_item_t;

static const gpio_pin_enum key_pins[KEY_COUNT] = {
    CAR_MENU_KEY_UP_PIN, CAR_MENU_KEY_DOWN_PIN,
    CAR_MENU_KEY_OK_PIN, CAR_MENU_KEY_BACK_PIN
};

static const car_speed_tier_params_struct speed_table[CAR_SPEED_TIER_COUNT] = {
    {110.0f,  72.0f, 52.0f, 0.24f, 12.0f, 1.0f, 0.0f},
    {140.0f, 100.0f, 82.0f, 0.20f, 12.0f, 1.0f, 0.0f},
    {160.0f, 115.0f, 92.0f, 0.20f, 12.0f, 1.0f, 0.0f},
    {175.0f, 130.0f, 90.0f, 0.20f, 14.0f, 1.2f, 0.0f}
};

static const char *speed_names[CAR_SPEED_TIER_COUNT] = {
    "CONSERVATIVE", "NORMAL", "FAST", "SPRINT"
};

static const menu_item_t menu_items[CAR_MENU_ITEM_COUNT] = {
    {CAR_TASK_1_VIDEO,       "TASK 1 VIDEO"},
    {CAR_TASK_2_RACE,        "TASK 2 RACE"},
    {CAR_TASK_3_ROUND_TRIP,  "TASK 3 BALL"},
    {CAR_TASK_4_AB_BALANCE,  "TASK 4 A-B"},
    {CAR_TASK_5_LAP_BALANCE, "TASK 5 LAP"},
    {CAR_TASK_6_ARBITRARY,   "TASK 6 RESERVED"},
    {CAR_TASK_RESET_YAW,     "CALIBRATE"},
    {CAR_TASK_DIAGNOSTIC,    "DIAGNOSTIC"}
};

static key_state_t keys[KEY_COUNT];
static car_menu_clear_fn clear_fn;
static car_menu_text_fn text_fn;
static uint8 opened;
static uint8 speed_opened;
static uint8 selected;
static uint8 first_visible;
static uint8 redraw;
static car_task_t pending_task;
static car_speed_tier_t current_speed = CAR_SPEED_TIER_FAST;
static car_speed_tier_t pending_speed = CAR_SPEED_TIER_FAST;

static uint8 task_uses_chassis(car_task_t task)
{
    return (CAR_TASK_2_RACE == task)
        || (CAR_TASK_4_AB_BALANCE == task)
        || (CAR_TASK_5_LAP_BALANCE == task)
        || (CAR_TASK_6_ARBITRARY == task);
}

static void scan_key(uint8 index)
{
    key_state_t *key = &keys[index];
    uint8 level = gpio_get_level(key_pins[index]);
    if(level != key->raw)
    {
        key->raw = level;
        key->debounce = 0U;
    }
    else if(key->debounce < MENU_DEBOUNCE_TICKS)
    {
        key->debounce++;
        if((MENU_DEBOUNCE_TICKS == key->debounce) && (level != key->stable))
        {
            key->stable = level;
            if(GPIO_LOW == level)
            {
                key->held = 0U;
                key->long_consumed = 0U;
            }
            else
            {
                if(!key->long_consumed)
                {
                    key->short_event = 1U;
                }
                key->held = 0U;
            }
        }
    }
    if(GPIO_LOW == key->stable && key->held < MENU_LONG_TICKS)
    {
        key->held++;
        if(MENU_LONG_TICKS == key->held)
        {
            key->long_event = 1U;
            key->long_consumed = 1U;
        }
    }
}

static uint8 take_short(key_id_t id)
{
    uint8 value = keys[id].short_event;
    keys[id].short_event = 0U;
    return value;
}

static uint8 take_long(key_id_t id)
{
    uint8 value = keys[id].long_event;
    keys[id].long_event = 0U;
    return value;
}

static void update_window(void)
{
    if(selected < first_visible) first_visible = selected;
    if(selected >= first_visible + CAR_MENU_VISIBLE_ROWS)
        first_visible = selected - CAR_MENU_VISIBLE_ROWS + 1U;
}

const char *car_menu_speed_tier_name(car_speed_tier_t tier)
{
    return (tier < CAR_SPEED_TIER_COUNT) ? speed_names[tier] : "INVALID";
}

void car_menu_init(car_menu_clear_fn clear_screen, car_menu_text_fn show_text)
{
    uint8 i;
    clear_fn = clear_screen;
    text_fn = show_text;
    opened = 1U;
    speed_opened = 0U;
    selected = 0U;
    first_visible = 0U;
    redraw = 1U;
    pending_task = CAR_TASK_NONE;
    for(i = 0U; i < KEY_COUNT; ++i)
    {
        gpio_init(key_pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
        memset(&keys[i], 0, sizeof(keys[i]));
        keys[i].raw = gpio_get_level(key_pins[i]);
        keys[i].stable = GPIO_HIGH;
    }
}

car_task_t car_menu_update(void)
{
    uint8 i;
    for(i = 0U; i < KEY_COUNT; ++i) scan_key(i);

    if(take_long(KEY_BACK) || take_short(KEY_BACK))
    {
        if(speed_opened)
        {
            speed_opened = 0U;
            pending_task = CAR_TASK_NONE;
            redraw = 1U;
            return CAR_TASK_NONE;
        }
        car_menu_open();
        return CAR_TASK_STOP;
    }
    if(!opened)
    {
        if(take_short(KEY_OK)) return CAR_TASK_START;
        take_short(KEY_UP);
        take_short(KEY_DOWN);
        return CAR_TASK_NONE;
    }

    if(speed_opened)
    {
        if(take_short(KEY_UP))
            pending_speed = (0U == pending_speed)
                ? (CAR_SPEED_TIER_COUNT - 1U)
                : (car_speed_tier_t)(pending_speed - 1U);
        if(take_short(KEY_DOWN))
            pending_speed = (car_speed_tier_t)
                (((uint8)pending_speed + 1U) % CAR_SPEED_TIER_COUNT);
        if(take_short(KEY_OK))
        {
            car_task_t task = pending_task;
            current_speed = pending_speed;
            pending_task = CAR_TASK_NONE;
            speed_opened = 0U;
            car_menu_close();
            return task;
        }
        redraw = 1U;
        car_menu_render();
        return CAR_TASK_NONE;
    }

    if(take_short(KEY_UP))
    {
        selected = (0U == selected) ? (CAR_MENU_ITEM_COUNT - 1U) : (selected - 1U);
        update_window();
        redraw = 1U;
    }
    if(take_short(KEY_DOWN))
    {
        selected = (selected + 1U) % CAR_MENU_ITEM_COUNT;
        update_window();
        redraw = 1U;
    }
    if(take_short(KEY_OK))
    {
        car_task_t task = menu_items[selected].task;
        if(task_uses_chassis(task))
        {
            pending_task = task;
            pending_speed = current_speed;
            speed_opened = 1U;
            redraw = 1U;
            car_menu_render();
            return CAR_TASK_NONE;
        }
        car_menu_close();
        return task;
    }
    car_menu_render();
    return CAR_TASK_NONE;
}

void car_menu_open(void)
{
    opened = 1U;
    speed_opened = 0U;
    pending_task = CAR_TASK_NONE;
    redraw = 1U;
}

void car_menu_close(void)
{
    opened = 0U;
    speed_opened = 0U;
    redraw = 0U;
}

void car_menu_render(void)
{
    uint8 row;
    char line[24];
    if(!opened || !redraw || NULL == clear_fn || NULL == text_fn) return;
    clear_fn(0x00U);
    if(speed_opened)
    {
        text_fn(0U, "SELECT SPEED");
        snprintf(line, sizeof(line), "> %s", speed_names[pending_speed]);
        text_fn(2U, line);
        snprintf(line, sizeof(line), "TASK %u", (unsigned)(pending_task - CAR_TASK_1_VIDEO + 1U));
        text_fn(4U, line);
        text_fn(6U, "OK START BACK CANCEL");
    }
    else
    {
        for(row = 0U; row < CAR_MENU_VISIBLE_ROWS; ++row)
        {
            uint8 index = first_visible + row;
            if(index >= CAR_MENU_ITEM_COUNT) break;
            snprintf(line, sizeof(line), "%c%s", (index == selected) ? '>' : ' ', menu_items[index].name);
            text_fn((uint8)(row * 2U), line);
        }
    }
    redraw = 0U;
}

uint8 car_menu_is_open(void) { return opened; }

const char *car_menu_task_name(car_task_t task)
{
    uint8 i;
    for(i = 0U; i < CAR_MENU_ITEM_COUNT; ++i)
        if(menu_items[i].task == task) return menu_items[i].name;
    return "NONE";
}

car_speed_tier_t car_menu_get_speed_tier(void) { return current_speed; }

const car_speed_tier_params_struct *car_menu_get_speed_tier_params(car_speed_tier_t tier)
{
    if(tier >= CAR_SPEED_TIER_COUNT) tier = CAR_SPEED_TIER_FAST;
    return &speed_table[tier];
}
