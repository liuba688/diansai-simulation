#include "car_menu.h"

#define CAR_MENU_DEBOUNCE_TICKS          (3U)
#define CAR_MENU_LONG_PRESS_TICKS        (100U)
#define CAR_MENU_LINE_CHARS              (21U)

typedef enum
{
    CAR_MENU_KEY_UP = 0,
    CAR_MENU_KEY_DOWN,
    CAR_MENU_KEY_OK,
    CAR_MENU_KEY_BACK,
    CAR_MENU_KEY_COUNT
} car_menu_key_t;

typedef struct
{
    car_task_t task;
    const char *name;
} car_menu_item_t;

typedef struct
{
    uint8 raw_level;
    uint8 stable_level;
    uint8 debounce_ticks;
    uint16 hold_ticks;
    uint8 short_event;
    uint8 long_event;
} car_menu_key_state_t;

static const gpio_pin_enum car_menu_key_pins[CAR_MENU_KEY_COUNT] =
{
    CAR_MENU_KEY_UP_PIN,
    CAR_MENU_KEY_DOWN_PIN,
    CAR_MENU_KEY_OK_PIN,
    CAR_MENU_KEY_BACK_PIN
};

/*
 * This table is the only place that needs editing after a contest problem is
 * chosen. Reserved entries deliberately do nothing until a task is assigned.
 */
static const car_menu_item_t car_menu_items[CAR_MENU_ITEM_COUNT] =
{
    { CAR_TASK_LINE_FOLLOW, "LINE FOLLOW" },
    { CAR_TASK_ANGLE_HOLD,  "ANGLE HOLD"  },
    { CAR_TASK_RESERVED_03, "EMPTY TASK 03" },
    { CAR_TASK_RESERVED_04, "EMPTY TASK 04" },
    { CAR_TASK_RESERVED_05, "EMPTY TASK 05" },
    { CAR_TASK_RESERVED_06, "EMPTY TASK 06" },
    { CAR_TASK_RESERVED_07, "EMPTY TASK 07" },
    { CAR_TASK_RESERVED_08, "EMPTY TASK 08" }
};

static car_menu_key_state_t key_states[CAR_MENU_KEY_COUNT];
static car_menu_clear_fn menu_clear_screen;
static car_menu_text_fn menu_show_text;
static uint8 menu_open;
static uint8 selected_index;
static uint8 first_visible_index;
static uint8 redraw_pending;

static void car_menu_scan_key (uint8 index)
{
    car_menu_key_state_t *key = &key_states[index];
    uint8 level = gpio_get_level(car_menu_key_pins[index]);

    if(level != key->raw_level)
    {
        key->raw_level = level;
        key->debounce_ticks = 0;
    }
    else if(key->debounce_ticks < CAR_MENU_DEBOUNCE_TICKS)
    {
        key->debounce_ticks ++;
        if(key->debounce_ticks == CAR_MENU_DEBOUNCE_TICKS)
        {
            if(key->stable_level != level)
            {
                key->stable_level = level;
                if(GPIO_LOW == level)
                {
                    key->hold_ticks = 0;
                    key->long_event = 0;
                }
                else
                {
                    if((key->hold_ticks < CAR_MENU_LONG_PRESS_TICKS)
                       && !key->long_event)
                    {
                        key->short_event = 1;
                    }
                    key->hold_ticks = 0;
                    key->long_event = 0;
                }
            }
        }
    }

    if(GPIO_LOW == key->stable_level)
    {
        if(key->hold_ticks < CAR_MENU_LONG_PRESS_TICKS)
        {
            key->hold_ticks ++;
            if(key->hold_ticks == CAR_MENU_LONG_PRESS_TICKS)
            {
                key->long_event = 1;
            }
        }
    }
}

static uint8 car_menu_take_short_event (car_menu_key_t key)
{
    uint8 event = key_states[key].short_event;
    key_states[key].short_event = 0;
    return event;
}

static uint8 car_menu_take_long_event (car_menu_key_t key)
{
    if(1U == key_states[key].long_event)
    {
        /* Value 2 remembers that this press was already consumed. */
        key_states[key].long_event = 2U;
        return 1U;
    }
    return 0U;
}

static void car_menu_update_window (void)
{
    if(selected_index < first_visible_index)
    {
        first_visible_index = selected_index;
    }
    else if(selected_index >= (first_visible_index + CAR_MENU_VISIBLE_ROWS))
    {
        first_visible_index = selected_index - CAR_MENU_VISIBLE_ROWS + 1U;
    }
}

void car_menu_init (car_menu_clear_fn clear_screen, car_menu_text_fn show_text)
{
    uint8 i;

    menu_clear_screen = clear_screen;
    menu_show_text = show_text;
    menu_open = 1;
    selected_index = 0;
    first_visible_index = 0;
    redraw_pending = 1;

    for(i = 0; i < CAR_MENU_KEY_COUNT; i ++)
    {
        gpio_init(car_menu_key_pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
        key_states[i].raw_level = GPIO_HIGH;
        key_states[i].stable_level = GPIO_HIGH;
        key_states[i].debounce_ticks = 0;
        key_states[i].hold_ticks = 0;
        key_states[i].short_event = 0;
        key_states[i].long_event = 0;
    }
}

car_task_t car_menu_update (void)
{
    uint8 i;

    for(i = 0; i < CAR_MENU_KEY_COUNT; i ++)
    {
        car_menu_scan_key(i);
    }

    /*
     * BACK is always a safety stop. A long press fires without waiting for
     * release; a short press fires on release.
     */
    if(car_menu_take_long_event(CAR_MENU_KEY_BACK)
       || car_menu_take_short_event(CAR_MENU_KEY_BACK))
    {
        car_menu_open();
        return CAR_TASK_STOP;
    }

    if(!menu_open)
    {
        car_menu_take_short_event(CAR_MENU_KEY_UP);
        car_menu_take_short_event(CAR_MENU_KEY_DOWN);
        car_menu_take_short_event(CAR_MENU_KEY_OK);
        return CAR_TASK_NONE;
    }

    if(car_menu_take_short_event(CAR_MENU_KEY_UP))
    {
        if(0 == selected_index)
        {
            selected_index = CAR_MENU_ITEM_COUNT - 1U;
        }
        else
        {
            selected_index --;
        }
        car_menu_update_window();
        redraw_pending = 1;
    }

    if(car_menu_take_short_event(CAR_MENU_KEY_DOWN))
    {
        selected_index ++;
        if(selected_index >= CAR_MENU_ITEM_COUNT)
        {
            selected_index = 0;
        }
        car_menu_update_window();
        redraw_pending = 1;
    }

    if(car_menu_take_short_event(CAR_MENU_KEY_OK))
    {
        car_task_t task = car_menu_items[selected_index].task;
        car_menu_close();
        return task;
    }

    if(redraw_pending)
    {
        car_menu_render();
    }

    return CAR_TASK_NONE;
}

void car_menu_open (void)
{
    menu_open = 1;
    redraw_pending = 1;
}

void car_menu_close (void)
{
    menu_open = 0;
    redraw_pending = 0;
}

void car_menu_render (void)
{
    uint8 row;
    char line[CAR_MENU_LINE_CHARS + 1U];

    if(!menu_open || (0 == menu_clear_screen) || (0 == menu_show_text))
    {
        return;
    }

    menu_clear_screen(0x00);

    for(row = 0; row < CAR_MENU_VISIBLE_ROWS; row ++)
    {
        uint8 item_index = first_visible_index + row;
        uint8 column;

        for(column = 0; column < CAR_MENU_LINE_CHARS; column ++)
        {
            line[column] = ' ';
        }
        line[CAR_MENU_LINE_CHARS] = '\0';

        if(item_index < CAR_MENU_ITEM_COUNT)
        {
            const char *name = car_menu_items[item_index].name;
            uint8 source_index = 0;
            uint8 target_index = 2;

            line[0] = (item_index == selected_index) ? '>' : ' ';
            while((name[source_index] != '\0')
                  && (target_index < CAR_MENU_LINE_CHARS))
            {
                line[target_index] = name[source_index];
                source_index ++;
                target_index ++;
            }
        }
        menu_show_text(row * 2U, line);
    }

    redraw_pending = 0;
}

uint8 car_menu_is_open (void)
{
    return menu_open;
}

const char *car_menu_task_name (car_task_t task)
{
    uint8 i;

    if(CAR_TASK_STOP == task)
    {
        return "STOP";
    }

    for(i = 0; i < CAR_MENU_ITEM_COUNT; i ++)
    {
        if(car_menu_items[i].task == task)
        {
            return car_menu_items[i].name;
        }
    }
    return "UNKNOWN";
}
