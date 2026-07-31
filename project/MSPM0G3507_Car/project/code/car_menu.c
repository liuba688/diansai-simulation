#include "car_menu.h"

#define CAR_MENU_DEBOUNCE_TICKS          (3U)
#define CAR_MENU_LONG_PRESS_TICKS        (100U)
#define CAR_MENU_DOUBLE_CLICK_TICKS      (35U)
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
    uint8 released_level;
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
 * Speed tier parameter table.  Each tier bundles all tunables so switching
 * at the track-side does not require a recompile.
 *
 * Conservative : low-speed 110 RPM stability baseline
 * Normal       : 140 RPM (tested, stable)
 * Fast         : 160 RPM (full-load stable baseline)
 * Sprint       : 175 RPM (17-18 s target)
 */
static const car_speed_tier_params_struct car_speed_tier_table[CAR_SPEED_TIER_COUNT] =
{
    /* CONSERVATIVE */
    {
        110.0f,   /* straight_rpm   */
        72.0f,    /* max_curve_rpm  */
        52.0f,    /* min_curve_rpm  */
        0.24f,    /* line_kp        */
        12.0f,    /* speed_kp       */
        1.0f,     /* speed_ki       */
        0.0f      /* speed_kd       */
    },
    /* NORMAL */
    {
        140.0f,   /* straight_rpm   */
        100.0f,   /* max_curve_rpm  */
        82.0f,    /* min_curve_rpm  */
        0.20f,    /* line_kp        */
        12.0f,    /* speed_kp       */
        1.0f,     /* speed_ki       */
        0.0f      /* speed_kd       */
    },
    /* FAST */
    {
        160.0f,   /* straight_rpm   */
        115.0f,   /* max_curve_rpm  */
        92.0f,    /* min_curve_rpm  */
        0.20f,    /* line_kp        */
        12.0f,    /* speed_kp       */
        1.0f,     /* speed_ki       */
        0.0f      /* speed_kd       */
    },
    /* SPRINT */
    {
        175.0f,   /* straight_rpm   */
        130.0f,   /* max_curve_rpm  */
        90.0f,    /* min_curve_rpm  */
        0.20f,    /* line_kp        */
        14.0f,    /* speed_kp       */
        1.2f,     /* speed_ki       */
        0.0f      /* speed_kd       */
    }
};

static car_speed_tier_t current_speed_tier = CAR_SPEED_TIER_SPRINT;

/*
 * Menu item table.
 * CALIBRATE first (IMU must settle before racing), then competition
 * tasks, then diagnostics and settings.
 */
static const car_menu_item_t car_menu_items[CAR_MENU_ITEM_COUNT] =
{
    { CAR_TASK_RESET_YAW,    "CALIBRATE"    },
    { CAR_TASK_RACE_LINE,    "RACE LINE"    },
    { CAR_TASK_STATIC_BALL,  "STATIC BALL"  },
    { CAR_TASK_AB_BALANCE,   "A-B BALANCE"  },
    { CAR_TASK_LAP_BALANCE,  "LAP BALANCE"  },
    { CAR_TASK_ANGLE_HOLD,   "ANGLE HOLD"   },
    { CAR_TASK_ODOMETER_QUERY, "ODOMETER"   },
    { CAR_TASK_SPEED_TIER,   "SPEED TIER"   }
};

static car_menu_key_state_t key_states[CAR_MENU_KEY_COUNT];
static car_menu_clear_fn menu_clear_screen;
static car_menu_text_fn menu_show_text;
static uint8 menu_open;
static uint8 selected_index;
static uint8 first_visible_index;
static uint8 redraw_pending;
static uint8 raw_key_mask;
static uint8 speed_menu_open;
static car_speed_tier_t speed_menu_selection;
#if CAR_MENU_SINGLE_B21_MODE
static uint8 single_key_click_pending;
static uint16 single_key_click_gap;
#endif

static const char *car_menu_speed_tier_name(car_speed_tier_t tier)
{
    static const char *names[CAR_SPEED_TIER_COUNT] =
    {
        "CONSERVATIVE", "NORMAL", "FAST", "SPRINT"
    };
    return names[tier];
}

static void car_menu_scan_key (uint8 index)
{
    car_menu_key_state_t *key = &key_states[index];
    gpio_pin_enum pin = car_menu_key_pins[index];
#if CAR_MENU_SINGLE_B21_MODE
    if(CAR_MENU_KEY_DOWN == index)
    {
        pin = CAR_MENU_SINGLE_B21_PIN;
    }
#endif
    uint8 level = gpio_get_level(pin);
    /* All four external keys are wired active-low. */
    uint8 pressed_level = GPIO_LOW;
#if CAR_MENU_SINGLE_B21_MODE
    if(CAR_MENU_KEY_DOWN == index)
    {
        /* B21 is electrically fixed: released=1, pressed=0. */
        pressed_level = GPIO_LOW;
    }
#endif

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
                if(pressed_level == level)
                {
                    key->hold_ticks = 0;
                    key->long_event = 0;
                }
                else if(key->released_level == level)
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

    if(pressed_level == key->stable_level)
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

/*
 * Display the current speed tier name and key parameters for ~1 second,
 * then re-render the menu.
 */
static void car_menu_show_speed_tier (void)
{
    char line[CAR_MENU_LINE_CHARS + 1U];
    const car_speed_tier_params_struct *p;
    static const char *tier_names[CAR_SPEED_TIER_COUNT] =
    {
        "CONSERVATIVE",
        "NORMAL",
        "FAST",
        "SPRINT"
    };

    p = &car_speed_tier_table[current_speed_tier];

    menu_clear_screen(0x00U);

    menu_show_text(0U, "SPEED TIER");
    menu_show_text(2U, tier_names[current_speed_tier]);

    sprintf(line, "STR=%u MAX=%u MIN=%u",
            (unsigned int)p->straight_rpm,
            (unsigned int)p->max_curve_rpm,
            (unsigned int)p->min_curve_rpm);
    menu_show_text(4U, line);

    sprintf(line, "L-Kp=%.2f S-Kp=%.1f",
            (double)p->line_kp,
            (double)p->speed_kp);
    menu_show_text(6U, line);

    system_delay_ms(1200U);
    redraw_pending = 1U;
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
    raw_key_mask = 0U;
    speed_menu_open = 0U;
    speed_menu_selection = current_speed_tier;
#if CAR_MENU_SINGLE_B21_MODE
    single_key_click_pending = 0U;
    single_key_click_gap = 0U;
#endif

    for(i = 0; i < CAR_MENU_KEY_COUNT; i ++)
    {
        gpio_init(car_menu_key_pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
    }
    system_delay_ms(2U);
    for(i = 0; i < CAR_MENU_KEY_COUNT; i ++)
    {
        key_states[i].raw_level =
            gpio_get_level(car_menu_key_pins[i]);
        /* Start from the electrically released state; do not infer polarity
         * from a possibly-held key during power-up. */
        key_states[i].stable_level = GPIO_HIGH;
        key_states[i].released_level = GPIO_HIGH;
        key_states[i].debounce_ticks = 0;
        key_states[i].hold_ticks = 0;
        key_states[i].short_event = 0;
        key_states[i].long_event = 0;
    }
#if CAR_MENU_SINGLE_B21_MODE
    /* B21 is the only physical menu key in this build. */
    /* The on-board B21 key is active-low: released=1, pressed=0. */
    gpio_init(CAR_MENU_SINGLE_B21_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    key_states[CAR_MENU_KEY_DOWN].raw_level = gpio_get_level(CAR_MENU_SINGLE_B21_PIN);
    key_states[CAR_MENU_KEY_DOWN].stable_level = key_states[CAR_MENU_KEY_DOWN].raw_level;
    key_states[CAR_MENU_KEY_DOWN].released_level = GPIO_HIGH;
#endif
}

car_task_t car_menu_update (void)
{
    uint8 i;
    uint8 new_raw_mask = 0U;

#if CAR_MENU_SINGLE_B21_MODE
    /* B21: short press = next item, long press = OK, double click = BACK. */
    {
        car_menu_key_state_t *key = &key_states[CAR_MENU_KEY_DOWN];
        uint8 level = gpio_get_level(CAR_MENU_SINGLE_B21_PIN);

        /* Dedicated B21 scanner: low=pressed, high=released. */
        if((GPIO_LOW == level) && (GPIO_HIGH == key->stable_level))
        {
            key->stable_level = GPIO_LOW;
            key->hold_ticks = 0U;
            key->long_event = 0U;
        }
        if((GPIO_LOW == level) && (GPIO_LOW == key->stable_level))
        {
            if(key->hold_ticks < CAR_MENU_LONG_PRESS_TICKS)
            {
                key->hold_ticks++;
                if(key->hold_ticks == CAR_MENU_LONG_PRESS_TICKS)
                {
                    key->long_event = 1U;
                }
            }
        }
        if((GPIO_HIGH == level) && (GPIO_LOW == key->stable_level))
        {
            if((key->hold_ticks < CAR_MENU_LONG_PRESS_TICKS)
               && (0U == key->long_event))
            {
                key->short_event = 1U;
            }
            key->stable_level = GPIO_HIGH;
            key->hold_ticks = 0U;
            key->long_event = 0U;
        }
    }
    if(single_key_click_pending && (single_key_click_gap < CAR_MENU_DOUBLE_CLICK_TICKS))
    {
        single_key_click_gap++;
    }
    else if(single_key_click_pending)
    {
        single_key_click_pending = 0U;
    }

    if(car_menu_take_long_event(CAR_MENU_KEY_DOWN))
    {
        single_key_click_pending = 0U;
        if(speed_menu_open)
        {
            current_speed_tier = speed_menu_selection;
            speed_menu_open = 0U;
            redraw_pending = 1U;
            car_menu_render();
            return CAR_TASK_NONE;
        }
        if(menu_open)
        {
            car_task_t task = car_menu_items[selected_index].task;
            if(CAR_TASK_SPEED_TIER == task)
            {
                speed_menu_selection = current_speed_tier;
                speed_menu_open = 1U;
                redraw_pending = 1U;
                car_menu_render();
                return CAR_TASK_NONE;
            }
            car_menu_close();
            return task;
        }
        car_menu_open();
        return CAR_TASK_NONE;
    }
    if(car_menu_take_short_event(CAR_MENU_KEY_DOWN))
    {
        if(single_key_click_pending)
        {
            single_key_click_pending = 0U;
            car_menu_open();
            return CAR_TASK_STOP;
        }
        single_key_click_pending = 1U;
        single_key_click_gap = 0U;
        if(menu_open)
        {
            selected_index = (uint8)((selected_index + 1U) % CAR_MENU_ITEM_COUNT);
            car_menu_update_window();
            redraw_pending = 1U;
            car_menu_render();
        }
        return CAR_TASK_NONE;
    }
    if(redraw_pending)
    {
        car_menu_render();
    }
    return CAR_TASK_NONE;
#endif

    for(i = 0; i < CAR_MENU_KEY_COUNT; i ++)
    {
        car_menu_scan_key(i);
        if(GPIO_HIGH == gpio_get_level(car_menu_key_pins[i]))
        {
            new_raw_mask |= (uint8)(1U << i);
        }
    }
    if(new_raw_mask != raw_key_mask)
    {
        raw_key_mask = new_raw_mask;
        redraw_pending = 1U;
    }

#if CAR_MENU_KEY_DIAGNOSTIC_MODE
    if(redraw_pending)
    {
        car_menu_render();
    }
    return CAR_TASK_NONE;
#else
    /* BACK exits the speed submenu; elsewhere it is the safety stop. */
    if(speed_menu_open)
    {
        if(car_menu_take_long_event(CAR_MENU_KEY_BACK)
           || car_menu_take_short_event(CAR_MENU_KEY_BACK))
        {
            speed_menu_open = 0U;
            redraw_pending = 1U;
            return CAR_TASK_NONE;
        }

        if(car_menu_take_short_event(CAR_MENU_KEY_UP))
        {
            if(0U == speed_menu_selection)
            {
                speed_menu_selection = CAR_SPEED_TIER_COUNT - 1U;
            }
            else
            {
                speed_menu_selection--;
            }
            redraw_pending = 1U;
        }
        if(car_menu_take_short_event(CAR_MENU_KEY_DOWN))
        {
            speed_menu_selection = (car_speed_tier_t)
                (((uint8)speed_menu_selection + 1U)
                 % CAR_SPEED_TIER_COUNT);
            redraw_pending = 1U;
        }
        if(car_menu_take_short_event(CAR_MENU_KEY_OK))
        {
            current_speed_tier = speed_menu_selection;
            speed_menu_open = 0U;
            redraw_pending = 1U;
        }
        if(redraw_pending)
        {
            car_menu_render();
        }
        return CAR_TASK_NONE;
    }

    /* BACK is the safety stop when the main menu is active or hidden. */
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

        if(CAR_TASK_SPEED_TIER == task)
        {
            speed_menu_selection = current_speed_tier;
            speed_menu_open = 1U;
            redraw_pending = 1U;
            return CAR_TASK_NONE;
        }

        if(CAR_TASK_ODOMETER_QUERY == task)
        {
            /* ODOMETER stays in-menu — the handler shows the value
             * briefly and calls car_menu_render(). */
            return task;
        }

        car_menu_close();
        return task;
    }

    if(redraw_pending)
    {
        car_menu_render();
    }

    return CAR_TASK_NONE;
#endif
}

void car_menu_open (void)
{
    menu_open = 1;
    speed_menu_open = 0U;
    redraw_pending = 1;
}

void car_menu_close (void)
{
    menu_open = 0;
    speed_menu_open = 0U;
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

    if(speed_menu_open)
    {
        const car_speed_tier_params_struct *p =
            &car_speed_tier_table[speed_menu_selection];

        menu_show_text(0U, "SELECT SPEED");
        menu_show_text(2U, car_menu_speed_tier_name(speed_menu_selection));
        sprintf(line, "STR=%u MAX=%u",
                (unsigned int)p->straight_rpm,
                (unsigned int)p->max_curve_rpm);
        menu_show_text(4U, line);
        menu_show_text(6U, "UP/DN SEL OK SAVE");
        redraw_pending = 0U;
        return;
    }

#if CAR_MENU_KEY_DIAGNOSTIC_MODE
    {
        char diagnostic_line[22];
        static const char hex_digits[] = "0123456789ABCDEF";

        menu_show_text(0U, "KEY TEST 0731");
        sprintf(diagnostic_line, "UP B13:%u DN B23:%u",
                (raw_key_mask >> CAR_MENU_KEY_UP) & 1U,
                (raw_key_mask >> CAR_MENU_KEY_DOWN) & 1U);
        menu_show_text(2U, diagnostic_line);
        sprintf(diagnostic_line, "OK B26:%u BK B27:%u",
                (raw_key_mask >> CAR_MENU_KEY_OK) & 1U,
                (raw_key_mask >> CAR_MENU_KEY_BACK) & 1U);
        menu_show_text(4U, diagnostic_line);
        sprintf(diagnostic_line, "RAW K:%c TIER:%u",
                hex_digits[raw_key_mask & 0x0FU],
                (unsigned int)current_speed_tier);
        menu_show_text(6U, diagnostic_line);
        redraw_pending = 0U;
        return;
    }
#else
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
#endif
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
    if(CAR_TASK_ODOMETER_QUERY == task)
    {
        return "ODOMETER";
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

car_speed_tier_t car_menu_get_speed_tier (void)
{
    return current_speed_tier;
}

const car_speed_tier_params_struct
    *car_menu_get_speed_tier_params (car_speed_tier_t tier)
{
    if(tier >= CAR_SPEED_TIER_COUNT)
    {
        tier = CAR_SPEED_TIER_SPRINT;
    }
    return &car_speed_tier_table[tier];
}
