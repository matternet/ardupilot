//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

enum precision_land_state_t {
    PRECISION_LAND_STATE_NOT_ACTIVE = 0,
    PRECISION_LAND_STATE_DESCEND_AND_SEARCH,
    PRECISION_LAND_STATE_STOP_AND_CENTER,
    PRECISION_LAND_STATE_DESCEND_TO_COMMIT_POINT,
    PRECISION_LAND_STATE_FINAL_LAND,
    PRECISION_LAND_STATE_SUCCESS,
    PRECISION_LAND_STATE_FAILED_ASCENDING,
    PRECISION_LAND_STATE_FAILED
};

struct precision_land_state_transition_t {
    enum precision_land_state_t from;
    enum precision_land_state_t to;
};

static const struct precision_land_state_transition_t precision_land_valid_transitions[] = {
    {PRECISION_LAND_STATE_NOT_ACTIVE, PRECISION_LAND_STATE_DESCEND_AND_SEARCH},
    {PRECISION_LAND_STATE_NOT_ACTIVE, PRECISION_LAND_STATE_STOP_AND_CENTER},

    {PRECISION_LAND_STATE_DESCEND_AND_SEARCH, PRECISION_LAND_STATE_STOP_AND_CENTER},
    {PRECISION_LAND_STATE_DESCEND_AND_SEARCH, PRECISION_LAND_STATE_FAILED},
    {PRECISION_LAND_STATE_DESCEND_AND_SEARCH, PRECISION_LAND_STATE_NOT_ACTIVE},

    {PRECISION_LAND_STATE_STOP_AND_CENTER, PRECISION_LAND_STATE_DESCEND_TO_COMMIT_POINT},
    {PRECISION_LAND_STATE_STOP_AND_CENTER, PRECISION_LAND_STATE_FAILED},
    {PRECISION_LAND_STATE_STOP_AND_CENTER, PRECISION_LAND_STATE_NOT_ACTIVE},

    {PRECISION_LAND_STATE_DESCEND_TO_COMMIT_POINT, PRECISION_LAND_STATE_FINAL_LAND},
    {PRECISION_LAND_STATE_DESCEND_TO_COMMIT_POINT, PRECISION_LAND_STATE_FAILED_ASCENDING},
    {PRECISION_LAND_STATE_DESCEND_TO_COMMIT_POINT, PRECISION_LAND_STATE_NOT_ACTIVE},

    {PRECISION_LAND_STATE_FINAL_LAND, PRECISION_LAND_STATE_SUCCESS},
    {PRECISION_LAND_STATE_FINAL_LAND, PRECISION_LAND_STATE_NOT_ACTIVE},

    {PRECISION_LAND_STATE_FAILED_ASCENDING, PRECISION_LAND_STATE_FAILED},
    {PRECISION_LAND_STATE_FAILED_ASCENDING, PRECISION_LAND_STATE_NOT_ACTIVE},
};

#define PRECISION_LAND_NUM_VALID_TRANSITIONS (sizeof(precision_land_valid_transitions)/sizeof(precision_land_valid_transitions[0]))

static enum precision_land_state_t precision_land_state;

void Copter::precision_land_init()
{
    copter.precland.init();
    precision_land_state = PRECISION_LAND_STATE_NOT_ACTIVE;
}

void Copter::precision_land_update_estimator()
{
    int32_t height_above_terrain_cm;
    bool rangefinder_height_above_terrain_cm_valid = get_rangefinder_height_above_terrain(height_above_terrain_cm);

    // use range finder altitude if it is valid, else try to get terrain alt, else use height above home
    if (!rangefinder_height_above_terrain_cm_valid) {
        if (!terrain_use() || !current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, height_above_terrain_cm)) {
            height_above_terrain_cm = current_loc.alt;
        }
    }

    copter.precland.update(height_above_terrain_cm, rangefinder_height_above_terrain_cm_valid);
}

void Copter::precision_land_begin()
{

}

void Copter::precision_land_update_controls()
{
    float climb_rate_desired;

    switch(precision_land_state) {
        case PRECISION_LAND_STATE_NOT_ACTIVE:
            break;
        case PRECISION_LAND_STATE_DESCEND_AND_SEARCH:
            break;
        case PRECISION_LAND_STATE_STOP_AND_CENTER:
            break;
        case PRECISION_LAND_STATE_DESCEND_TO_COMMIT_POINT:
            break;
        case PRECISION_LAND_STATE_FINAL_LAND:
            break;
        case PRECISION_LAND_STATE_SUCCESS:
            break;
        case PRECISION_LAND_STATE_FAILED_ASCENDING:
            break;
        case PRECISION_LAND_STATE_FAILED:
            break;
    }
}

bool Copter::precision_land_state_transition_valid(enum precision_land_state_t next_state)
{
    for (uint8_t i=0; i<PRECISION_LAND_NUM_VALID_TRANSITIONS; i++) {
        if (precision_land_valid_transitions[i].from == precision_land_state && precision_land_valid_transitions[i].to == next_state) {
            return true;
        }
    }
    return false;
}

bool Copter::precision_land_set_state(enum precision_land_state_t next_state)
{
    if (!precision_land_state_transition_valid(next_state)) {
        return false;
    }

    precision_land_state = next_state;
}

#endif
