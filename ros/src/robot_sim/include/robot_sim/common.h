// Copyright © 2018 Naoki Akai. All rights reserved.

#ifndef __COMMON_H__
#define __COMMON_H__

#include <vector>

typedef struct
{
    double x, y, yaw;
} pose_t;

typedef struct
{
    bool is_active;
    pose_t pose;
    double v;
    double size;
    int id;
} moving_object_t;





// belows are used for semantic scan-based localization

#define WALL_MAP_FRAME ("/wall")
#define DOOR_MAP_FRAME ("/door")
#define TABLE_MAP_FRAME ("/table")
#define MAX_LANDMARK_ID (3)
#define UNKONW_OBSTACLE_ID (4)
#define FREE_SPACE_ID (1000)
#define MIN_DYNAMIC_OBSTACLE_ID (1001)
#define MAX_DYNAMIC_OBSTACLE_ID (65535)

// UNKNOWN must be numbers of semantic labels (NOTE: FREE_SPACE is not counted as semantics)
enum ObjectID
{
    LANDMARK = 0, // unknown static landmarks which do not have specific semantic label
    WALL = 1,
    DOOR = 2,
    TABLE = 3,
    UNKNOWN = 4, // dynamic objects which do not exist on a given landmark map
    FREE_SPACE = 1000,
    MIN_OBJECT_ID = 1001,
    MAX_OBJECT_ID = 65535
};

inline void get_semantic_point_color(int type, unsigned char* r, unsigned char* g, unsigned char* b)
{
    if (type == LANDMARK)
        *r = 0, *g = 0, *b = 0;
    else if (type == WALL)
        *r = 255, *g = 0, *b = 0;
    else if (type == DOOR)
        *r = 0, *g = 255, *b = 0;
    else if (type == TABLE)
        *r = 0, *g = 0, *b = 255;
    else
        *r = 255, *g = 255, *b = 255; // unknown
}

inline int get_semantic_label_num(void)
{
    return (UNKNOWN + 1);
}

inline void get_corresponding_semantic_prob_indexes(int scan_index, int* start_index, int* end_index)
{
    *start_index = (UNKNOWN + 1) * scan_index;
    *end_index = (UNKNOWN + 1) * scan_index + UNKNOWN;
}

inline int get_corresponding_semantic_prob_index(int scan_index, int type)
{
    if (type > UNKNOWN)
        type = UNKNOWN;
    return (UNKNOWN + 1)  * scan_index + type;
}

#endif /* __COMMON_H__ */
