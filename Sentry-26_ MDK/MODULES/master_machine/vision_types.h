#ifndef VISION_TYPES_H
#define VISION_TYPES_H

typedef enum { COLOR_NONE = 0, COLOR_BLUE = 1, COLOR_RED = 2 } Enemy_Color_e;
typedef enum { VISION_MODE_AIM = 0, VISION_MODE_SMALL_BUFF = 1, VISION_MODE_BIG_BUFF = 2 } Work_Mode_e;
typedef enum {
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10, SMALL_AMU_15 = 15, BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18, SMALL_AMU_30 = 30,
} Bullet_Speed_e;

#endif /* VISION_TYPES_H */
