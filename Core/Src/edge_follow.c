#include "edge_follow.h"

static int16_t sp_left       = EDGE_SP_LEFT_DEFAULT;
static int16_t sp_right      = EDGE_SP_RIGHT_DEFAULT;
static int16_t last_error    = 0;
static uint8_t lost_flag     = 0;

void EdgeFollow_Init(int16_t sp_left_in, int16_t sp_right_in)
{
    sp_left    = sp_left_in;
    sp_right   = sp_right_in;
    last_error = 0;
    lost_flag  = 0;
}

void EdgeFollow_Reset(int16_t seed_error)
{
    last_error = seed_error;
    lost_flag  = 0;
}

int32_t EdgeFollow_GetPosition(const volatile uint16_t *norm, EdgeSide_t side, uint8_t *found)
{
    // Quet tu phia NGOAI vao: mep trai quet 4->0, mep phai quet 0->4
    int8_t  step = (side == EDGE_SIDE_LEFT) ? -1 : +1;
    int8_t  i    = (side == EDGE_SIDE_LEFT) ? (EDGE_NUM_SENSORS - 1) : 0;
    int32_t pos_prev = 0;
    int32_t v_prev   = 0;

    *found = 1;
    for (uint8_t k = 0; k < EDGE_NUM_SENSORS; k++, i += step)
    {
        int32_t v   = norm[i];
        int32_t pos = 2000 - 1000 * i;          // khop weights[] cua rpr220.c

        if (v >= EDGE_THRESH)
        {
            if (k == 0)
            {
                // Mat ngoai cung da den san: mep nam ngoai mang
                return pos + (int32_t)side * EDGE_OUTSIDE_OFFSET;
            }
            // Noi suy tuyen tinh diem cat nguong giua mat truoc (trang) va mat nay (den)
            return pos_prev + (pos - pos_prev) * (EDGE_THRESH - v_prev) / (v - v_prev);
        }
        pos_prev = pos;
        v_prev   = v;
    }

    *found = 0;                                  // Khong mat nao den: mat line
    return 0;
}

int16_t EdgeFollow_GetError(const volatile uint16_t *norm, EdgeSide_t side)
{
    uint8_t found;
    int32_t e = EdgeFollow_GetPosition(norm, side, &found);

    if (!found)
    {
        lost_flag = 1;
        return last_error;                       // Giu sai so cu, khong ep ve 0
    }

    lost_flag  = 0;
    last_error = (side == EDGE_SIDE_LEFT) ? (int16_t)(e + sp_left)
                                          : (int16_t)(e - sp_right);
    return last_error;
}

uint8_t EdgeFollow_IsLost(void)
{
    return lost_flag;
}