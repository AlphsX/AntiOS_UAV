/*
 * animation.h -- AltiOS UAV Boot Animation
 * Arduino UNO R4 WiFi -- LED Matrix 12x8
 */

#include <stdint.h>

const unsigned long frames[][4] = {

    // ── Phase 1: Center dot ──────────────────────────────
    {0x00000000, 0x00060000, 0x00000000, 120},

    // ── Phase 2: Diamond expand ──────────────────────────
    {0x00000000, 0x00060000, 0x00000060, 80},
    {0x00000060, 0x00060000, 0x00000060, 80},
    {0x00000060, 0x0FF60FF0, 0x00000060, 80},
    {0x00000FF0, 0x0FF60FF0, 0x0FF00000, 80},

    // ── Phase 3: Sweep outward ───────────────────────────
    {0x00000FF0, 0xFFF60FF0, 0x0FF00000, 60},
    {0x0FF00FF0, 0xFFF6FFF0, 0xFFF00FF0, 60},
    {0xFFF00FF0, 0xFFFFFFF0, 0xFFF0FFF0, 60},
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 180},

    // ── Phase 4: Wipe from edges to center ───────────────
    {0x7FF7FF00, 0xFFFFFF00, 0x7FF7FF00, 50},
    {0x3FE3FE00, 0xFFFFFE00, 0x3FE3FE00, 50},
    {0x1FC1FC00, 0x7FFFC00, 0x1FC1FC00, 50},
    {0x0F80F800, 0x3FFF8000, 0x0F80F800, 50},
    {0x07007000, 0x1FF70000, 0x07007000, 50},
    {0x00000000, 0x00060000, 0x00000000, 80},

    // ── Phase 5: Final pulse x2 ──────────────────────────
    {0x00000000, 0x00000000, 0x00000000, 60},
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 80},
    {0x00000000, 0x00000000, 0x00000000, 60},
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 60},

    // ── End marker ───────────────────────────────────────
    {0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF},
};