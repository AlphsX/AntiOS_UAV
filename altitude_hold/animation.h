/*
 * animation.h -- AltiOS UAV Boot Animation
 * Arduino UNO R4 WiFi -- LED Matrix 12x8
 */

#include <stdint.h>

const unsigned long frames[][4] = {

    // ── Phase 1: Center dot ──────────────────────────────
    // Single dot at center (col 5-6, row 3-4)
    {0x00000000, 0x00060000, 0x00000000, 120},

    // ── Phase 2: Diamond expand ──────────────────────────
    // Small cross
    {0x00000000, 0x00060000, 0x00000060, 80},
    // Cross grows
    {0x00000060, 0x00060000, 0x00000060, 80},
    // Wider cross
    {0x00000060, 0x0FF60FF0, 0x00000060, 80},
    // Fill center block
    {0x00000FF0, 0x0FF60FF0, 0x0FF00000, 80},

    // ── Phase 3: Sweep outward ───────────────────────────
    // Expand to full width center rows
    {0x00000FF0, 0xFFF60FF0, 0x0FF00000, 60},
    // Add top and bottom rows
    {0x0FF00FF0, 0xFFF6FFF0, 0xFFF00FF0, 60},
    // Nearly full
    {0xFFF00FF0, 0xFFFFFFF0, 0xFFF0FFF0, 60},
    // All on
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 180},

    // ── Phase 4: Wipe from edges to center ───────────────
    // Remove outer columns (col 0 and 11)
    {0x7FF7FF00, 0xFFFFFF00, 0x7FF7FF00, 50},
    // Remove next columns
    {0x3FE3FE00, 0xFFFFFE00, 0x3FE3FE00, 50},
    // Shrink more
    {0x1FC1FC00, 0x7FFFC00, 0x1FC1FC00, 50},
    // Center band remains
    {0x0F80F800, 0x3FFF8000, 0x0F80F800, 50},
    // Thin center
    {0x07007000, 0x1FF70000, 0x07007000, 50},
    // Just center dot again
    {0x00000000, 0x00060000, 0x00000000, 80},

    // ── Phase 5: Final pulse x2 ──────────────────────────
    {0x00000000, 0x00000000, 0x00000000, 60},
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 80},
    {0x00000000, 0x00000000, 0x00000000, 60},
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 60},

    // ── End marker ───────────────────────────────────────
    {0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF},

};