#ifndef GCS_COMM_H
#define GCS_COMM_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} gcs_rgb_t;

typedef struct {
    uint8_t motor_speed;  // 0-100
    gcs_rgb_t rgb;
} gcs_command_t;

/**
 * Initialize GCS communication
 */
void gcs_comm_init(void);

/**
 * Start GCS communication task
 */
void gcs_comm_start(void);

/**
 * Process RGB color and map to discrete LEDs
 */
void gcs_process_rgb(gcs_rgb_t rgb);

/**
 * Apply motor speed to all motors
 */
void gcs_apply_motor_speed(uint8_t speed_percent);

#endif // GCS_COMM_H