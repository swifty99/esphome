#pragma once

// Shared constants for LIN communication
#define LIN_MAX_DATA_SIZE 8
#define LIN_MAX_DISCOVERED_PIDS 32
#define LIN_MAX_ID_RQ_COUNT 16
#define LIN_MAX_RESPONSES 16

typedef enum { LIN_MODE_MASTER, LIN_MODE_SLAVE, LIN_MODE_LISTENER } lin_mode_t;
