#ifndef COMMON_H
#define COMMON_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// CONSTANTS AND LIMITS

// Grid dimensions (can be overridden by config)
#define MAX_GRID_X 50
#define MAX_GRID_Y 50
#define MAX_GRID_Z 10

// Population and GA limits
#define MAX_POPULATION 200
#define MAX_GENERATIONS 1000
#define MAX_ROBOTS 10
#define MAX_PATH_LENGTH 500

// Maximum moves in a chromosome (action sequence)
#define MAX_MOVES 300

// Worker process limits
#define MAX_WORKERS 16

// Cell types
#define CELL_EMPTY 0
#define CELL_OBSTACLE 1

// Risk levels (computed from heat)
#define RISK_NONE 0
#define RISK_LOW 1
#define RISK_MEDIUM 2
#define RISK_HIGH 3
#define RISK_CRITICAL 4

// Robot status
#define ROBOT_IDLE 0
#define ROBOT_MOVING 1
#define ROBOT_RESCUING 2
#define ROBOT_RETURNING 3
#define ROBOT_STUCK 4

// Sensor thresholds for survivor detection
#define CO2_THRESHOLD_LOW 600.0     // Possible survivor nearby
#define CO2_THRESHOLD_HIGH 800.0    // Definite survivor
#define CO2_AMBIENT 400.0           // Normal ambient CO2
#define CO2_SURVIVOR_SOURCE 1000.0  // CO2 at survivor location
#define CO2_DIFFUSION_RATE 0.3      // How much CO2 spreads to neighbors

// Heat thresholds for risk assessment
#define HEAT_SAFE 40.0      // Below this: safe
#define HEAT_WARM 60.0      // 40-60: warm (low risk)
#define HEAT_HOT 80.0       // 60-80: hot (medium risk)
#define HEAT_VERY_HOT 100.0 // 80-100: very hot (high risk)
#define HEAT_EXTREME 120.0  // Above 100: extreme (critical risk/impassable)

// DIRECTION ENUM (26 possible moves in 3D space)

typedef enum
{
    // Horizontal movements (8 directions, z=0)
    NORTH = 0,      // (0, +1, 0)
    SOUTH,          // (0, -1, 0)
    EAST,           // (+1, 0, 0)
    WEST,           // (-1, 0, 0)
    NE,             // (+1, +1, 0) - Northeast
    NW,             // (-1, +1, 0) - Northwest
    SE,             // (+1, -1, 0) - Southeast
    SW,             // (-1, -1, 0) - Southwest

    // Vertical movements (2 directions)
    UP,             // (0, 0, +1)
    DOWN,           // (0, 0, -1)

    // Diagonal-vertical movements (16 directions)
    NORTH_UP,       // (0, +1, +1)
    NORTH_DOWN,     // (0, +1, -1)
    SOUTH_UP,       // (0, -1, +1)
    SOUTH_DOWN,     // (0, -1, -1)
    EAST_UP,        // (+1, 0, +1)
    EAST_DOWN,      // (+1, 0, -1)
    WEST_UP,        // (-1, 0, +1)
    WEST_DOWN,      // (-1, 0, -1)
    NE_UP,          // (+1, +1, +1)
    NE_DOWN,        // (+1, +1, -1)
    NW_UP,          // (-1, +1, +1)
    NW_DOWN,        // (-1, +1, -1)
    SE_UP,          // (+1, -1, +1)
    SE_DOWN,        // (+1, -1, -1)
    SW_UP,          // (-1, -1, +1)
    SW_DOWN         // (-1, -1, -1)
} Direction;

#define NUM_DIRECTIONS 26

// CORE DATA STRUCTURES

// 3D coordinate/vector
typedef struct
{
    int x;
    int y;
    int z;
} Vec3;

// Grid cell information
typedef struct
{
    int type;         // CELL_EMPTY or CELL_OBSTACLE
    float heat;       // Heat sensor reading in °C
    float co2;        // CO2 sensor reading in ppm
    int risk;         // Risk level (0-4) - computed from heat
    int has_survivor; // 1 if survivor detected - computed from co2
    int visited;      // For path tracking (0 or 1)
    int survivor_helped; // 1 if survivor already received supplies (multi-trip feature)
} Cell;

// Robot structure (with action-based path)
typedef struct
{
    int id;                     // Robot identifier
    Vec3 start_pos;             // Starting position
    Vec3 current_pos;           // Current position

    // Action sequence (genes)
    Direction moves[MAX_MOVES]; // Sequence of directions to move
    int num_moves;              // Number of moves in the sequence

    // Generated path (decoded from moves, not part of genes)
    Vec3 path[MAX_PATH_LENGTH]; // Path as sequence of coordinates
    int path_length;            // Number of steps in path

    int status;            // ROBOT_IDLE, ROBOT_MOVING, etc.
    int survivors_rescued; // Count of survivors this robot rescued
    float time_taken;      // Time taken to complete path
    
    // Multi-trip supply delivery fields
    int has_supplies;      // 0 or 1 (can only carry supplies for ONE survivor)
    int trips_completed;   // Number of successful delivery trips
    Vec3 entry_point;      // Entry/exit point (typically 0,0,0)
    
    // Fog of war fields (obstacle discovery)
    int discovered_obstacles[MAX_PATH_LENGTH];  // Indices of discovered obstacle cells
    int num_discovered;                         // Count of discovered obstacles
} Robot;

// Chromosome (represents solution: paths for all robots)
typedef struct
{
    Robot robots[MAX_ROBOTS]; // Array of robots with their paths
    int num_robots;           // Number of robots in this chromosome
    double fitness;           // Fitness score
    int survivors_reached;    // Total survivors reached
    int cells_covered;        // Total unique cells covered
    double total_path_length; // Sum of all robot path lengths (Euclidean)
    double total_risk;        // Sum of risks encountered
    double total_time;        // Total time taken
} Chromosome;

// VEC3 UTILITY FUNCTIONS

// Compare two Vec3 positions for equality
static inline int vec3_equals(Vec3 a, Vec3 b)
{
    return (a.x == b.x && a.y == b.y && a.z == b.z);
}

// DIRECTION UTILITY FUNCTIONS

// Convert direction to movement delta
static inline Vec3 direction_to_delta(Direction dir)
{
    switch (dir)
    {
    // Horizontal (z=0)
    case NORTH:      return (Vec3){0, 1, 0};
    case SOUTH:      return (Vec3){0, -1, 0};
    case EAST:       return (Vec3){1, 0, 0};
    case WEST:       return (Vec3){-1, 0, 0};
    case NE:         return (Vec3){1, 1, 0};
    case NW:         return (Vec3){-1, 1, 0};
    case SE:         return (Vec3){1, -1, 0};
    case SW:         return (Vec3){-1, -1, 0};

    // Vertical
    case UP:         return (Vec3){0, 0, 1};
    case DOWN:       return (Vec3){0, 0, -1};

    // Diagonal-vertical
    case NORTH_UP:   return (Vec3){0, 1, 1};
    case NORTH_DOWN: return (Vec3){0, 1, -1};
    case SOUTH_UP:   return (Vec3){0, -1, 1};
    case SOUTH_DOWN: return (Vec3){0, -1, -1};
    case EAST_UP:    return (Vec3){1, 0, 1};
    case EAST_DOWN:  return (Vec3){1, 0, -1};
    case WEST_UP:    return (Vec3){-1, 0, 1};
    case WEST_DOWN:  return (Vec3){-1, 0, -1};
    case NE_UP:      return (Vec3){1, 1, 1};
    case NE_DOWN:    return (Vec3){1, 1, -1};
    case NW_UP:      return (Vec3){-1, 1, 1};
    case NW_DOWN:    return (Vec3){-1, 1, -1};
    case SE_UP:      return (Vec3){1, -1, 1};
    case SE_DOWN:    return (Vec3){1, -1, -1};
    case SW_UP:      return (Vec3){-1, -1, 1};
    case SW_DOWN:    return (Vec3){-1, -1, -1};

    default:         return (Vec3){0, 0, 0};
    }
}

// Get direction name (for debugging/display)
static inline const char *direction_name(Direction dir)
{
    switch (dir)
    {
    case NORTH:      return "N";
    case SOUTH:      return "S";
    case EAST:       return "E";
    case WEST:       return "W";
    case NE:         return "NE";
    case NW:         return "NW";
    case SE:         return "SE";
    case SW:         return "SW";
    case UP:         return "UP";
    case DOWN:       return "DOWN";
    case NORTH_UP:   return "N_UP";
    case NORTH_DOWN: return "N_DN";
    case SOUTH_UP:   return "S_UP";
    case SOUTH_DOWN: return "S_DN";
    case EAST_UP:    return "E_UP";
    case EAST_DOWN:  return "E_DN";
    case WEST_UP:    return "W_UP";
    case WEST_DOWN:  return "W_DN";
    case NE_UP:      return "NE_UP";
    case NE_DOWN:    return "NE_DN";
    case NW_UP:      return "NW_UP";
    case NW_DOWN:    return "NW_DN";
    case SE_UP:      return "SE_UP";
    case SE_DOWN:    return "SE_DN";
    case SW_UP:      return "SW_UP";
    case SW_DOWN:    return "SW_DN";
    default:         return "???";
    }
}

// UTILITY FUNCTIONS (inline for performance)

// Convert 3D coordinates to 1D array index
static inline int coord_to_index(int x, int y, int z, int grid_x, int grid_y)
{
    return x + y * grid_x + z * grid_x * grid_y;
}

// Convert 1D index back to 3D coordinates
static inline Vec3 index_to_coord(int index, int grid_x, int grid_y)
{
    Vec3 pos;
    pos.z = index / (grid_x * grid_y);
    int remainder = index % (grid_x * grid_y);
    pos.y = remainder / grid_x;
    pos.x = remainder % grid_x;
    return pos;
}

// Check if two positions are equal
static inline int vec3_equal(Vec3 a, Vec3 b)
{
    return (a.x == b.x && a.y == b.y && a.z == b.z);
}

// Calculate Euclidean distance between two positions
static inline double euclidean_distance(Vec3 a, Vec3 b)
{
    int dx = b.x - a.x;
    int dy = b.y - a.y;
    int dz = b.z - a.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Calculate Manhattan distance between two positions
static inline int manhattan_distance(Vec3 a, Vec3 b)
{
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
}

// Check if position is within grid bounds
static inline int is_within_bounds(Vec3 pos, int grid_x, int grid_y, int grid_z)
{
    return (pos.x >= 0 && pos.x < grid_x &&
            pos.y >= 0 && pos.y < grid_y &&
            pos.z >= 0 && pos.z < grid_z);
}

// Check if two positions are adjacent (including diagonals in 3D - 26 neighbors)
static inline int are_adjacent(Vec3 a, Vec3 b)
{
    int dx = abs(b.x - a.x);
    int dy = abs(b.y - a.y);
    int dz = abs(b.z - a.z);

    // Adjacent if all deltas are 0 or 1, and at least one is non-zero
    return (dx <= 1 && dy <= 1 && dz <= 1) && (dx + dy + dz > 0);
}

// Compute risk level from heat
static inline int compute_risk_from_heat(float heat)
{
    if (heat < HEAT_SAFE)
        return RISK_NONE;
    if (heat < HEAT_WARM)
        return RISK_LOW;
    if (heat < HEAT_HOT)
        return RISK_MEDIUM;
    if (heat < HEAT_VERY_HOT)
        return RISK_HIGH;
    return RISK_CRITICAL;
}

// Detect survivor from CO2 level
static inline int detect_survivor_from_co2(float co2)
{
    return (co2 >= CO2_THRESHOLD_HIGH) ? 1 : 0;
}

// Calculate total path length using Euclidean distance
static inline double calculate_path_length(Vec3 *path, int path_length)
{
    if (path_length < 2)
        return 0.0;

    double total_length = 0.0;
    for (int i = 0; i < path_length - 1; i++)
    {
        total_length += euclidean_distance(path[i], path[i + 1]);
    }
    return total_length;
}

#endif // COMMON_H