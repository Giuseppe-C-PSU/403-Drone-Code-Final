#ifndef NAVIGATION_H
#define NAVIGATION_H

class Nav {
public:
    // Navigation parameters
    float targetPos[3] = {0,0,0}; // Target position
    float currentPos[3] = {0,0,0}; // Current position
    float tolerance = 0.5; // Tolerance for reaching the target
    float initialPos[3] = {0,0,0}; // Initial position
    bool atTarget = false; // Flag to check  tif target is reached
    bool fireAlreadyDetected = false; // Flago check if fire is already detected
    int waypointIndex = 0; // Index of the current waypoint

    float waypoints[13][3] = {
        // Grid is describes as such:
        /*
                back wall

            |  B           C  |
            |                 |
            |                 |
    ^       |     F     G     |
    |       |                 |
    X+      |        C        |
            |               X |
    Y+ -->  |     E     H     |
            |                 |
            |                 |
            |  A  (0,I,1)  D  |

            "Front" (where net is)
        */

        // These are in ft, and just guesses for now:
        {-2.7, 0, 4},   // Waypoint 0 (Takeoff)
        {-10, 5, 4},   // Waypoint A 
        {10, 5, 4},  // Waypoint B
        {10, -5, 4}, // Waypoint C
        {-10, -5, 4},  // Waypoint D
        {-5, 2, 4},   // Waypoint E
        {5, 2, 4},  // Waypoint F
        {5, -2, 4}, // Waypoint G
        {-5, -2, 4},  // Waypoint H
        {0, 0, 4},   // Waypoint iota (Above fire)
        {0, 0, 3},   // Waypoint X (Fire)
        {0, 0, 4},   // Waypoint I 
        {0, 0, 4}    // Waypoint 1 (Land)
    };

    float delta_x;
    float delta_y;
    float delta_z;
    
    void initializeNavigation();
    void updatePosition();
    void setTarget(float targetPos[3]);
};

#endif 