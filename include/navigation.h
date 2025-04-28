#ifndef NAVIGATION_H
#define NAVIGATION_H

class nav {
private:
    // Navigation parameters
    float targetPos[3] = {0,0,0}; // Target position
    float currentPos[3] = {0,0,0}; // Current position
    float tolerance = 0.1; // Tolerance for reaching the target
    float initialPos[3] = {0,0,0}; // Initial position
    bool atTarget = false; // Flag to check  tif target is reached
    bool fireAlreadyDetected = false; // Flago check if fire is already detected
    float delta_x;
    float delta_y;
    float delta_z;
    void initializeNavigation();
    void updatePosition();
    void setTarget(float targetPos[3]);
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
    X+      |                 |
            |               X |
    Y+ -->  |     E     H     |
            |                 |
            |                 |
            |  A  (0,I,1)  D  |

            "Front" (where net is)
        */

        // These are in ft, and just guesses for now:
        {0, 0, 5},   // Waypoint 0 (Takeoff)
        {0, -5, 5},  // Waypoint A
        {15, -5, 5}, // Waypoint B
        {15, 5, 5},  // Waypoint C
        {0, 5, 5},   // Waypoint D
        {5, -2, 5},  // Waypoint E
        {10, -2, 5}, // Waypoint F
        {10, 2, 5},  // Waypoint G
        {5, 2, 5},   // Waypoint H
        {0, 0, 5},   // Waypoint I 
        {0, 0, 5},   // Waypoint iota (Above fire)
        {0, 0, 3},   // Waypoint X (Fire)
        {0, 0, 0}    // Waypoint 1 (Land)
    };

public:
    nav() {
        initializeNavigation();
    }
    ~nav() {
        // Destructor
    }

};





#endif 