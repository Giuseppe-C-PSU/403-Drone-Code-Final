// This is for the nagation for the drone itself
#include "datalink.h"
#include "navigation.h"
#include "thermal.h"

extern Thermal therm;

struct datalinkMessageOptitrack_ref* mocap = &obDatalinkMessageOptitrack;

void nav::initializeNavigation(){
    // Initialize navigation parameters
    initialPos[0] = mocap->pos_x;
    initialPos[1] = mocap->pos_y;
    initialPos[2] = mocap->pos_z; 
}

void nav::setTarget(float targetPos[3]){
    this->targetPos[0] = targetPos[0];
    this->targetPos[1] = targetPos[1];
    this->targetPos[2] = targetPos[2];
    atTarget = false; // Reset the target reached flag
    // Serial.print("Target set to: ");
    // Serial.print(targetPos[0]);
    // Serial.print(", ");
    // Serial.print(targetPos[1]);
    // Serial.print(", ");
    // Serial.println(targetPos[2]);
}

void nav::updatePosition() {
    currentPos[0] = mocap->pos_x;
    currentPos[1] = mocap->pos_y;
    currentPos[2] = mocap->pos_z;

    delta_x = abs(targetPos[0] - currentPos[0]);
    delta_y = abs(targetPos[1] - currentPos[1]);
    delta_z = abs(targetPos[2] - currentPos[2]);
    
    // Serial.print("Delta X: ");
    // Serial.print(delta_x);
    // Serial.print(", Delta Y: ");
    // Serial.print(delta_y);
    // Serial.print(", Delta Z: ");
    // Serial.println(delta_z);

    if(therm.fireDetected){
        if(fireAlreadyDetected){
            return; // Fire already detected, do nothing
        } else {
            waypoints[11][0] = currentPos[0];
            waypoints[11][1] = currentPos[1];
            waypoints[12][0] = currentPos[0];
            waypoints[12][1] = currentPos[1];
            setTarget(waypoints[12]);
            // Serial.println("Fire detected, setting target to waypoint iota.");
            fireAlreadyDetected = true;
        }
        
    }



    if (delta_x < tolerance && delta_y < tolerance && delta_z < tolerance) {
        // Target reached
        // Serial.println("Target reached!");
        atTarget = true;
    } 

    if (atTarget)
    {
        if(waypointIndex == 12){
            therm.deploySuppressant = true; 
        }
        waypointIndex++;
        setTarget(waypoints[waypointIndex]);
        // Serial.print("Moving to waypoint");
    }
    
}


