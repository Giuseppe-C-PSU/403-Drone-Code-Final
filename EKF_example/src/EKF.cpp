#include "EKF.h"
#include <Arduino.h>
#include "sensors.h"
#include "sensor_prelim.h"

#include <cmath>  // For std::fabs
#include <algorithm>  // For std::swap

extern Sensors sens;
// Constructor to initialize the EKF
EKF::EKF() {
  // Initialize the state vectors derivative vectors
  //may need to add values for the initial position values
  for (int i = 0; i < STATE_DIM; i++) {
      if (i < 10){ //adding the biases to the initial value
        x_predicted[i] = 0;
        x_corrected[i] = 0;
      }
      if(i >= 10 && i < 12){
        x_predicted[i] = sens.bias.acc[i-10];
        x_corrected[i] = sens.bias.acc[i-10];
      }
      if(i>12){
        x_predicted[i] = sens.bias.gyr[i-13];
        x_corrected[i] = sens.bias.gyr[i-13];
      }
      x_dot_init[i] = 0;
      x_dot_next[i] = 0;
      
  }

  // Initialize the state covariance matrices (P_)
  for (int i = 0; i < STATE_DIM; i++) {
      for (int j = 0; j < STATE_DIM; j++) {
        if(i == j)
        {
          P_predicted[i][j] = 1;  // Identity matrix
          P_corrected[i][j] = 1;
          P_dot[i][j] = 1;
        }
        else 
        {
          P_predicted[i][j] = 0;  // Identity matrix
          P_corrected[i][j] = 0;
          P_dot[i][j] = 0;

        }
      }
  }

  // Initialize the state transition matrix (F_)
  for (int i = 0; i < STATE_DIM; i++) {
      for (int j = 0; j < STATE_DIM; j++) {
          F_[i][j] = 0; // matrix of 0s will be initialized in predict function
      }
  }

  // Initialize the process noise covariance matrix (Q_)
  for (int i = 0; i < STATE_DIM; i++) {
      for (int j = 0; j < STATE_DIM; j++) {
          Q_[i][j] = (i == j) ? 0.1 : 0;  // Diagonal with small values
      }
  }

  // Initialize the measurement matrix (H_)
  for (int i = 0; i < MEAS_DIM; i++) {
      for (int j = 0; j < STATE_DIM; j++) {
         H_[i][j] = 0;
           
      }
  }

  // Initialize the measurement noise covariance matrix (R_)
  for (int i = 0; i < MEAS_DIM; i++) {
      for (int j = 0; j < MEAS_DIM; j++) {
          R_[i][j] = (i == j) ? 1 : 0;  // Diagonal with small values
      }
  }

  // Initialize the Kalman gain matrix (K_)
  for (int i = 0; i < STATE_DIM; i++) {
      for (int j = 0; j < MEAS_DIM; j++) {
          K_[i][j] = 0;
      }
  }

}

// Prediction step
void EKF::predict( float dt) {
  //finding derivative part for state vector prediction
  xdot(x_predicted, x_dot_next);

  // Eq(10) from paper
  for (int i = 0; i < STATE_DIM; i++) {
        x_predicted[i] = x_corrected[i] + (dt/2 * (x_dot_next[i] + x_dot_init[i]));
  }

  F_finder(x_predicted, F_);

  float FP[STATE_DIM][STATE_DIM]; float PF_transpose[STATE_DIM][STATE_DIM]; float F_transpose[STATE_DIM][STATE_DIM];
  float temp[STATE_DIM][STATE_DIM];
  matrixInit(FP); matrixInit(PF_transpose); matrixInit(F_transpose); matrixInit(temp);

  matrixTranspose(F_, F_transpose);
  
  //eq(11) finding Pdot
  matrixMultiply(F_,P_predicted,FP);
  matrixMultiply(P_predicted,F_transpose,PF_transpose);
  matrixAdd(FP,PF_transpose,temp);
  //Pdot = F*P + P*F' + Q
  matrixAdd(temp, Q_, P_dot);

  // P- = P+ + dt * Pdot
  for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            P_predicted[i][j] = P_corrected[i][j] + dt * P_dot[i][j];
        }
  }   


  // making xi = xi+1 for the next iteration
  for (int i = 0; i<16; i++)
    {x_dot_init[i] = x_dot_next[i];}

  // ALL PRINTING HAPPENS BELOW HERE

  //Printing the predicted result for debugging
  Serial.print("x_predicted = ");
  vectorPrint(x_predicted);

  // Serial.print("P_predicted = ");
  // matrixPrint(P_predicted);
  
  
}

// Update step
void EKF::update(float z[MEAS_DIM]) {
    H_maker(H_);

    // Creating all the intermediate matrices needed
    float H_transpose[STATE_DIM][MEAS_DIM]; float temp_inv[MEAS_DIM][MEAS_DIM]; float HP[MEAS_DIM][STATE_DIM];
    float PH_transpose[STATE_DIM][MEAS_DIM]; float HPH[MEAS_DIM][MEAS_DIM]; float HPH_R[MEAS_DIM][MEAS_DIM];
    float temp_PH[STATE_DIM][MEAS_DIM]; float KH[STATE_DIM][STATE_DIM]; float I_KH[STATE_DIM][STATE_DIM];
    matrixInit(H_transpose); matrixInit(temp_inv); matrixInit(HP); matrixInit(PH_transpose);
    matrixInit(HPH);matrixInit(HPH_R); matrixInit(temp_PH);
    matrixInit(KH); matrixInit(I_KH);

    // Creating all intermediate vectors
    float h[MEAS_DIM]; float y[MEAS_DIM]; float Ky[STATE_DIM];
    vectInit(h, MEAS_DIM); vectInit(y, MEAS_DIM); vectInit(Ky, STATE_DIM);

    // Kalman Gain calculation
    matrixTranspose(H_, H_transpose);
    matrixMultiply(P_predicted, H_transpose, PH_transpose);
    matrixMultiply(H_, PH_transpose, HPH);
    matrixAdd(HPH, R_, HPH_R);
    matrixInverseGaussianElimination(HPH_R, temp_inv);
    matrixMultiply(PH_transpose, temp_inv, K_);
    // //printing outputs for the things
    // Serial.println("H_transpose:");
    // matrixPrint(H_transpose);
    // Serial.println("PH_transpose:");
    // matrixPrint(PH_transpose);
    // Serial.println("HPH:");
    // matrixPrint(HPH);
    // Serial.println("HPH_R:");
    // matrixPrint(HPH_R);
    // Serial.println("temp_inv:");
    // matrixPrint(temp_inv);
    // Serial.println("K:");
    // matrixPrint(K_);
   
    // Additional stability checks for Kalman Gain
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            if (abs(K_[i][j]) > 1e3 || isnan(K_[i][j])) {
                Serial.println("Numerical instability detected in Kalman Gain.");
                return;
            }
        }
    }

    

    // Measurement residual
    for (int i = 0; i < MEAS_DIM; i++) {
        h[i] = x_predicted[i];
    }

    for (int i = 0; i < 10; i++) {
        y[i] = z[i] - h[i];
    }

    // State update
    vectorMatrixMultiply(K_, y, Ky);
    for (int i = 0; i < 16; i++) {
        x_corrected[i] = x_predicted[i] + Ky[i];
    }

    // Covariance update
    matrixMultiply(K_, H_, KH);

    // Ensure KH values are within a reasonable range before subtraction
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            if (KH[i][j] < -1e3 || KH[i][j] > 1e3) {
                Serial.println("Numerical instability detected in KH matrix.");
                return;
            }
        }
    }

    matrixSubtract(I, KH, I_KH);
    matrixMultiply(I_KH, P_predicted, P_corrected);
    

    // ALL PRINTING HAPPENS BELOW HERE
    // Print updated covariance matrix
    // Serial.print("y = ");
    // vectorPrint(y);

    // Serial.println("P_corrected:");
    // matrixPrint(P_corrected);

    // Print updated state vector
    Serial.println("x_corrected:");
    vectorPrint(x_corrected);

    


}



// Print current state
void EKF::printState() {
    Serial.print("x_corrected = ");
    for (int i = 0; i < STATE_DIM; i++) {
        Serial.print(x_corrected[i]);
        Serial.print(" ");
    }
    Serial.println();
}

//may need to change how quaternions are gotten idk yet
void EKF::xdot(float X[STATE_DIM] , float x_dot[STATE_DIM] )
{
  //creating the variables
  float w_bias[3]; float acc_bias[3]; float acc_IMU[3]; float w_IMU[3]; float q[4];
  float grav[3] = { 0, 0, -9.81}; float qdot[4]; float vel[3]; float vdot[3];  
  //making all vectors zero vectors
  vectInit(w_bias, 3); vectInit(acc_bias, 3); vectInit(acc_IMU, 3); vectInit(w_IMU, 3); vectInit(q, 4);
  vectInit(qdot, 4); vectInit(vel, 3); vectInit(vdot, 3);

  //initializes variables
  for(int i = 0; i<3; i++)
  {
    w_bias[i] = X[i+13];
    acc_bias[i] = X[i+10];
    acc_IMU[i] = sens.data.acc[i];
    w_IMU[i] = sens.data.gyr[i];
    
  }
  for(int i = 0; i < 4; i++){
    //is q from sensor or from state vector model
    //q[i] = sens.data.quat[i];
    q[i] = X[i];
  }

  float A[4][4] = { 
        {0, -(w_IMU[0] - w_bias[0]), -(w_IMU[1] - w_bias[1]), -(w_IMU[2] - w_bias[2])},
        {w_IMU[0] - w_bias[0], 0, w_IMU[2] - w_bias[2], -(w_IMU[1] - w_bias[1])},
        {w_IMU[1] - w_bias[0], -(w_IMU[2] - w_bias[2]), 0, w_IMU[0] - w_bias[0]},
        {w_IMU[2] - w_bias[2], w_IMU[1] - w_bias[1], -(w_IMU[0] - w_bias[0]), 0}
  };

  float B[4][4]; 

  matrixInit(B);
  // for(int i = 0; i < 4; i++)
  // {
  //   for(int j = 0; j < 4; j++)
  //   {
  //     Serial.print(B[i][j]);
  //     Serial.print(", ");
  //   }
  //   Serial.println();
  // }
  //for q dot, multiplies A by 0.5 to get B
  for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            B[i][j] = 0.5 * A[i][j];
        }
  }

  //qdot = B*q = 0.5*A*q
 for (int i = 0; i < 4; i++) {
        qdot[i] = 0;
        for (int j = 0; j < 4; j++) {
            qdot[i] += B[i][j] * q[j];
        }
 }

  vel[0] = X[7]; 
  vel[1] = X[8];
  vel[2] = X[9];
  
 float Tb_i[3][3] = {
                    {1-2*( sq(q[2]) + sq(q[3]) ), 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2])},
                    {2*(q[1]*q[2] + q[0]*q[3]), 1-2*( sq(q[1]) + sq(q[3]) ), 2*(q[2]*q[3] - q[0]*q[1])},
                    {2*(q[1]*q[3] - q[0]*q[2]), 2*(q[2]*q[3] - q[0]*q[1]), 1-2*( sq(q[1]) + sq(q[2]) ) }
  };

  //acc_IMU - acc_bias
  float acc_no_bias[3] = {0, 0, 0}; //intialiazing the vector to 0s 
  for (int i = 0; i < 3; i++) {
        acc_no_bias[i] = acc_IMU[i] - acc_bias[i];
 }

  // Tb_i * acc_no_bias = Tb_i * (accIMU - ba) 
  float result[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            result[i] += Tb_i[i][j] * acc_no_bias[j];
        }
  }

  // result + g= acceleration
  for (int i = 0; i < 4; i++) {
        vdot[i] = result[i] + grav[i];
 }

  //assigning all values to the xdot vector
  for (int i = 0; i < STATE_DIM; i++)
  
  x_dot[0] = qdot[0];
  x_dot[1] = qdot[1];
  x_dot[2] = qdot[2];
  x_dot[3] = qdot[3];
  x_dot[4] = vel[0];
  x_dot[5] = vel[1];
  x_dot[6] = vel[2];
  x_dot[7] = vdot[0];
  x_dot[8] = vdot[1];
  x_dot[9] = vdot[2];
  x_dot[10] = 0;
  x_dot[11] = 0;
  x_dot[12] = 0;
  x_dot[13] = 0;
  x_dot[14] = 0;
  x_dot[15] = 0; 
  


}

void EKF::F_finder(float X[STATE_DIM], float F[STATE_DIM][STATE_DIM]){

 float w_bias[3]; float acc_bias[3]; float acc_IMU[3]; float w_IMU[3]; float q[4];  
 float a[3]; // acceleration accounting for biases
  //initializes variables
  for(int i = 0; i<3; i++)
  {
    w_bias[i] = X[i+13];
    acc_bias[i] = X[i+10];
    acc_IMU[i] = sens.data.acc[i];
    w_IMU[i] = sens.data.gyr[i];
    a[i] = acc_bias[i] - acc_IMU[i]; // a = ba - aIMU
  }
  for(int i = 0; i < 4; i++){//is q from sensor or from state vector model?????
    //q[i] = sens.data.quat[i];
    q[i] = X[i];
  }


  // multiplying by the 1/2 in the for loop
    float F11[4][4] = {
    {0, w_bias[0] - w_IMU[0], w_bias[1] - w_IMU[1], w_bias[2] - w_IMU[2]},
    {-w_bias[0] + w_IMU[0], 0, -w_bias[2] + w_IMU[2], -w_bias[1] + w_IMU[1]},
    {-w_bias[1] + w_IMU[1], -w_bias[2] + w_IMU[2], 0, -w_bias[0] + w_IMU[0]},
    {-w_bias[2] + w_IMU[2], -w_bias[1] + w_IMU[1], -w_bias[0] + w_IMU[0], 0}
  };
  // multiplying by the 1/2 in the for loop
  float F15[4][3] = {
    {q[1], q[2], q[3]},
    {-q[0], q[3], -q[2]},
    {-q[3], -q[0], q[1]},
    {q[2], -q[1], -q[0]}
  };
  float F31[3][4] =  {
    {2*(q[3]*a[1] - q[2]*a[2]), 2*(q[2]*a[1] + q[3]*a[2]), 2*(2*q[2]*a[0] - q[1]*a[1] - q[0]*a[2]), 2*(2*q[3]*a[0] + q[0]*a[1] - q[1]*a[2])    },
    {2*(q[1]*a[2] - q[3]*a[0]), 2*(2*q[1]*a[1] - q[2]*a[0] + q[0]*a[2]), -2*(q[1]*a[0] + q[3]*a[2]), 2*(2*q[3]*a[2] - q[0]*a[0] - q[2]*a[2])   },
    {2*(q[2]*a[0] - q[1]*a[1]), 2*(2*q[1]*a[2] - q[3]*a[0] - q[0]*a[1]), 2*(2*q[2]*a[2] - q[3]*a[1] + q[0]*a[0]), -2*(q[1]*a[0] + q[2]*a[1])   }
  };
  float F34[3][3]=  {
    {-1+2*(sq(q[2]) + sq(q[3])), -2*(q[1]*q[2] - q[0]*q[3]), -2*(q[1]*q[3] + q[0]*q[2]) },
    {-2*(q[0]*q[3] + q[1]*q[2]), -1+2*(sq(q[1]) + sq(q[3])), -2*(q[2]*q[3] - q[0]*q[1]) },
    {-2*(q[1]*q[3] + q[0]*q[2]), -2*(q[0]*q[1] + q[2]*q[3]), -1+2*(sq(q[1]) + sq(q[2])) },
  };

  //rows 0-3, columns 0-3
  //F11 is in F  4x4
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      F[i][j] = 0.5 * F11[i][j];
    }
  }

  //F15 is in F 4x3
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      F[i][j+13] = 0.5 * F15[i][j];
    }
  }

  //F31 is in F 3x4
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      F[i+7][j] = 0.5 * F31[i][j];
    }
  }

  //F34 is in F
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      F[i+7][j+10] = F34[i][j];
    }
  }
  // I is in F
  float I[3][3]= {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      F[i+4][j+7] = I[i][j];;
    }
  }

  //for testing
  // Serial.println("F: = ");
  //     for (int i = 0; i < 16; i++) {
  //         for (int j = 0; j < 16; j++) {
  //             Serial.print(F[i][j]);
  //             Serial.print(", ");
  //         }
  //         Serial.println();
  //     }



}

void EKF::H_maker(float H[MEAS_DIM][STATE_DIM])
{
  for(int i = 0; i < MEAS_DIM; i++)
  {
    for(int j = 0; j < STATE_DIM; j++)
    {
      if(i == j && i < 7){
        H[i][j] = 1;
      }
    }
  }
  
  

  // Serial.println("H =");
  // for(int i = 0; i < MEAS_DIM; i++)
  // {
  //   for(int j = 0; j < STATE_DIM; j++)
  //   {
  //       Serial.print(H[i][j]);
  //       Serial.print(", ");
  //   }
  //   Serial.println();
  // }



}


//initializes vectors and matrices with 0;
template <int na, int ma>
void EKF::matrixInit(float (&a)[na][ma])
{
  for (int i = 0; i < na; i++)
  {
    for (int j = 0; j < ma; j++)
    {
      a[i][j] = 0;
    }
  }
}

void EKF::vectInit(float* vect, int size)
{
  for(int i = 0; i < size; i++)
  {
    vect[i] = 0;
  }

}

// MATRIX OPERATIONS BELOW
//n is rows, m is columns a and b correspond to the matrix being multiplied
template <int na, int ma, int nb, int mb>
void EKF::matrixMultiply(float (&a)[na][ma], float (&b)[nb][mb], float (&result)[na][mb]) {
    
  for (int i = 0; i < na; i++) {
        for (int j = 0; j < mb; j++) {
            result[i][j] = 0;
            for (int k = 0; k < ma; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

template <int na, int ma, int nb, int mb>
void EKF::matrixAdd(float (&a)[na][ma], float (&b)[nb][mb], float (&result)[na][mb]){
  for (int i = 0; i < na; i++)
  {
    for (int j = 0; j < mb; j++)
    {
      result[i][j] = a[i][j] + b[i][j];
    }
  }
}

template <int na, int ma, int nb, int mb>
void EKF::matrixSubtract(float (&a)[na][ma], float (&b)[nb][mb], float (&result)[na][mb]){
  for (int i = 0; i < na; i++)
  {
    for (int j = 0; j < mb; j++)
    {
      result[i][j] = a[i][j] - b[i][j];
    }
  }
}

template <int n>
bool EKF::matrixInverseGaussianElimination(float (&a)[n][n], float (&result)[n][n]) {
    // Augment matrix 'a' with identity matrix
    float augmented[n][2 * n];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            augmented[i][j] = a[i][j];
            augmented[i][j + n] = (i == j) ? 1.0f : 0.0f;  // Identity matrix
        }
    }

    // Perform Gaussian Elimination
    for (int i = 0; i < n; i++) {
        // Find the pivot row (partial pivoting)
        int maxRow = i;
        for (int j = i + 1; j < n; j++) {
            if (fabs(augmented[j][i]) > fabs(augmented[maxRow][i])) {
                maxRow = j;
            }
        }

        // Swap the rows if needed
        if (i != maxRow) {
            for (int j = 0; j < 2 * n; j++) {
                std::swap(augmented[i][j], augmented[maxRow][j]);
            }
        }

        // Make the pivot element 1 and eliminate column below it
        float pivot = augmented[i][i];
        if (fabs(pivot) < 1e-6) {
            Serial.println("Matrix is singular, can't compute inverse!");
            return false;  // Singular matrix, no inverse
        }

        // Normalize the pivot row
        for (int j = 0; j < 2 * n; j++) {
            augmented[i][j] /= pivot;
        }

        // Eliminate all other rows
        for (int j = 0; j < n; j++) {
            if (j != i) {
                float factor = augmented[j][i];
                for (int k = 0; k < 2 * n; k++) {
                    augmented[j][k] -= augmented[i][k] * factor;
                }
            }
        }
    }

    // Extract the inverse matrix from the augmented matrix
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result[i][j] = augmented[i][j + n];  // The right half is the inverse
        }
    }

    return true;
}


template <int na, int ma>
void EKF::matrixTranspose(float (&a)[na][ma], float (&result)[ma][na])
{
  for (int i = 0; i < na; i++)
  {
    for (int j = 0; j < ma; j++)
    {
      result[j][i] = a[i][j];
    }
  }
}

template <int na, int ma, int vb>
void EKF::vectorMatrixMultiply(float (&a)[na][ma], float (&b)[vb], float (&result)[na]){
  for (int i = 0; i < na; i++)
  {
    result[i] = 0;
    for (int j = 0; j < ma; j++)
    {
      result[i] += a[i][j] * b[j];
    }
  }
}


template <int na, int ma>
void EKF::matrixPrint(float (&a)[na][ma]) {  
  for (int i = 0; i < na; i++) {
        for (int j = 0; j < ma; j++) {
            Serial.print(a[i][j]);
            Serial.print(", ");
        }
        Serial.println();
    }
}

template <int na>
void EKF::vectorPrint(float (&a)[na]) {
    for (int i = 0; i < na; i++) {
        Serial.print(a[i]);
        Serial.print(", ");
    }
    Serial.println();
}


