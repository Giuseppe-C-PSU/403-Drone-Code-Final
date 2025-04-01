#ifndef EKF_H
#define EKF_H

// You can change the dimensions here
#define STATE_DIM 16  // Number of states
#define MEAS_DIM 7   // Number of measurements

//template <int na, int ma, int nb, int mb>
class EKF {
public:
    EKF();  // Constructor to initialize the EKF

    void predict( float dt);  // Prediction step
    void update(float z[MEAS_DIM]);  // Update step


    // State Vector
    float x_predicted[STATE_DIM]; // [q, x, v, ba, bw]
    float x_corrected[STATE_DIM];
    // State Covariance Matrix (P_)
    float P_predicted[STATE_DIM][STATE_DIM];
    float P_corrected[STATE_DIM][STATE_DIM];
    float P_dot[STATE_DIM][STATE_DIM];
    // State Transition Matrix (F_)
    float F_[STATE_DIM][STATE_DIM];

    // Process Noise Covariance Matrix (Q_)
    float Q_[STATE_DIM][STATE_DIM];

    // Measurement Matrix (H_)
    float H_[MEAS_DIM][STATE_DIM];

    // Measurement Noise Covariance Matrix (R_)
    float R_[MEAS_DIM][MEAS_DIM];

    // Kalman Gain Matrix (K_)
    float K_[STATE_DIM][MEAS_DIM];

    // derivate vectors 
    float x_dot_init[STATE_DIM];
    float x_dot_next[STATE_DIM];

    //identity matrix
    float I[STATE_DIM][STATE_DIM] = {
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}
    };


    //PREDICTION STEP FUNCTIONS    
    void xdot(float X[STATE_DIM] , float x_dot[STATE_DIM] );
    void F_finder(float X[STATE_DIM], float F[STATE_DIM][STATE_DIM]);

    // UPDATE STEP FUNCTIONS
    void H_maker(float H[MEAS_DIM][STATE_DIM]);
      
    //INITIALIZATION FUNCTIONS
    template <int na, int ma>
    void matrixInit(float (&a)[na][ma]);
    void vectInit(float* vect, int size);

    // MATRIX OPERATIONS BELOW
    template <int na, int ma, int nb, int mb>
    void matrixMultiply(float (&a)[na][ma], float (&b)[nb][mb], float (&result)[na][mb]);
    
    template <int na, int ma, int nb, int mb>
    void matrixAdd(float (&a)[na][ma], float (&b)[nb][mb], float (&result)[na][mb]);

    template <int na, int ma, int nb, int mb>
    void matrixSubtract(float (&a)[na][ma], float (&b)[nb][mb], float (&result)[na][mb]);

    template <int n>
    bool matrixInverseGaussianElimination(float (&a)[n][n], float (&result)[n][n]);

    template <int na, int ma>
    void matrixTranspose(float (&a)[na][ma], float (&result)[ma][na]);

    template <int na, int ma, int vb>
    void vectorMatrixMultiply(float (&a)[na][ma], float (&b)[vb], float (&result)[na]);

    template<int na>
    void normalize(float (&a)[na], float (&result)[na]);

    template <int na, int ma>
    void matrixPrint(float (&a)[na][ma]);

    template <int na>
    void vectorPrint(float (&a)[na]);
};

#endif  // EKF_H