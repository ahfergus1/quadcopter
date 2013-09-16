/*============================================================================*/
//   IMU_controller.ino - Lawrence Cushman and Andrew Simpson, 2013.           /
//                      - github: lawrencecushman                              /
/*============================================================================*/


#include "L3G4200D.h"
#include "HMC5883L.h"
#include "ADXL345.h"

#include <MatrixMath.h>
#include <Wire.h>

L3G4200D gyro;
HMC5883L magnetometer;
ADXL345  accelerometer;

long timer = 0;

#define N 15 // Number of Dimensions of the state vector
#define M 9  // DoF for the IMU (3x3-axis sensors)
#define L 4  // Number of control outputs (4 motors)

float dt = 0.01; // delta Time (seconds) //TODO: implement actual dt

/*===== initialize Kalman variables/ constants =====*/
float x[N] = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0}; // The State's initial (horrible) estimate

// Error Covariance Matrix 
float P[N][N] = {{1000,0,0, 0,0,0,    0,0,0,    0,0,0,    0,0,0    }, // x
                 {0,1000,0, 0,0,0,    0,0,0,    0,0,0,    0,0,0    }, // y
                 {0,0,1000, 0,0,0,    0,0,0,    0,0,0,    0,0,0    }, // z                   
                 {0,0,0,    1000,0,0, 0,0,0,    0,0,0,    0,0,0    }, // x_vel
                 {0,0,0,    0,1000,0, 0,0,0,    0,0,0,    0,0,0    }, // y_vel
                 {0,0,0,    0,0,1000, 0,0,0,    0,0,0,    0,0,0    }, // z_vel
                 {0,0,0,    0,0,0,    1000,0,0, 0,0,0,    0,0,0    }, // x_accel
                 {0,0,0,    0,0,0,    0,1000,0, 0,0,0,    0,0,0    }, // y_accel
                 {0,0,0,    0,0,0,    0,0,1000, 0,0,0,    0,0,0    }, // z_accel
                 {0,0,0,    0,0,0,    0,0,0,    1000,0,0, 0,0,0    }, // roll
                 {0,0,0,    0,0,0,    0,0,0,    0,1000,0, 0,0,0    }, // pitch                  
                 {0,0,0,    0,0,0,    0,0,0,    0,0,1000, 0,0,0    }, // yaw
                 {0,0,0,    0,0,0,    0,0,0,    0,0,0,    1000,0,0 }, // roll_vel
                 {0,0,0,    0,0,0,    0,0,0,    0,0,0,    0,1000,0 }, // pitch_vel
                 {0,0,0,    0,0,0,    0,0,0,    0,0,0,    0,0,1000 }};// yaw_vel

 // Measurement Equation - relates x to z     TODO:Set up H
 //               pos     vel     accel   angle   angle'
float H[M][N] = {{0,0,0,  0,0,0,  0,0,0,  0,0,0,  1,0,0  }, // gyro x
                 {0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,1,0  }, // gyro y
                 {0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,1  }, // gyro z                   
                 {0,0,0,  0,0,0,  0,0,0,  1,0,0,  0,0,0  }, // mag x
                 {0,0,0,  0,0,0,  0,0,0,  0,1,0,  0,0,0  }, // mag y
                 {0,0,0,  0,0,0,  0,0,0,  0,0,1,  0,0,0  }, // mag z
                 {0,0,0,  0,0,0,  1,0,0,  0,0,0,  0,0,0  }, // accel x
                 {0,0,0,  0,0,0,  0,1,0,  0,0,0,  0,0,0  }, // accel y
                 {0,0,0,  0,0,0,  0,0,1,  0,0,0,  0,0,0  }};// accel z

// Process Noise Covariance TODO:Setu Up Q
//    make this a diagonal matrix with the variance of each of the states along the diagonal
float Q[N][N] = {{2,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0  }, // x
                 {0,2,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0  }, // y
                 {0,0,2,  0,0,0,  0,0,0,  0,0,0,  0,0,0  }, // z                   
                 {0,0,0,  2,0,0,  0,0,0,  0,0,0,  0,0,0  }, // x_vel
                 {0,0,0,  0,2,0,  0,0,0,  0,0,0,  0,0,0  }, // y_vel
                 {0,0,0,  0,0,2,  0,0,0,  0,0,0,  0,0,0  }, // z_vel
                 {0,0,0,  0,0,0,  2,0,0,  0,0,0,  0,0,0  }, // x_accel
                 {0,0,0,  0,0,0,  0,2,0,  0,0,0,  0,0,0  }, // y_accel
                 {0,0,0,  0,0,0,  0,0,2,  0,0,0,  0,0,0  }, // z_accel
                 {0,0,0,  0,0,0,  0,0,0,  2,0,0,  0,0,0  }, // roll
                 {0,0,0,  0,0,0,  0,0,0,  0,2,0,  0,0,0  }, // pitch                  
                 {0,0,0,  0,0,0,  0,0,0,  0,0,2,  0,0,0  }, // yaw
                 {0,0,0,  0,0,0,  0,0,0,  0,0,0,  2,0,0  }, // roll_vel
                 {0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,2,0  }, // pitch_vel
                 {0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,2  }};// yaw_vel

// Measurement Noise Covariance TODO:Set Up R
//                gyro    mag   accel
float R[M][M] = {{2,0,0, 0,0,0, 0,0,0}, // gyro x
                 {0,2,0, 0,0,0, 0,0,0}, // gyro y
                 {0,0,2, 0,0,0, 0,0,0}, // gyro z
                 {0,0,0, 2,0,0, 0,0,0}, // mag  x
                 {0,0,0, 0,2,0, 0,0,0}, // mag  y
                 {0,0,0, 0,0,2, 0,0,0}, // mag  z
                 {0,0,0, 0,0,0, 2,0,0}, // accel x
                 {0,0,0, 0,0,0, 0,2,0}, // accel y
                 {0,0,0, 0,0,0, 0,0,2}};// accel z

// State Equation Matrix
float A[N][N] = {{1,0,0, dt,0,0, 0.5*dt*dt,0,0, 0,0,0,  0,0,0 },  // x
                 {0,1,0, 0,dt,0, 0,0.5*dt*dt,0, 0,0,0,  0,0,0 },  // y
                 {0,0,1, 0,0,dt, 0,0,0.5*dt*dt, 0,0,0,  0,0,0 },  // z                   
                 {0,0,0, 1,0,0,  dt,0,0,        0,0,0,  0,0,0 },  // x_vel
                 {0,0,0, 0,1,0,  0,dt,0,        0,0,0,  0,0,0 },  // y_vel
                 {0,0,0, 0,0,1,  0,0,dt,        0,0,0,  0,0,0 },  // z_vel
                 {0,0,0, 0,0,0,  1,0,0,         0,0,0,  0,0,0 },  // x_accel
                 {0,0,0, 0,0,0,  0,1,0,         0,0,0,  0,0,0 },  // y_accel
                 {0,0,0, 0,0,0,  0,0,1,         0,0,0,  0,0,0 },  // z_accel
                 {0,0,0, 0,0,0,  0,0,0,         1,0,0,  dt,0,0},  // roll
                 {0,0,0, 0,0,0,  0,0,0,         0,1,0,  0,dt,0},  // pitch                  
                 {0,0,0, 0,0,0,  0,0,0,         0,0,1,  0,0,dt},  // yaw
                 {0,0,0, 0,0,0,  0,0,0,         0,0,0,  1,0,0 },  // roll_vel
                 {0,0,0, 0,0,0,  0,0,0,         0,0,0,  0,1,0 },  // pitch_vel
                 {0,0,0, 0,0,0,  0,0,0,         0,0,0,  0,0,1 }}; // yaw_vel
                 
//float u[L];    // control outputs
//float B[N][L]; // control output to state mapping               

float A_transpose[N][N] = {};   
float H_transpose[N][M];
float I[N][N]; // NxN Identity Matrix       
float K[N][M]; // Kalman Gain
float z[M];    // Sensor Measurements

// Preallocated Temporary variable matrices
long gyroData[3]; 
long magData[3]; 
long accelData[3]; 
float result[N]; 
float result2[N][N]; 
float result3[N][N]; 
float result4[M][N]; 
float result5[M][M]; 
float result6[M][M]; 
float result7[N][M];   
float result8[M]; 
float result9[M]; 
float result10[N]; 
float result11[N]; 
float result12[N][N]; 
float result13[N][N]; 
float result14[N][N]; 

//=============== Main ================= 
void setup() {
  Serial.begin(9600);
  Wire.begin();

  Matrix.Transpose((float*)A, N,N, (float*)A_transpose); // Store A_Transpose
//  Matrix.Transpose((float*)H, M,N, (float*)H_transpose); // Store H_Transpose
// 
//  gyro.setupGyro();
//  magnetometer.setupMagnetometer();
//  accelerometer.setupAccel();
//
//  fillIdentityMatrix();   
}

void loop(){
  magnetometer.updateMagValuesWithRepeatedStart();
  gyro.updateGyroValuesWithRepeatedStart();
  accelerometer.updateAccelValuesWithRepeatedStart();

  gyro.getGyroValues(gyroData);
  magnetometer.getMagnetometerValues(magData);
  accelerometer.getAccelerometerValues(accelData);

  // TODO: Convert sensor readings to useful units
  
//  // Store sensor readings in z
//  for (int i=0; i<3; i++){
//    z[i] = gyroData[i];
//    z[3+i] = accelData[i];
//    z[6+i] = magData[i];
//  }
//  
//  // ===== KALMAN FILTER ===== 
//  // Project the state ahead
//  //       x = A*x + B*u
//  Matrix.Multiply((float*)A,(float*)x, N,N,1, (float*)result); // A*x = result
//  Matrix.Copy((float*)result, N, 1, (float*)x);                // x = A*x
//  
//  // Project the error Covariance Ahead
//  //       P = A*P*A_transpose + Q
//  Matrix.Multiply((float*)A,(float*)P, N,N,N, (float*)result2);                 // A*P = result2
//  Matrix.Multiply((float*)result2,(float*)A_transpose, N,N,N, (float*)result3); // result2*A_transpose = result 3,
//  Matrix.Add((float*)result3,(float*)Q, N,N, (float*)P);                        // P = result3 + Q
//
//  // Compute the Kalman Gain
//  //       K = P*H_transpose * (H*P*H_transpose + R)_inverse
//  Matrix.Multiply((float*)H,(float*)P, M,N,N, (float*)result4);       // result4 = H*P
//  Matrix.Multiply((float*)result4,(float*)H, M,N,M, (float*)result5); // result5 = result4*H_Transpose
//  Matrix.Add((float*)result5,(float*)R, M,M, (float*)result6);        // result6 = result5 + R
//  Matrix.Invert((float*)result6, M);                                  // result6 = result6_inverse
//  Matrix.Multiply((float*)P,(float*)H_transpose, N,N,M, (float*)result7);  // result7 = P*H_transpose
//  Matrix.Multiply((float*)result7,(float*)result6, N,M,M, (float*)K);   // K = result6*result7
//
//  // Update Estimate with Measurement z
//  //       x = x + K*(z - H*x)
//  Matrix.Multiply((float*)H,(float*)x, M,N,1, (float*)result8);       // result8 = H*x
//  Matrix.Subtract((float*)z, (float*)result8, M,1, (float*)result9);  // result9 = z - result8
//  Matrix.Multiply((float*)K,(float*)result9, N,M,1, (float*)result10);// result10 = K*result9
//  Matrix.Add((float*)x, (float*)result10, N,1, (float*)result11);     // result11 = x + result10
//  Matrix.Copy((float*)result11, N, 1, (float*)x);                     // x = result11
//  
//  // Update the Error Covariance
//  //       P = (I - K*H)*P
//  Matrix.Multiply((float*)K,(float*)H, N,M,1, (float*)result12);        // result12 = K*H
//  Matrix.Subtract((float*)I, (float*)result12, M,1, (float*)result13);  // result13 = I - result12
//  Matrix.Multiply((float*)result13,(float*)P, N,N,N, (float*)result14); // result14 = result13*P
//  Matrix.Copy((float*)result14, N, N, (float*)P);                       // P = result14
  
  
  // Print Gyro Data
  for (int i=0; i< 3; i++){
     Serial.print(gyroData[i]);
     Serial.print(",");
  }
  
  // Print Magnetometer Data
  for (int i=0; i< 3; i++){
     Serial.print(magData[i]);
     Serial.print(",");
  }
  
  // Print Accelerometer Data
  for (int i=0; i< 3; i++){
     Serial.print(accelData[i]);
     Serial.print(",");
  }
  
//  // Print State
//  for (int i=0; i<N ; i++){
//     Serial.print(x[i]);
//     Serial.print(",");
//  }
  Serial.println(";");
}

void fillIdentityMatrix(){
  for(int i=0; i<N; i++){
    I[i][i] = 1.0; 
  }
}
