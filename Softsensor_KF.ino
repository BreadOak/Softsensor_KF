void setup() {
  Serial.begin(115200);
}
float sensorValue;
float filterValue;
float KFValue;
float voltage;
float sensitivity = 0.1;

// *KalmanFilter Parameter*
float Zk; 
float x_Predict; 
float p_Predict; 
float Q = pow(0.01 , 2);
float R = pow(0.5 , 2);
float A = 1;
float B_uk = 0;
float H = 1;
float Xk = 320;
float Pk = 1.0;
float K_gain = 1.0;

void loop() {
  sensorValue = analogRead(A0);
  //filterValue = filterValue * (1 - sensitivity) + sensorValue * sensitivity;
  //voltage = filterValue * (5.0 / 1023.0);
  
  // *KalmanFilter*
  Zk = sensorValue;
  x_Predict = A*Xk + B_uk;                   // prediction -> state
  p_Predict = A*Pk + Q;                      // prediction -> error covariance
  K_gain = p_Predict/(H*p_Predict + R);      // update Kalman gain 
  Xk = x_Predict + K_gain*(Zk - x_Predict);  // estimation -> state 
  Pk = (1 - K_gain)*p_Predict;               // estimation -> error covariance
  KFValue = Xk;
  voltage = KFValue * (5.0 / 1023.0);
  
  //Serial.println(filterValue);
  //Serial.println(sensorValue);
  //Serial.println(voltage);
  Serial.println(KFValue);

  delay(3);
}
