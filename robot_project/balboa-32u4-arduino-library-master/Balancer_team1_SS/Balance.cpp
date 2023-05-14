
#include <Wire.h>
#include "Balance.h"
#include <math.h>
int32_t gYZero;
int32_t angle; 
int32_t angle_a;// millidegrees
int32_t angleRate; // degrees/s

int32_t speed_SP=0;
int32_t pos_SP=0;

int32_t distanceLeft;
int32_t speedLeft;
int32_t driveLeft;
int32_t distanceRight;
int32_t speedRight;
int32_t driveRight;
int16_t motorSpeed;
bool isBalancingStatus = false;
bool balanceUpdateDelayedStatus;
int32_t K;
int32_t angle_compl;
int32_t angle_complementaire;
int32_t previous_angle;
int32_t angleRate_milli_Radian;

//position differenciation et observer (LS3)

int32_t phi;   
int32_t phiLeft;           //mrads
int32_t phiRight;           //mrads

int32_t phidot;         //mrads/s
int32_t phidot_obs;     //mrads/s
int32_t z_k;
int32_t dutySum;


bool isBalancing()
{
  return isBalancingStatus;
}

bool balanceUpdateDelayed()
{
  return balanceUpdateDelayedStatus;
}

void balanceSetup()
{
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200); 
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // Wait for IMU readings to stabilize.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    
    total += imu.g.y;
    delay(1);
  }

  gYZero = total / CALIBRATION_ITERATIONS;
  
}
void lyingDown()
{
  // Reset things so it doesn't go crazy.
  motorSpeed = 0;
  distanceLeft = 0;
  distanceRight = 0;
  motors.setSpeeds(0, 0);

  if (angleRate > -2 && angleRate < 2)
  {
    // It's really calm, so use the accelerometer to measure the
    // robot's rest angle.  The atan2 function returns a result
    // in radians, so we multiply it by 180000/pi to convert it
    // to millidegrees.
    angle = atan2(imu.a.z, imu.a.x) * 57296;
    

    distanceLeft = 0;
    distanceRight = 0;
  }
}

int integrateGyro()
{
  // Convert from full-scale 1000 deg/s to deg/s.
  angleRate = (imu.g.y - gYZero) / 29;
  angleRate_milli_Radian = angleRate*(PI/180); // angle mrad

  angle += angleRate * UPDATE_TIME_MS;
  
  //Serial.print("angle");
  //Serial.println(angle);
  return angleRate_milli_Radian;
}

int accelerometer()
  {
    angle_a = atan2(imu.a.z,imu.a.x)*1000; //en mrad
    //Serial.println(angle_a);  
    return angle_a;
    }

void complematory_algorithm()
{
  if (previous_angle==0){
    previous_angle = accelerometer();
  }
  
  K=(UPDATE_TIME_MS*1000)/filter_time_MS;

  angle_compl = ((1000-K)*(previous_angle+UPDATE_TIME_MS *angleRate/1000) + K*angle_a)/1000;// en mrad
  angle_compl = angle_compl;
  previous_angle = angle_compl;
  angle_complementaire=angle_compl*180/PI;// en mdeg

  return angleRate_milli_Radian;
}

void balance()
{
  // Adjust toward angle=0 with timescale ~10s, to compensate for
  // gyro drift.  More advanced AHRS systems use the
  // accelerometer as a reference for finding the zero angle, but
  // this is a simpler technique: for a balancing robot, as long
  // as it is balancing, we know that the angle must be zero on
  // average, or we would fall over.
  angle = angle * 999 / 1000;

    int32_t u = 0;
  // This variable measures how close we are to our basic
  // balancing goal - being on a trajectory that would cause us
  // to rise up to the vertical position with zero speed left at
  // the top.  This is similar to the fallingAngleOffset used
  // for LED feedback and a calibration procedure discussed at
  // the end of Balancer.ino.
  //
  // It is in units of millidegrees, like the angle variable, and
  // you can think of it as an angular estimate of how far off we
  // are from being balanced.
  int32_t risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;

  // Combine risingAngleOffset with the distance and speed
  // variables, using the calibration constants defined in
  // Balance.h, to get our motor response.  Rather than becoming
  // the new motor speed setting, the response is an amount that
  // is added to the motor speeds, since a *change* in speed is
  // what causes the robot to tilt one way or the other.

  if (motorSpeed > MOTOR_SPEED_LIMIT)
  {
    motorSpeed = MOTOR_SPEED_LIMIT;
  }
  if (motorSpeed < -MOTOR_SPEED_LIMIT)
  {
    motorSpeed = -MOTOR_SPEED_LIMIT;
  }

  // Adjust for differences in the left and right distances; this
  // will prevent the robot from rotating as it rocks back and
  // forth due to differences in the motors, and it allows the
  // robot to perform controlled turns.
  int16_t distanceDiff = distanceLeft - distanceRight;

  motors.setSpeeds(
    motorSpeed + distanceDiff * DISTANCE_DIFF_RESPONSE / 100,
    motorSpeed - distanceDiff * DISTANCE_DIFF_RESPONSE / 100);
}
 

void posR()
{
  static int16_t lastCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;

   phiLeft=(distanceLeft*1000*2*PI)/1321;          // on convertit de count par tour a mrads
   phiRight=(distanceRight*1000*2*PI)/1321;         // on convertit de count par tour a mrads
   phi=(phiLeft+phiRight)/2; //mrads               // moyenne de la position des 2 roues
   //Serial.println(phi);
   
   //Differentiation
   phidot= 0.5*(speedLeft+speedRight)*(1000/UPDATE_TIME_MS)*1000*2*PI/1321; //[mrads/s] 
   Serial.print(phidot);
   Serial.print(",");
}

void Obsreduit()
{
  dutySum =1000*2*motorSpeed/MOTOR_SPEED_LIMIT;                            //Tension moyenne des deux roues             
  
  phidot_obs=z_k+L1*angle_compl+L2* angleRate_milli_Radian+L3*phi;
  phidot_obs=phidot_obs/1000;                                              //[mrads/s] 
  
  z_k=K1*phidot_obs+K2_1*angle_compl+K2_2* angleRate_milli_Radian+K2_3*phi+dutySum; 
  
  Serial.println(phidot_obs); 
}

void integrateEncoders()
{
  static int16_t lastCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;
}

void StateSpace()
{
  // Adjust toward angle=0 with timescale ~10s, to compensate for
  // gyro drift.  More advanced AHRS systems use the
  // accelerometer as a reference for finding the zero angle, but
  // this is a simpler technique: for a balancing robot, as long
  // as it is balancing, we know that the angle must be zero on
  // average, or we would fall over.
  angle = angle * 999 / 1000;

//position sans action integrale
  //dutySum=(N_pos*pos_SP - K_pos1*angle_compl - K_pos2*angleRate_milli_Radian - K_pos3*phi - K_pos4*phidot_obs)/1000;  //permet de calculer la tension d'entrée grâce à la formule récursive du contrôleur d'état de position

  //Vitesse sans AI
  //dutySum=(N_speed*speed_SP - K_speed1*angle_compl - K_speed2*angleRate_milli_Radian*1000 - K_speed3*phidot)/1000;       //permet de calculer la tension d'entrée grâce à la formule récursive du contrôleur d'état de vitesse
  
  motorSpeed = (dutySum*MOTOR_SPEED_LIMIT/1000/2); //pour avoir la vitesse d'une seule roue


// SATURATION
  if (motorSpeed > MOTOR_SPEED_LIMIT)
  {
    motorSpeed = MOTOR_SPEED_LIMIT;
    //dutySum=2000;
  }
  if (motorSpeed < -MOTOR_SPEED_LIMIT)
  {
    motorSpeed = -MOTOR_SPEED_LIMIT;
    //dutySum=2000;
  }

  // Adjust for differences in the left and right distances; this
  // will prevent the robot from rotating as it rocks back and
  // forth due to differences in the motors, and it allows the
  // robot to perform controlled turns.
  int16_t distanceDiff = distanceLeft - distanceRight;

  motors.setSpeeds(
    motorSpeed + distanceDiff * DISTANCE_DIFF_RESPONSE / 100,
    motorSpeed - distanceDiff * DISTANCE_DIFF_RESPONSE / 100);
}

void balanceDrive(int16_t leftSpeed, int16_t rightSpeed)
{
  driveLeft = leftSpeed;
  driveRight = rightSpeed;
}

void balanceDoDriveTicks()
{
  distanceLeft -= driveLeft;
  distanceRight -= driveRight;
  speedLeft -= driveLeft;
  speedRight -= driveRight;
}

void balanceResetEncoders()
{
  distanceLeft = 0;
  distanceRight = 0;
}

void balanceUpdateSensors()
{
  imu.read();
  //integrateGyro();
  //accelerometer();
  posR();
  Obsreduit();
  complematory_algorithm();
  //integrateEncoders();
}

void balanceUpdate()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();
  static uint8_t count = 0;

  // Perform the balance updates at 100 Hz.
  if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  balanceUpdateSensors();
  balanceDoDriveTicks();

  if (isBalancingStatus)
  {
    //balance();
    StateSpace();

    // Stop trying to balance if we have been farther from
    // vertical than STOP_BALANCING_ANGLE for 5 counts.
    if (abs(angle) > STOP_BALANCING_ANGLE)
    {
      if (++count > 5)
      {
        isBalancingStatus = false;
        count = 0;
      }
    }
    else
    {
      count = 0;
    }
  }
  else
  {
    lyingDown();

    // Start trying to balance if we have been closer to
    // vertical than START_BALANCING_ANGLE for 5 counts.
    if (abs(angle) < START_BALANCING_ANGLE)
    {
      if (++count > 5)
      {
        isBalancingStatus = true;
        count = 0;
      }
    }
    else
    {
      count = 0;
    }
  }
}
