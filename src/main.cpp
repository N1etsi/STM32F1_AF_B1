///////////////v0.4 Rev2
///////////////fast cycle, only mpu (close to 1ms)


#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define nCh 8
#define usCyc 5000
#define msCyc 5.0f

#define MPU_ADDR 0x68

#define Ro 0
#define Pi 1
#define Ya 2
#define Kp 0
#define Ki 1
#define Kd 2
#define X 0
#define Y 1
#define Z 2

#define STDBY -10
#define GND 0
#define AIR 1
#define SHUT -5

//pin declaration
#define rx PA8
#define escLF PA0
#define escLB PA2
#define escRF PA1
#define escRB PA3
#define redL PB14
#define greenL PB12
#define INTERRUPT_PIN PB5
#define stL PC13

//modes
byte mode = AIR; //STDBY by default

//loop
uint32_t loopSt;

//control loop
float Kte[3][3];
float setP[3];
float altsetP[3]; //alternate pid settings
float err[3];
float last[3];
float mem[3];
int32_t pidOut[3];

//ppm
uint32_t rxPrev, rxCurr, rxCH;
uint32_t chArr[nCh+1], ppmRE;
int chCounter=0;

//pwm
uint32_t escLFt, escLBt, escRFt, escRBt, throttle;

///////////
//SENSORS//
///////////
//mpu6050
float accRes = (float)(4.0f / 32768.0f); //accel resolution calibration aaa
float gyrRes = (float)(500.0f / 32768.0f); //gyro resolution calibration aaa
int16_t rawAG[6];
float gyr[3], gyrBias[3] = {0.0, 0.0, 0.0};
float acc[3], accBias[3] = {0.0, 0.0, 0.0};
uint32_t cycTim;
float yaw, pitch, roll; //Madgwick Quaternion Filter Attitude Results 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float beta = sqrt(3.0f / 4.0f) * PI * (40.0f / 180.0f);
float zeta = sqrt(3.0f / 4.0f) * PI * (2.0f / 180.0f);
float deltat = msCyc / 1000.0f;

//aux function declaration
void pidK();
void pinDecl();
void initRX(void);
void setRX();
void timerS(void);
void escOut();
void escInit();
void calcPID();
void setMPU();
void calibMPU();
void quaternionFilter();
void readMPU();
void varClear();
void debugPrint();
void tiltCheck();

//Setup 
void setup()
{
	//Init, lights off

	//Comms startup
  //Serial.begin(460800);
	Wire.setClock(400000);
	Wire.begin();
	
  delay(1000);
	
	//FC startup
	pidK();
	pinDecl();
	timerS();
  setRX();

  digitalWrite(redL, HIGH);
	digitalWrite(greenL, HIGH);
  digitalWrite(stL, HIGH);
	
	//Sensor start
	setMPU();

  delay(500);
	//Calibrate Sensors (might remove)
	calibMPU();

	//Light Signal Ready
	digitalWrite(redL, LOW);
	digitalWrite(greenL, LOW);
	delay(250);
	digitalWrite(redL, HIGH);
	digitalWrite(greenL, HIGH);
	delay(200);
	digitalWrite(redL, LOW);
	digitalWrite(greenL, LOW);
	delay(600);
	digitalWrite(redL, HIGH);
	digitalWrite(greenL, HIGH);
	delay(200);
	digitalWrite(greenL, LOW);
	loopSt= micros();
}

//Main Loop
void loop()
{
  if((micros()-loopSt)>1000.0)   digitalWrite(stL, LOW);
	while( (micros()-loopSt) < (usCyc));
	loopSt = micros();
	//MODE INDEPENDANT
	readMPU();
	quaternionFilter();
	tiltCheck();
  //debugPrint();
	//STDBY MODE
	if(mode == AIR)
	{
    calcPID();
    escOut();

    digitalWrite(redL, LOW);
    digitalWrite(greenL, HIGH);
    
	}
	else if(mode == GND)
	{
    escInit();
    varClear();
    digitalWrite(redL, HIGH);
    digitalWrite(greenL, LOW);
    setP[Ya]=yaw; // initial position
    digitalWrite(greenL, LOW);
    digitalWrite(redL, LOW);
	}
	else if(mode == STDBY)
	{
    escInit();
    varClear();
    digitalWrite(redL, HIGH);
    digitalWrite(greenL, LOW);
	}
	else //catch exceptions, same as stdby?
	{
    escInit();
    varClear();
    digitalWrite(redL, HIGH);
    digitalWrite(greenL, HIGH);
	}
}
//END of Main Loop
//////////////////
//Aux functions 
void pidK()
{
  // most stable 4.0 / 0.03 / 150
  //old (had oscillations) most stable until now : 6.0 / 0.08 / 150.0
  // will try baseflight default values 4.0 / 0.03 / 23.0
  setP[Ro]=0;
  setP[Pi]=0;
  setP[Ya]=0;

  Kte[Ro][Kp]=1.3; //1.3 seems to low, 10 seems to high
  Kte[Ro][Ki]=0.05;
  Kte[Ro][Kd]=15.0;

  Kte[Pi][Kp]=1.3;
  Kte[Pi][Ki]=0.05;
  Kte[Pi][Kd]=15.0;

  Kte[Ya][Kp]=0.2;
  Kte[Ya][Ki]=0.00;
  Kte[Ya][Kd]=5;
}

void pinDecl()
{
  //PPM RX
  pinMode(rx, INPUT);
  //PWM ESC
  pinMode(escLF, PWM);
  pinMode(escLB, PWM);
  pinMode(escRB, PWM);
  pinMode(escRF, PWM);
  //LED status
  pinMode(redL, OUTPUT);
  pinMode(greenL, OUTPUT);
  pinMode(stL, OUTPUT);
  digitalWrite(redL, LOW);
  digitalWrite(greenL, LOW);
}

void initRX(void)
{
  rxCurr= TIMER1_BASE->CCR1;
  if(rxCurr<rxPrev) rxCH = (0xFFFF - rxPrev) + rxCurr;
  else rxCH = rxCurr - rxPrev;
  rxPrev=rxCurr;

  if(rxCH>3000) chCounter=0;
  else if(rxCH<0) rxCH+=0xFFFF;

  chArr[chCounter]=rxCH;
  if(chCounter==8 && mode!=SHUT)
  {
    if(chArr[chCounter]<850) mode =STDBY;
    else if(chArr[chCounter]<1200) mode=GND;
    else if(chArr[chCounter]<1800) mode=AIR;
    else mode=SHUT;
  }
  else if(chCounter==8 && mode == SHUT) chArr[chCounter]=0;
  chCounter++;
}
void setRX()
{
  chArr[0] = 1000.0;
  chArr[1] = 1500.0;
  chArr[2] = 1500.0;
  chArr[3] = 1000.0;
  chArr[4] = 1500.0;
  chArr[5] = 1000.0;
  chArr[6] = 1000.0;
  chArr[7] = 1000.0;
  chArr[8] = 1000.0;
}

void timerS(void)
{
  //setup timer2-PWM and timer1-ppm
  //Timer1.attachCompare1Interrupt(initRX);
  Timer1.attachInterrupt(TIMER_CH1, initRX);
  TIMER1_BASE->CR1 = TIMER_CR1_CEN;
  TIMER1_BASE->CR2 = 0;
  TIMER1_BASE->SMCR = 0;
  TIMER1_BASE->DIER = TIMER_DIER_CC1IE;
  TIMER1_BASE->EGR = 0;
  TIMER1_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  TIMER1_BASE->CCMR2 = 0;
  TIMER1_BASE->CCER = TIMER_CCER_CC1E;
  TIMER1_BASE->PSC = 71;
  TIMER1_BASE->ARR = 0xFFFF;
  TIMER1_BASE->DCR = 0;

  TIMER2_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = 0;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE | (0b110 << 12) | TIMER_CCMR1_OC2PE;
  TIMER2_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE | (0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 5000;
  TIMER2_BASE->DCR = 0;

  //pwm init default values
  TIMER2_BASE->CCR1 = 1000;
  TIMER2_BASE->CCR2 = 1000;
  TIMER2_BASE->CCR3 = 1000;
  TIMER2_BASE->CCR4 = 1000;
}

void escOut()
{ 
  /*
  Serial.print(escLFt);
  Serial.print(" ");
  Serial.print(escRFt);
  Serial.print(" ");
  Serial.print(escLBt);
  Serial.print(" ");
  Serial.print(escRBt);
  Serial.println(" ");
  */

  TIMER2_BASE->CCR1 = escLFt;
  TIMER2_BASE->CCR2 = escRFt;
  TIMER2_BASE->CCR3 = escLBt;
  TIMER2_BASE->CCR4 = escRBt;
  TIMER2_BASE->CNT=5000;
}

void escInit()
{
  TIMER2_BASE->CCR1 = 1000;
  TIMER2_BASE->CCR2 = 1000;
  TIMER2_BASE->CCR3 = 1000;
  TIMER2_BASE->CCR4 = 1000;
  TIMER2_BASE->CNT=5000;
}

void calcPID()
{
  throttle=chArr[3];
  setP[Ro] = ((float)chArr[1] - 1500) / 15.0;
  setP[Pi]= -1.0*((float)chArr[2] - 1500) / 15.0;
  if(abs(chArr[4]-1500) < 20 );
  else
  {
    setP[Ya]+= ((float)chArr[4] - 1500) / 1500.0; //needs testing on rate
    if(setP[Ya] > 180) setP[Ya] -= 360.0;
    else if (setP[Ya] < -180) setP[Ya]+=360.0;
  }
  
  //roll pitch
  err[Ro] = roll - setP[Ro];
  mem[Ro] += err[Ro];
  pidOut[Ro] = Kte[Ro][Kp] * err[Ro] + Kte[Ro][Ki] * mem[Ro] + Kte[Ro][Kd] * (err[Ro] - last[Ro]);
  last[Ro] = err[Ro];
  if(pidOut[Ro] > 400)  pidOut[Ro] = 400;
  else if(pidOut[Ro] < (-400)) pidOut[Ro] = (-400);

   //pid pitch
  err[Pi] = pitch - setP[Pi];
  mem[Pi] += err[Pi];
  pidOut[Pi] = Kte[Pi][Kp] * err[Pi] + Kte[Pi][Ki] * mem[Pi] + Kte[Pi][Kd] * (err[Pi] - last[Pi]);
  last[Pi] = err[Pi];
  if(pidOut[Pi] > 400)  pidOut[Pi] = 400;
  else if(pidOut[Pi] < (-400)) pidOut[Pi] = (-400);

  //pid yaw
  err[Ya] = yaw - setP[Ya];
  if(abs(err[Ya]) > 180 ) err[Ya] *= -1;
  //mem[Ya]+=err[Ya];
  pidOut[Ya] = Kte[Ya][Kp] * err[Ya] + Kte[Ya][Kd] * (err[Ya] - last[Ya]); //+ Kte[Ya][Ki]*mem[Ya] + Kte[Ya][Kd]*(err[Ya]-last[Ya]);
  last[Ya] = err[Ya];
  if(pidOut[Ya] > 400)  pidOut[Ya] = 400;
  else if(pidOut[Ya] < (-400)) pidOut[Ya] = (-400);

  //output
  escLFt = (uint32_t) min(max(throttle + pidOut[Pi] - pidOut[Ro] + pidOut[Ya], 1200), 1800);
  escRFt = (uint32_t) min(max(throttle + pidOut[Pi] + pidOut[Ro] - pidOut[Ya], 1200), 1800);
  escLBt = (uint32_t) min(max(throttle - pidOut[Pi] - pidOut[Ro] - pidOut[Ya], 1200), 1800);
  escRBt = (uint32_t) min(max(throttle - pidOut[Pi] + pidOut[Ro] + pidOut[Ya], 1200), 1800);

}

void setMPU()
{
  //Power Management 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); //power mngmt bit
  Wire.write(0x00); //no sleep mode
  Wire.endTransmission();

  //Sample rate divider and frequency -> set to 1kHz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);
  Wire.write(0x04);//0x04 -> 200Hz // 0x07 -> 1kHz
  Wire.endTransmission();

  //set gyro sensitivity
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08); //set to 500deg/s max aaa
  Wire.endTransmission();

  //set accel sensitivity
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08); //set to 4g max aaa
  Wire.endTransmission();
}

void calibMPU()
{
  float bias[6];
  int n = 100;
  bias[0] = 0.0;
  bias[1] = 0.0;
  bias[2] = 0.0;
  bias[3] = 0.0;
  bias[4] = 0.0;
  bias[5] = 0.0;

  for(int i=0; i<20; i++) readMPU();
  digitalWrite(stL, HIGH);
  for(int i=0; i<n; i++)
  {
    readMPU();
    bias[0] += acc[X];
    bias[1] += acc[Y];
    bias[2] += acc[Z];
    bias[3] += gyr[X];
    bias[4] += gyr[Y];
    bias[5] += gyr[Z];
    //Serial.println(i);
  }
  digitalWrite(stL, LOW);
  accBias[X] = bias[0] / n;
  accBias[Y] = bias[1] / n;
  accBias[Z] = bias[2] / n;
  accBias[Z] -= 1.0f;

  gyrBias[X] = bias[3] / n;
  gyrBias[Y] = bias[4] / n;
  gyrBias[Z] = bias[5] / n;

}

void readMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); //first data address
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14); //6+2+6(acc, temp, gyr)
  while(Wire.available() < 14);
  rawAG[0] = (int16_t)((Wire.read() << 8) | Wire.read());
  rawAG[1] = (int16_t)((Wire.read() << 8) | Wire.read());
  rawAG[2] = (int16_t)((Wire.read() << 8) | Wire.read());

  rawAG[3] = (int16_t)((Wire.read() << 8) | Wire.read()); //temp value, trash it

  rawAG[3] = (int16_t)((Wire.read() << 8) | Wire.read());
  rawAG[4] = (int16_t)((Wire.read() << 8) | Wire.read());
  rawAG[5] = (int16_t)((Wire.read() << 8) | Wire.read());

  acc[X] = ((float)rawAG[0])*accRes - accBias[X];
  acc[Y] = ((float)rawAG[1])*accRes - accBias[Y];
  acc[Z] = ((float)rawAG[2])*accRes - accBias[Z];

  gyr[X] = ((float)rawAG[3])*gyrRes - gyrBias[X];
  gyr[Y] = ((float)rawAG[4])*gyrRes - gyrBias[Y];
  gyr[Z] = ((float)rawAG[5])*gyrRes - gyrBias[Z];

  while(Wire.available()) Wire.read();
}

void quaternionFilter()
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx=0, gbiasy=0, gbiasz=0;        // gyro bias error

  //Degrees to Radians
  gyr[X] *= PI/180.0f; 
  gyr[Y] *= PI/180.0f;
  gyr[Z] *= PI/180.0f;

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;

  // Normalise accelerometer measurement
  norm = sqrt(acc[X] * acc[X] + acc[Y] * acc[Y] + acc[Z] * acc[Z]);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  acc[X] *= norm;
  acc[Y] *= norm;
  acc[Z] *= norm;
  
  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - acc[X];
  f2 = _2q1 * q2 + _2q3 * q4 - acc[Y];
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - acc[Z];
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;
  
  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;
  
  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
  
  // Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  gyr[X] -= gbiasx;
  gyr[Y] -= gbiasy;
  gyr[Z] -= gbiasz;
  
  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gyr[X] - _halfq3 * gyr[Y] - _halfq4 * gyr[Z];
  qDot2 =  _halfq1 * gyr[X] + _halfq3 * gyr[Z] - _halfq4 * gyr[Y];
  qDot3 =  _halfq1 * gyr[Y] - _halfq2 * gyr[Z] + _halfq4 * gyr[X];
  qDot4 =  _halfq1 * gyr[Z] + _halfq2 * gyr[Y] - _halfq3 * gyr[X];

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(beta * hatDot1)) * deltat;
  q2 += (qDot2 -(beta * hatDot2)) * deltat;
  q3 += (qDot3 -(beta * hatDot3)) * deltat;
  q4 += (qDot4 -(beta * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

  // Intrinsic Euler Angles
  yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  
  yaw *= 180.0f / PI;
  pitch *= 180.0f / PI;
  roll *= 180.0f / PI;
}

void debugPrint()
{
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.print(yaw);
  Serial.print("/");
  Serial.print(setP[Ya]);
  Serial.print("/");
  Serial.print(chArr[4]);
  Serial.print("/");
  Serial.print(err[Ya]);
  Serial.print("/");
  Serial.print(pidOut[Ya]);
  Serial.println(" ");

}

void tiltCheck()
{
  if((roll > 60.0) || (roll < -60.0) || (pitch > 60.0) || (pitch < -60.0) )mode = SHUT;

}

void varClear()
{
  mem[Ro]=0;
  mem[Pi]=0;
  last[Ro]=0;
  last[Pi]=0;
}