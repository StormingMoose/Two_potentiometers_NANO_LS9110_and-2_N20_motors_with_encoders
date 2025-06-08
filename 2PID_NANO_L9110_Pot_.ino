
// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.
//https://github.com/curiores/ArduinoTutorials/blob/main/MultipleEncoders/SimplePositionPID/SimplePositionPID.ino

// StormingMoose added the potentiometers to control output targets
// video proof   https://www.youtube.com/shorts/rNg9tuDR440


// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
  
};

// How many motors
#define NMOTORS 2

// Pins                         motor one has pins 4,3, as input 20 and 21 are the driver pins, motor 2 has pins 3,4 as input and 10 and 11 as output to the motor driver
const int enca[] = {2,3};  
const int encb[] = {5,4};
const int pwm[] = {9,10};
const int in1[] = {6,11};

// Globals
long prevT = 0;
volatile int posi[] = {0,0};

// PID class instances
SimplePID pid[NMOTORS];

//Pontentiometers
const int pot[] = {A1,A2};

void setup() {
  Serial.begin(9600);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);

    pinMode(pot[k],INPUT);

    pid[k].setParams(1,0.01,0,255);
  }
  
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
}

void loop() {

  // set target position
  int target[NMOTORS];

  target[0] =  30*analogRead(pot[0]);  // 2750*sin(prevT/1e6);  // analogRead(pot[0]);  //  2750*sin(prevT/1e6);  //
  target[1] =  2*analogRead(pot[1]);  //target[0];  // 2750*sin(prevT/1e6);   // analogRead(pot[1]);  // 2750*sin(prevT/1e6);   //


  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position

  int pos[NMOTORS];
  noInterrupts(); // disable interrupts temporarily while reading
  for(int k = 0; k < NMOTORS; k++){
     pos[k] = posi[k];
    }
  interrupts(); // turn interrupts back on
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k]);
  }

}

void setMotor(int dir, int pwr, int pwm, int in1){
  if(dir == 1){
    digitalWrite(in1,HIGH);
    analogWrite(pwm,255 - pwr);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    analogWrite(pwm,pwr);
  }
  else{
    digitalWrite(in1,LOW);
    analogWrite(pwm,LOW);
  }  
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}
