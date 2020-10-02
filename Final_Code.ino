
/***************PINS***************/
/********MOTOR PINS********/
//Right Motor
const int BIN1 = 8;
const int BIN2 = 9;
const int PWMB = 10;

//Left Motor
const int AIN1 = 13;
const int AIN2 = 12;
const int PWMA = 11;



/********SENSORS********/
//Ultrasonic
const int backTrigPin = 2;
const int backEchoPin = 3;
const int leftTrigPin = 7;
const int leftEchoPin = 6;
const int rightTrigPin = 5;
const int rightEchoPin = 4;

//Flex Sensor
const int flFlex = A0;
const int frFlex = A1;
const int fFlex = A2;

//Light Sensors
const int flPhoto = A3;
const int frPhoto = A4;
const int fPhoto = A5;



/***************VARIABLES***************/
//MOTOR SPEED
const int MAX_MOTOR_SPEED = 165;

//FLEX SENSOR DIFF
const int FLEX_SENSOR_MIN_DIFF = 35;

//ULTRASONIC ACCURATE DISTANCE
const float  MAX_ACCURATE_DISTANCE = 20.000;

//PHOTORESISTOR DIFF
const int PHOTO_MAX_DIFF = 50;
const int PHOTO_MIN_DIFF = 20;

//TIMEOUT
unsigned int startTime;
unsigned int timeBreak;

//COLLISONS
bool fCollision = false; //If equal to true there is a collision, if equal to false there is no collision
bool frCollision = false;
bool flCollision = false;
bool bCollision = false;
float ultrasonicCollisionDistance = 3.5;

//FLEX RESTING STATE
int fRest;
int flRest;
int frRest;

//RANDOM TURN DIRECTION
bool turnDirection;

//Choosing Turn Direction While Eliminating False Values
int timesThrough = 0;



/////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  delay(10000);
  /***************SERIAL***************/
  Serial.begin(9600);
  
  /***************PIN MODES***************/
  /********MOTOR PINS********/
  //Right Motor
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  //Left Motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);



  /********SENSORS********/
  //Rear Ultrasonic
  pinMode(backTrigPin, OUTPUT);
  pinMode(backEchoPin, INPUT);

  //Left Ultrasonic
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);

  //Right Ultrasonic
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  
  //Flex Sensor
  pinMode(flFlex, INPUT);
  pinMode(frFlex, INPUT);
  pinMode(fFlex, INPUT);

  //Photoresistor
  pinMode(flPhoto, INPUT);
  pinMode(frPhoto, INPUT);
  pinMode(fPhoto, INPUT);
  

/*ONLY FOR DEBUGGING
  while (!Serial) {
    ;
  }
  */
  fRest = analogRead(fFlex);
  flRest = analogRead(flFlex);
  frRest = analogRead(frFlex);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  driveForward ();
  scanCollisions ();
  while (fCollision == false && frCollision == false && flCollision == false) {
    scanCollisions();
    adjustDirection();
  }

  if (fCollision == true) {
    frontCollision();
    } else if (frCollision == true) {//COLLISION TO THE RIGHT OF THE ROBOT
    stopMotors();
    dimeLeft();
    while (frCollision == true) {
      scanFrontRightCollision();
    }
    delay (175);
    stopMotors();
    
  } else if (flCollision == true) {//COLLISION TO THE LEFT OF THE ROBOT
    stopMotors();
    dimeRight();
    while (flCollision == true) {
      scanFrontLeftCollision();
    }
    delay (175);
    stopMotors();
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void driveForward () {
  //RIGHT MOTOR
  analogWrite(PWMB, MAX_MOTOR_SPEED);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  //LEFT MOTOR
  analogWrite(PWMA, MAX_MOTOR_SPEED);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void driveBackward () {
  //RIGHT MOTOR
  digitalWrite (BIN1, LOW);
  digitalWrite (BIN2, HIGH);
  analogWrite (PWMB, MAX_MOTOR_SPEED);

  //LEFT MOTOR
  digitalWrite (AIN1, LOW);
  digitalWrite (AIN2, HIGH);
  analogWrite (PWMA, MAX_MOTOR_SPEED);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void stopMotors () {
  //RIGHT MOTOR
    digitalWrite (BIN1, LOW);
    digitalWrite (BIN2, LOW);

    //LEFT MOTOR
    digitalWrite (AIN1, LOW);
    digitalWrite (AIN2, LOW);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void pivotRight () {
  //RIGHT MOTOR
  digitalWrite (BIN1, LOW);
  digitalWrite (BIN2, LOW);
  analogWrite (PWMB, MAX_MOTOR_SPEED);

  //LEFT MOTOR
  digitalWrite (AIN1, HIGH);
  digitalWrite (AIN2, LOW);
  analogWrite (PWMA, MAX_MOTOR_SPEED);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void dimeRight () {
  //RIGHT MOTOR
  digitalWrite (BIN1, LOW);
  digitalWrite (BIN2, HIGH);
  analogWrite (PWMB, MAX_MOTOR_SPEED);

  //LEFT MOTOR
  digitalWrite (AIN1, HIGH);
  digitalWrite (AIN2, LOW);
  analogWrite (PWMA, MAX_MOTOR_SPEED);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void reversePivotRight() {
  //RIGHT MOTOR
  digitalWrite (BIN1, LOW);
  digitalWrite (BIN2, LOW);
  analogWrite (PWMB, MAX_MOTOR_SPEED);

  //LEFT MOTOR
  digitalWrite (AIN1, LOW);
  digitalWrite (AIN2, HIGH);
  analogWrite (PWMA, MAX_MOTOR_SPEED);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void pivotLeft () {
  //RIGHT MOTOR
  digitalWrite (BIN1, HIGH);
  digitalWrite (BIN2, LOW);
  analogWrite (PWMB, MAX_MOTOR_SPEED);

  //LEFT MOTOR
  digitalWrite (AIN1, LOW);
  digitalWrite (AIN2, LOW);
  analogWrite (PWMA, MAX_MOTOR_SPEED);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void dimeLeft () {
  //RIGHT MOTOR
  digitalWrite (BIN1, HIGH);
  digitalWrite (BIN2, LOW);
  analogWrite (PWMB, MAX_MOTOR_SPEED);

  //LEFT MOTOR
  digitalWrite (AIN1, LOW);
  digitalWrite (AIN2, HIGH);
  analogWrite (PWMA, MAX_MOTOR_SPEED);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void reversePivotLeft () {
  //RIGHT MOTOR
  digitalWrite (BIN1, LOW);
  digitalWrite (BIN2, HIGH);
  analogWrite (PWMB, MAX_MOTOR_SPEED);

  //LEFT MOTOR
  digitalWrite (AIN1, LOW);
  digitalWrite (AIN2, LOW);
  analogWrite (PWMA, MAX_MOTOR_SPEED);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void scanBackCollision () {
  bCollision = false;
  
  float echoTime;
  float calcDistance;

  digitalWrite (backTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite (backTrigPin, LOW);
  echoTime = pulseIn(backEchoPin, HIGH);
  calcDistance = (echoTime/148); //Distance is calculated in inches 
  
  if (calcDistance > ultrasonicCollisionDistance) {
    bCollision = false;
    return;
    
  } else if (calcDistance <= ultrasonicCollisionDistance) {
    bCollision = true;
    return;
    
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void scanFrontCollision() {
  fCollision = false;
  int b = analogRead(fFlex);
  if (b <= (fRest - FLEX_SENSOR_MIN_DIFF)) {
    fCollision = true;
  } else {
    fCollision = false;
  }
  /* FOR DEBUGGING
  Serial.print("Front: ");
  Serial.print(b);
  Serial.print(" ,");
  Serial.println(fCollision);
  delay(500);
  */
  
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void scanFrontLeftCollision() {
  int b = analogRead(flFlex);
  if (b <= (flRest - FLEX_SENSOR_MIN_DIFF)) {
    flCollision = true;
  } else {
    flCollision = false;
  }
  /*
  Serial.print("Front Left: ");
  Serial.print(b);
  Serial.print(" , ");
  Serial.print("Rest: ");
  Serial.print(flRest);
  Serial.print (" , ");
  Serial.println(flCollision);
  delay(500);
  */

}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void scanFrontRightCollision() {
  int b = analogRead(frFlex);
  if (b <= (frRest - FLEX_SENSOR_MIN_DIFF)) {
    frCollision = true;
  } else {
    frCollision = false;
  }
  /*
  Serial.print("Front Right: ");
  Serial.print(b);
  Serial.print(" ,");
  Serial.println(frCollision);
  delay(500);
  */

}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void scanCollisions() {
  scanFrontCollision();
  scanFrontRightCollision();
  scanFrontLeftCollision();
  Serial.println(" ");
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
float readLeftUltra() {
  float echoTime;
  float calcDistance;

  digitalWrite (leftTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite (leftTrigPin, LOW);
  echoTime = pulseIn(leftEchoPin, HIGH);
  calcDistance = (echoTime/148); //Distance is calculated in inches 
  return calcDistance;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
float readRightUltra() {
  float echoTime;
  float calcDistance;

  digitalWrite (rightTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite (rightTrigPin, LOW);
  echoTime = pulseIn(rightEchoPin, HIGH);
  calcDistance = (echoTime/148); //Distance is calculated in inches 
  return calcDistance;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void chooseTurnDirection() {
  float l = readLeftUltra();
  delay(50);
  float r = readRightUltra();
  //Serial.print("Times Through: ");
  //Serial.print(timesThrough);
  //Serial.print(". Left: ");
  //Serial.print(l);
  //Serial.print (". Right: ");
  //Serial.println(r);
  //Serial.print("Turn Direction: ");

  if (timesThrough <= 2) {
    if (l < MAX_ACCURATE_DISTANCE && r < MAX_ACCURATE_DISTANCE) {
      //0 = Left turn
      //1 = Right Turn
      if (l > r) {
        turnDirection = 0;
        timesThrough = 0;
        //Serial.println ("Left");
        //Serial.println();
      } else if (r > l) {
        turnDirection = 1;
        timesThrough = 0;
        //Serial.println ("Right");
        //Serial.println();

      } else if (r == l) {
        timesThrough ++;

        //Serial.println ("Random");
        //Serial.println();
        chooseTurnDirection();
      }
      timesThrough = 0;
    } else if (l >= MAX_ACCURATE_DISTANCE && r < MAX_ACCURATE_DISTANCE) {
      turnDirection = 0;
      timesThrough = 0;
      //Serial.println ("Left");
      //Serial.println();
    } else if (l < MAX_ACCURATE_DISTANCE && r >= MAX_ACCURATE_DISTANCE) {
      turnDirection = 1;
      timesThrough = 0;
      //Serial.println ("Right");
      //Serial.println();

    } else {
      timesThrough ++;
      //Serial.println ("Random");
      //Serial.println();
      chooseTurnDirection();
    }
  } else {
    turnDirection = random(0, 2);
    //Serial.print("Random");
    //Serial.println();
    timesThrough = 0;
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void setTimeBreak(int t){
  timeBreak = t + millis();
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void frontCollision() {
  stopMotors();
    // 0 = Left Turn
    // 1 = Right Turn
    chooseTurnDirection();
    driveBackward();
    scanBackCollision();
    scanFrontCollision();
    while(fCollision == true && bCollision == false) {
      scanBackCollision();
      scanFrontCollision();
    }
    if (fCollision == false) {
      delay(250);
      stopMotors();
    } else if (bCollision == true) {
      stopMotors();
    }
    
    switch (turnDirection){
      case 1:
        reversePivotLeft();
        setTimeBreak(300);
        scanFrontCollision();
        scanFrontLeftCollision();
        while(fCollision == false && millis() < timeBreak) {
          scanFrontCollision();
          scanFrontLeftCollision();
        }
        if (fCollision == true) {
          frontCollision();
        } else {
          stopMotors();
          break;
        }
      case 0:
        reversePivotRight();
        setTimeBreak(300);
        scanFrontCollision();
        while(fCollision == false && millis() < timeBreak) {
          scanFrontCollision();
        }
        if (fCollision == true) {
          frontCollision();
        } else {
          stopMotors();
          break;
        } 
    }     
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
void adjustDirection() {
  int r = analogRead(frPhoto);
  int l = analogRead(flPhoto);
  int f = analogRead(fPhoto);
  if (f > 900) {
    stopMotors();
    while (analogRead(fPhoto) > 900) {
      
    }
  } else if (r - PHOTO_MAX_DIFF > f && r > l) { // Right side is the brightest
    stopMotors();
    dimeRight();
    setTimeBreak(1000);
    while (analogRead(fPhoto) + PHOTO_MIN_DIFF < r && millis() <= timeBreak) {
      //Do nothing 
    }
    stopMotors();
    driveForward();
  } else if (l - PHOTO_MAX_DIFF > f && l > r ) { // Left side is the brightest
    stopMotors();
    dimeLeft();
    setTimeBreak(1000);
    while (analogRead(fPhoto) + PHOTO_MIN_DIFF < l && millis() <= timeBreak) {
      //Do nothing
    }
    stopMotors();
    driveForward();
  } else { // Front is the brightest but less than 900
    // Do nothing
  }
}
