//Including all the necessary libraries to used for this assignment.
#include <Wire.h>
#include <avr/wdt.h>
#include "MPU6050.h"
#include "MPU6050_getdata.h"
#include "DeviceDriverSet_xxx0.h"
// #include "MPU6050_tockn.h"

//Creating a variable to store the number of rows and coloumns in the occupancy grid.
const int GRIDDIMENSIONS = 10;
const int NUM_MEASUREMENTS = 5;

//Setting the MPU6050 Accelegryo component.
MPU6050_getdata AppMPU6050getdata;

//Setting the MPU6050 Accelegryo component.
// MPU6050 mpu6050(Wire);

//Creating a new servo object.
DeviceDriverSet_Servo AppServo;

//Creating and initialising the variable to store the initial time.
float initial_time = 0.0;
float angle = 0.0;
float current_angle = 0.0;

//Creating a variable to store the duratrion of time passed since the start of the program.
long duration;

//Creating variables for x and y axis.
int x = 0;
int y = 0;

int moveForwardCount = 0;
int turnRightCount = 0;
int turnLeftCount = 0;
int squaresX = 0;
int squaresY = 0;

int toReturnX = -1;
int toReturnY = -1;

//Original/starting variables for X and Y.
int Ox = 10;
int Oy = 10;

//Variables for X and Y when robot has localised itself.
int Nx = Ox;
int Ny = Oy;

int cornerType = 0;

//Grid 10x10 to act as occupancy grid.
int GRID1[GRIDDIMENSIONS][GRIDDIMENSIONS];

//Grid 20x20 to act as localisation and mapping grid.
int GRID2[20][20];

int freeCell[6] = {-1};

bool turnedLeft = false;
bool turnedRight = false;
bool foward = false;
bool back = false;
bool left = false;
bool right = false;
bool stop = false;
bool robotLocalised = false;
bool foundCorner = false;

//Starting function for the program.
void setup() 
{
  //Initialising the serial port.
  Serial.begin(9600);

  //Defining the pins to be used by the board.
  #define PIN_Motor_STBY 3
  #define PIN_Motor_PWMA 5  
  #define PIN_Motor_AIN_1 7
  #define PIN_Motor_PWMB 6
  #define PIN_Motor_BIN_1 8
  #define PIN_ITR20001xxxL A2
  #define PIN_ITR20001xxxM A1
  #define PIN_ITR20001xxxR A0
  #define TrackingDetection_S 40
  #define TrackingDetection_E 270
  #define echoPin 12 
  #define trigPin 13


  //Defining the pi numerical constant.
  #define PI 3.1415926535897932384626433832795

  //Initialising the pin modes for all the individual component pins.
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialising the accelegyro component.
  AppMPU6050getdata.MPU6050_dveInit();

  //Calibrating the accelegyro sensor.
  AppMPU6050getdata.MPU6050_calibration();

  Wire.begin();
  // mpu6050.begin();
  // mpu6050.calcGyroOffsets(true);

  //Initialising the servo to point directly forward.
  AppServo.DeviceDriverSet_Servo_Init(90);

  //Setting the initial time.
  initial_time = millis();

  initialiseGrid1();
  
  initialiseGrid2();

  initialiseRobotPosition();

  float cur_angle;
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&cur_angle);

  AppMPU6050getdata.MPU6050_calibration();

  delay(3000);
}


void SignalLocalised()
{
  for(int i = 0; i < 10;i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(1000);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(1000);
  }
}



void Map()
{
  delay(3000);  

  PanServo();

  delay(1000);

  CheckFree();

  printGrid2();

  Serial.print("Nx: ");
  Serial.println(Nx);

  Serial.print("Ny: ");
  Serial.println(Ny);

  Serial.print("Cells travelled in the y-axis: ");
  Serial.println(y);

  Serial.print("Cells travelled in the x-axis: ");
  Serial.println(x);

  Serial.println("===================================================================================================");
}



int IdentifyCorner()
{
  //The robot is in corner (0,10).
  if((Nx <= 10) && (Ny >= 10))
  {
    // Serial.println("(0,10)");
    cornerType = 1;
    foundCorner = true;
    stopMov();
    LocaliseRobot(Nx, Ny, cornerType);
    stopMov();
  }

  //The robot is in corner (0,0).
  else if((Nx <= 10) && (Ny <= 10))
  {
    // Serial.println("(0,0)");
    cornerType = 2;
    foundCorner = true;
    stopMov();
    LocaliseRobot(Nx, Ny, cornerType);
    stopMov();
  }

  //The robot is in corner (10,10).
  else if((Nx >= 10) && (Ny >= 10))
  {
    // Serial.println("(10,10)");
    cornerType = 3;
    foundCorner = true;
    stopMov();
    LocaliseRobot(Nx, Ny, cornerType);
    stopMov();
  }

  //The robot is in corner (10,0).
  else if((Nx >= 10) && (Ny <= 10))
  {
    // Serial.println("(10,0)");
    cornerType = 4;
    foundCorner = true;
    stopMov();
    LocaliseRobot(Nx, Ny, cornerType);
    stopMov();
  }

  return cornerType;
}



void LocaliseRobot(int numX, int numY, int type)
{
  int differenceX = 0;
  int differenceY = 0;
  int startingPointX = 0;
  int startingPointY = 0;

  //Handling the case that the robot is at the corner (0,10).
  if(type == 1) //(0,10)
  {
    differenceX = numX - 0;
    differenceY = numY - 10;

    startingPointX = 10 - differenceX;
    startingPointY = 10 - differenceY;
  }

  //Handling the case that the robot is at the corner (0,0).
  else if(type == 2) // (0,0)
  {
    differenceX = numX - 0;
    differenceY = numY - 0;

    startingPointX = (10 - differenceX) + 1;
    startingPointY = (10 - differenceY) + 1;
  }

  //Handling the case that the robot is at the corner (10,10).
  else if(type == 3) // (10,10)
  {
    differenceX = numX - 10;
    differenceY = numY - 10;

    startingPointX = 10 - differenceX;
    startingPointY = 10 - differenceY;
  }

  //Handling the case that the robot is at the corner (10,0).
  else if(type == 4) // (10,0)
  {
    differenceX = numX - 10;
    differenceY = numY - 0;

    startingPointX = 10 - differenceX - 1;
    startingPointY = 10 - differenceY;
  } 
  
  //Displaying information to the console.
  Serial.print("The robot started at position: ");
  Serial.print("(");
  Serial.print(startingPointX);
  Serial.print(",");
  Serial.print(startingPointY);
  Serial.println(")");

  stopMov();

  robotLocalised = true;
  foundCorner = true;

  GRID2[startingPointX][startingPointY] = 3;
}



//Function to move the motors forward.
//Parameters: Speed for the right pair of motors.
//            Speed for the left pair of motors.
void Fwd(uint8_t right, uint8_t left) 
{
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, left);
  analogWrite(PIN_Motor_PWMA, right);
  digitalWrite(PIN_Motor_STBY, HIGH); 
}



//Function to spin the tires and robot on its own axis to the left.
//Parameters: Speed for the right pair of motors.
//            Speed for the left pair of motors.
void SpinLeft(uint8_t right, uint8_t left) 
{
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, left);
  analogWrite(PIN_Motor_PWMA, right);
  digitalWrite(PIN_Motor_STBY, HIGH);
}



//Function to spin the tires and robot on its own axis to the right.
//Parameters: Speed for the right pair of motors.
//            Speed for the left pair of motors.
void SpinRight(uint8_t right, uint8_t left) 
{
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, left);
  analogWrite(PIN_Motor_PWMA, right);
  digitalWrite(PIN_Motor_STBY, HIGH); 
}



//Function to stop the robot whilst moving.
void stopMov()
{
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
  digitalWrite(PIN_Motor_STBY, LOW);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, LOW);
}



//Function to turn the servo directly forward, to the left and to the right.
void PanServo()
{
  initialiseFreeCellArray();
  
  //Turning the servo to the FRONT.
  AppServo.DeviceDriverSet_Servo_control(90);
  if(checkObstacle() == true)
  {
    freeCell[0] = -1;
    freeCell[1] = -1;

    //Checking robot heading.
    int direction = BearingCheck();
    
    //Robot facing forward.
    if(direction == 1)
    {
      GRID2[Nx][Ny+1] = 2;
    }

    //Robot facing backward.
    else if(direction == 2)
    {
      GRID2[Nx][Ny-1] = 2;      
    }

    //Robot facing Right.    
    else if(direction == 3)
    {
      GRID2[Nx+1][Ny] = 2;
    }

    //Robot Facing Left.
    else if(direction == 4)
    {
      GRID2[Nx-1][Ny] = 2;
    }
  }
  //No obstacle infront of the robot.
  else
  {
    //Checking robot heading.
    int direction = BearingCheck();
    
    //Robot facing North.
    if(direction == 1)
    {
      freeCell[0] = (Nx);
      freeCell[1] = (Ny+1);
    }

    //Robot facing South.
    else if(direction == 2)
    {
      freeCell[0] = (Nx);
      freeCell[1] = (Ny-1);
    }

    //Robot facing East.    
    else if(direction == 3)
    {
      freeCell[0] = (Nx+1);
      freeCell[1] = (Ny);
    }

    //Robot Facing West.
    else if(direction == 4)
    {
      freeCell[0] = (Nx-1);
      freeCell[1] = (Ny);
    }
  }



  //Turning the servo to the RIGHT.
  AppServo.DeviceDriverSet_Servo_control(0);
  if(checkObstacle() == true)
  {
    freeCell[2] = -1;
    freeCell[3] = -1;

    //Checking robot heading.
    int direction = BearingCheck();
    
    //Robot facing North.
    if(direction == 1)
    {
      GRID2[Nx+1][Ny] = 2;
    }

    //Robot facing South.
    else if(direction == 2)
    {
      GRID2[Nx-1][Ny] = 2;    
    }

    //Robot facing East.    
    else if(direction == 3)
    {
      GRID2[Nx][Ny-1] = 2;
    }

    //Robot Facing West.
    else if(direction == 4)
    {
      GRID2[Nx][Ny+1] = 2;
    }
  }
  else
  {
    //Checking robot heading.
    int direction = BearingCheck();
    
    //Robot facing North.
    if(direction == 1)
    {
      freeCell[2] = (Nx+1);
      freeCell[3] = (Ny);
    }

    //Robot facing South.
    else if(direction == 2)
    {     
      freeCell[2] = (Nx-1);
      freeCell[3] = (Ny);
    }

    //Robot facing East.    
    else if(direction == 3)
    {
      freeCell[2] = (Nx);
      freeCell[3] = (Ny-1);
    }

    //Robot Facing West.
    else if(direction == 4)
    {
      freeCell[2] = (Nx);
      freeCell[3] = (Ny+1);
    }
  }

  

  //Turning the servo to the LEFT.
  AppServo.DeviceDriverSet_Servo_control(180);
  if(checkObstacle() == true)
  {
    freeCell[4] = -1;
    freeCell[5] = -1;

    //Checking robot heading.
    int direction = BearingCheck();
    
    //Robot facing North.
    if(direction == 1)
    {
      GRID2[Nx-1][Ny] = 2;
    }

    //Robot facing South.
    else if(direction == 2)
    {
      GRID2[Nx+1][Ny] = 2;      
    }

    //Robot facing East.    
    else if(direction == 3)
    {
      GRID2[Nx][Ny+1] = 2;
    }

    //Robot Facing West.
    else if(direction == 4)
    {
      GRID2[Nx][Ny-1] = 2;
    }
  }
  else
  {
    //Checking robot heading.
    int direction = BearingCheck();
    
    //Robot facing North.
    if(direction == 1)
    {
      freeCell[4] = (Nx-1);
      freeCell[5] = (Ny);
    }

    //Robot facing South.
    else if(direction == 2)
    {
      freeCell[4] = (Nx+1);
      freeCell[5] = (Ny);  
    }

    //Robot facing East.    
    else if(direction == 3)
    {
      freeCell[4] = (Nx);
      freeCell[5] = (Ny+1);
    }

    //Robot Facing West.
    else if(direction == 4)
    {
      freeCell[4] = (Nx);
      freeCell[5] = (Ny-1);
    }
  }
  
  CheckCorner();

  AppServo.DeviceDriverSet_Servo_control(90);
}



void CheckCorner()
{
  if(freeCell[0] == -1  && freeCell[1] == -1 && freeCell[2] == -1 && freeCell[3] == -1)
  {
    // Serial.print("Identified corner: ");
    IdentifyCorner();
    stopMov();
  }
  if(freeCell[0] == -1 && freeCell[1] == -1 && freeCell[4] == -1 && freeCell[5] == -1)
  {
    // Serial.print("Identified corner: ");
    IdentifyCorner();
    stopMov();
  }
}



void CheckFree()
{
  int preferredX = -1;
  int preferredY = -1;
  int direction = -1;
  bool undisc = false;
  
  for(int i = 0; i < 6; i += 2)
  {
    if(freeCell[i] != -1 && freeCell[i+1] != -1)
    {
      // Serial.println("Free Cell is not empty");
      
      if(GRID2[freeCell[i]][freeCell[i+1]] == 1)   
      {
        // Serial.println("This cell has been previously discovered before.");

        Serial.print("preferredX value: ");
        Serial.println(preferredX);

        Serial.print("preferredY value: ");
        Serial.println(preferredY);    
           
        if(preferredX == -1)
        {
          preferredX = freeCell[i];
          preferredY = freeCell[i+1];
          
          if(GRID2[freeCell[i]][freeCell[i+1]] == 2)
          {
            direction = 1;
          }
          else
          {
            direction = 0;            
          }
        }
      }        

      else
      {
        if(undisc == false)
        {
          undisc = true;
          preferredX = freeCell[i];
          preferredY = freeCell[i+1];

          GRID2[freeCell[i]][freeCell[i+1]] = 1;  

          //front
          if (i == 0)
          {
            direction = 0;
          }

          //right
          else if (i == 2)
          {
            direction = 1;              
          }
          
          //left.
          else if (i == 4)
          {
            direction = 2;
          }
        }
        else
        {
          GRID2[freeCell[i]][freeCell[i+1]] = 1;                
        }          
      }
    }
  }

  toReturnX = preferredX;
  toReturnY = preferredY;

  // Serial.print("Chosen direction: ");
  // Serial.println(direction);

  if(foundCorner != true)
  {
    //Since the path is clear, the prioritised direction is straight, therefore the robot will kepp moving forward.
    if(direction == 0)
    {
      moveOneCell();
    }

    //Executing evasive manouvers by turning right and moving one cell.
    else if(direction == 1)
    {
      float cur_angle;
      AppMPU6050getdata.MPU6050_dveGetEulerAngles(&cur_angle);
      turnRight(cur_angle);
      moveOneCell();
    }
    
    //Executing evasive manouvers by turning left and moving one cell.
    else if(direction == 2)
    {
      float cur_angle;
      AppMPU6050getdata.MPU6050_dveGetEulerAngles(&cur_angle);
      turnLeft(cur_angle);
      moveOneCell();
    }
  }

  
  printFreeCellArray();

  initialiseFreeCellArray();
}



void printFreeCellArray()
{
  // Serial.println("FreeCell Array:");
  for(int i = 0; i < 6; i++)
  {
    Serial.println(freeCell[i]);
  }
}



void initialiseFreeCellArray()
{
  for(int i = 0; i < 6; i++)
  {
    freeCell[i] == -1;
  }
}



//Function which checks is there is an obstacle infront of the sensor.
bool checkObstacle()
{
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  long distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  
  //Detects the object within 27 cm of the sensor.
  if (distance < 27)
  {
    //Object detected - stop car
    stopMov();
    return true;
  }

  //Default value.
  return false;
}



void initialiseGrid1()
{
  // loop through array's rows
  for (int i = 0; i < GRIDDIMENSIONS; ++i) 
  {
    // loop through columns of current row
    for (int j = 0; j < GRIDDIMENSIONS; ++j)
    {
      GRID1[i][j] = 0;
    }
  } 
}



void initialiseGrid2()
{
  // loop through array's rows
  for (int i = 0; i < 20; ++i) 
  {
    // loop through columns of current row
    for (int j = 0; j < 20; ++j)
    {
      GRID2[i][j] = 0;
    }
  } 
}



void initialiseRobotPosition()
{
  GRID2[Ox][Oy] = 1;
}



void printGrid2() 
{
  Serial.println("Current 20x20 grid contents:");
  
  // loop through array's rows
  for (int i = 0; i < 20; ++i) 
  {
    // loop through columns of current row
    for (int j = 0; j < 20; ++j)
    {
      Serial.print(GRID2[i][j]);
      Serial.print(" ");
    }
    Serial.print('\n') ; // start new line of output
  } 
}

void printGrid1() 
{
  // loop through array's rows
  for (int i = 0; i < 10; i++) 
  {
    // loop through columns of current row
    for (int j = 0; j < 10; i++)
    {
      Serial.print(GRID1[i][j]);
      Serial.print(" ");
    }
    Serial.print('\n') ; // start new line of output
  } 
} 



void moveOneCell()
{
  int new_dir = BearingCheck();

  switch(new_dir)
  {
    //Forward - Add Y.
    case 1:
      Ny = Ny + 1;
      y = y + 1;
    break;

    //Backward - Subtract Y.
    case 2:
      Ny = Ny - 1;
      y = y - 1;
    break;

    //Right - Add X.
    case 3:
      Nx = Nx + 1;
      x = x + 1;
    break;

    //Left - Subtract X.
    case 4:
      Nx = Nx - 1;
      x = x - 1;
    break;
        
    default:
    break;    
  }
  
  // Fwd(85,70);
  // delay(1250); 

  //For fully charged battery:
  Fwd(80,72);
  delay(1090); 
  stopMov();

  GRID2[Nx][Ny] = 1;

  int direction = BearingCheck();
  Serial.print("Current direction after moving one cell: ");
  Serial.println(direction);
}



void turnLeft(float current_angle)
{
  bool stop = false;

  float target = current_angle - 75.0;

  while(!stop)
  {
    float angle;
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&angle);
    // Serial.println(angle);

    if(angle <= target)
    {
      stop = true;
      stopMov();
      break;
    }

    SpinLeft(60,58);
  }

  int direction = BearingCheck();
}



void turnRight(float current_angle)
{
  bool stop = false;

  float target = current_angle + 78.0;

  while(!stop)
  {
    float angle;
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&angle);
    // Serial.println(angle);

    if(angle >= target)
    {
      stop = true;
      stopMov();
      break;
    }

    SpinRight(60,50);
  }

  int direction = BearingCheck();
}



int BearingCheck()
{
  int curDir = 0;
  
  float newAngle;

  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&newAngle);

  // Serial.print("Current angle: ");
  // Serial.println(newAngle);

  if(newAngle > 360)
  {
    newAngle = 360 - newAngle;
  }
  else if(newAngle < -360)
  {
    newAngle = 360 + newAngle;
  }

  if((newAngle < 45 && newAngle > -45) || (newAngle < -315 && newAngle > 305))
  {
    // Serial.println("Robot is facing North");
    foward = true;
    back = false;
    left = false;
    right = false;
    curDir = 1;
    // Serial.println("");
  }

  else if((newAngle < -225 && newAngle > -135) || (newAngle > 135 && newAngle < 225))
  {
    // Serial.println("Robot is facing South");
    foward = false;
    back = true;
    left = false;
    right = false;
    curDir = 2;
    // Serial.println("");
  }

  else if((newAngle > 45 && newAngle < 135) || (newAngle < -315 && newAngle > -225))
  {
    // Serial.println("Robot is facing East");
    foward = false;
    back = false;
    left = false;
    right = true;
    curDir = 3;
    // Serial.println("");
  }

  else if((newAngle > 225 && newAngle < 315) || (newAngle < -45 && newAngle > -135))
  {
    // Serial.println("Robot is facing West");
    foward = false;
    back = false;
    left = true;
    right = false;
    curDir = 4;
    // Serial.println("");
  }

  return curDir;
}



void SendInfoToPi()
{
  while(true)
  {
    if(Serial.available() > 0)
    {
      printGrid2();      
    }
  }
}



void loop()
{
  float newAngle;
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&newAngle);

  while(robotLocalised != true)
  {
    Map();
    int direction = BearingCheck();
  }

  if(robotLocalised == true)
  {
    SignalLocalised();
    SendInfoToPi();
  }

  int direction = BearingCheck();
}