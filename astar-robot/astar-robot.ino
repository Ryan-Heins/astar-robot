#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <avr/pgmspace.h>
#include <Servo.h>

/*track 1*/
#define GRAPH_WIDTH 3
#define GRAPH_HEIGHT 6
#define SIZE_OBSTACLE_X 0
#define START_DIRECTION 0
//0 = north, 1 = east, 2 = south, 3 = west
const PROGMEM uint16_t obstacleX[] = {};
const PROGMEM uint16_t obstacleY[] = {};
const PROGMEM uint8_t srcXandY[] = {0, 0};
const PROGMEM uint8_t destXandY[] = {0, 5};
/*track 2
#define GRAPH_WIDTH 4
#define GRAPH_HEIGHT 8
#define SIZE_OBSTACLE_X 0
#define START_DIRECTION 0
//0 = north, 1 = east, 2 = south, 3 = west
const PROGMEM uint16_t obstacleX[] = {};
const PROGMEM uint16_t obstacleY[] = {};
const PROGMEM uint8_t srcXandY[] = {2, 1};
const PROGMEM uint8_t destXandY[] = {2, 5};
*/
/*track 3
#define GRAPH_WIDTH 4
#define GRAPH_HEIGHT 8
#define SIZE_OBSTACLE_X 0
#define START_DIRECTION 0
//0 = north, 1 = east, 2 = south, 3 = west
const PROGMEM uint16_t obstacleX[] = {};
const PROGMEM uint16_t obstacleY[] = {};
const PROGMEM uint8_t srcXandY[] = {0, 0};
const PROGMEM uint8_t destXandY[] = {2, 5};
*/
/*track 4 bonus
#define GRAPH_WIDTH 4
#define GRAPH_HEIGHT 8
#define SIZE_OBSTACLE_X 0
#define START_DIRECTION 0
//0 = north, 1 = east, 2 = south, 3 = west
const PROGMEM uint16_t obstacleX[] = {};
const PROGMEM uint16_t obstacleY[] = {};
const PROGMEM uint8_t srcXandY[] = {2, 2};
const PROGMEM uint8_t destXandY[] = {0, 7};
*/

//used by isObstacle, isDest, isSource, getH

//ultrasonic pins
const PROGMEM int trig = 9;
const PROGMEM int echo = 10;
//servo pin
const PROGMEM int controlPin = 7;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(513, 1);
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(513, 2);

//test functions
void printCoor(unsigned int G[]) {
  for (int i = 0; i < GRAPH_HEIGHT * GRAPH_WIDTH; i++) {
    Serial.print("Graph Index: ");
    Serial.print(i);
    Serial.print(" X: ");
    Serial.print(getXCoor(i));
    Serial.print(" Y: ");
    Serial.print(getYCoor(i));
    
    Serial.print("\n");
  }  
}
void printGraphArray(unsigned int G[]) {
  //printCoor(G);
  for (int i = 0; i < GRAPH_HEIGHT * GRAPH_WIDTH; i++) {
    Serial.print("X: ");
    Serial.print(getXCoor(i));
    Serial.print("\tY: ");
    Serial.print(getYCoor(i));
    Serial.print("\tIndex ");
    Serial.print(": ");
    Serial.print(i);
    /*
    Serial.print("\tCalculated Index: ");
    Serial.print(getIndex(getXCoor(i),getYCoor(i))); 
    */
    Serial.print("\t");
    for (int j = 15; j >= 0; j--) {
      Serial.print(bitRead(G[i], j)); 
    }
    Serial.print("\tg: ");
    Serial.print(getG(G, i));
    Serial.print("\tH: ");
    Serial.print(getH(i)); 
    Serial.print("\n");
  }
}

//given graph index, sets G value
void setG(unsigned int G[], int graph_index, unsigned int g) {
  for(int i = 5; i < 14; i++) {
    bitWrite(G[graph_index], i, bitRead(g, i - 5)); 
  }
}

//gets G value for given graph_index
unsigned int getG(unsigned int G[], int graph_index) {
  unsigned int g = 0;
  for (int i = 5; i < 14; i++) {
    bitWrite(g, i-5, bitRead(G[graph_index], i));
  }
  return g;
}

//returns h value given index of node
unsigned char getH(int i) {
  unsigned char h = 0;
  if (destXandY[0] > getXCoor(i)) {
    h += (destXandY[0] - getXCoor(i));
  } else {
    h += (getXCoor(i) - destXandY[0]);
  }
  if (destXandY[1] > getYCoor(i)) {
    h += (destXandY[1] - getYCoor(i));
  } else {
    h += (getYCoor(i) - destXandY[1]);
  }
  if ((destXandY[0] != getXCoor(i)) && (destXandY[1] != getYCoor(i))) {
    h += 1;
  }
  return h;
}

unsigned int cost(unsigned int G[], int graph_index) {
  return (getH(graph_index) + getG(G, graph_index));
}

//gets graph_index given x and y coordinate
unsigned int getIndex(unsigned int x, unsigned int y) {
  return (y * GRAPH_WIDTH + x);
}

//returns x or y coor given index of node
unsigned char getXCoor(int graph_index) {
  return (graph_index % GRAPH_WIDTH);
}
unsigned char getYCoor(int graph_index) {
  return (graph_index / GRAPH_WIDTH);
}

//returns true if node is dest node
bool isDest(int graph_index) {
  if (getIndex(destXandY[0], destXandY[1]) == graph_index) {
    return true;
  } else {
    return false;
  }
}
//returns true if node is source node
bool isSource(int graph_index) {
  if (getIndex(srcXandY[0], srcXandY[1]) == graph_index) {
    return true;
  } else {
    return false;
  }
}

//returns true if given graph_index is a known obstacle
bool isObstacle(int graph_index) {
  bool retval;
  //Serial.println(graph_index);
  for (int i = 0; i < SIZE_OBSTACLE_X; i++) {
    /*
    Serial.print(getXCoor(graph_index));
    Serial.print(" =? ");
    Serial.println(pgm_read_word_near(obstacleX + i));
    Serial.print(getYCoor(graph_index));
    Serial.print(" =? ");
    Serial.println(pgm_read_word_near(obstacleY + i));
    */
    if ((getXCoor(graph_index) == pgm_read_word_near(obstacleX + i)) && (getYCoor(graph_index) == pgm_read_word_near(obstacleY + i))) {
      /*
      Serial.print("Obstacle found at x: ");
      Serial.print(getXCoor(graph_index));
      Serial.print(" y: ");
      Serial.println(getYCoor(graph_index));
      */
      return true;
    } else {
      retval = false;
    }
  }
  return retval;
}

bool queueEmpty(unsigned int G[]) {
  for (int i = 0; i < GRAPH_HEIGHT * GRAPH_WIDTH; i++) {
    if (bitRead(G[i], 3) == 1) {
      return false;
    }
  }
  return true;
}

void setDirection(unsigned int G[], int graph_index, unsigned int direction) {
  if (direction == 0) {
    bitWrite(G[graph_index], 0, 0);
    bitWrite(G[graph_index], 1, 0);
  }
  if (direction == 1) {
    bitWrite(G[graph_index], 0, 1);
    bitWrite(G[graph_index], 1, 0);
  }
  if (direction == 2) {
    bitWrite(G[graph_index], 0, 0);
    bitWrite(G[graph_index], 1, 1);
  }
  if (direction == 3) {
    bitWrite(G[graph_index], 0, 1);
    bitWrite(G[graph_index], 1, 1);
  }
}

//sets direction to nodes optimal previous node
void setParentDirection(unsigned int G[], int graph_index, unsigned int direction) {
  if (direction == 0) {
    bitWrite(G[graph_index], 14, 0);
    bitWrite(G[graph_index], 15, 0);
  }
  if (direction == 1) {
    bitWrite(G[graph_index], 14, 1);
    bitWrite(G[graph_index], 15, 0);
  }
  if (direction == 2) {
    bitWrite(G[graph_index], 14, 0);
    bitWrite(G[graph_index], 15, 1);
  }
  if (direction == 3) {
    bitWrite(G[graph_index], 14, 1);
    bitWrite(G[graph_index], 15, 1);
  }
}

//returns the direction the node is pointing
unsigned int nodePointing(unsigned int G[], int graph_index) {
  unsigned int direction = 0;
  if (bitRead(G[graph_index], 0) == 1) direction += 1;
  if (bitRead(G[graph_index], 1) == 1) direction += 2;
  /*
  Serial.print("nodePointing index ");
  Serial.print(graph_index);
  Serial.print(" direction : ");
  Serial.println(direction);
  */
  return direction;
}

unsigned int adjacentCost(unsigned int G[], unsigned int node, unsigned int adj_dir) {
  unsigned int dist;
  if (nodePointing(G, node) == adj_dir) {
    dist = 1; //straight
  } else if ((nodePointing(G, node) - 1 == adj_dir) || (nodePointing(G, node) + 1 == adj_dir)) {
    dist = 2; //90
  } else if ((nodePointing(G, node) - 3 == adj_dir) || (nodePointing(G, node) + 3 == adj_dir)) {
    dist = 2; //90  
  } else {
    dist = 3; //180
  }
  /*
  Serial.print("Current Node points ");
  Serial.println(nodePointing(G, node));
  Serial.print("Direction to adjacent ");
  Serial.println(adj_dir);
  Serial.print("Distance to adjacent ");
  Serial.println(dist);
  */
  return dist;
}

 //**********A star************//
void shortestPath(unsigned int path[], unsigned int G[], unsigned int source) {
  unsigned int retval;
  unsigned int minPriorityIndex = 0;
  bool done = false;

  unsigned int minCost, minG, minH, adjcost, adjG, adjH, currG, currH, currCost;
 
  while (!done) {
    if (source == 0) { 
      minPriorityIndex = -1;
    } else {
      minPriorityIndex = 0;
    }
    minPriorityIndex = 0;
    for (int i = 0; i < GRAPH_HEIGHT * GRAPH_WIDTH; i++) {
      currG = getG(G, i);
      currH = getH(i);
      currCost = currH + currG;
      minG = getG(G, minPriorityIndex);
      minH = getH(minPriorityIndex);
      minCost = minG + minH;
      if (bitRead(G[minPriorityIndex], 2) == 1) {
        minCost = 511;
      }
/*
      Serial.print("minPriorityIndex: ");
      Serial.print(minPriorityIndex);
      Serial.print("\tcurrent index: ");
      Serial.println(i);

      Serial.print("currG: ");
      Serial.print(currG);
      Serial.print("\tcurrH: ");
      Serial.print(currH);
      Serial.print("\tcurrCost: ");
      Serial.print(currCost);
      Serial.print("\tminG: ");
      Serial.print(minG);
      Serial.print("\tminH: ");
      Serial.print(minH);
      Serial.print("\tminCost: ");
      Serial.println(minCost);
*/
      if (((currCost) <= (minCost))&& (bitRead(G[i], 3) == 1) && (bitRead(G[i], 4) == 0)) {
        /*
        Serial.print("index, min:  ");
        Serial.print(i);
        Serial.print(", ");
        Serial.println(minPriorityIndex);
        */
      }      

      if ((currCost <= minCost) && (bitRead(G[i], 3) == 1) && (bitRead(G[i], 4) == 0)) {
        //if new min prior value found and is in queue and is not an obstacle
        //Serial.println(bitRead(G[i], 2));
        if (bitRead(G[i], 2) == 0) {
          minPriorityIndex = i; 
          /*
          Serial.print("\nminPriorityIndex = ");
          Serial.println(minPriorityIndex);
          Serial.print("minPriorityIndex G: ");
          Serial.println(getG(G, minPriorityIndex));   
          */
        }
          
        
      }
      /*
      if (((getG(G, i) + getH(i)) < (getG(G, minPriorityIndex) + getH(minPriorityIndex)))&& (bitRead(G[i], 3) == 1)) {
        Serial.println("cost less");
      }
      if (((getG(G, i) + getH(i)) < (getG(G, minPriorityIndex) + getH(minPriorityIndex))) && (bitRead(G[i], 3) == 1) && (bitRead(G[i], 4) == 0)) {
        //if new min prior value found and is in queue and is not an obstacle
        minPriorityIndex = i; 
        
        Serial.print("\nminPriorityIndex = ");
        Serial.println(minPriorityIndex);
        Serial.print("minPriorityIndex G: ");
        Serial.println(getG(G, minPriorityIndex));     
      }
      */
    }
    bitWrite(G[minPriorityIndex], 3, 0);      //sets not in queue
    bitWrite(G[minPriorityIndex], 2, 1);      //sets was in queue
    
    //for each adjacent
    /*
    Serial.print("(");
    Serial.print(getXCoor(minPriorityIndex));
    Serial.print(",");
    Serial.print(getYCoor(minPriorityIndex));
    Serial.print(") adjacent's:\n");
    */
    for (int i = getYCoor(minPriorityIndex) - 1; i <= getYCoor(minPriorityIndex) + 1; i++) {
      
      if (i < 0) continue;
      if (i == getYCoor(minPriorityIndex)) {
        for (int j = getXCoor(minPriorityIndex) - 1; j <= getXCoor(minPriorityIndex) + 1; j++) {
          if (j < 0 || j == getXCoor(minPriorityIndex)) continue;
          if (j >= GRAPH_WIDTH) break;
          //left and right adjacents(x = j, y = i)
          if (bitRead(G[getIndex(j, i)], 4) == 1) continue;
          /*
          Serial.print("(");
          Serial.print(j);
          Serial.print(",");
          Serial.print(i);
          Serial.print(") \n");
          */
          //if adjacent is destination, set directions, and be done
          if (isDest(getIndex(j, i))) {
            if (j == getXCoor(minPriorityIndex) - 1) {  //dest is to left, set destination
              setParentDirection(G, getIndex(j, i), 1);
              setDirection(G, getIndex(j, i), 3);
            } else {  //else dest is to right
              setParentDirection(G, getIndex(j, i), 3); 
              setDirection(G, getIndex(j, i), 1);
            }
            done = true;
          } else {
            if (j == getXCoor(minPriorityIndex) - 1) {  //adjacent to left
              //if adjacentdistance + minPriorityIndex G < adjacents g--> update
              if ((adjacentCost(G, minPriorityIndex, 3) + getG(G, minPriorityIndex)) < getG(G, getIndex(j, i))) {
                //if (bitRead(G[getIndex(j, i)], 2) == 0){  //only update if wasn't in queue
                  setG(G, getIndex(j, i), (getG(G, minPriorityIndex) + adjacentCost(G, minPriorityIndex, 3)));  //updates g
                  /*
                  Serial.print("g value set to: ");
                  Serial.println(getG(G, minPriorityIndex) + adjacentCost(G, minPriorityIndex, 3));
                  */
                  bitWrite(G[getIndex(j, i)], 3, 1);  //puts adjacent in queue
                  //setDirection(G, minPriorityIndex, 3);
                  setDirection(G, getIndex(j, i), 3);
                  setParentDirection(G, getIndex(j, i), 1);
                //}
              }               
            } else {  //adjacent to right
              if ((adjacentCost(G, minPriorityIndex, 1) + getG(G, minPriorityIndex)) < getG(G, getIndex(j, i))) {
                //if (bitRead(G[getIndex(j, i)], 2) == 0){  //only update if wasn't in queue
                  bitWrite(G[getIndex(j, i)], 3, 1);  //puts adjacent in queue
                  setG(G, getIndex(j, i), (getG(G, minPriorityIndex) + adjacentCost(G, minPriorityIndex, 1)));  //updates g
                  /*
                  Serial.print("g value set to: ");
                  Serial.println(getG(G, minPriorityIndex) + adjacentCost(G, minPriorityIndex, 1));
                  */
                  //setDirection(G, minPriorityIndex, 1);
                  setDirection(G, getIndex(j, i), 1);                
                  setParentDirection(G, getIndex(j, i), 3);
                //}
              }                
            }     
          }       
        }
      }
      if (done == true) break;
      if (i == getYCoor(minPriorityIndex)) continue;
      if (i >= GRAPH_HEIGHT) break;
      //above and below adjacents(x = getXCoor(minPriorityIndex), y = i)
      if (bitRead(G[getIndex(getXCoor(minPriorityIndex), i)], 4) == 1) continue;
      /*
      Serial.print("(");
      Serial.print(getXCoor(minPriorityIndex));
      Serial.print(",");
      Serial.print(i);
      Serial.print(") \n");
      */
      //if adjacent is destination, set directions, in path, and be done
      if (isDest(getIndex(getXCoor(minPriorityIndex), i))) {  //if neighbor is destination
        if (i == getYCoor(minPriorityIndex) - 1) {  //dest is below, set dir north
          setParentDirection(G, getIndex(getXCoor(minPriorityIndex), i), 0);
          setDirection(G, getIndex(getXCoor(minPriorityIndex), i), 2);
        } else {  //else dest is above, set dir north
          setParentDirection(G, getIndex(getXCoor(minPriorityIndex), i), 2);
          setDirection(G, getIndex(getXCoor(minPriorityIndex), i), 0);
        }
        done = true;
      } else {
        if (i == getYCoor(minPriorityIndex) - 1) {  //adjacent below
          //if adjacentdistance + minPriorityIndex G < adjacents g--> update
          if ((adjacentCost(G, minPriorityIndex, 2) + getG(G, minPriorityIndex)) < getG(G, getIndex(getXCoor(minPriorityIndex), i)))  {
            //if (bitRead(G[getIndex(minPriorityIndex, i)], 2) == 0){  //only update if wasn't in queue
              setG(G, getIndex(getXCoor(minPriorityIndex), i), (getG(G, minPriorityIndex) + adjacentCost(G, minPriorityIndex, 2))); //updates g
              /*
              Serial.print("g value set to: ");
              Serial.println(getG(G, minPriorityIndex) + adjacentCost(G, minPriorityIndex, 2));
              */
              bitWrite(G[getIndex(getXCoor(minPriorityIndex), i)], 3, 1); //puts adjacent in queue
              //setDirection(G, minPriorityIndex, 2);
              setDirection(G, getIndex(getXCoor(minPriorityIndex), i), 2);
              setParentDirection(G, getIndex(getXCoor(minPriorityIndex), i), 0);
            //}
          }         
        } else {  //adjacent above
          if ((adjacentCost(G, minPriorityIndex, 0) + getG(G, minPriorityIndex)) < getG(G, getIndex(getXCoor(minPriorityIndex), i))) {
            //if (bitRead(G[getIndex(minPriorityIndex, i)], 2) == 0){  //only update if wasn't in queue
              setG(G, getIndex(getXCoor(minPriorityIndex), i), (getG(G, minPriorityIndex) + adjacentCost(G, minPriorityIndex, 0))); //updates g
              /*
              Serial.print("g value set to: ");
              Serial.println(getG(G, minPriorityIndex) + adjacentCost(G, minPriorityIndex, 0));
              */
              bitWrite(G[getIndex(getXCoor(minPriorityIndex), i)], 3, 1); //puts adjacent in queue
              //setDirection(G, minPriorityIndex, 0);
              setDirection(G, getIndex(getXCoor(minPriorityIndex), i), 0);
              setParentDirection(G, getIndex(getXCoor(minPriorityIndex), i), 2);
            //}
          }        
        }
      }
    }
    //printGraphArray(G);
  }

  //build path
  unsigned int prev_optimal_dir;
  
  if (bitRead(G[getIndex(destXandY[0], destXandY[1])], 15) == 0 && bitRead(G[getIndex(destXandY[0], destXandY[1])], 14) == 0) {
    prev_optimal_dir = 0;
  } else if (bitRead(G[getIndex(destXandY[0], destXandY[1])], 15) == 0 && bitRead(G[getIndex(destXandY[0], destXandY[1])], 14) == 1) {
    prev_optimal_dir = 1;
  } else if (bitRead(G[getIndex(destXandY[0], destXandY[1])], 15) == 1 && bitRead(G[getIndex(destXandY[0], destXandY[1])], 14) == 0) {
    prev_optimal_dir = 2;
  } else {
    prev_optimal_dir = 3;
  }
  /*
  Serial.print(getIndex(destXandY[0], destXandY[1]));
  Serial.print("'s optimal previous: ");
  Serial.println(prev_optimal_dir);
  */
  //printGraphArray(G);
  unsigned int prev_optimal = getIndex(destXandY[0], destXandY[1]), path_count = 0;
  
  while (prev_optimal >= 0) {
    /*
    Serial.print("prev_optimal: ");
    Serial.print(prev_optimal);
    Serial.print(" prev_optimal_dir: ");
    Serial.println(prev_optimal_dir);
    */
    path[path_count] = prev_optimal;
    /*
    Serial.print(prev_optimal);
    */
    if (prev_optimal_dir == 0) {
      prev_optimal += GRAPH_WIDTH;
    } else if (prev_optimal_dir == 1) {
      prev_optimal += 1;
    } else if (prev_optimal_dir == 2) {
      prev_optimal -= GRAPH_WIDTH;
    } else {
      prev_optimal -= 1;
    }
    /*
    Serial.print("/");
    Serial.println(prev_optimal);
    */
    if (bitRead(G[prev_optimal], 15) == 0 && bitRead(G[prev_optimal], 14) == 0) {
      prev_optimal_dir = 0;
    } else if (bitRead(G[prev_optimal], 15) == 0 && bitRead(G[prev_optimal], 14) == 1) {
      prev_optimal_dir = 1;
    } else if (bitRead(G[prev_optimal], 15) == 1 && bitRead(G[prev_optimal], 14) == 0) {
      prev_optimal_dir = 2;
    } else {
      prev_optimal_dir = 3;
    }
    path_count++;
    if (prev_optimal == source) {
      path[path_count] = prev_optimal;
      break;
    } 
  }
}

bool step12() {
  long duration = 0;
  int steps = 1438;
  bool obstacle = false;
  
  int light_count = 0;

  for (; steps >= 0; steps--) {
    duration = 0;
    leftMotor->onestep(FORWARD, DOUBLE);
    rightMotor->onestep(FORWARD, DOUBLE);
    //delayMicroseconds(90);

    //Serial.print("Photosensor: ");
    //Serial.println(analogRead(1));
    
    if (analogRead(1) < 11) {
      if (light_count >= 15) {
        //Serial.println("obstacle true 1");
        obstacle = true; 
        break;
      } else {
        light_count++;
      }
    }    
  }
  
  if (obstacle == true) {  
    //Serial.println("Obstacle Found");
    for (int i = 1438 - steps; i > 0; i--) {
      leftMotor->onestep(BACKWARD, DOUBLE);
      rightMotor->onestep(BACKWARD, DOUBLE);
      delayMicroseconds(100);
    }
  }
  return obstacle;
}
/*
bool step12(unsigned int G[], int graph_index, int ahead) {
  Servo servo;
  servo.attach(controlPin);
  
  long duration = 0;
  int steps = 1438;
  bool obstacle = false;
  int angle = 0;
  
  int light_count = 0;
  int ultra_count = 0;

  if (look(graph_index, ahead) == 1) {
    servo.write(180);
    delay(200);
  } else if (look(graph_index, ahead) == 3) {
    servo.write(0);
    delay(200);
  }
  servo.write(90);
  delay(200);
  for (int i = 0; i < 100; i++) {
      digitalWrite(trig, LOW);
      delayMicroseconds(5);       //low pulse with delay ensures clean high pulse
      digitalWrite(trig, HIGH);   //high pulse of 10 or more ms triggers sensor
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      
      if (digitalRead(echo) != HIGH) {
        duration = pulseIn(echo, HIGH, 4000);
      }
      if (duration <= 1200 || duration >= 3400) { //range ~ 1-2 feet
        //Serial.print("Duration: ");
        //Serial.println(duration);
        if (ultra_count > 0) {
          ultra_count--;
        }
        continue;
      } else {
        if (duration >= 1200 && duration <= 3400) {
          obstacle = true;
          //Serial.print("Duration: ");
          //Serial.println(duration);
          ultra_count++;
        }
        
        if (ultra_count >= 5) {
          //set direction of obstacle (left, right, straight)
          if (angle == 180) { //obstacle to the left, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != 0)){
               bitWrite(G[graph_index - 1] , 4, 1);
               
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH], 4, 1);
               
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
              
            }
          } else if (angle == 90) { //obstacle straight, check if obstacle is in graph
            obstacleinFront = true;
            if ((nodePointing(G, graph_index) == 0) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)){
               bitWrite(G[graph_index + GRAPH_WIDTH] , 4, 1);
               
            } else if ((nodePointing(G, graph_index) == 1) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               
            } else if ((nodePointing(G, graph_index) == 2) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               
            } else if ((nodePointing(G, graph_index) == 3) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
               
            }
          } else if (angle == 0) {  //obstacle to the right, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)){
               bitWrite(G[graph_index + 1] , 4, 1);
               
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
              
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
              
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH ], 4, 1);
              
            }
          }
          break;
        }
      }
    }

    servo.write(180);
    delay(200);
    for (int i = 0; i < 100; i++) {
      digitalWrite(trig, LOW);
      delayMicroseconds(5);       //low pulse with delay ensures clean high pulse
      digitalWrite(trig, HIGH);   //high pulse of 10 or more ms triggers sensor
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      
      if (digitalRead(echo) != HIGH) {
        duration = pulseIn(echo, HIGH, 4000);
      }
      if (duration <= 1400 || duration >= 3000) { //range ~ 1-2 feet
        //Serial.print("Duration: ");
        //Serial.println(duration);
        if (ultra_count > 0) {
          ultra_count--;
        }
        continue;
      } else {
        if (duration >= 1400 && duration <= 3000) {
          obstacle = true;
          //Serial.print("Duration: ");
          //Serial.println(duration);
          ultra_count++;
        }
        
        if (ultra_count >= 5) {
          //set direction of obstacle (left, right, straight)
          if (angle == 180) { //obstacle to the left, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != 0)){
               bitWrite(G[graph_index - 1] , 4, 1);
               
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            }
          } else if (angle == 90) { //obstacle straight, check if obstacle is in graph
            obstacleinFront = true;
            if ((nodePointing(G, graph_index) == 0) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)){
               bitWrite(G[graph_index + GRAPH_WIDTH] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
               obstacle = true;
            }
          } else if (angle == 0) {  //obstacle to the right, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)){
               bitWrite(G[graph_index + 1] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH ], 4, 1);
               obstacle = true;
            }
          }
          break;
        }
      }
    }

    servo.write(90);
    delay(200);
    for (int i = 0; i < 100; i++) {
      digitalWrite(trig, LOW);
      delayMicroseconds(5);       //low pulse with delay ensures clean high pulse
      digitalWrite(trig, HIGH);   //high pulse of 10 or more ms triggers sensor
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      
      if (digitalRead(echo) != HIGH) {
        duration = pulseIn(echo, HIGH, 4000);
      }
      if (duration <= 1400 || duration >= 3000) { //range ~ 1-2 feet
        //Serial.print("Duration: ");
        //Serial.println(duration);
        if (ultra_count > 0) {
          ultra_count--;
        }
        continue;
      } else {
        if (duration >= 1400 && duration <= 3000) {
          obstacle = true;
          //Serial.print("Duration: ");
          //Serial.println(duration);
          ultra_count++;
        }
        
        if (ultra_count >= 5) {
          //set direction of obstacle (left, right, straight)
          if (angle == 180) { //obstacle to the left, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != 0)){
               bitWrite(G[graph_index - 1] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            }
          } else if (angle == 90) { //obstacle straight, check if obstacle is in graph
            obstacleinFront = true;
            if ((nodePointing(G, graph_index) == 0) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)){
               bitWrite(G[graph_index + GRAPH_WIDTH] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
               obstacle = true;
            }
          } else if (angle == 0) {  //obstacle to the right, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)){
               bitWrite(G[graph_index + 1] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH ], 4, 1);
               obstacle = true;
            }
          }
          break;
        }
      }
    }

    servo.write(0);
    delay(200);
    for (int i = 0; i < 100; i++) {
      digitalWrite(trig, LOW);
      delayMicroseconds(5);       //low pulse with delay ensures clean high pulse
      digitalWrite(trig, HIGH);   //high pulse of 10 or more ms triggers sensor
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      
      if (digitalRead(echo) != HIGH) {
        duration = pulseIn(echo, HIGH, 4000);
      }
      if (duration <= 1400 || duration >= 3000) { //range ~ 1-2 feet
        //Serial.print("Duration: ");
        //Serial.println(duration);
        if (ultra_count > 0) {
          ultra_count--;
        }
        continue;
      } else {
        if (duration >= 1400 && duration <= 3000) {
          obstacle = true;
          //Serial.print("Duration: ");
          //Serial.println(duration);
          ultra_count++;
        }
        
        if (ultra_count >= 5) {
          //set direction of obstacle (left, right, straight)
          if (angle == 180) { //obstacle to the left, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != 0)){
               bitWrite(G[graph_index - 1] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            }
          } else if (angle == 90) { //obstacle straight, check if obstacle is in graph
            obstacleinFront = true;
            if ((nodePointing(G, graph_index) == 0) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)){
               bitWrite(G[graph_index + GRAPH_WIDTH] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
               obstacle = true;
            }
          } else if (angle == 0) {  //obstacle to the right, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)){
               bitWrite(G[graph_index + 1] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH ], 4, 1);
               obstacle = true;
            }
          }
          break;
        }
      }
    }

    servo.write(90);

/*
  while (angle <= 180) {
    servo.write(angle);
    delay(300);
    for (int i = 0; i < 100; i++) {
      digitalWrite(trig, LOW);
      delayMicroseconds(5);       //low pulse with delay ensures clean high pulse
      digitalWrite(trig, HIGH);   //high pulse of 10 or more ms triggers sensor
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      
      if (digitalRead(echo) != HIGH) {
        duration = pulseIn(echo, HIGH, 4000);
      }
      if (duration <= 1400 || duration >= 3000) { //range ~ 1-2 feet
        //Serial.print("Duration: ");
        //Serial.println(duration);
        if (ultra_count > 0) {
          ultra_count--;
        }
        continue;
      } else {
        if (duration >= 1400 && duration <= 3000) {
          //Serial.print("Duration: ");
          //Serial.println(duration);
          ultra_count++;
        }
        
        if (ultra_count >= 5) {
          //set direction of obstacle (left, right, straight)
          if (angle == 180) { //obstacle to the left, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != 0)){
               bitWrite(G[graph_index - 1] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            }
          } else if (angle == 90) { //obstacle straight, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)){
               bitWrite(G[graph_index + GRAPH_WIDTH] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
               obstacle = true;
            }
          } else if (angle == 0) {  //obstacle to the right, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)){
               bitWrite(G[graph_index + 1] , 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
               obstacle = true;
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH ], 4, 1);
               obstacle = true;
            }
          }
          break;
        }
      }
    }
    if (obstacle == false) {
      angle += 90; 
    } else {
      break;
    }
  }
  servo.write(90);

  
  if (obstacleinFront == false) {
    for (; steps >= 0; steps--) {
      duration = 0;
      leftMotor->onestep(FORWARD, DOUBLE);
      rightMotor->onestep(FORWARD, DOUBLE);
      //delayMicroseconds(100);
  
      //Serial.print("Photosensor: ");
      //Serial.println(analogRead(1));
      
      if (analogRead(1) < 11) {
        if (light_count >= 15) {
          obstacle = true; 
          break;
        } else {
          light_count++;
        }
      }
    }
    if (obstacle == true) {  
      //Serial.println("Obstacle Found");
      for (int i = 1438 - steps; i > 0; i--) {
        leftMotor->onestep(BACKWARD, DOUBLE);
        rightMotor->onestep(BACKWARD, DOUBLE);
        delayMicroseconds(100);
      }
      //set obstacle straight ahead as obstacle
      if (nodePointing(G, graph_index) == 0) {
        bitWrite(G[graph_index + GRAPH_WIDTH], 4, 1); 
      } else if (nodePointing(G, graph_index) == 1) {
        bitWrite(G[graph_index + 1], 4, 1);
      } else if (nodePointing(G, graph_index) == 2) {
        bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
      } else if (nodePointing(G, graph_index) == 3) {
        bitWrite(G[graph_index - 1], 4, 1);
      }
    }
  }
  //step forward without sensors
  //Serial.println("Went straight");
  for (int i = 1438; i > 0; i--) {
    leftMotor->onestep(FORWARD, DOUBLE);
    rightMotor->onestep(FORWARD, DOUBLE);
    delayMicroseconds(100);
  }

  return obstacleinFront;
}
*/

//looks in direction needed to go, checks for obstacle
bool look(unsigned int G[], unsigned int graph_index, unsigned int dir) {
  Servo servo;
  servo.attach(controlPin);
  long duration = 0;
  bool obstacle = false; 
  int ultra_count = 0;
  
  int direction = 0;
  if (graph_index == 0 && dir == 1) {
    //look right
    direction = 1;
  }
  if (graph_index == 0 && dir == 3) {
    //look left
    direction = 2;
  }
  if (graph_index == 1 && dir == 0) {
    //look left
    direction = 2;
  }
  if (graph_index == 1 && dir == 2) {
    //look right
    direction = 1;
  }
  if (graph_index == 2 && dir == 1) {
    //look left
    direction = 2;
  }
  if (graph_index == 2 && dir == 3) {
    //look right
    direction = 1;
  }
  if (graph_index == 3 && dir == 0) {
    //look right
    direction = 1;
  }
  if (graph_index == 3 && dir == 2) {
    //look left
    direction = 2;
  } 

  if (direction == 1) {
    servo.write(0);
    delay(200);
  } else if (direction == 0) {
    servo.write(180);
    delay(200);
  }

  for (int i = 0; i < 100; i++) {
      digitalWrite(trig, LOW);
      delayMicroseconds(5);       //low pulse with delay ensures clean high pulse
      digitalWrite(trig, HIGH);   //high pulse of 10 or more ms triggers sensor
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      
      if (digitalRead(echo) != HIGH) {
        duration = pulseIn(echo, HIGH, 4000);
      }
      if (duration <= 700 || duration >= 3600) { //range ~ 1-2 feet
        //Serial.print("Duration: ");
        //Serial.println(duration);
        if (ultra_count > 0) {
          ultra_count--;
        }
        continue;
      } else {
        if (duration >= 700 && duration <= 3600) {
          obstacle = true;
          //Serial.print("Duration: ");
          //Serial.println(duration);
          ultra_count++;
        }
        
        if (ultra_count >= 5) {
          //set direction of obstacle (left, right, straight)
          if (direction == 2) { //obstacle to the left, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != 0)){
               bitWrite(G[graph_index - 1] , 4, 1);
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH], 4, 1);
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
               bitWrite(G[graph_index + 1], 4, 1);
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
            }
          } else if (direction == 1) {  //obstacle to the right, check if obstacle is in graph
            if ((nodePointing(G, graph_index) == 0) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)){
               bitWrite(G[graph_index + 1] , 4, 1);
            } else if ((nodePointing(G, graph_index) == 1) && (getYCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
            } else if ((nodePointing(G, graph_index) == 2) && (getXCoor(graph_index) != 0)) {
               bitWrite(G[graph_index - 1], 4, 1);
            } else if ((nodePointing(G, graph_index) == 3) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)) {
               bitWrite(G[graph_index + GRAPH_WIDTH ], 4, 1);
            }
          }
          break;
        }
      }
    }
    return obstacle;
}
 
bool lookStraight(unsigned int G[], unsigned int graph_index) {
  Servo servo;
  servo.attach(controlPin);
  long duration = 0;
  int steps = 1438;
  bool obstacle = false; 
  int ultra_count = 0;

  servo.write(90);
  delay(200);
  
  for (int i = 0; i < 100; i++) {
    digitalWrite(trig, LOW);
    delayMicroseconds(5);       //low pulse with delay ensures clean high pulse
    digitalWrite(trig, HIGH);   //high pulse of 10 or more ms triggers sensor
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    if (digitalRead(echo) != HIGH) {
      duration = pulseIn(echo, HIGH, 4000);
    }
    if (duration <= 700 || duration >= 3600) { //range ~ 1-2 feet
      //Serial.print("Duration: ");
      //Serial.println(duration);
      if (ultra_count > 0) {
        ultra_count--;
      }
      continue;
    } else {
      if (duration >= 700 && duration <= 3600) {
        obstacle = true;
        //Serial.print("Duration: ");
        //Serial.println(duration);
        ultra_count++;
      }
      
      if (ultra_count >= 5) {
        if ((nodePointing(G, graph_index) == 0) && (getYCoor(graph_index) != GRAPH_HEIGHT - 1)){
             bitWrite(G[graph_index + GRAPH_WIDTH] , 4, 1);
        } else if ((nodePointing(G, graph_index) == 1) && (getXCoor(graph_index) != GRAPH_WIDTH - 1)) {
             bitWrite(G[graph_index + 1], 4, 1);
        } else if ((nodePointing(G, graph_index) == 2) && (getYCoor(graph_index) != 0)) {
             bitWrite(G[graph_index - GRAPH_WIDTH], 4, 1);
        } else if ((nodePointing(G, graph_index) == 3) && (getXCoor(graph_index) != 0)) {
             bitWrite(G[graph_index - 1], 4, 1);
        }
        break;
      }
    }
  }
  return obstacle;
}


void point(unsigned int strt, unsigned int stp) {
  delay(200);
  if (strt == 0 && stp == 1) {
    turnRight();
  }
  if (strt == 0 && stp == 2) {
    turnLeft();
    turnLeft();
  }
  if (strt == 0 && stp == 3) {
    turnLeft();
  }
  if (strt == 1 && stp == 0) {
    turnLeft();
  }
  if (strt == 1 && stp == 2) {
    turnRight();
  }
  if (strt == 1 && stp == 3) {
    turnLeft();
    turnLeft();
  }
  if (strt == 2 && stp == 0) {
    turnLeft();
    turnLeft();
  }
  if (strt == 2 && stp == 1) {
    turnLeft();
  }
  if (strt == 2 && stp == 3) {
    turnRight();
  }
  if (strt == 3 && stp == 0) {
    turnRight();
  }
  if (strt == 3 && stp == 1) {
    turnLeft();
    turnLeft();
  }
  if (strt == 3 && stp == 2) {
    turnLeft();
  }
}

//turn right or left 90 degrees
void turnLeft() {
  //Serial.println("Turned Left");
  for (int i = 513; i > 0; i--) {
    rightMotor->onestep(BACKWARD, MICROSTEP);
    leftMotor->onestep(FORWARD, MICROSTEP);
    delayMicroseconds(100);
  }
}

void turnRight() {
  //Serial.println("Turned Right");
  for (int i = 513; i > 0; i--) {
    leftMotor->onestep(BACKWARD, MICROSTEP);
    rightMotor->onestep(FORWARD, MICROSTEP);
    delayMicroseconds(100);
  }
}

void traversePath(unsigned int G[], unsigned int path[], unsigned int path_count) {
  bool obstacleFound = false;
  int i = path_count;
  for (i = path_count; i > 0; i--) {
    /*
    Serial.print("i: ");
    Serial.print(i);
    Serial.print("\tpath[i]: ");
    Serial.print(path[i]);
    Serial.print("\tpath[i-1]: ");
    Serial.print(path[i-1]);
    Serial.print("\tnodePointing(G, path[i]): ");
    Serial.print(nodePointing(G, path[i]));
    Serial.print("\tnodePointing(G, path[i-1]): ");
    Serial.println(nodePointing(G, path[i-1]));
    */
    if (nodePointing(G, path[i]) != nodePointing(G, path[i - 1])) { //if two sequential nodes are not in same direction
      obstacleFound = look(G, nodePointing(G, path[i]), nodePointing(G, path[i - 1]));
      if (obstacleFound == true) {
        bitWrite(G[path[i - 1]], 4, 1);
        break;
      }
      point(nodePointing(G, path[i]), nodePointing(G, path[i - 1]));
      obstacleFound = step12();
    } else {  //else just go straight
      obstacleFound = lookStraight(G, path[i]);
      if (obstacleFound == true) {
        bitWrite(G[path[i - 1]], 4, 1);
        break;
      }
      obstacleFound = step12();
    }
    if (obstacleFound == true) {
      bitWrite(G[path[i - 1]], 4, 1);
      break;
    }
  }
  if (obstacleFound == true) {
    reinitialize(G, path[i]);
  }
}

void reinitialize(unsigned int G[], unsigned int new_source) {
  unsigned int path_count = 0;
  bitWrite(G[new_source], 3, 1);  //puts new source in queue
  bitWrite(G[new_source], 2, 0);
  setG(G, new_source, 0); 
  for (int i = 0; i < GRAPH_HEIGHT * GRAPH_WIDTH; i++) {
    //if not source, set g to max value
    setDirection(G, i, 0);
    setParentDirection(G, i, 0);
    if (i != new_source) {
      setG(G, i, -1);
      bitWrite(G[i], 3, 0);
      bitWrite(G[i], 2, 0);
    }
  }
  unsigned int path[GRAPH_HEIGHT * GRAPH_WIDTH] = {0};

  //Serial.println("finding new shortest path with source");
  //Serial.println(new_source);
  //printGraphArray(G);
  shortestPath(path, G, new_source);
  //Serial.println("found new shortest path");
  for (int i = 0; i < GRAPH_HEIGHT * GRAPH_WIDTH; i++) {
    if (path[i] == new_source) {
      path_count = i;
    }
  }
  /*
  Serial.print("Reinitialized Path_count: ");
  Serial.println(path_count);
  for (int i = path_count; i > 0; i--) {
    Serial.print(path[i]);
    Serial.print("-->");
  }
  Serial.println("");
  */
  //Serial.println("5");
  //need to point in right direction
  traversePath(G, path, path_count);  
  //Serial.println("5back");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(500000);  

  AFMS.begin(1600);    //default frequency 1.6KHz
  Wire.setClock(400000);      //I2C freq to 400KHz

  leftMotor->release();
  rightMotor->release();
  
  //ultrasonic pins
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  //BUTTON START CODE
  pinMode(2, OUTPUT); //left led
  pinMode(3, OUTPUT); //right led
  pinMode(8, INPUT); //button
  Serial.println("Press button when ready...");
  while(1) {
    if (digitalRead(8) == HIGH) {
      digitalWrite(2, HIGH);
      digitalWrite(3, HIGH);  
      break;
    }  
  }
  
  //initialize
  unsigned int G[GRAPH_HEIGHT * GRAPH_WIDTH] = {0};
  unsigned int source;

  //set start direction, in queue, in path, and initial G value for initial source
  for(int i = 0; i < 2; i++) {
    bitWrite(G[getIndex(srcXandY[0], srcXandY[1])], i, bitRead(START_DIRECTION, i)); 
    bitWrite(G[getIndex(srcXandY[0], srcXandY[1])], 3, 1);
    bitWrite(G[getIndex(srcXandY[0], srcXandY[1])], 2, 1);
    setG(G, getIndex(srcXandY[0], srcXandY[1]), 0); 
    source = getIndex(srcXandY[0], srcXandY[1]);
  }
  //sets initial g values for all nodes except source 
  //sets source in queue, in path, and g = 0
  for (int i = 0; i < GRAPH_HEIGHT * GRAPH_WIDTH; i++) {
    //if not source, set g to max value
    if (!isSource(i)) setG(G, i, -1);
  }
  
  unsigned int path[GRAPH_HEIGHT * GRAPH_WIDTH];

  //printGraphArray(G);
  
  shortestPath(path, G, source);
  
  unsigned int path_count = 0;
  for (int i = 0; i < GRAPH_HEIGHT * GRAPH_WIDTH; i++) {
    if (path[i] == source) {
      path_count = i;
    }
  }

  
  //gives path index in order
  for (int i = path_count; i >= 0; i--) {
    Serial.print(path[i]);
    Serial.print(" -->");
  }
  
  traversePath(G, path, path_count);
  /*
  Serial.println(path_count);
  for (int i = path_count; i > 0; i--) {
    Serial.println(path[i]);
  }
  */



/*
  bool obstacles;
  while (!obstacles) {
    obstacles = step12();
  }
  Serial.println("Obstacle detected");
*/



  leftMotor->release();
  rightMotor->release();

/*
  //gives path index in order
  for (int i = path_count; i >= 0; i--) {
    Serial.print(path[i]);
    Serial.print(" -->");
  }
  printGraphArray(G);
*/
/*
  for (int i = 0; i <= path_count + 1; i++) {
    Serial.println(path[i]);
  }

  Serial.println("\n");
  printGraphArray(G);
*/  
  /* printing out obstacles
  unsigned int displayint;
  for (int i = 0; i < SIZE_OBSTACLE_X; i++) {
    Serial.print("obstacleX[");
    Serial.print(i);
    Serial.print("]: ");
    displayint = pgm_read_word_near(obstacleX + i);
    Serial.println(displayint);
    Serial.print("obstacleY[");
    Serial.print(i);
    Serial.print("]: ");
    displayint = pgm_read_word_near(obstacleY + i);
    Serial.println(displayint);
  }*/
}

void loop() {
  // put your main code here, to run repeatedly
}
