#include <serialize.h>
#include <stdarg.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "packet.h"
#include "constants.h"
#include "Arduino.h"

#define S0  7
#define S1  8
#define S2  9
#define S3  13
#define OUT 12 

#define TIMEOUT 30000
#define TRIGGER 4
#define ECHO 7
#define speedOfSound 343

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
}TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

#define pi 3.1415954

#define ALEX_LENGTH 17
#define ALEX_BREADTH 10

float alexDiagnal = sqrt(ALEX_LENGTH * ALEX_LENGTH + ALEX_BREADTH * ALEX_BREADTH);
float alexCirc = PI * alexDiagnal /2;

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      192 //to be adjusted

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.65 //to be measured

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF (1 << 5)   // PD Pin 5 - OC0B
#define LR (1 << 6)   // PD Pin 6 - OC0A
#define RF (1 << 3)   //11 PB Pin 3 - OC1B
#define RR (1 << 2)   //10 PB Pin 2 - OC2A

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right enconder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//distance
unsigned long deltaDist;
unsigned long newDist;

//angle
unsigned long deltaTicks;
unsigned long targetTicks;

//color sensor
volatile int color; //variable to store the color index of the identified color to send back to the Rpi
volatile int rgb_values[3] = {0,0,0}; //array to store the measured R, G, B values of the identified color 
int diodeArray[3][2] = {{0,0},{0,1},{1,1}}; //array to store the switching pattern for pins S2 and S3 to turn on the different photodiodes
int colours[2][3] = {{255,0,0}, {0,255,0}}; //array which store the standard RGB values for key colors
int sensor_values[3][2] = {{143, 95}, {89, 59}, {129, 85}}; //array storing the {max, min} sensor values for photodiodes of R, G, B filters 
int colour_id[2] = {1, 2}; //array storing the index for key colors, in this case 1-Red, 2-Green
int tolerance = 15; //tolerance value to determine the range of acceptance for the measured RGB values when comparingto standard RGB values

//ultrasonic
volatile unsigned long duration; //varaible to store the time taken for the USS triggering pulse to return 
volatile int distance; //variable to store the calculated distance between Alex and an obstacle
volatile int ultrasonic;//variable to store the calcualted distance to send back to the Rpi

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    statusPacket.params[0] = leftForwardTicks;
    statusPacket.params[1] = rightForwardTicks;
    statusPacket.params[2] = leftReverseTicks; 
    statusPacket.params[3] = rightReverseTicks;
    statusPacket.params[4] = leftForwardTicksTurns;
    statusPacket.params[5] = rightForwardTicksTurns;
    statusPacket.params[6] = leftReverseTicksTurns;
    statusPacket.params[7] = rightReverseTicksTurns; 
    statusPacket.params[8] = forwardDist;
    statusPacket.params[9] = reverseDist;
    statusPacket.params[10] = color;
    statusPacket.params[11] = ultrasonic;
    sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf (char *format, ...) {
    va_list args;
    char buffer[128];

    va_start(args, format);
    vsprintf(buffer, format, args);
    sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
  
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
    if (dir == FORWARD) { 
      leftForwardTicks++; 
      forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC); 
    }
    else if (dir == BACKWARD) { 
      leftReverseTicks++; 
      reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC); 
    }
    else if (dir == LEFT) {
      leftReverseTicksTurns++; 
    }
    else if (dir == RIGHT) {
      leftForwardTicksTurns++;
    } 
}

void rightISR()
{
    if (dir == FORWARD) {
      rightForwardTicks++; 
    }
    else if (dir == BACKWARD) {
      rightReverseTicks++; 
    }
    else if (dir == LEFT) {
      rightForwardTicksTurns++; 
    } 
    else if (dir == RIGHT) {
      rightReverseTicksTurns++; 
    } 
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  cli();
  EICRA |= 0b00001010;
  EIMSK |= 0b00000011;
  sei();
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}



// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  unsigned long baudRate = 9600;
  unsigned int b;
  b = (unsigned int) round (F_CPU / (16.0 * baudRate)) - 1;
  UBRR0H = (unsigned char) (b >> 8);
  UBRR0L = (unsigned char) b;

  //Async mode
  //No parity
  //1 stop bit
  //8N1 configuration 
  //bit 0 (UCPOL0) is always 0
  UCSR0C = 0b00000110;

  UCSR0A = 0;  
  //Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  UCSR0B = 0b00011000;
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  do {

    while ((UCSR0A & 0b10000000) == 0);

      buffer[count] = UDR0;
    
  } while (buffer[count++] != '\0');

  return count;
}

void writeSerial(const char *buffer, int len)
{
  for (int i = 0; i < len; i++) {
    while ((UCSR0A & 0b00100000) == 0);
      UDR0 = buffer[i];
  }
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
   DDRD |= (LF|LR);
   DDRB |= (RF|RR);
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  //setting up timer 0 for left motor
  TCNT0 = 0; //start counter from 0
  TCCR0B = 0b00000011; //clk/64 source

  //setting up timer 1 for right motor
  TCNT1 = 0;
  TCCR1B = 0b00000011; //clk/64 source
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;
  
  int val = pwmVal(speed);

  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist=9999999;
  newDist=forwardDist + deltaDist;

  OCR0A = 0;
  OCR0B = val;
  OCR1A = 0;
  OCR1B = val;

  TCCR0A = 0b00100001; //left motor forward
  TCCR1A = 0b00100001; //right motor forward
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD;
  
  int val = pwmVal(speed);

  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist=9999999;
  newDist=forwardDist + deltaDist;
  
  OCR0A = val;
  OCR0B = 0;
  OCR1A = val;
  OCR1B = 0;

  TCCR0A = 0b10000001; //left motor reverse
  TCCR1A = 0b10000001; //right motor reverse
  
}

unsigned long computeDeltaTicks(float ang) 
{
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  
  int val = pwmVal(speed);

  if (ang == 0)
    deltaTicks = 999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = rightReverseTicksTurns + deltaTicks;

  OCR0A = val;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = val;

  TCCR0A = 0b10000001; //left motor reverse
  TCCR1A = 0b00100001; //right motor forward
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  
  int val = pwmVal(speed);

  if (ang == 0)
    deltaTicks = 999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = rightReverseTicksTurns + deltaTicks;

  OCR0A = 0;
  OCR0B = val;
  OCR1A = val;
  OCR1B = 0;

  TCCR0A = 0b00100001; //left motor forward
  TCCR1A = 0b10000001; //right motor reverse
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;

  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;

  TCCR0A = 0b00000001; //left motor forward
  TCCR1A = 0b00000001; //right motor forward
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0; 
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  
  forwardDist = 0;
  reverseDist = 0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  switch (which) {
    case 0: leftForwardTicks = 0;
      break;
    case 1: rightForwardTicks = 0;
      break;
    case 2: leftReverseTicks = 0;
      break;
    case 3: rightReverseTicks = 0;
      break;
    case 4: leftForwardTicksTurns = 0;
      break;
    case 5: rightForwardTicksTurns = 0;
      break;
    case 6: leftReverseTicksTurns = 0;
      break;
    case 7: rightReverseTicksTurns = 0;
      break;
  }
}

// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
        sendOK();
        stop();
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      clearCounters();
      break;
              
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  cli();
  //color sensor
  //Set F1,F2, DIODE1, DIODE2 to output, OUT to input
  DDRB |= 0b00100001;
  DDRB &= ~0b00010000;

  //ultrasonic
  DDRD |= 0b00010000;  //set trig to output
  PORTD &= ~0b00010000; //set trig to low
  DDRD &= ~0b10000000; //set echo to input
    
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void ultra_sonic() {
   //Turn Trig on and off to send a pulse
   PORTD |= 0b00010000;
   delayMicroseconds(10);
   PORTD &= ~0b00010000;
   delayMicroseconds(10); 
   
   duration = pulseIn(ECHO, HIGH, TIMEOUT); //Measure the duration taken for pulse to return back to USS 
   ultrasonic = ((duration / 2) / 1000000) * speedOfSound * 100; //Calculate the distance travelled by the pulse using (duration/2) * speedOfSound (in cm/microseconds)
}

//Function to determine the resultant pulse period (the duration for the pulse to stay low). Used together with color sensor
unsigned long get_period() {
  //Set Timer 2
  TCNT2 = 0;
  TCCR2A = 0;

  //If the current pulse on the OUTPIN is HIGH, wait for pulse on the OUT pin to go from HIGH to LOW
  if (PINB & 0b00010000 == 1) {
    while (PINB & 0b00010000 == 1);
  }

  //Start Timer
  TCCR2B = 0b00000011;

  //Wait for Timer to go from LOW TO HIGH again
  while (PINB & 0b00010000 == 0);

  //Stop Timer
  TCCR2B = 0;

  //Calculate the duration  of LOW period. Since prescalar factor of the timer is set to 64, each increment of TCNT is 4 microseconds
  return TCNT2 * 4;
}

int colour_sensing(){

  //Loop is done to switch to the R, G, B photodiodes and take respective measurements
  for (int i=0; i<3; i++){

    // On/off S2 according to the current switching pattern
    if (diodeArray[i][0] == 0) {
      PORTB &= ~0b00000001;
      
    }
    else {
      PORTB |= 0b00000001;
    }

    // On/off S3 according to the current switching pattern
    if (diodeArray[i][1] == 0) {
      PORTB &= ~0b00100000;
    }
    else {
      PORTB |= 0b00100000;
    }

    //Obtain the measured sensor value through get_period(), then map it to the standard 0-255 RGB scale
    rgb_values[i] = map(get_period(), sensor_values[i][0], sensor_values[i][1], 0, 255);
  }

  //Loop through the standard list of RGB values to look for a match
  for (int n=0; n < 2; n++) {
    char found = 'Y'; //Flag to determine whether a match is found. Pulled to Y initially to force program into loop
    for (int x =0; x < 3; x++) { //Loop through all 3 RGB values to cross-check
      if (!(colours[n][x] - tolerance < rgb_values[x] &&  rgb_values[x] < colours[n][x] + tolerance)) { //cross-checking whether the measured RGB values fall into the accepted range
        found = 'N'; //When any of the RGB values fall out of the accepted range, flag is pulled to N to exit the checking loop
        break;
      }
    }
    if (found = 'Y') { //Check whether the flag has be altered. If no, all RGB values fall in the acceptable criteria and there's a match 
      return colour_id[n]; //Return the index of the matching color
    }
  }
}
    
void loop() {  
  color = colour_sensing();

  ultra_sonic();

  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 

  if (deltaDist > 0)
  {
    if (dir == FORWARD) {
      if (forwardDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD) {
      if (reverseDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0)
  {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
      
}
