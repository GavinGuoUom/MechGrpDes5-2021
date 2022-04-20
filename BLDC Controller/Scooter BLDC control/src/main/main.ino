#include <Arduino.h>

#ifndef DEBUG
#define DEBUG
#endif

// define port numer of each pin to mannually control the registor
#define UH 0
#define UL 1
#define VH 2
#define VL 3
#define WH 4
#define WL 5

// define output pin
#define PWM_H_PIN 5 // OC0B
#define PWM_L_PIN 6 // OC0A

#define UH_PIN 8 // PB0
#define UL_PIN 9
#define VH_PIN 10
#define VL_PIN 11
#define WH_PIN 12
#define WL_PIN 13 // PB5

#define CAP_SW 7 //Controling the capacitor

// define input pin
// from switch
#define DRIVE_SW_PIN 2 // INT0
#define BRAKE_SW_PIN 3 // INT1
// from hall sensor
#define HALL_U_PIN 14 // PC0
#define HALL_V_PIN 15 // PC1
#define HALL_W_PIN 16 // PC2

//from voltage sensor
#define BAT_VO_PIN A3

bool drive_flag;
bool brake_flag;
uint8_t hall_sen;
uint8_t switch_patt;
uint8_t bat_vin;

void drive_sw_interrupt();
void brake_sw_interrupt();


void setup() {
  // setup pin mode
  pinMode(PWM_H_PIN, OUTPUT);
  pinMode(PWM_L_PIN, OUTPUT);
  pinMode(CAP_SW, OUTPUT);
  /*
  pinMode(UH_PIN, OUTPUT);
  pinMode(UL_PIN, OUTPUT);
  pinMode(VH_PIN, OUTPUT);
  pinMode(VL_PIN, OUTPUT);
  pinMode(WH_PIN, OUTPUT);
  pinMode(WL_PIN, OUTPUT);
  */

  // from control swtiches
  pinMode(DRIVE_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DRIVE_SW_PIN), drive_sw_interrupt, CHANGE);
  pinMode(BRAKE_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BRAKE_SW_PIN), brake_sw_interrupt, CHANGE);
  // from sensor
  /*
  pinMode(HALL_U_PIN, INPUT_PULLUP);
  pinMode(HALL_V_PIN, INPUT_PULLUP);
  pinMode(HALL_W_PIN, INPUT_PULLUP);
  */
  // Enable pin change interrupt
  PCICR |= 1 << PCIE1;
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);
  PORTC |= (1<<PC0) | (1<<PC1) | (1<<PC2); // pinMode()
  // from voltage
  //  pinMode(BAT_VO_PIN, INPUT_PULLUP);

}

void loop() {
  // put your main code here, to run repeatedly:
  bat_vin = analogRead(BAT_VO_PIN);

  switch (hall_sen)
  {
    // HW HV HU
  case 1: //0b001
    switch_patt = (1<<UH) | (1<<VL);
    break;
  case 3: //0b011
    switch_patt = (1<<UH) | (1<<WL);
    break;
  case 2: //0b010
    switch_patt = (1<<VH) | (1<<WL);
    break;
  case 6: //0b110
    switch_patt = (1<<VH) | (1<<UL);
    break;
  case 4: //0b100
    switch_patt = (1<<WH) | (1<<UL);
    break;
  case 5: //0b101
    switch_patt = (1<<WH) | (1<<VL);
    break;
  default:
    break;
  }
  if(drive_flag)
  {
    PORTB = switch_patt;
    analogWrite(PWM_H_PIN, 255);
    analogWrite(PWM_L_PIN, 255);
  }

}

void drive_sw_interrupt()
{
  drive_flag = digitalRead(DRIVE_SW_PIN);
}

void brake_sw_interrupt()
{
  brake_flag = digitalRead(BRAKE_SW_PIN);
}

ISR(PCINT1_vect)
{
  hall_sen = PINC; // W||V||U
}
