
/*
*   GoKart DC Motor Driver - 21st September 2023 - James Fotherby
*
*   Runs on a Arduino Pro-Mini
*   This program reads a throttle voltage (0.8-4.0v from a Hall effect pedal) on pin A0
*   This is used to generate a torque demand - ie. a motor current setpoint
*   A PI controller takes the measured current (from a 50A ACS758 Hall effect current sensor connected to pin A3) and the setpoint to generate an output PWM Duty cycle value
*
*   Timer 1 is configured to produce a 4kHz phase correct PWM signal which feeds the HW-039 H-Bridge driver via pins 9 and 10. Pins 7 and 12 enable to drivers.
*
*   There is a forward reverse switch conected to pin 5 - We only allow the motor to change direction when there is no current flowing and no voltage applied to the motor
*
*   We synchronise our ADC samples with our timer1 PWM and use interrupts:
*     - If our PWM Duty is >50% we take ADC readings in the middle of the high periods else we sample in the middle of the low periods
*     - This ensures we sample as far away as possible from the switching transients to get the best and most consistent current readings.
*     - We switch alternately between sampling the throttle and motor current ADC channels
*     - So we sample at the PWM Frequency of 4kHz (current measurements at 2kHz and throttle measurements at 2kHz)
*
*   The ADC Throttle values range between: 185-680
*   The Current sensor reports about 512. +ve values represent a forward torque at 10/Amp. Hence an ADC reading of 612 equates to +10A of motor current
*
*/

#define         Fwd_Rev_Pin   5
#define         Throttle_En   6     // I ran out of Vcc pins, so I just set this to high and use it to supply power to the throttle module (which only draws 6mA) 
#define         PWM_A_Enable  7

#define         PWM_T1A       9
#define         PWM_T1B       10

#define         PWM_B_Enable  12

#define         Throttle_In   A0
#define         Current_Sense A3

#include <PID_v1.h>
#include "Average.h"

double I_Setpoint = 512, I_Measured, D_Output;
double Kp = 4.5, Ki = 50, Kd = 0;

Average         FilterI(0);                                                 
Average         FilterThrottle(0);
PID             PID(&I_Measured, &D_Output, &I_Setpoint, Kp, Ki, Kd, DIRECT);

int             duty = 0;
byte            Reverse_Flag = 0;

void setup() {
  noInterrupts();

  // Configure filters and PI controllers
  FilterI.Set_Fraction_Filter_Const(0.9, 0.1);                                // Configure an exponential filter (ie. RC low pass filter) for the ADC readings 
  FilterI.Set_Fraction_Filter(512.0);                                         // 0-512-1024 where 512 represents 0 current so intiate the filter here

  FilterThrottle.Set_Fraction_Filter_Const(0.9, 0.1);                         // Filter for the throttle input too                       

  PID.SetMode(AUTOMATIC);                                                     // Configure our PIC controller
  PID.SetOutputLimits(0,1999);
  PID.SetSampleTime(10);

  // Configure Timer1 for PWM output
  ICR1 = 1999; 
  TCCR1A = 0b10100010;
  TCCR1B = 0b00010001;                                                        // 16MHz XO, No prescaler and 1/1000 up/down (phase correct) count leads to a 4kHz PWM wave.
  TIMSK1 = 0b00100001;                                                        // Enable Interrupts when timer counter at top and bottom (ie. middle of the PWM high and low sections)                                                    
  OCR1A = 0;
  OCR1B = 0;

  // Configure ADC Module
  ADMUX = B01000000;                                                          // Set V_Ref to be AVcc, Set channel to A0
  ADCSRA = B10001111;                                                         // Enable ADC, Enable conversion complete interrupt, set 128 Prescaler -> ADC Clk = 125KHz

  interrupts(); 

  // Configure Pins
  pinMode(PWM_T1A, OUTPUT);
  pinMode(PWM_T1B, OUTPUT);

  pinMode(PWM_A_Enable, OUTPUT);
  pinMode(PWM_B_Enable, OUTPUT);

  pinMode(Throttle_En, OUTPUT); 
  pinMode(Fwd_Rev_Pin, INPUT_PULLUP);

  pinMode(Throttle_In, INPUT);  
  pinMode(Current_Sense, INPUT); 

  digitalWrite(PWM_A_Enable, HIGH);
  digitalWrite(PWM_B_Enable, HIGH); 

  digitalWrite(Throttle_En, HIGH);  

  // Debugging
  //Serial.begin(115200);
}
  
// -------------------------------------------------------------------------------------------------------------------------
void loop() {
  static unsigned long Iterate_PID = millis();
  static unsigned long Debug_Time = millis();
  
  // Iterate our Torque Controller at regular 10 millisecond intervals
  long Next_Iterate_PID = Iterate_PID - millis();
  if(Next_Iterate_PID <= 0)
  {
    Iterate_PID += 10;                                                          // Iterate next in 10ms

    I_Setpoint = FilterThrottle.FF_Current - 225.0;                             // Translate the throttle values 

    if(duty > 1500) {                                                           // At higher speeds reduce the allowed torque to 20A (We have a 25A battery fuse)
      I_Setpoint = constrain(I_Setpoint, -30.0, 200.0);
    }
    else if(duty > 1000) {
      I_Setpoint = constrain(I_Setpoint, -30.0, 300.0);                         // Allow 30A at medium speeds (battery current will be <25A due to DC/DC conversion)
    }    
    else {
      I_Setpoint = constrain(I_Setpoint, -30.0, 360.0);                         // Allow 36A at low speeds
    } 

    if(Reverse_Flag == 0) {
      I_Measured = FilterI.FF_Current - 512.0;
      PID.Compute();
      duty = (int)D_Output;
    
      OCR1A = 0;
      OCR1B = duty;
    }
    else {
      I_Measured = 512.0 - FilterI.FF_Current;
      PID.Compute();
      duty = (int)D_Output;

      OCR1A = duty;
      OCR1B = 0;
    }
  }

  // Debug / reverse switch
  long Next_Debug = Debug_Time - millis();
  if(Next_Debug <= 0)
  {
    Debug_Time += 100;

    if(I_Measured < 10.0 && duty < 100 && I_Setpoint < 0.0) {                   // Only allow us to change the motor direction if there is no current or voltage to the motor (ie. We're stationary)
      if(digitalRead(Fwd_Rev_Pin))  {
        Reverse_Flag = 0;
      }
      else  {
        Reverse_Flag = 1;
      }
    }

    //Serial.print(I_Setpoint); Serial.print(",");
    //Serial.print(I_Measured); Serial.print(",");
    //Serial.println(Reverse_Flag);
  }
}

//  ######################################################################################################
//  ------------------------------------------------------------------------------------------------------
//  ######################################################################################################
// This code runs when the PWM signal is in the middle of its HIGH period
ISR(TIMER1_OVF_vect)
{
  if(duty < 1000)                                                               
    return;

  if(ADCSRA & 0b01000000)                                                       // If the ADC is still performing a conversion (it shouldn't be) then return
    return;   
      
  if(ADMUX & 1)  {
    ADMUX = B01000000;     
    ADCSRA |= B01000000;  
  } 
  else  {     
    ADMUX = B01000011;
    ADCSRA |= B01000000;   
  }
}

// This code runs when the PWM signal is in the middle of its LOW period
ISR(TIMER1_CAPT_vect)
{
  if(duty >= 1000)                                                              // When duties are low we want to sample in the middle of the low period because it's longer than the high period 
    return;

  if(ADCSRA & 0b01000000)                                                       // If the ADC is still performing a conversion (it shouldn't be) then return
    return;   
      
  if(ADMUX & 1)  {
    ADMUX = B01000000;     
    ADCSRA |= B01000000;  
  } 
  else  {     
    ADMUX = B01000011;
    ADCSRA |= B01000000;   
  }
}

// Interrupt service routine for ADC conversion complete
ISR(ADC_vect) 
{    
  unsigned char adcl = ADCL;
  unsigned char adch = ADCH;

  float ADC_Value = (float)((adch << 8) | adcl);

  if(ADMUX & 1)
    FilterI.Fraction_Filter(ADC_Value);
  else  
    FilterThrottle.Fraction_Filter(ADC_Value);
}






























