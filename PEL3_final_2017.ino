 /* PELS3 - August 14, 2017- Universal Electronic Lead Screw Code. This code is to be used on and Electronic Lead Screw outlined in: "An Electronic Lead screw for a D6000 Wabeco Lathe", by D.B Plewes, 
  Digital Machinist, Vol 11, No.4, 6-19, 2016). This uses a 1024 pulse/spindle-rev encoder on the Wabeco D6000 lathe and assumes a 400 steps/revolution attached to the lead screw.  
  This uses floating point arithmetic by comparing the desired thread position and against the discrete position which has been delivered by the stepper motor.  
  The code updates the step pulses as needed to keep the same position between the desired position at that delivered. The selection between metric and imperial threads is 
  achieved by a push-button on a rotary encoder while the value of the thread/turning parameters are achieved by rotating the rotary switch.
    
 The inputs to the Arduino which are needed is as follows:
  - knob rotary encoder - Q to Arduino pin 4, I to Arduino pin 5, button to Arduino pin 6. 
  - spindle rotary encoder goes to LS7184 chip.  Output from pin 6 of LS7184 goes to Arduino pin 2. 
  - input to step pin of stepper motor controller comes from Arduino pin 13. 
  - input to direction of stepper motor controller comes from the direction output of LS7184
  - output from Arduino pin 7 to change the output_pulses/input_pulses ratio to the LM7184 chip via line 'range_select_pin' 
  
The button is used to put the ELS into a programming mode so that the menu parameter can be changed.  Once the parameter is found we push the button again to lock in this parameter
so that it can't change during operation. All the thread parameters are put into one big menu file which includes: Turning, Imperial threads and Metric threads. The approximate cut depth 
for the cross-slide set at 59.5 degrees are shown on the LCD display for each thread size in mm's.

 This program is free software. It can be redistributed and/or modified under the terms of the GNU General Public License as published by the Free Software Foundation. 
 This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the GNU General Public License for more details. You can read a copy of the GNU General Public License at http://www.gnu.org/licenses/. 
  
 
  */  

  #include <Wire.h>
  #include <Adafruit_RGBLCDShield.h>
 Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
 
 // User parameters to be altered depending on lathe parameters...................................................... 
 // To use this code one must replace these parameters with correct values for your lathe. These are 'spindle_encoder_resolution','lead_screw_pitch' and 'motor_steps'.  
 // For example, if you had a spindle encoder of 200 step/rev... then edit the current 'spindle_encoder_resolution=1024' to read 'spindle_encoder_resolution=200'.
 // The applies to the 'lead_screw_pitch parameter' and the 'motor_steps' parameter. I have used numbers which apply to a Wabeco D6000 lathe. You must edit for your particular lathe. 
 
 int spindle_encoder_resolution=1024;   // the number of pulses per revolution of the spindle encoder
 float lead_screw_pitch=4.0;            // the pitch of the lathe lead screw in mm
 int motor_steps=400;                   // the number of steps per revolution of the lead screw
 float pitch=0.085;                     // the pitch to be cut in millimeters.  It also defines the lathe pitch for turning when first power on. 
 
 //......................................................................................................................
 // System parameters which need no alteration. 
 int stepper_pin=13;                                //the output pin for the stepper pin for the motor driver
 int encoder0PinA = 5;                              //the input pin for knob rotary encoder 'I' input
 int encoder0PinB = 4;                              //the input pin for knob rotary encoder 'Q' input
 int buttonPin=6;                                   // the button for the knob rotary encoder push button line
 int range_select_pin=7;                            // the pin to LS7184 from mode 0 to mode 2, permanently set to LOW for this code. Legacy from PELS1
 int menu = 1;                                      // the parameter for the menu select
 int encoder0PinALast = LOW;                        //a dummy parameter for the rotary encoder quad select algorithm
 int n = HIGH;                                      //another dummy parameter for the rotary encoder quad select algorithm
 int newButtonState = 0;                            //a parameter for the button push algorithm
 int oldButtonState=0;                              //a parameter for the button push algorithm
 int mode_select=1;                                 // a parameter to define the programming versus operation settings
 int tpi;                                           // a paremter to define the number of threads/inch

 float depth;                                       // a parameter to define the thread depth in mm on the compound slide. This is set at 75% of the pitch which seems to work
 float pitch_factor=0.75;                           // a parameter to define how deep to push the oblique cutter for each thread pitch in mm. May differ depending on thread design. This one works
 volatile long input_counter=0;                     //a parameter for the interrupt to count input pulses
 volatile float factor;                             // the ratio of needs steps/rev for stepper to spindle_encoder_resolution for each thread pitch we pick, this is calculated in the programme 
 volatile long delivered_stepper_pulses=0;          //number of steps delivered to the lead screw stepper motor
 volatile float calculated_stepper_pulses=0;        //number of steps we should have delivered for a given lead screw pitch
 
 void setup()
 { 
   pinMode (encoder0PinA,INPUT);                        //input for the Q channel of the switch rotary encoder
   pinMode (encoder0PinB,INPUT);                        //input for the I channel of the switch rotary encoder
   pinMode (buttonPin,INPUT_PULLUP);                    //input for the button of the switch rotary encoder
   pinMode (range_select_pin,OUTPUT);                   // set up digital pin "range_select_pin" to digital output to change the state of the LS7184 quad chip with HIGH giving 4x pulse number and LOW giving 1x pulse number. Legacy from PELS1
   lcd.begin(16,2);                                     // initializes the LCD display
   pinMode(stepper_pin, OUTPUT);                        // sets up the stepper_pin as an output for the stepper pulses
   attachInterrupt(0, count, RISING);                    // enable the interrupt for Arduino pin 2 which is interrupt "0"
   digitalWrite(range_select_pin,LOW);                   // sets ground to pin 6 of LS7184 chip to range select for LS7184 chip for a factor of 1.  
   factor= (motor_steps*pitch)/(lead_screw_pitch*spindle_encoder_resolution); //initial factor when turning on the ELS to deliver a turning operation of "medium" pitch
   
   
 //.................This next section starts the system to Nnormal Imperial Turning...............................................
   lcd.setCursor(0,0);                                    
   lcd.print("Turning");
   lcd.setCursor(0,1);
   lcd.print("Normal    ");
 } 
    
   
   
 void count()    //this is the interrupt routine for the floating point division algorithm
  {
    input_counter++;                                                            // increments a counter for the number of spindle pulses received
    calculated_stepper_pulses=round(factor*input_counter);                      // calculates the required number of stepper pulses which should have occured based on the number spindle pulses (input_counter number)
    if((calculated_stepper_pulses>delivered_stepper_pulses)&&(mode_select==0))  // if the calculated number of pulses is greated than the delivered pulses, we deliver one more stepper pulse only if mode_select is set for lathe (==0)
       {
      digitalWrite(stepper_pin,HIGH);                              // turns the stepper_pin output pin to HIGH
      delayMicroseconds(10);                                       // keeps that level HIGH for 10 microseconds
      digitalWrite(stepper_pin,LOW);                               // turns the stepper_pin output pin to LOW
      delivered_stepper_pulses++;                                  // increment the number of delivered_stepper_pulses to reflect the pulse just delivered
      }
  }

 void thread_parameters()                                           //this defines the parameters for the thread and turning for both metric and imperial threads
 { 
   newButtonState = digitalRead(buttonPin);                     // Get the current state of the button
      if (newButtonState == HIGH && oldButtonState == LOW)     // Has the button gone high since we last read it?
         { mode_select=!mode_select;}
                      
         if (mode_select == 0)                                  //mode_select==0 for lathe operation which I call "lathe"   
           {       
         lcd.setCursor(11,1);
         lcd.print("lathe");
            }
          else
            {
             mode_select=1; 
          lcd.setCursor(11,1);                                  // mode_select==1 for parameter selection which I call "prog" for programme
          lcd.print(" prog");
            }
         oldButtonState = newButtonState;   
            
  if(mode_select==1)
          {
           n = digitalRead(encoder0PinA);                                        //Selecting the Thread and Turning Parameters
                 if ((encoder0PinALast == LOW) && (n == HIGH)) {                 //true if button got pushed?
                       if (digitalRead(encoder0PinB) == LOW) {                   //this is the quadrature routine for the rotary encoder
                       menu++;
                 } else {
                       menu--;
                 }
                 Serial.println(menu);
                  if(menu>35){                                      //the next four lines allows the rotary select to go around the menu as a loop in either direction
                             menu=35;
                             }
                  if(menu<1){
                             menu=1;
                             }
                                 switch(menu) {
                                             case(1):     pitch=0.085;                  break;  // Normal Turning
                                             case(2):     pitch=0.050;                  break;  // Fine Turning
                                             case(3):     pitch=0.160;                  break;  // Coarse Turning
                               //...........................................................................................imperial data              
                                             case(4):     tpi=11;   break;
                                             case(6):     tpi=12;   break;
                                             case(7):     tpi=13;   break;
                                             case(8):     tpi=16;   break;
                                             case(9):     tpi=18;   break;
                                             case(10):    tpi=20;   break;
                                             case(11):    tpi=24;   break;
                                             case(12):    tpi=28;   break;
                                             case(13):    tpi=32;   break;
                                             case(14):    tpi=36;   break;
                                             case(15):    tpi=40;   break;
                                             case(16):    tpi=42;   break;
                                             case(17):    tpi=44;   break;
                                             case(18):    tpi=48;   break;
                                             case(19):    tpi=52;   break;
                              //.............................................................................................metric data               
                                             case(20):    pitch=0.4;   break;      
                                             case(21):    pitch=0.5;   break;      
                                             case(22):    pitch=0.7;   break;      
                                             case(23):    pitch=0.75;  break;      
                                             case(24):    pitch=0.8;   break;      
                                             case(25):    pitch=1.0;   break;      
                                             case(26):    pitch=1.25;  break;      
                                             case(27):    pitch=1.5;   break;      
                                             case(28):    pitch=1.75;  break;      
                                             case(29):    pitch=2.0;   break;      
                                             case(30):    pitch=2.5;   break;      
                                             case(31):    pitch=3.0;   break;      
                                             case(32):    pitch=3.5;   break;      
                                             case(33):    pitch=4.0;   break;      
                                             case(34):    pitch=5.0;   break;
                                             case(35):    pitch=7.0;   break;
                                                }
                                                     if(menu<4){
                                                     factor= (motor_steps*pitch)/(lead_screw_pitch*spindle_encoder_resolution); 
                                                     switch(menu) {
                                                           case(1):     lcd.setCursor(0,0);     lcd.print("Turning         ");     lcd.setCursor(0,1);     lcd.print("Normal     ");     break;
                                                           case(2):     lcd.setCursor(0,0);     lcd.print("Turning         ");     lcd.setCursor(0,1);     lcd.print("Fine       ");     break;
                                                           case(3):     lcd.setCursor(0,0);     lcd.print("Turning         ");     lcd.setCursor(0,1);     lcd.print("Coarse     ");     break;
                                                           
                                                                   }}
                                                     else
                                                      {
                                                      if(menu<20)
                                                      {
                                                        depth=pitch_factor*25.4/tpi;                      //the depth of cut in mm on the compound slide I need for each thread pitch.  I use this during operation rather than looking it up each time
                                                        factor= motor_steps*25.4/(tpi*lead_screw_pitch*spindle_encoder_resolution);            //the imperial factor needed to account for details of lead screw pitch, stepper motor #pulses/rev and encoder #pulses/rev
                                                        lcd.setCursor(0,0);  lcd.print("Imperial ");  lcd.print(tpi);       lcd.print(" tpi ");
                                                        lcd.setCursor(0,1);  lcd.print("depth="); lcd.print(depth);     lcd.print(" mm"); 
                                                      }
                                                     else
                                                      {
                                                        depth=pitch_factor*pitch;                          //the depth of cut in mm on the compound slide
                                                        factor=pitch*motor_steps/(lead_screw_pitch*spindle_encoder_resolution);         //the metric factor needed to account for details of lead screw pitch, stepper motor #pulses/rev and encoder #pulses/rev
                                                        lcd.setCursor(0,0);     lcd.print("Metric ");  lcd.print(pitch);     lcd.print(" mm");
                                                        lcd.setCursor(0,1);     lcd.print("depth=");         lcd.print(depth);     lcd.print(" mm");
                                                      }
                                                      }
                                                     
                 }
              delivered_stepper_pulses=0;
              input_counter=0;  
          }
                                                       
                  encoder0PinALast = n; 
              
 }
 
  
   void loop()
   {
   thread_parameters();
   }
   

 
