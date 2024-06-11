#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>

// The below pins are used for the VAP2 PCB. Designed by C.G. Hoogendijk for Delta Proto BV.
// It is based on an Arduino M0  (not PRO)

#define TEMP1   A0  // PA2   6 analog inputs for temperature sensors TP1000
#define TEMP2   A1  // PB8
#define TEMP3   A2  // PB9
#define TEMP4   A3  // PA4
#define TEMP5   A4  // PA5
#define TEMP6   A5  // PB2

#define LED0    4   // PA14 debug leds
#define LED1    5   // PA15

#define MISO    18  // PA12  SPI connection to TMC5160A for stepper control
#define MOSI    21  // PB10
#define SCK     20  // PB11
#define CS0n    43  // PA16
#define ENn     13  // PA17

#define STEP    6   // PA20
#define DIR     7   // PA21

#define HEAT1   0   // PA10 24V dc output for controlling heating elements thru solid state relay
#define HEAT2   1   // PA11

#define FANB    35  // PB22 24V dc output for controlling cooling FAN's
#define FANT    36  // PB23

#define END_A   17  // PA23 input pull-up  EndSwitches 
#define END_B   16  // PA22 input pull-up

#define SPARE   12  // PA19 input pull-up   
#define LIT     11  // PA16 input pull-up  Input for detecting LIT open or closed
#define PWR      8  // PA18 input is 24Volt present low when no power applied to stepper motor or temp sensors

#define R_SENSE 0.075f // Current sensor resistor value in ohm used on PCB

uint8_t test;

constexpr uint32_t steps_per_mm = 80;  // TBD

TMC5160Stepper driver = TMC5160Stepper(CS0n, R_SENSE, MOSI, MISO, SCK ); 
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP, DIR);
void StopCheck(void *arg);

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital output pins
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(HEAT1, OUTPUT);
  pinMode(HEAT2, OUTPUT);
  pinMode(FANT, OUTPUT);
  pinMode(FANB, OUTPUT);

  // initialize digital input pins
  pinMode(END_A, INPUT_PULLUP);
  pinMode(END_B, INPUT_PULLUP);
  pinMode(LIT, INPUT_PULLUP);
  pinMode(SPARE, INPUT_PULLUP);
  pinMode(PWR, INPUT);

  // pins for controlling stepper motor
  digitalWrite(ENn, HIGH);
  pinMode(ENn, OUTPUT);
  digitalWrite(CS0n, HIGH);
  pinMode(CS0n, OUTPUT);
  driver.begin();             // Initiate pins and registeries
  driver.rms_current(1000);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver.pwm_autoscale(1);
  driver.microsteps(16);

  stepper.setMaxSpeed(100*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(400*steps_per_mm); // 2000mm/s^2
  stepper.setEnablePin(ENn);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();

  // serial debug over USB
  SerialUSB.begin(115200);  //start Serial in case we need to print debugging info
  Serial.println("Start...");
  analogReadResolution(12);

    // xTaskCreate(
    //     StopCheck,
    //     "stopchecker", // Task name
    //     1024,       // Stack size
    //     NULL,       // Task parameters
    //     1,          // Task Priority
    //     NULL); 

        
}

void StopCheck(void *arg){
  bool old_End_A = digitalRead(END_A);
  bool old_End_B = digitalRead(END_B);
  SerialUSB.println("STOPTASK test");
  for (;;)
  {
    digitalWrite(LED1, HIGH);  // turn the LED on (HIGH is the voltage level)
    digitalWrite(LED0, LOW);   // turn the LED on (HIGH is the voltage level)
    SerialUSB.println("STOPTASK RUNNING");
    if (old_End_A != digitalRead(END_A) && digitalRead(END_A) == 0|| old_End_B != digitalRead(END_B) && digitalRead(END_B))
    {
      SerialUSB.println("STOP");
      stepper.stop();
    }

    delay(10);                      // wait for a second
    digitalWrite(LED1, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(LED0, HIGH);   // turn the LED off by making the voltage LOW 
    old_End_A = digitalRead(END_A);
    old_End_B = digitalRead(END_B);
  }
  
}

// The PT1000 is connected to gnd on one side
// and to a 15K resistor connected to 18V
// When temp is 0 degrees the sensor is 1K by definition
// This means the current to the 15K resistor is 18Volt/(15K+1K)
// is 1,125mA. On the ADC we will measure 1,125 Volt
// With a 3.3V ref this means for 10bits result 349
// So 349 is 0 degrees and the voltage over the 15K resistor is 18k*1,125mA=16,875V
// In the voltage range from 0 to 3,3 Volt the current thriuh this resistor is:
// 18V/15k = 1,2mA (18-3.3)V/15K = 0,98mA  (almost current source of +/-1mA)
// The restance we need to measure is 0  1000,0 ohm   1,125V  349/1396
//                                  100  1385,1 ohm   1,522V  472/1888
//                                  200  1758,6 ohm   1,889V  586/2344
//                                  300  2120,5 ohm   2,229V  692/2768
//                                  400  2470,8 ohm   2,546V  790/3160
//                                  500  2809,8 ohm   2,840V  872/3488
    
// Trendline according to https://mycurvefit.com/
//   0,00002563354x^2 + 0,1115863x - 204,183

// PT1000 resistance table from https://www.sterlingsensors.co.uk/pt1000-resistance-table

int32_t cal_temperature( uint32_t adc_in)
{
   adc_in -= 14;  // if no voltage applied to input have has some small baseline of 14
   adc_in = adc_in*104000/100000; // add 4%

   int32_t result = (adc_in*adc_in)/39011;  //   0,00002563354x^2
   result += (adc_in*111586)/1000000;       // + 0,1115863x
   result -= 204;                           // - 204,183
   return result;
}

// the loop function runs over and over again forever
// This particulair loop is for testing PCB only!!!
void loop()
 {
  test++;

  String input = SerialUSB.readStringUntil('\n');
  SerialUSB.println(input);

  // digitalWrite(LED1, HIGH);  // turn the LED on (HIGH is the voltage level)
  // digitalWrite(LED0, LOW);   // turn the LED on (HIGH is the voltage level)


  // delay(10);                      // wait for a second
  // digitalWrite(LED1, LOW);   // turn the LED off by making the voltage LOW
  // digitalWrite(LED0, HIGH);   // turn the LED off by making the voltage LOW 

  if ((input == "MOT_UP" && digitalRead(END_A) == false) || (input == "MOT_UP" && (digitalRead(END_B) && digitalRead(END_A)) == true))
  {
    stepper.setCurrentPosition(0);
    SerialUSB.println("moving up");
    stepper.disableOutputs();
    stepper.move(500); // Move 100mm
    stepper.enableOutputs();
    while (stepper.run())
    {
      ;
    }
    stepper.disableOutputs();
    stepper.move(100 * 10000); // Move 100mm
    stepper.enableOutputs();
    while (stepper.run())
    {
      SerialUSB.println("moving up");
      if ( digitalRead(END_A) == LOW || digitalRead(END_B) == LOW)
      {
        SerialUSB.println("End switch triggered, stopping motor");
        stepper.stop();
        break;
      }
    }
  }
  else if ((input == "MOT_DOWN" && digitalRead(END_B) == false) || (input == "MOT_DOWN" && (digitalRead(END_B) && digitalRead(END_A)) == true))
  {
    stepper.setCurrentPosition(0);
    SerialUSB.println("moving down");
    stepper.disableOutputs();
    stepper.move(-500); // Move 100mm
    stepper.enableOutputs();
    while (stepper.run())
    {
      ;
    }

    stepper.disableOutputs();
    stepper.moveTo(-100 * 10000); // Move 100mm
    stepper.enableOutputs();
    while (stepper.run())
    {
      SerialUSB.println("moving down");
      if ( digitalRead(END_A) == LOW || digitalRead(END_B) == LOW)
      {
        SerialUSB.println("End switch triggered, stopping motor");
        stepper.stop();
        break;
      }
    }
  }

}
