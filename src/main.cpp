#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>
#include <Wire.h>  // Include Wire library if using I2C

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

#define RX    35  // PB22 24V dc output for controlling cooling FAN's
#define TX    36  // PB23

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
  // pinMode(TX, OUTPUT);
  // pinMode(RX, INPUT);

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
  driver.rms_current(500);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver.pwm_autoscale(1);
  driver.microsteps(16);

  stepper.setMaxSpeed(20*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(300*steps_per_mm); // 2000mm/s^2
  stepper.setEnablePin(ENn);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();

  analogReadResolution(12);

  SerialUSB.begin(9600);
  Serial.begin(9600, SERIAL_8N1);
}

void StopCheck(void *arg){
  bool old_End_A = digitalRead(END_A);
  bool old_End_B = digitalRead(END_B);
  Serial1.println("STOPTASK test");
  for (;;)
  {
    digitalWrite(LED1, HIGH);  // turn the LED on (HIGH is the voltage level)
    digitalWrite(LED0, LOW);   // turn the LED on (HIGH is the voltage level)
    Serial1.println("STOPTASK RUNNING");
    if (old_End_A != digitalRead(END_A) && digitalRead(END_A) == 0 || old_End_B != digitalRead(END_B) && digitalRead(END_B))
    {
      Serial1.println("STOP");
      stepper.stop();
    }

    delay(10);                      // wait for a second
    digitalWrite(LED1, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(LED0, HIGH);   // turn the LED off by making the voltage LOW 
    old_End_A = digitalRead(END_A);
    old_End_B = digitalRead(END_B);
  }
}

int32_t cal_temperature(uint32_t adc_in)
{
   adc_in -= 14;  // if no voltage applied to input have has some small baseline of 14
   adc_in = adc_in * 104000 / 100000; // add 4%

   int32_t result = (adc_in * adc_in) / 39011;  //   0,00002563354x^2
   result += (adc_in * 111586) / 1000000;       // + 0,1115863x
   result -= 204;                           // - 204,183
   return result;
}

void loop()
{
  Serial.println("TS1:200");
  test++;

  String input = Serial.readStringUntil('\n');
  // digitalWrite(LED1, HIGH);
  //  digitalWrite(LED1, LOW);

  SerialUSB.println(input);
  if (input.startsWith("M:UP"))
  {
        // Blink LED0 and LED1 for visual indication
    digitalWrite(LED1, HIGH);
    SerialUSB.println("LED HGIGHHGHGHGHHG");
  }
    if (input.startsWith("M:DOWN"))
  {
        // Blink LED0 and LED1 for visual indication
    digitalWrite(LED1, LOW);
    SerialUSB.println("LED LOWWWWW");
  }
    // digitalWrite(LED0, HIGH);
    // delay(500);  // Wait half a second
    // digitalWrite(LED0, LOW);
    // delay(500);  // Wait half a second
  
  delay(100);

  // if ((input == "MOT_UP" && digitalRead(END_A) == false) || (input == "MOT_UP" && (digitalRead(END_B) && digitalRead(END_A)) == true))
  // {
  //   stepper.setCurrentPosition(0);
  //   Serial1.println("moving up");
  //   stepper.disableOutputs();
  //   stepper.move(1000); // Move 100mm
  //   stepper.enableOutputs();
  //   while (stepper.run())
  //   {
  //     ;
  //   }
  //   stepper.disableOutputs();
  //   stepper.move(100 * 100000); // Move 100mm
  //   stepper.enableOutputs();
  //   while (stepper.run())
  //   {
  //     Serial1.println("moving up");
  //     if ( digitalRead(END_A) == LOW || digitalRead(END_B) == LOW)
  //     {
  //       Serial1.println("End switch triggered, stopping motor");
  //       stepper.stop();
  //       break;
  //     }
  //   }
  // }
  // else if ((input == "MOT_DOWN" && digitalRead(END_B) == false) || (input == "MOT_DOWN" && (digitalRead(END_B) && digitalRead(END_A)) == true))
  // {
  //   stepper.setCurrentPosition(0);
  //   Serial1.println("moving down");
  //   stepper.disableOutputs();
  //   stepper.move(-1000); // Move 100mm
  //   stepper.enableOutputs();
  //   while (stepper.run())
  //   {
  //     ;
  //   }

  //   stepper.disableOutputs();
  //   stepper.moveTo(-100 * 100000); // Move 100mm
  //   stepper.enableOutputs();
  //   while (stepper.run())
  //   {
  //     Serial1.println("moving down");
  //     if ( digitalRead(END_A) == LOW || digitalRead(END_B) == LOW)
  //     {
  //       Serial1.println("End switch triggered, stopping motor");
  //       stepper.stop();
  //       break;
  //     }
  //   }
  // }
}
