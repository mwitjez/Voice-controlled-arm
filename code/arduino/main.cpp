#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Arduino.h>

#define DIR_BASE_PIN 9     // Direction
#define STEP_BASE_PIN 8    // Step
#define DIR_JOINT1_PIN A1  // Direction
#define STEP_JOINT1_PIN A0 // Step
#define DIR_JOINT2_PIN A3  // Direction
#define STEP_JOINT2_PIN A2 // Step
#define CS_BASE_PIN 10     // Chip select mottor base
#define CS_JOINT1_PIN 7    // Chip select mottor joint 1
#define CS_JOINT2_PIN 4    // Chip select mottor joint 1
#define SW_MOSI 11         // Software Master Out Slave In (MOSI)
#define SW_MISO 12         // Software Master In Slave Out (MISO)
#define SW_SCK 13          // Software Slave Clock (SCK)
#define SERVO_PIN 3

#define R_SENSE 0.11f // Match to your driver- SilentStepStick series use 0.11

// Select your stepper driver type
TMC2130Stepper driver_base = TMC2130Stepper(CS_BASE_PIN, R_SENSE);     // Hardware SPI
TMC2130Stepper driver_joint1 = TMC2130Stepper(CS_JOINT1_PIN, R_SENSE); // Hardware SPI
TMC2130Stepper driver_joint2 = TMC2130Stepper(CS_JOINT2_PIN, R_SENSE); // Hardware SPI

constexpr uint32_t steps_per_mm = 80;

AccelStepper stepper_base = AccelStepper(stepper_base.DRIVER, STEP_BASE_PIN, DIR_BASE_PIN);
AccelStepper stepper_joint1 = AccelStepper(stepper_joint1.DRIVER, STEP_JOINT1_PIN, DIR_JOINT1_PIN);
AccelStepper stepper_joint2 = AccelStepper(stepper_joint2.DRIVER, STEP_JOINT2_PIN, DIR_JOINT2_PIN);

Servo servo_gripper;
int servo_start_pos;
int servo_pos;

String msg;

void close_gripper()
{
  for (servo_pos = 0; servo_pos <= 50; servo_pos += 1)
  {                                 // goes from 0 degrees to 50 degrees
    servo_gripper.write(servo_pos); // tell servo to go to position in variable 'pos'
    delay(15);                      // waits 15 ms for the servo to reach the position
  }
}

void open_gripper()
{
  Serial.println("Opening..");
  for (servo_pos = 50; servo_pos >= 0; servo_pos -= 1)
  {                                 // goes from 0 degrees to 50 degrees
    servo_gripper.write(servo_pos); // tell servo to go to position in variable 'pos'
    delay(15);                      // waits 15 ms for the servo to reach the position
  }
}

void rotate_base(int degrees, String dir)
// 44.44 =  5*16*200/360
// 5 - przełożenie
// 16 - rozdzielczość mikrokroków
// 200 - podstawowa rozdzielczość silnika
{
  if (dir == "right")
  {
    stepper_base.move(-degrees * 44.44);
  }
  else
  {
    stepper_base.move(degrees * 44.44);
  }
  stepper_base.runToPosition();
}

void move_joint1(int degrees)
{
  // 355.55 =  5*16*200/360
  // 40 - przełożenie
  // 16 - rozdzielczość mikrokroków
  // 200 - podstawowa rozdzielczość silnika
  stepper_joint1.move(degrees * 355.55);
  stepper_joint1.runToPosition();
}

void move_joint2(int degrees)
{
  // 853.33 =  5*16*200/360
  // 2.4*40 - przełożenie
  // 16 - rozdzielczość mikrokroków
  // 200 - podstawowa rozdzielczość silnika
  stepper_joint2.move(degrees * 853.33);
  stepper_joint2.runToPosition();
}

void readSerialPort()
{
  msg = "";
  if (Serial.available())
  {
    delay(10);
    while (Serial.available() > 0)
    {
      msg += (char)Serial.read();
    }
    Serial.flush();
  }
}

void setup()
{
  SPI.begin();
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Start...");

  // konfiguracja pierwszego sterownika
  pinMode(CS_BASE_PIN, OUTPUT);
  digitalWrite(CS_BASE_PIN, HIGH);
  driver_base.begin();          // Initiate pins and registeries
  driver_base.rms_current(600); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver_base.en_pwm_mode(1);   // Enable extremely quiet stepping
  driver_base.pwm_autoscale(1);
  driver_base.microsteps(16);

  stepper_base.setMaxSpeed(50 * steps_per_mm);      // 100mm/s @ 80 steps/mm
  stepper_base.setAcceleration(500 * steps_per_mm); // 2000mm/s^2
  stepper_base.setPinsInverted(false, false, true);
  stepper_base.enableOutputs();
  digitalWrite(CS_BASE_PIN, LOW);
  // konfiguracja drugiego sterownika
  pinMode(CS_JOINT1_PIN, OUTPUT);
  digitalWrite(CS_JOINT1_PIN, HIGH);
  driver_joint1.begin();          // Initiate pins and registeries
  driver_joint1.rms_current(600); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver_joint1.en_pwm_mode(1);   // Enable extremely quiet stepping
  driver_joint1.pwm_autoscale(1);
  driver_joint1.microsteps(16);

  stepper_joint1.setMaxSpeed(100 * steps_per_mm);     // 100mm/s @ 80 steps/mm
  stepper_joint1.setAcceleration(500 * steps_per_mm); // 2000mm/s^2
  stepper_joint1.setPinsInverted(false, false, true);
  stepper_joint1.enableOutputs();
  digitalWrite(CS_JOINT1_PIN, LOW);
  // konfiguracja trzeciego sterownika
  pinMode(CS_JOINT2_PIN, OUTPUT);
  digitalWrite(CS_JOINT2_PIN, HIGH);
  driver_joint2.begin();          // Initiate pins and registeries
  driver_joint2.rms_current(600); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver_joint2.en_pwm_mode(1);   // Enable extremely quiet stepping
  driver_joint2.pwm_autoscale(1);
  driver_joint2.microsteps(16);

  stepper_joint2.setMaxSpeed(50 * steps_per_mm);       // 100mm/s @ 80 steps/mm
  stepper_joint2.setAcceleration(1000 * steps_per_mm); // 2000mm/s^2
  stepper_joint2.setPinsInverted(false, false, true);
  stepper_joint2.enableOutputs();
  digitalWrite(CS_JOINT2_PIN, LOW);

  // konfiguracja serwomechanizmu
  servo_gripper.write(0);
  servo_gripper.attach(SERVO_PIN);
  servo_start_pos = servo_gripper.read();
}

void loop()
{
  readSerialPort();
  if (msg == "open")
  {
    open_gripper();
    Serial.println("Opening...");
  }
  else if (msg == "close")
  {
    close_gripper();
    Serial.println("Closing...");
  }
  else if (msg == "stop")
  {
    stepper_base.stop();
    stepper_joint1.stop();
    stepper_joint2.stop();
    Serial.println("Stoping..");
  }
  else if (msg.startsWith("rotate"))
  {
    int degrees = msg.substring(6, 8).toInt();
    String direction = msg.substring(8);
    Serial.println("Moving...");
    rotate_base(degrees, direction);
  }
  else if (msg.startsWith("move_joint1"))
  {
    int degrees = msg.substring(11).toFloat();
    Serial.println("Moving joint1...");
    Serial.println(degrees);
    move_joint1(degrees);
  }
  else if (msg.startsWith("move_joint2"))
  {
    int degrees = msg.substring(11).toFloat();
    Serial.println("Moving joint2...");
    Serial.println(degrees);
    move_joint2(degrees);
  }
  else if (msg != "")
  {
    Serial.println("Unknown message: ");
    Serial.println(msg);
  }
  delay(500);
}
