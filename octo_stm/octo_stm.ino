/**

  (SBC communication)
   RX1-TX1 pins:
     PA9  : Tx1
     PA10 : Rx1

  (Distance Sensor - Left)
   RX1-TX1 pins:
     PA2 : Tx2
     PA3 : Rx2

  (Distance Sensor - Right)
   RX1-TX1 pins:
     PA11 : Tx6
     PA12 : Rx6

  (Wire draw sensor)
  I2C2 pins:
     PB6  : SCL1_PIN
     PB7  : SDA1_PIN

  (12V DC motor cytron)
  DC Motor pins:
      PA7 : ACT_A1_PIN
      PA6 : ACT_A2_PIN

  (Flow sensor)
  interrupt pin:
      PA0 : FLOW_SENSOR_PIN

  (Lin Servo)
    attach pin:
      PB13 : LINEAR_SERVO_PIN

  (Switching)
    RELAY_1 (PB0)   : 1
    RELAY_2 (PB1)   : 2
    RELAY_3 (PB2)   : 3
    RELAY_4 (PB10)  : 4


*/

/*************************** <> ***************************/

/*************************** <include header files> ***************************/
#include "stm_include.h"
#include "ros_include.h"

/*************************** <std function declarations> ***************************/
void gpioInit();
void controlLinServo();
void getWire();
void getDistanceL();
void getDistanceR();
void getFlow();
void flowPulseCounter();

/*************************** <ros function declarations> ***************************/
void init_pub_sub_serv();
void callback_motor_toggle(const Trigger::Request &req, Trigger::Response &res);
void callback_relay_toggle(const RelayControl::Request &req, RelayControl::Response &res);


/*
   ros-handle
*/
ros::NodeHandle nh;

/*
   Publisher
*/
std_msgs::Int64 distL;
ros::Publisher distL_pub("dist_l", &distL);

std_msgs::Int64 distR;
ros::Publisher distR_pub("dist_r", &distR);

std_msgs::Float32 wireVal;
ros::Publisher wire_pub("wire_value", &wireVal);

/*
   Server
*/
ros::ServiceServer<Trigger::Request, Trigger::Response> motor_trig("motor_trig", &callback_motor_toggle);       // 12V DC motor
ros::ServiceServer<RelayControl::Request, RelayControl::Response> relay_trig("relay_toggle_channel", &callback_relay_toggle); // switching


/*************************** <setup> ***************************/
void setup() {

  SerialL.begin(9600);
  SerialR.begin(9600);

  nh.getHardware()->setBaud(57600);

  if (digitalPinToInterrupt(FLOW_SENSOR_PIN) == NOT_AN_INTERRUPT)
  {
    debugln("Pin PA0 is not an interrupt capable.");
  } else
  {
    debugln("Pin PA0 is an interrupt capable.");
  }

  gpioInit();
  nh.initNode();
  init_pub_sub_serv();

}


/*************************** <loop> ***************************/
void loop() {

  controlLinServo();
  getWire();
  getDistanceL();
  getDistanceR();
  getFlow();

  nh.spinOnce();
  delay(1);
}


/*************************** <std function definitions> ***************************/
void gpioInit()
{
  /** I2C */
  Wire.setSDA(SDA1_PIN);
  Wire.setSCL(SCL1_PIN);
  Wire.begin();

  // draw wire
  D_WIRE.begin();
  D_WIRE.setGain(0);


  // 12V dc motor cytron
  pinMode(ACT_A1_PIN, OUTPUT);
  pinMode(ACT_A2_PIN, OUTPUT);

  // lin-servo (pwm-out)
  pinMode(LINEAR_SERVO_PIN, OUTPUT);

  // flow sensor
  pinMode(FLOW_SENSOR_PIN, INPUT);

  // inbuilt led
  pinMode(LED_PIN, OUTPUT);

  // switch
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);




  // 12V DC motor (cytron)
  digitalWrite(ACT_A1_PIN, LOW);
  digitalWrite(ACT_A2_PIN, LOW);

  analogWrite(LINEAR_SERVO_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);

  digitalWrite(RELAY_1, LOW);
  digitalWrite(RELAY_2, LOW);
  digitalWrite(RELAY_3, LOW);
  digitalWrite(RELAY_4, LOW);

  // flow sensor pulse count
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseCounter, RISING);

}


void controlLinServo()
{
  //test

  if (millis() - prev_servo_millis > SERVO_DELAY)
  {
    if (servoPose <= 0) {
      servoFlag = true;
    } else if (servoPose >= 255) {
      servoFlag = false;
    }

    debug("servoFlag: ");
    debug(servoFlag);
    debug(" | servoPose:" );
    debugln(servoPose);

    analogWrite(LINEAR_SERVO_PIN, servoPose);

    if (servoFlag == true) {
      servoPose++;
    } else {
      servoPose--;
    }
    prev_servo_millis = millis();
  }
}

void getWire()
{
  if (millis() - prev_wire_millis > WIRE_DELAY)
  {

    int16_t adcVal = D_WIRE.readADC(0);
    float mVolt = adcVal * (ADS_GAIN / ADC_VAL_15);
    float mAmp = mVolt / MV_TO_AMP;

    debug("ADC: ");
    debug(adcVal);
    debug(" | mVolt: ");
    debug(mVolt);
    debug(" | mAmp: ");
    debugln(mAmp);

    wireVal.data = mAmp;
    wire_pub.publish(&wireVal);
    prev_wire_millis = millis();
  }

}

void getDistanceL()
{

  if (SerialL.available() > 0)
  {
    delay(4);

    if (SerialL.read() == 0xff)
    {
      dataL_buffer[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        dataL_buffer[i] = SerialL.read();
      }

      csL = dataL_buffer[0] + dataL_buffer[1] + dataL_buffer[2];

      if (dataL_buffer[3] == csL) {
        distanceL = (dataL_buffer[1] << 8) + dataL_buffer[2];

        debug("distL: ");
        debug(distanceL);
        debugln(" mm");

        distL.data = distanceL;
        distL_pub.publish(&distL);
      }
    }
  }
}

void getDistanceR()
{

  if (SerialR.available() > 0)
  {
    delay(4);

    if (SerialR.read() == 0xff)
    {
      dataR_buffer[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        dataR_buffer[i] = SerialR.read();
      }

      csR = dataR_buffer[0] + dataR_buffer[1] + dataR_buffer[2];

      if (dataR_buffer[3] == csR) {
        distanceR = (dataR_buffer[1] << 8) + dataR_buffer[2];

        debug("distR: ");
        debug(distanceR);
        debugln(" mm");

        distR.data = distanceR;
        distR_pub.publish(&distR);
      }
    }
  }

}
void getFlow()
{
  if (millis() - prev_flow_millis > FLOW_DELAY) {

    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));

    flowRate = ((FLOW_DELAY / (millis() - prev_flow_millis)) * pulseCount) / FLOW_CONSTANT;
    prev_flow_millis = millis();


    flowRate_ml = (flowRate / 60) * 1000;
    totalVolume  += flowRate_ml;

    debug("flowRate: ");
    debug(flowRate_ml);
    debug(" mL/s | ");


    debug("flowVolume: ");
    debug(totalVolume);
    debugln(" mL");
    pulseCount = 0;

    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseCounter, RISING);
  }
}

void flowPulseCounter()
{
  pulseCount++;
}





/*************************** <ros function definitions> ***************************/

void init_pub_sub_serv()
{
  // Publisher
  nh.advertise(distL_pub);
  nh.advertise(distR_pub);
  nh.advertise(wire_pub);

  //Service
  nh.advertiseService(motor_trig);
  nh.advertiseService(relay_trig);
}


void callback_motor_toggle(const Trigger::Request &, Trigger::Response &res)
{
  if (motorFlag)
  {
    digitalWrite(ACT_A1_PIN, HIGH);
    digitalWrite(ACT_A2_PIN, LOW);
    res.message = "motor: on";
  }
  else
  {
    digitalWrite(ACT_A1_PIN, LOW);
    digitalWrite(ACT_A2_PIN, LOW);
    res.message = "motor: off";
  }
  motorFlag = !motorFlag;

  res.success = true;
}


void callback_relay_toggle(const RelayControl::Request &req, RelayControl::Response &res)
{
  switch (req.data)
  {
    // turn off everything
    case 0:
      digitalWrite(RELAY_1, LOW);
      digitalWrite(RELAY_2, LOW);
      digitalWrite(RELAY_3, LOW);
      digitalWrite(RELAY_4, LOW);
      break;

    // RELAY_1
    case 1:
      digitalWrite(RELAY_1, HIGH - digitalRead(RELAY_1));
      break;

    // RELAY_2
    case 2:
      digitalWrite(RELAY_2, HIGH - digitalRead(RELAY_2));
      break;

    // RELAY_3
    case 3:
      digitalWrite(RELAY_3, HIGH - digitalRead(RELAY_3));
      break;

    // RELAY_4
    case 4:
      digitalWrite(RELAY_4, HIGH - digitalRead(RELAY_4));
      break;

    default:
      break;
  }

  digitalWrite(LED_PIN, HIGH - digitalRead(LED_PIN));
  res.response = true;
}

//  -- END OF FILE --
