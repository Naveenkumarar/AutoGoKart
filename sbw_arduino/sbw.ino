#include <AccelStepper.h>
#include <MultiStepper.h>

#include <AS5047P.h>
#define LED_PIN LED_BUILTIN

#define AS5047P_CHIP_SELECT_PORT 7 
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/point_stamped.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__PointStamped input;

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
  }
}


AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);


#define DIR_PIN 11
#define STEP_PIN 12

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
MultiStepper steppers;

float relative_position = 97.2;
float right_limit = 0.4;
float left_limit = 194;

int maxLimit = 90;
int minLimit = -90;
float encoder_value = 0;
int spr = 4000;

float map_val(float value, int spr){
  
  float leftspan = 360.0;
  float rightspan = float(spr);
  float valueScaled = value / leftspan;
  return valueScaled * rightspan;
  
}

float prev = 0;
float angle = 0.0;
void subscription_callback(const void * msgin){

  const geometry_msgs__msg__PointStamped * msg_in = (const geometry_msgs__msg__PointStamped *)msgin;
   float rad_angle = msg_in->point.x;
  angle = rad_angle * 57296/1000;
  angle = angle * 3.23;

  if(prev != angle)
  {
    if(angle > 90) angle = 90;
    if(angle < -90) angle = -90;
    
    float change = prev - angle;
    prev = angle;

    angle = change;

    digitalWrite(LED_PIN, (int(angle) > 0) ? LOW : HIGH);
    Serial.println(angle);
  
    if(angle<0)
      digitalWrite(DIR_PIN,LOW);
    else
      digitalWrite(DIR_PIN,HIGH);
      
    int step_input = int(map_val(abs(angle),spr));
    
    for (int i=0;i<step_input;i++){
    digitalWrite(STEP_PIN,HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN,LOW);
    delayMicroseconds(1000);
    
    }
    
  }

}
void setup() {

  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();
//
//  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
//    RCCHECK(rclc_node_init_default(&node, "sbw", "", &support));
//debugln("HIIIIII");
size_t domain_id = 5;
  
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options,allocator));
  
  RCCHECK(rcl_init_options_set_domain_id(&init_options,domain_id));
  
  // create init_options
  
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  rcl_node_options_t node_ops = rcl_node_get_default_options();

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));



  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "sbw_feedback"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped),
    "sbw_control"));


  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &input, &subscription_callback, ON_NEW_DATA));

  pinMode(LED_PIN, OUTPUT);
     pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
  
  Serial.begin(115200);

  stepper.setMaxSpeed(100);
  steppers.addStepper(stepper);
  
  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }

  encoder_value = as5047p.readAngleDegree();
  int diff = relative_position - int(encoder_value);

  if(diff<0)
      digitalWrite(DIR_PIN,HIGH);
    else
      digitalWrite(DIR_PIN,LOW);
      
  int step_input = int(map_val(abs(diff),spr));
    
    for (int i=0;i<step_input;i++){
    digitalWrite(STEP_PIN,HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN,LOW);
    delayMicroseconds(1000);
    
    }
  
}


void loop() {
  msg.data = as5047p.readAngleDegree();
  Serial.println(msg.data);     
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));


    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
