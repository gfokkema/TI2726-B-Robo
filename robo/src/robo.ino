#include "ros.h"
#include "geometry_msgs/Point32.h"
#include "motor.h"
#include "dualmotor.h"

// Left engine pins
#define LEFTREV   7
#define LEFTEN	 24
#define LEFTFWD   6

// Right engine pins
#define RIGHTREV  3
#define RIGHTEN  25
#define RIGHTFWD  2

// Proximity sensor pins
#define ECHO     22
#define TRIGGER  23

class NewHardware : public ArduinoHardware {
public:
	NewHardware() : ArduinoHardware(&Serial1, 57600) {};
};

Motor m1(LEFTREV, LEFTEN, LEFTFWD);
Motor m2(RIGHTREV, RIGHTEN, RIGHTFWD);
DualMotor motor(&m1, &m2);

ros::NodeHandle_<NewHardware> nh;
ros::Subscriber<geometry_msgs::Point32> cmd_vel_sub("cmd_vel", cmd_vel_cb);

/*******************
 * DEBUG VARIABLES *
 *******************/
long pulse_start = 0, pulse_end = 0;
bool pulse_dirty = false;

long timer_start = 0, timer_end = 0;
bool timer_start_dirty = false;
bool timer_end_dirty   = false;
/*******************/

void cmd_vel_cb(const geometry_msgs::Point32& cmd_vel_msg)
{
	Serial.print("speed: ");
	Serial.println(cmd_vel_msg.x);
	Serial.print("angular: ");
	Serial.println(cmd_vel_msg.z);
	
	motor.update(cmd_vel_msg.x, cmd_vel_msg.z);
}

void pulse_start_cb()
{
	pulse_start = micros();
}

void pulse_end_cb()
{
	pulse_end = micros();
	pulse_dirty = true;
}

ISR(TIMER1_COMPA_vect)
{
	digitalWrite(TRIGGER, LOW);
	timer_end = micros();
	// TODO: This should not be necessary!
	// TCNT1 = 0;
	timer_end_dirty = true;
}

ISR(TIMER1_COMPB_vect)
{
	digitalWrite(TRIGGER, HIGH);
	timer_start = micros();
	timer_start_dirty = true;
}

void setup()
{
	pinMode(ECHO,     INPUT);
	pinMode(TRIGGER, OUTPUT);
	
	// WGM1   = 0100 (ctc)
	// CS1    =  101 (1024 prescaler)
	// TIMSK1 = 0110 (interrupt on OCR1A, interrupt on OCR1B)
	// overflow frequency: 16 MHz / 1024 / 4096 = 4 Hz
	// desired width:      16 ms per 256 ms --> 4096 / 16 = 256
	noInterrupts();           // disable all interrupts
	OCR1A = 4095;
	OCR1B = 3839;
	TCCR1A = _BV(WGM12);
	TCCR1B = _BV(CS12) | _BV(CS10);
	TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);
	interrupts();             // enable all interrupts
	
	attachInterrupt(ECHO, pulse_start_cb, RISING);
	attachInterrupt(ECHO, pulse_end_cb,  FALLING);

	nh.initNode();
	nh.subscribe(cmd_vel_sub);
}

void loop()
{
	if (timer_start_dirty)
	{
		Serial.print("start: ");
		Serial.println(timer_start);
		timer_start_dirty = false;
	}
	
	if (timer_end_dirty)
	{
		Serial.print("end:   ");
		Serial.println(timer_end);
		timer_end_dirty = false;
	}
	
	if (pulse_dirty)
	{
		// Speed of sound is 340 m/s or 29 cm/microsecond
		// The pulse travels back and forth, so we divide this by 2
		long pulse_dt = pulse_end - pulse_start;
		int pulse_distance = pulse_dt / 29 / 2;
		
		Serial.println("Yay the whole chain works!");
		Serial.print("Distance to obstacle: ");
		Serial.println(pulse_distance);
		pulse_dirty = false;
	}
	
	nh.spinOnce();
}
