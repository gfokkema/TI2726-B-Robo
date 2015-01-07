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

long pulse_start = 0, pulse_end = 0;
bool pulse_dirty = false;
bool timer_dirty = false;

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

ISR(TIMER2_COMPA_vect)
{
	digitalWrite(TRIGGER, HIGH);
	timer_dirty = true;
}

ISR(TIMER2_COMPB_vect)
{
	digitalWrite(TRIGGER, LOW);
}

void setup()
{
	pinMode(ECHO,     INPUT);
	pinMode(TRIGGER, OUTPUT);
	
	// WGM2   = 010 (ctc)
	// CS2    = 100 (64 prescaler)
	// TIMSK2 = 011 (interrupt on ctc, interrupt on overflow)
	// frequency: 16 MHz / 64 / 256 = 1024Hz (4 clock overflows per second)
	// desired width: 10 ms per 250 ms --> 256 / 25 = 11
	noInterrupts();           // disable all interrupts
	OCR2A = 255;
	OCR2B = 11;
	TCCR2A = _BV(WGM21);
	TCCR2B = _BV(CS22);
	TIMSK2 = _BV(OCIE2A) | _BV(OCIE2B);
	interrupts();             // enable all interrupts
	
	attachInterrupt(ECHO, pulse_start_cb, RISING);
	attachInterrupt(ECHO, pulse_end_cb,  FALLING);

	nh.initNode();
	nh.subscribe(cmd_vel_sub);
}

void loop()
{
	if (timer_dirty)
	{
		Serial.println("every 250 ms");
		
		timer_dirty = false;
	}
	
	if (pulse_dirty)
	{
		Serial.println("Yay the whole chain works!");
		// Speed of sound is 340 m/s or 29 cm/microsecond
		// The pulse travels back and forth, so we divide this by 2
		long pulse_dt = pulse_end - pulse_start;
		int pulse_distance = pulse_dt / 29 / 2;
		
		pulse_dirty = false;
	}
	
	nh.spinOnce();
}
