#include <LiquidCrystal.h>
#include "DHT.h"
#include <OneWire.h> 
#include <PID_v1.h>
#include <EEPROM.h>

//menu stuff
const char string0[] PROGMEM = "Set fan to 100%";
const char string1[] PROGMEM = "Set fan to 90%";
const char string2[] PROGMEM = "Set fan to 0%";
const char string3[] PROGMEM = "Set fan %";
const char string4[] PROGMEM = "Lock current temp";
const char string5[] PROGMEM = "Set to dew point";
const char string6[] PROGMEM = "Set target temp";
const char string7[] PROGMEM = "Use last temp";
const char string8[] PROGMEM = "Save current temp";
const char string9[] PROGMEM = "Back";

class Printer
{
public:
	inline void begin(int)
	{
	}
	
	template <class T>
	inline void print(T)
	{
	}
	
	template <class T>
	inline void println(T)
	{
	}
	
	inline int available(void)
	{
		return 0;
	}
	
	inline float parseFloat(void)
	{
		return 0;
	}
};

Printer PRINTER;

#define Serial PRINTER

const char* const string_table[] PROGMEM = {string0, string1, string2, string3, string4, string5, string6, string7, string8, string9};

enum Level
{
	kInfo,
	kOptionsPresent,
	kInOptionsList,
	kInOption,
	kToReturn,
};

int current_pos = 0;
Level level = kInfo;

//button stuff
int buttonState[3];
int lastButtonState[3] = {LOW, LOW, LOW};
long lastDebounceTime[3] = {0, 0, 0};
const long debounceDelay = 50;

//my pins
//d3 and d5 are free
const int DS18S20_Pin = 6;
const int fan_pin = 11;
const int DHTPIN = 12;
const int buttonPin[3] = {16, 14, 15};

LiquidCrystal lcd(4, 2, 7, 8, 9, 10);

//temperature probe
OneWire ds(DS18S20_Pin);

#define DHTTYPE DHT11

//temp and humidity
DHT dht(DHTPIN, DHTTYPE);

#define LAST_TEMPS 16
float last_probe_t[LAST_TEMPS];

double Setpoint, Input, Output;
double Kp=2, Ki=0.1, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

float getTemp(){
	//returns the temperature from one DS18S20 in DEG Celsius

	byte data[12];
	byte addr[8];

	if ( !ds.search(addr))
	{
		//no more sensors on chain, reset search
		ds.reset_search();
		return -1000;
	}

	if ( OneWire::crc8( addr, 7) != addr[7])
	{
		Serial.println("CRC is not valid!");
		return -1000;
	}

	if ( addr[0] != 0x10 && addr[0] != 0x28)
	{
		Serial.print("Device is not recognized");
		return -1000;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44,1); // start conversion, with parasite power on at the end

	byte present = ds.reset();
	ds.select(addr);    
	ds.write(0xBE); // Read Scratchpad


	for (int i = 0; i < 9; i++)
	{ // we need 9 bytes
		data[i] = ds.read();
	}

	ds.reset_search();

	byte MSB = data[1];
	byte LSB = data[0];

	float tempRead = ((MSB << 8) | LSB); //using two's compliment
	float TemperatureSum = tempRead / 16;

	return TemperatureSum;
}

float get_dew_point(float t, float h)
{
	float dp = 243.04*(log(h / 100)
		+ ((17.625 * t) / (243.04 + t))) / (17.625 - log(h / 100)
		- ((17.625 * t) / (243.04 + t)));
		
	return dp;
}

bool button_check(int id)
{
	// read the state of the switch into a local variable:
	int reading = !digitalRead(buttonPin[id]);
	
//	Serial.println(reading);

	// check to see if you just pressed the button 
	// (i.e. the input went from LOW to HIGH),  and you've waited 
	// long enough since the last press to ignore any noise:  

	// If the switch changed, due to noise or pressing:
	if (reading != lastButtonState[id])
	{
		// reset the debouncing timer
		lastDebounceTime[id] = millis();
	}

	// save the reading.  Next time through the loop,
	// it'll be the lastButtonState:
	lastButtonState[id] = reading; 

	if ((millis() - lastDebounceTime[id]) > debounceDelay)
	{
		// whatever the reading is at, it's been there for longer
		// than the debounce delay, so take it as the actual current state:

		// if the button state has changed:
		if (reading != buttonState[id]) {
		buttonState[id] = reading;

		return true;
		}
	}

	return false;
}

void setup()
{
	Serial.begin(9600);

	dht.begin();
	
	lcd.begin(20, 4);
	lcd.clear();
	
	lcd.print(F("build date"));
	lcd.setCursor(0, 1);
	lcd.print(F(__DATE__));
	lcd.setCursor(0, 2);
	lcd.print(F(__TIME__));
	
	delay(1500);
	lcd.clear();

	//fan to full power
	pinMode(fan_pin, OUTPUT);
	analogWrite(fan_pin, 255);
	
	for (int count = 0; count < 3; count++)
		pinMode(buttonPin[count], INPUT_PULLUP);

	for (int count = 0; count < LAST_TEMPS; count++)
		last_probe_t[count] = getTemp();

	myPID.SetMode(MANUAL);
	myPID.SetSampleTime(1000);
	myPID.SetOutputLimits(0, 100);
}

float target = 0.0f;
float eps = 0.2f;
int fan_power = 100;
int fan_max = 100;
int fan_min = 0;
int to_read = 0;

long last_entry = 0;
void loop()
{
	bool change = false;
	
	change |= button_check(0);
	change |= button_check(1);
	change |= button_check(2);
	
	if (change)
	{
		bool re_run = false;
		bool have_printed_poll_option = false;
		
		do
		{			
			//if re-entering, let the buttons go to unpressed then re-poll the buttons
			if (re_run)
			{
				while (buttonState[0] || buttonState[1] || buttonState[2])
				{
					button_check(0);
					button_check(1);
					button_check(2);
				}	
			
				button_check(0);
				button_check(1);
				button_check(2);
				
				if (level == kInOption && (current_pos == 3 || current_pos == 6) && have_printed_poll_option)
					if (!(buttonState[0] || buttonState[1] || buttonState[2]))
						continue;
			}
			
			int cursor = 0;
			
			switch (level)
			{
				case kInfo:
					re_run = false;
				
	//				Serial.println("kInfo");
					if (buttonState[2] == HIGH)
					{
						current_pos = 0;
						level = kOptionsPresent;
						re_run = true;
					}
				
					break;
		
				case kOptionsPresent:
				{
					re_run = false;
				
					lcd.clear();
	//				Serial.println("kOptionsPresent");
					char longest_string[sizeof(string8) + 1];
					Serial.println(current_pos);
					strcpy_P(longest_string, (char *)pgm_read_word(&(string_table[current_pos])));

					lcd.print(longest_string); lcd.setCursor(0, ++cursor);
					lcd.print(F("Select?"));
				
					level = kInOptionsList;
				
					break;
				}
			
				case kInOptionsList:
				{
					re_run = false;
				
	//				Serial.println("kInOptionsList");
					int orig_p = current_pos;
					int orig_l = level;
				
					if (buttonState[0] == HIGH)
						current_pos--;
					else if (buttonState[1] == HIGH)
						current_pos++;
					else if (buttonState[2] == HIGH)
						level = kInOption;

					if (current_pos < 0)
						current_pos = sizeof(string_table) / sizeof(const char *) - 1;
					else if (current_pos == sizeof(string_table) / sizeof(const char *))
						current_pos = 0;
					
					if (orig_p != current_pos)
						level = kOptionsPresent;
					
					if (orig_l != level)
						re_run = true;
					
					break;
				}
		
				case kInOption:
					re_run = false;
					
					lcd.clear();
				
	//				Serial.println("kInOption");
					switch (current_pos)
					{
						case 0:
							fan_power = 100;
							myPID.SetMode(MANUAL);
							level = kToReturn;
							break;
						case 1:
							fan_power = 90;
							myPID.SetMode(MANUAL);
							level = kToReturn;
							break;
						case 2:
							fan_power = 0;
							myPID.SetMode(MANUAL);
							level = kToReturn;
							break;
						case 3:
							if (buttonState[1] == HIGH)
								fan_power += 5;
							else if (buttonState[0] == HIGH)
								fan_power -= 5;
							else if (buttonState[2] == HIGH)
								level = kOptionsPresent;
							
							if (fan_power > 100)
								fan_power = 100;
							else if (fan_power < 0)
								fan_power = 0;
								
							have_printed_poll_option = true;
							
							lcd.print("fan speed: ");
							lcd.print(fan_power); lcd.setCursor(0, ++cursor);
							
							myPID.SetMode(MANUAL);
						
							re_run = true;
							break;
						case 4:
							target = getTemp();
							myPID.SetMode(AUTOMATIC);
							level = kToReturn;
							break;
						case 5:
						{
							float t = dht.readTemperature();
							float h = dht.readHumidity();
							target = get_dew_point(t, h);

							Serial.print(F("setting to dew point "));
							Serial.print(target); Serial.print(F(" ")); Serial.print(t); Serial.print(F(" ")); Serial.println(h);

							myPID.SetMode(AUTOMATIC);
							level = kToReturn;
							break;
						}
						case 6:
							if (buttonState[1] == HIGH)
								target += 0.5f;
							else if (buttonState[0] == HIGH)
								target -= 0.5f;
							else if (buttonState[2] == HIGH)
								level = kOptionsPresent;
							
							if (target > 50)
								target = 50;
							else if (target < -50)
								target = -50;
								
							have_printed_poll_option = true;
							
							lcd.print("target temp: ");
							lcd.print(target); lcd.setCursor(0, ++cursor);
							
							myPID.SetMode(AUTOMATIC);
						
							re_run = true;
							break;
						case 7:
							((unsigned char *)&target)[0] = EEPROM.read(0);
							((unsigned char *)&target)[1] = EEPROM.read(1);
							((unsigned char *)&target)[2] = EEPROM.read(2);
							((unsigned char *)&target)[3] = EEPROM.read(3);
						
							lcd.print("loaded value "); lcd.print(target); lcd.setCursor(0, ++cursor);
							level = kToReturn;
							
							myPID.SetMode(AUTOMATIC);
							break;
						case 8:
							EEPROM.write(0, ((unsigned char *)&target)[0]);
							EEPROM.write(1, ((unsigned char *)&target)[1]);
							EEPROM.write(2, ((unsigned char *)&target)[2]);
							EEPROM.write(3, ((unsigned char *)&target)[3]);
							lcd.print("stored value "); lcd.print(target); lcd.setCursor(0, ++cursor);

							level = kToReturn;
							break;
						case 9:
							level = kInfo;
							break;
					};
				
					if (!(current_pos == 9 || current_pos == 3 || current_pos == 6))
					{
						lcd.print("Return to menu?"); lcd.setCursor(0, ++cursor);
						re_run = true;
					}
					break;
				
				case kToReturn:
					re_run = false;
				
	//				Serial.println("kToReturn");
					
					if (buttonState[0] == HIGH
						|| buttonState[1] == HIGH
						|| buttonState[2] == HIGH)
					{
						level = kOptionsPresent;
						re_run = true;
						
						if (buttonState[2] == HIGH)
							current_pos = 9;
					}
					break;
			}
		}
		while (re_run);
	}
	
	if (millis() - last_entry > 1000)
	{
		last_entry = millis();
		
		lcd.home();

		float h = dht.readHumidity();
		float t = dht.readTemperature();
		float probe_t = getTemp();

		for (int count = 0; count < LAST_TEMPS - 1; count++)
			last_probe_t[count] = last_probe_t[count + 1];

		last_probe_t[LAST_TEMPS - 1] = probe_t;

		Input = probe_t;
		Setpoint = target;

		myPID.Compute();
		Serial.print(Output); Serial.print(F(" "));

		if (myPID.GetMode() == AUTOMATIC)
			fan_power = Output;
		analogWrite(fan_pin, map(fan_power, 0, 100, 15, 255));

		float temp_vel = (last_probe_t[LAST_TEMPS - 1] - last_probe_t[0]) / LAST_TEMPS;
		float distance = target - probe_t;
		float est = distance / temp_vel;

		double dp = get_dew_point(t, h);

		if (level == kInfo)
		{
			lcd.print(F("temperature "));

			lcd.setCursor(13, 0);
			lcd.print(F("       "));
			lcd.setCursor(13, 0);
			lcd.print((int)t);
			lcd.print(F("/"));
			lcd.print(probe_t);
			lcd.setCursor(0, 1);

			lcd.print(F("humidity "));
			lcd.setCursor(13, 1);
			lcd.print(F("   "));
			lcd.setCursor(13, 1);
			lcd.print((int)h);
			lcd.setCursor(0, 2);

			lcd.print(F("dew point "));
			lcd.setCursor(13, 2);
			lcd.print(F("      "));
			lcd.setCursor(13, 2);
			lcd.print(dp);
			lcd.setCursor(0, 3);
		
			if (myPID.GetMode() == AUTOMATIC)
			{
				lcd.print(F("target"));
				lcd.setCursor(6, 3);
				lcd.print(F("              "));
				lcd.setCursor(8, 3);
				lcd.print(target);
				lcd.print(F("/"));
				lcd.print(fan_power);
				lcd.print(F("%"));
			}
			else
			{
				lcd.print(F("fan power"));
				lcd.setCursor(9, 3);
				lcd.print(F("           "));
				lcd.setCursor(13, 3);
				lcd.print(fan_power);
				lcd.print(F("%"));
			}
		}

		Serial.print(F("temperature ")); Serial.print((int)t); Serial.print(" "); Serial.print(probe_t);
		Serial.print(F(" humidity ")); Serial.print((int)h);
		Serial.print(F(" dew point ")); Serial.print(dp);
		Serial.print(F(" fan power ")); Serial.print(fan_power);
		Serial.print(F(" target ")); Serial.print(target);
		Serial.print(F(" vel ")); Serial.print(temp_vel); Serial.print(F(" deg/sec"));
		Serial.print(F(" distance ")); Serial.print(distance);
		Serial.print(F(" est ")); Serial.print(est); Serial.print(F(" ")); 
		Serial.print(Kp); Serial.print(F(" ")); Serial.print(Ki); Serial.print(F(" ")); Serial.print(Kd);
		Serial.print(F(" mode ")); Serial.print(myPID.GetMode() == AUTOMATIC ? F("auto") : F("manual"));
		Serial.println(F(""));

		if (Serial.available() > 0)
		{
			float f = Serial.parseFloat();
			switch (to_read)
			{
			case 0:
				Kp = f; break;
			case 1:
				Ki = f; break;
			case 2:
				Kd = f; break;
			case 3:
				target = f; break;
			}
		
			to_read++;
			if (to_read == 4)
				to_read = 0;

			myPID.SetTunings(Kp, Ki, Kd);
		}
	}
}

