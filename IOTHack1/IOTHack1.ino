#include "Wire.h"
#include <Suli.h>
#include <Wire.h>
#include "Seeed_LED_Bar_Arduino.h"

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"

//Wifi Init Code
#include <LTask.h>
#include <LWiFi.h>
#include <LWiFiClient.h>

#define WIFI_AP "BASH"
#define WIFI_PASSWORD "BASH@infocomm"
#define WIFI_AUTH LWIFI_WPA  // choose from LWIFI_OPEN, LWIFI_WPA, or LWIFI_WEP.
#define SITE_URL "iot-blackhat.herokuapp.com"

#define SAMPLETIME 1 //sampling time in millis

//LEDBAR Code
#define BAR_CLK  9
#define BAR_DATA 8

#define ACCELRANGEMAX 1.29

SeeedLedBar bar(BAR_CLK, BAR_DATA);                  // CLK, DTA

int level = 0;

//Temperature and Humidity Sensor Setup
#include "DHT.h"

#define DHTPIN 6     // what pin we're connected to

#define DHTTYPE DHT22   // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE);

//Wifi Request Variables 
char action[] = "POST ";  // Edit to build your command - "GET ", "POST ", "HEAD ", "OPTIONS " - note trailing space
char server[] = "things.ubidots.com";
//char path[] = "/api/v1.6/variables/55391dfd762542557e0e8775/values";  // Edit Path to include you source key
char path[] = "/api/v1.6/collections/values";  // Edit Path to include you source key
char token[] = "xVBQKsxi1Zdhr5Pxx805zwxeoFYH5fES9wgBwz3eThBR1zqStfDZPv7DQe4p";  // Edit to insert you API Token
int port = 80; // HTTP

LWiFiClient c;

float accel = 0.123;

// Here are the program variables
int num;                            // part of the length calculation
String le;                         // length of the payload in characters
String var;                        // This is the payload or JSON request



// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

#define sample_num_mdate  5000      

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;

volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;

float temp = 0.0;
float humid = 0.0;

volatile float avg_accel=0;
float max_accel=0;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  LTask.begin();
  LWiFi.begin();
  Serial.begin(115200);             // setup Serial port
  Wire.begin();
  
  // keep retrying until connected to AP
  Serial.println("Connecting to AP");
  while (0 == LWiFi.connect(WIFI_AP, LWiFiLoginInfo(WIFI_AUTH, WIFI_PASSWORD)))
  {
    delay(1000);
  }
  Serial.println("Connected to AP");
  
  dht.begin();
  bar.begin(BAR_CLK,BAR_DATA);
  bar.setLevel(5);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
	
	delay(1000);
	Serial.println("     ");  
}

int ledbar = 5;

void loop() 
{  
	getAccel_Data();
	getGyro_Data();
	getCompassDate_calibrated(); // compass data has been calibrated here 
	getHeading();				//before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .					
	getTiltHeading();  

        //Read humidity and Temperature
        dht.readHT(&temp, &humid);
//        
//        //Read dust sensor
//        dust_val = pulseIn(DUSTPIN,LOW);
//        dust_time = millis();
	
        avg_accel = abs((Axyz[0]+Axyz[1]+Axyz[2])/3);
        
        if(avg_accel>max_accel)
          max_accel=avg_accel;

	Serial.println("calibration parameter: ");
	Serial.print(mx_centre);
	Serial.print("         ");
	Serial.print(my_centre);
	Serial.print("         ");
	Serial.println(mz_centre);
	Serial.println("     ");
	
	
	Serial.println("Acceleration(g) of X,Y,Z:");
	Serial.print(Axyz[0]); 
	Serial.print(",");
	Serial.print(Axyz[1]); 
	Serial.print(",");
	Serial.println(Axyz[2]); 
	Serial.println("Gyro(degress/s) of X,Y,Z:");
	Serial.print(Gxyz[0]); 
	Serial.print(",");
	Serial.print(Gxyz[1]); 
	Serial.print(",");
	Serial.println(Gxyz[2]); 
	Serial.println("Compass Value of X,Y,Z:");
	Serial.print(Mxyz[0]); 
	Serial.print(",");
	Serial.print(Mxyz[1]); 
	Serial.print(",");
	Serial.println(Mxyz[2]);
	Serial.println("The clockwise angle between the magnetic north and X-Axis:");
	Serial.print(heading);
	Serial.println(" ");
	Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
	Serial.println(tiltheading);
        Serial.println("Average Accel:");
        Serial.println(avg_accel);
	Serial.println("Max Accel: ");
        Serial.println(max_accel);
        Serial.println("   ");
	Serial.println("Temperature: ");
        Serial.println(temp);
        Serial.println("Humidity: ");
        Serial.println(humid);
        Serial.println("   ");
	Serial.println("   ");
        Serial.println("   ");
        
        //find the led bar value
        ledbar = ((avg_accel/ACCELRANGEMAX)*10);
        if(ledbar>10)
          ledbar=10;
        bar.setLevel(ledbar);
        
        //Send the POST Request
        //Build the JSON
        //var = "{\"value\":"+String(avg_accel)+"}";
        //+",\"timestamp\":"+String(millis())+
        var = "{\"variable\":" "55391dfd762542557e0e8775" ",\"value\":"+String(avg_accel)+"}";
        //var = "{\"value\":"+String(avg_accel)+",\"Acceleration\":12.345,\"user\":\"toonistic@gmail.com\"}";
        
        num=var.length();               // How long is the payload
        le=String(num);                 //this is to calcule the length of var
        Serial.print("Connect to ");    // For the console - show you are connecting
        Serial.println(server);
        Serial.println("Connecting to WebSite");
        while (0 == c.connect(server, 80))
        {
          Serial.println("Re-Connecting to WebSite");
          delay(500);
        }
  
        Serial.println("Connected");  // Console monitoring

        c.print(action);                   // These commands build a JSON request for Ubidots but fairly standard
        c.print(path);                     // specs for this command here: http://ubidots.com/docs/api/index.html
        c.println(" HTTP/1.1");
        c.println(F("Content-Type: application/json"));
        c.print(F("Content-Length: "));
        c.println(le);
        c.print(F("X-Auth-Token: "));
        c.println(token);
        c.print(F("Host: "));
        c.println(server);
        c.println();
        c.println(var);  // The payload defined above
        c.println();
        c.println((char)26); //This terminates the JSON SEND with a carriage return
      	
        Serial.println("POST Posted: var = "+var);
        
        delay(SAMPLETIME);	
        
}


void getHeading(void)
{
  heading=180*atan2(Mxyz[1],Mxyz[0])/PI;
  if(heading <0) heading +=360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1]/cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh)/PI;
  if(yh<0)    tiltheading +=360;
}


void Mxyz_init_calibrated ()
{
	
	Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
	Serial.print("  ");
	Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
	Serial.print("  ");
	Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
	while(!Serial.find("ready"));	
	Serial.println("  ");
	Serial.println("ready");
	Serial.println("Sample starting......");
	Serial.println("waiting ......");
	
	get_calibration_Data ();
	
	Serial.println("     ");
	Serial.println("compass calibration parameter ");
	Serial.print(mx_centre);
	Serial.print("     ");
	Serial.print(my_centre);
	Serial.print("     ");
	Serial.println(mz_centre);
	Serial.println("    ");
}


void get_calibration_Data ()
{
		for (int i=0; i<sample_num_mdate;i++)
			{
			get_one_sample_date_mxyz();
			/*
			Serial.print(mx_sample[2]);
			Serial.print(" ");
			Serial.print(my_sample[2]);                            //you can see the sample data here .
			Serial.print(" ");
			Serial.println(mz_sample[2]);
			*/


			
			if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];			
			if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value			
			if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];		
			
			if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
			if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
			if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];
						
			}
			
			mx_max = mx_sample[1];
			my_max = my_sample[1];
			mz_max = mz_sample[1];			
					
			mx_min = mx_sample[0];
			my_min = my_sample[0];
			mz_min = mz_sample[0];
	

	
			mx_centre = (mx_max + mx_min)/2;
			my_centre = (my_max + my_min)/2;
			mz_centre = (mz_max + mz_min)/2;	
	
}






void get_one_sample_date_mxyz()
{		
		getCompass_Data();
		mx_sample[2] = Mxyz[0];
		my_sample[2] = Mxyz[1];
		mz_sample[2] = Mxyz[2];
}	


void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;//16384  LSB/g
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;//131 LSB(��/s)
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
	I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
	delay(10);
	I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
	
    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
	my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
	mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;	
	
	//Mxyz[0] = (double) mx * 1200 / 4096;
	//Mxyz[1] = (double) my * 1200 / 4096;
	//Mxyz[2] = (double) mz * 1200 / 4096;
	Mxyz[0] = (double) mx * 4800 / 8192;
	Mxyz[1] = (double) my * 4800 / 8192;
	Mxyz[2] = (double) mz * 4800 / 8192;
}

void getCompassDate_calibrated ()
{
	getCompass_Data();
	Mxyz[0] = Mxyz[0] - mx_centre;
	Mxyz[1] = Mxyz[1] - my_centre;
	Mxyz[2] = Mxyz[2] - mz_centre;	
}
