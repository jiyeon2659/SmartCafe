#include <dht.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Ethernet.h>

dht DHT;//make an instance for reading temp and humidity

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress server(104, 196, 137, 191); // smart cafe's server IP address
char cafeServer[] = "104. 196. 137. 191";

IPAddress ip(192, 168, 0, 177);

EthernetClient client;

//Measure pins
//Temp and humitity reading pins
int temp_and_humidity_pin = 2;

//Fine dust reading pins
int fine_dust_led_power   = 3;
int fine_dust_measure_pin = A0;
//----------------------------------

//Luminance reading pin
int lumi_pin = A1;

//pressure reading pins
int seat1_pin = A8;
int seat2_pin = A9;
int seat3_pin = A10;
int seat4_pin = A11;
int seat9_pin = A12;
int seat10_pin = A13;

//sensors reading synchronizer
unsigned long time_previous[2],
              time_current[2];//0 is for temp and humidity reading, 1 is for fine dust readings
              
unsigned long fine_dust_reading_interval = 1000 /*1 second*/  * 1;
unsigned long temp_and_humidity_reading_interval = fine_dust_reading_interval + 500;

int steps = 0;
//----------------------------------

//values to be readed
double temperature,
       humidity,
       dust_density,
       luminance,
       seat1_pressure,
       seat2_pressure,
       seat3_pressure,
       seat4_pressure,
       seat9_pressure,
       seat10_pressure;
//----------------------------------


void setup(){
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // start the Ethernet connection :
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // trying to configure using IP address instaed of DHCP:
    Ethernet.begin(mac, ip);
  }

  /*
  // give the Ethernet shield a second to initialize
  delay(1000);
  Serial.println("connecting...");

  
  // if you get a connection, report back via serial:
  if (client.connect(server, 80)) {
    Serial.println("connected");
  }
  */
  
  time_previous[0] = millis(); 
  time_previous[1] = millis();
  //fine dust sensor
  pinMode(fine_dust_led_power,OUTPUT);
}


void loop()
{
  // Fine dust reading 
  time_current[0] = millis();
  if (time_current[0] - time_previous[0] >= fine_dust_reading_interval) {
        time_previous[0] = time_current[0];
        dust_density = fine_dust_sensor_measurments(fine_dust_measure_pin, fine_dust_led_power);
        //mySerial.println(dust_density);
        steps++;
   }//end Fine dust reading 

  //Temp and Humidity reading
  int chk = DHT.read11(temp_and_humidity_pin);
  time_current[1] = millis();
  temperature = check_temperature();
  delay(1500);
  humidity = check_humidity();   
  if (time_current[1] - time_previous[1] >= temp_and_humidity_reading_interval) {
        time_previous[1] = time_current[1];
        steps++;
   }//end Temp and Humidity reading

   delay(500);
   luminance = check_luminance();

   seat1_pressure = check_seat_pressure(seat1_pin);
   seat2_pressure = check_seat_pressure(seat2_pin);
   seat3_pressure = check_seat_pressure(seat3_pin);
   seat4_pressure = check_seat_pressure(seat4_pin);
   seat9_pressure = check_seat_pressure(seat9_pin);
   seat10_pressure = check_seat_pressure(seat10_pin);

   //printing out the readed values
   if(steps == 2){
      Serial.println("**************************************");
      Serial.println("Temperature = " + String(temperature) + "%");
      
      Serial.println("Humidity = " + String(humidity) + "%");
      
      Serial.println("Dust Density (ug/m^3): " + String(dust_density));

      Serial.println("Luminance = " + String(luminance) + "%");
      
      Serial.println("Seat 1's pressure = " + String(seat1_pressure));
      Serial.println("Seat 2's pressure = " + String(seat2_pressure));
      Serial.println("Seat 3's pressure = " + String(seat3_pressure));
      Serial.println("Seat 4's pressure = " + String(seat4_pressure));
      Serial.println("Seat 9's pressure = " + String(seat9_pressure));
      Serial.println("Seat 10's pressure = " + String(seat10_pressure));

        // create a URI for the request (luminance, humidity, temperature, fine dust)

        if (client.connect(server, 80)) {
          Serial.println("Connected to sensor insert page...");
            client.print("GET /main/sensorInsert.php?");
            client.print("light=");
            client.print(luminance);
            client.print("&");
            client.print("humid=");
            client.print(humidity);
            client.print("&");
            client.print("tempe=");
            client.print(temperature);
            client.print("&");
            client.print("finedust=");
            client.println(dust_density);
                             
            client.println(" HTTP/1.1");
            client.println("Host: 104.196.137.191");
            client.println("Connection: close");
            client.println();
            client.println();
            Serial.println("Transfer from sensor insert page is terminated.");
            client.stop();
        }
        else {
          Serial.println("Connection to sensor insert page is failed.");
        }

        

        if (client.connect(server, 80)) {
          Serial.println("Connected to seat insert page...");
            client.print("GET /main/seatInsert.php?");
            client.print("c1=");
            client.print(seat1_pressure);
            client.print("&");
            client.print("c2=");
            client.print(seat2_pressure);
            client.print("&");
            client.print("c3=");
            client.print(seat3_pressure);
            client.print("&");
            client.print("c4=");
            client.print(seat4_pressure);
            client.print("&");
            client.print("c9=");
            client.print(seat9_pressure);
            client.print("&");
            client.print("c10=");
            client.println(seat10_pressure);

            client.println(" HTTP/1.1");
            client.println("Host: 109.196.137.191");
            client.println("Connection: close");
            client.println();
            client.println();
            Serial.println("Transer from seat insert page is terminated.");
            client.stop();
        }
        else {
          Serial.println("Connection to seat insert page is failed.");
        }
        
      steps = 0;
   }
}//end void loop

//temperature check
double check_temperature(){
  double read_temperature = DHT.temperature;
  return read_temperature;
}

//humidity check 
double check_humidity(){
  //int chk = DHT.read11(check_pin);
  double read_humidity = DHT.humidity;
  return read_humidity;
}

//fine dust check
double fine_dust_sensor_measurments(int measurePin, int ledPower){
  unsigned int samplingTime = 280;
  unsigned int deltaTime = 40;
  unsigned int sleepTime = 9680;

  double voMeasured = 0;
  double calcVoltage = 0;
  double dustDensity = 0;
  
  digitalWrite(ledPower, LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured*(5.0/1024);
  dustDensity = 1000.0 * (0.17*calcVoltage-0.1);//dust ensity in ug/m^3

  if (dustDensity < 0)
  {
    dustDensity = 0.00;
  }
  
  return dustDensity;
}


double check_luminance() {
  double read_luminance = analogRead(A1);
  double luminance_percent = map(read_luminance, 0, 1024, 0, 100);
  return luminance_percent;
}

double check_seat_pressure(int seat_pin) {
  double pressure = 0;
  double read_pressure = analogRead(seat_pin);
  //double mfsr_r18 = map(read_pressure, 0, 1024, 0, 255);
  if (read_pressure > 15) {
    pressure = 1;
  }
  else {
    pressure = 0;
  }
  return pressure;
}

