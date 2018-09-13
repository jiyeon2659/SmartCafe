#include <dht.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(4,5);
dht DHT;//make an instance for reading temp and humidity

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
int seat5_pin = A12;
int seat6_pin = A13;

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
       seat5_pressure,
       seat6_pressure;
//----------------------------------


void setup(){
  Serial.begin(9600);
  mySerial.begin(9600);
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
   seat5_pressure = check_seat_pressure(seat5_pin);
   seat6_pressure = check_seat_pressure(seat6_pin);

   //printing out the readed values
   if(steps == 2){

      Serial.println("**************************************");
      Serial.println("Temperature = " + String(temperature) + "%");
      mySerial.print("<");
      mySerial.print(temperature);
      mySerial.print(",");
      
      Serial.println("Humidity = " + String(humidity) + "%");
      mySerial.print(humidity);
      mySerial.print(",");
      
      Serial.println("Dust Density (ug/m^3): " + String(dust_density));
      mySerial.print(dust_density);
      //mySerial.print(",");
      mySerial.print(">");

      Serial.println("Luminance = " + String(luminance));
      
      Serial.println("Seat 1's pressure = " + String(seat1_pressure));
      Serial.println("Seat 2's pressure = " + String(seat2_pressure));
      Serial.println("Seat 3's pressure = " + String(seat3_pressure));
      Serial.println("Seat 4's pressure = " + String(seat4_pressure));
      Serial.println("Seat 5's pressure = " + String(seat5_pressure));
      Serial.println("Seat 6's pressure = " + String(seat6_pressure));
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

  if ( dustDensity < 0)
  {
    dustDensity = 0.00;
  }
  
  return dustDensity;
}


double check_luminance() {
  double read_luminance = analogRead(A1);
  return read_luminance;
}

double check_seat_pressure(int seat_pin) {
  double read_pressure = analogRead(seat_pin);
  double mfsr_r18 = map(read_pressure, 0, 1024, 0, 255);
  return read_pressure;
}

