#include <SPI.h>
#include "./src/ADF4351.hpp"

#define DATA_PIN 1 //not used on arduino
#define CLK_PIN 1 //NOT USED on arduino
#define SS_PIN 10


int main(){
  //connect to our serial port
  Serial.begin(115200);
  // connect to our frequency synthesizer
  ADF4351 myadf = ADF4351(DATA_PIN,CLK_PIN);
  // initialize the synthesizer
  Serial.println(myadf.init());
  delay(500);
  Serial.println("test");
  myadf.print_info();
  delay(1000);
  // set the power and write all of our registers
  myadf.set_output_power(RF_POW_m4);
  myadf.write_all();
  //set the initial frequency
  //myadf.set_freq(1000e6);
  myadf.set_freq(955e6);
  myadf.print_info();
  //I dislike the arduino scoping so I just put all of this in main
  while(1){
    // change the frequency when entered by the user
    uint32_t freq_hz_in;//,freq_khz_in;
    delay(500);
    Serial.println("Please enter frequency to set in Hz");
    while (Serial.available() == 0);
    freq_hz_in = (uint32_t)Serial.parseInt();
    Serial.println(freq_hz_in);
    myadf.set_freq(freq_hz_in);
  }
}

