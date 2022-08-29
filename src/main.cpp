#include <Arduino.h>
#include "driver/i2s.h"
#include <FFT.h>
#include <blink.h>
#include <notesAndFreq.h>
#include <i2sConfig.h>
#include <midiConfig.h>

// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/I2S/HiFreq_ADC/HiFreq_ADC.ino
// https://github.com/atomic14/esp32_audio/blob/master/i2s_sampling/src/main.cpp
// https://pastebin.com/ChnLivTK

// TODO: Probar asociar la nota al bean de la FFT directamente, para evitar hacer las divisiones.



blink ledBlink(LED_BUILTIN);


#define NUM_SAMPLES 8192
#define ARRAYSIZE(a)    (sizeof(a)/sizeof(a[0]))

uint16_t samples[NUM_SAMPLES];
//uint16_t offset = (int)ADC_INPUT * 0x1000 + 0xFFF;

//double vReal[NUM_SAMPLES];
//double vImag[NUM_SAMPLES];


const float ADQ_FREQ[] = {2.65185056891564,   // K < = 113
                          2.66782239071767,   // 113 < K < 303
                          2.66392499544865};  // K > 303
const int FFT_N = NUM_SAMPLES; // Must be a power of 2
float fft_input[FFT_N];
float fft_output[FFT_N];

float max_magnitude = 0;
float fundamental_freq = 0;
char note_name[4];
char print_buf[300];
size_t bytesRead = 0;

int notaMidi = 0;
const float MAG_MINIMA = 0.1;     // Mínimo valor que se considerará como que hay una nota
const float VADC_MEAN = 1777.00; // Valor medio del ADC, con la entrada a masa.
const float MAG_GAIN = 4.0 / (float)FFT_N / 1000;

bool findNote()
{
  if(max_magnitude > MAG_MINIMA)
  {
    notaMidi = 69 + (int)(12.0 * log2(fundamental_freq / 440));
    sprintf(note_name, "%s", NOTE_NAMES_ALL[notaMidi - MIDI_INDEX]);
    return true;
  }
  return false;
  
}

// Función Para sacar datos por el Serie
void SerialInfo(unsigned long interval)
{
	static unsigned long previousMillis = 0;        // will store last time LED was updated
	//const long interval = 1000;           // interval at which to blink (milliseconds)
	unsigned long currentMillis = millis();
	
	
	if(currentMillis - previousMillis > interval) 
	{
		previousMillis = currentMillis;
    
    sprintf(print_buf,"Nota: %s, Fundamental Freq: %.2f Hz, Mag: %.2f V, NotaMidi: %u \n ",
                    note_name, fundamental_freq,max_magnitude, notaMidi);
        Serial.print(print_buf);
    
	}
}

void i2sInit()
{
  esp_err_t err;
  
  err = adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_4); //step 1
  
  if (err != ESP_OK) {
    Serial.printf("Failed setting up adc channel: %d\n", err);
    while (true);
  }


  err = i2s_driver_install(I2S_NUM_0, &i2s_config,  0, NULL);  //step 2
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }

  err = i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_4);
    if (err != ESP_OK) {
    Serial.printf("Failed setting up adc mode: %d\n", err);
    while (true);
  }
  i2s_adc_enable(I2S_NUM_0);

}

void CalculateFFT()
{
  max_magnitude = 0;
  fundamental_freq = 0;

//  size_t bytesRead = 0;
  i2s_read(I2S_NUM_0, (void*)samples, sizeof(samples), &bytesRead, portMAX_DELAY); // no timeout

  fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
    
  for (int k = 0; k < FFT_N; k++)
  {
    real_fft_plan->input[k] = ((float)samples[k] - VADC_MEAN);
    //Serial.println(real_fft_plan->input[k]);
  }
 
  fft_execute(real_fft_plan);

  for (int k = 1 ; k < real_fft_plan->size / 2 ; k++)
  {
    /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
    float mag = sqrt(pow(real_fft_plan->output[2*k],2) + pow(real_fft_plan->output[2*k+1],2))/1;
    //float freq = k * 1.0 / adqTime;
    

    if(mag > max_magnitude)
    {
        max_magnitude = mag;
        fundamental_freq = k;
    }
  }
  if (fundamental_freq < 113)
  {
    fundamental_freq *= ADQ_FREQ[0];
  }
  else if(fundamental_freq < 303)
  {
    fundamental_freq *= ADQ_FREQ[1];
  }
  else
  {
    fundamental_freq *= ADQ_FREQ[2];
  }
  max_magnitude *= MAG_GAIN;

  fft_destroy(real_fft_plan);
}

// plays a MIDI note or Control change..  Doesn't check to see that
// cmd is greater than 127, or that data values are  less than 127:
// Examples: 
// cmd: NOTE_ON, data1: pitch, data2: 0 -127
// cmd: NOTE_OFF, data1: pitch, data2: 0
// cmd: CONTROL_CHANGE, data1: SUSTAIN, data2: 0 - 127
void sendMIDI(int cmd, int data1, int data2,int midi_channel) 
{
  cmd = cmd | char(midi_channel);    // merge channel number
  
	SerialMidi.write(cmd);
	SerialMidi.write(data1);
	SerialMidi.write(data2);
}

void setup()
{
  delay(2000);
  Serial.begin(115200);
  ledBlink.init();
  i2sInit();
  SerialMidi.begin(BAUDRATE_MIDI);
  Serial.println("Listo");
  //CalculateFFT();
  //while(1){}
}

void loop()
{
  CalculateFFT();
  if(findNote()){
    SerialInfo(100);
    sendMIDI(MIDI_CH_NOTE_ON,notaMidi,127,MIDI_CHANNEL);
  } 
  
  ledBlink.update(1000);
     
}