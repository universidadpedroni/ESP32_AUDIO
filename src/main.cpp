#include <Arduino.h>
#include "driver/i2s.h"
#include <FFT.h>
#include <blink.h>
#include <notesAndFreq.h>

// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/I2S/HiFreq_ADC/HiFreq_ADC.ino
// https://github.com/atomic14/esp32_audio/blob/master/i2s_sampling/src/main.cpp
// https://pastebin.com/ChnLivTK

blink ledBlink(LED_BUILTIN);

#define I2S_SAMPLE_RATE 11000
//#define ADC_INPUT ADC1_CHANNEL_0 // pin 32
#define ADC_INPUT ADC1_CHANNEL_5    // Para Doit v1.0
#define NUM_SAMPLES 1024
#define ARRAYSIZE(a)    (sizeof(a)/sizeof(a[0]))

uint16_t samples[NUM_SAMPLES];
uint16_t offset = (int)ADC_INPUT * 0x1000 + 0xFFF;

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];

const float adqTime = (float) NUM_SAMPLES / (float) I2S_SAMPLE_RATE / 2;
const int FFT_N = NUM_SAMPLES; // Must be a power of 2
float fft_input[FFT_N];
float fft_output[FFT_N];

float max_magnitude = 0;
float fundamental_freq = 0;
char note_name[4];
char print_buf[300];


bool findNote()
{
  
  for (int octava = 0; octava < CANTIDAD_DE_OCTAVAS; octava++)
  {
    for (int nota = 0; nota < 11; nota++)
    {
        
      if (fundamental_freq >= pow(2,octava) * (1 - DETUNE) * FREQ_OF_NOTES[nota] && fundamental_freq <= pow(2,octava) * (1 + DETUNE) * FREQ_OF_NOTES[nota])
      {
        // Nota encontrada
        //Serial.printf("Nota %s%i, ",NOTE_NAMES[nota],octava);
        sprintf(note_name, "%s%i", NOTE_NAMES[nota], octava);
        return true;
      }
      
    }

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
    sprintf(print_buf,"Nota: %s, Fundamental Freq: %.2f Hz, Mag: %.2f V \n ",
                    note_name, fundamental_freq,(max_magnitude)*4/FFT_N/1000);
        Serial.print(print_buf);
    
	}
}

void i2sInit()
{
  esp_err_t err;
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = NUM_SAMPLES,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0

  };
  err = adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_0); //step 1
  if (err != ESP_OK) {
    Serial.printf("Failed setting up adc channel: %d\n", err);
    while (true);
  }


  err = i2s_driver_install(I2S_NUM_0, &i2s_config,  0, NULL);  //step 2
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }

  err = i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
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

  fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
    
  for (int k = 0; k < FFT_N; k++)
  {
    real_fft_plan->input[k] = (float)samples[k] - (float)2048.0;
    //Serial.println(real_fft_plan->input[k]);
  }
    
  
  fft_execute(real_fft_plan);

  for (int k = 1 ; k < real_fft_plan->size / 2 ; k++)
  {
    /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
    float mag = sqrt(pow(real_fft_plan->output[2*k],2) + pow(real_fft_plan->output[2*k+1],2))/1;
    float freq = k * 1.0 / adqTime;
    if(mag > max_magnitude)
    {
        max_magnitude = mag;
        fundamental_freq = freq;
    }
  }
  
  fft_destroy(real_fft_plan);
  


}




void setup()
{
  delay(2000);
  Serial.begin(115200);
  //pinMode(GPIO_NUM_32,OUTPUT);
  ledBlink.init();
  i2sInit();
  Serial.println("Listo");
}

void loop()
{
  //digitalWrite(GPIO_NUM_32,HIGH);
  size_t bytesRead = 0;
  i2s_read(I2S_NUM_0, (void*)samples, sizeof(samples), &bytesRead, portMAX_DELAY); // no timeout
  CalculateFFT();
  findNote();
  if(findNote()) SerialInfo(100);
  //digitalWrite(GPIO_NUM_32,LOW);

  
  ledBlink.update(1000);
  
  
    
}