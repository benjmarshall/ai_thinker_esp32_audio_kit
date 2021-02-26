/*      
  Program for testing the line in of the AI Thinker ESP32 Audio Kit.
  This program will print the maximum levels being seen on the ADC.
    
  This program is built on the examples found in the yummyDSP lib: 
  https://github.com/garygru/yummyDSP
  
  ------

  This example is ripped off the AC101 Arduino library for the AIThinker ESP32-A1S: 
  https://github.com/Yveaux/AC101
      
  AC101 Codec driver library example.
  Uses the ESP32-A1S module with integrated AC101 codec, mounted on the ESP32 Audio Kit board:
  https://wiki.ai-thinker.com/esp32-audio-kit
  
  Copyright (C) 2019, Ivo Pullens, Emmission
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <YummyDSP.h>
#include <AudioDriver.h>
#include <Codecs/AC101/AC101.h>

#define I2S_BCK_PIN                 27
#define I2S_LRCLK_PIN               26
#define I2S_DOUT_PIN                25
#define I2S_DIN_PIN                 35

#define GPIO_PA_EN                  GPIO_NUM_21
#define GPIO_SEL_PA_EN              GPIO_SEL_21

#define PIN_VOL_UP                  (18)      // KEY 5
#define PIN_VOL_DOWN                (5)       // KEY 6
#define PIN_INPUT_SELECT            (23)      // KEY 4
#define PIN_GAIN_UP					(13)      // KEY 2
#define PIN_GAIN_DOWN				(19)      // KEY 3

static AC101 i2sCodec;
YummyDSP dsp;
StatisticsNode stats;

const int fs = 96000;
const int channelCount = 2;

static uint8_t volume = 12;
const uint8_t volume_step = 2;
static uint8_t gain = 160;
const uint8_t gain_step = 2;

int input_select = 1;

int debounce = 0;

void audioTask(void *);


void setup()
{
  Serial.begin(115200);

  Serial.printf("Connect to AC101 codec... ");
  // setup audio codec
  i2sCodec.setup(fs, channelCount, I2S_BCK_PIN, I2S_LRCLK_PIN, I2S_DOUT_PIN, I2S_DIN_PIN, GPIO_PA_EN);

  i2sCodec.SetVolumeSpeaker(volume);
  i2sCodec.SetVolumeHeadphone(volume);
  i2sCodec.SetMode(i2sCodec.MODE_LINE);
  //i2sCodec.DumpRegisters();

  // Enable amplifier
  pinMode(GPIO_PA_EN, OUTPUT);
  digitalWrite(GPIO_PA_EN, HIGH);

  // Configure keys on ESP32 Audio Kit board
  pinMode(PIN_INPUT_SELECT, INPUT_PULLUP);
  pinMode(PIN_VOL_UP, INPUT_PULLUP);
  pinMode(PIN_VOL_DOWN, INPUT_PULLUP);
  pinMode(PIN_GAIN_DOWN, INPUT_PULLUP);
  pinMode(PIN_GAIN_UP, INPUT_PULLUP);

  Serial.printf("Use KEY5/KEY6 for volume Up/Down\n");
  Serial.printf("Use KEY4 for Input Select\n");
  Serial.printf("Line In selected by default\n");
  Serial.printf("Use KEY2/KEY3 for ADC Gain Up/Down\n");

  // setup audio lib
  dsp.begin(fs);
   
  stats.begin(fs, channelCount);

  // add nodes to audio processing tree
  dsp.addNode(&stats);


  // run audio in dedicated task on cpu core 1
  xTaskCreatePinnedToCore(audioTask, "audioTask", 10000, NULL, 10, NULL, 1);
  // run stats loop
  xTaskCreate(stats_loop, "statsloop", 10000, NULL, 10, NULL);
  // run control task on another cpu  core with lower priority
  Serial.print("\nSetup done\n");

}

bool pressed( const int pin )
{
  if (millis() > (debounce + 500))
  {
    if (digitalRead(pin) == LOW)
    {
      debounce = millis();
      return true;
    }
  }
  return false;
}


void audioTask(void *) {

  Serial.print("Audio task\n");

  float sample = 0;

  while (true) {
	  
	i2sCodec.readBlock();

    for (int i = 0; i < AudioDriver::BufferSize; i++) {

      for (int ch = 0; ch < channelCount; ch++) {
		  
		sample = i2sCodec.readSample(i, ch);

        sample = dsp.process(sample, ch);

        i2sCodec.writeSample(sample, i, ch);
      }
    }

    i2sCodec.writeBlock();
  }
  vTaskDelete(NULL);
}


void loop()
{
  bool updateVolume = false;
  bool updateGain = false;

  delay(2);

  if (pressed(PIN_VOL_UP))
  {
    if (volume <= (63 - volume_step))
    {
      // Increase volume
      volume += volume_step;
      updateVolume = true;
    }
  }
  if (pressed(PIN_VOL_DOWN))
  {
    if (volume >= volume_step)
    {
      // Decrease volume
      volume -= volume_step;
      updateVolume = true;
    }
  }
  if (updateVolume)
  {
    // Volume change requested
    Serial.printf("Volume %d\n", volume);
    i2sCodec.SetVolumeSpeaker(volume);
    i2sCodec.SetVolumeHeadphone(volume);
  }
  if (pressed(PIN_INPUT_SELECT))
  {
	if (input_select == 0)
	{
		Serial.printf("Line In Selected\n");
		i2sCodec.SetMode(i2sCodec.MODE_LINE);
		input_select = 1;
		delay(100);
		stats.reset();
	}
	else if (input_select == 1)
	{
		Serial.printf("Mic Selected\n");
		i2sCodec.SetMode(i2sCodec.MODE_MIC);
		input_select = 0;
		delay(100);
		stats.reset();
	}
  }
  if (pressed(PIN_GAIN_UP))
  {
	if (gain <= (255 - gain_step))
    {
      // Increase gain
      gain += gain_step;
      updateGain = true;
    }
  }
  if (pressed(PIN_GAIN_DOWN))
  {
	if (gain >= gain_step)
    {
      // Increase gain
      gain -= gain_step;
      updateGain = true;
    }
  }
  if (updateGain)
  {
    // Gain change requested
    Serial.printf("ADC Gain %d\n", gain);
    i2sCodec.SetADCGain(gain);
  }
	
}

void stats_loop(void *) {
	
	Serial.printf("Stats Task\n");
	
	while (true) {
		
		stats.reset();
		
		delay(2000);
	
		Serial.printf("Max Values Seen L:%f, R:%f\n",stats.getMaxValue(0), stats.getMaxValue(1));
		//Serial.printf("Min Values Seen L:%f, R:%f\n",stats.getMinValue(0), stats.getMinValue(1));
	}
	
	vTaskDelete(NULL);

}

