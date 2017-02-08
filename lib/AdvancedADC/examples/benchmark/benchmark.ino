#include "AdvancedADC.h"
const uint16_t n_samples = 1000;
uint8_t buffer8[n_samples];
uint16_t buffer16[n_samples];
uint8_t channels[] = {0, 1};

void callback() { }

void setup() {
  unsigned long t_start, t_end;
  float sampling_rate = 50e3;
  
  Serial.begin(115200);
  Serial.println("A0=" + String(analogRead(0)));
  Serial.println("A1=" + String(analogRead(1)));

  AdvancedADC.enableDigitalInputs(false);

  for (uint8_t i = 2; i < 8; i++) {
    AdvancedADC.setPrescaler(i);
    Serial.print("\nprescaler=");
    Serial.println(1<<i);

    Serial.print("single channel, 8-bit, interrupt-triggered: ");
    AdvancedADC.setSamplingRate(sampling_rate);
    AdvancedADC.setBuffer(buffer8);
    AdvancedADC.setChannel(0);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer8[i]) + ", ");
    }
    Serial.println("...");

    Serial.print("single channel, 16-bit, interrupt-triggered: ");
    AdvancedADC.setSamplingRate(sampling_rate);
    AdvancedADC.setBuffer(buffer16);
    AdvancedADC.setChannel(0);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");

    Serial.print("single channel, 8-bit, free running mode: ");
    AdvancedADC.setAutoTrigger(true);
    AdvancedADC.setTriggerSource(ADC_ATS_FREE_RUNNING_MODE);
    AdvancedADC.setBuffer(buffer8);
    AdvancedADC.setChannel(0);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer8[i]) + ", ");
    }
    Serial.println("...");
  
    Serial.print("single channel, 8-bit, free running mode, callback: ");
    AdvancedADC.setAutoTrigger(true);
    AdvancedADC.setTriggerSource(ADC_ATS_FREE_RUNNING_MODE);
    AdvancedADC.setChannel(0);
    AdvancedADC.registerCallback(&callback8);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer8[i]) + ", ");
    }
    Serial.println("...");

    Serial.print("single channel, 16-bit, free running mode: ");
    AdvancedADC.setAutoTrigger(true);
    AdvancedADC.setTriggerSource(ADC_ATS_FREE_RUNNING_MODE);
    AdvancedADC.setBuffer(buffer16);
    AdvancedADC.setChannel(0);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");
    
    Serial.print("single channel, 16-bit, free running mode, callback: ");
    AdvancedADC.setAutoTrigger(true);
    AdvancedADC.setTriggerSource(ADC_ATS_FREE_RUNNING_MODE);
    AdvancedADC.setChannel(0);
    AdvancedADC.registerCallback(&callback16);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");
  
    Serial.print("single channel, 8-bit, normal mode: ");
    AdvancedADC.setAutoTrigger(false);
    AdvancedADC.setBuffer(buffer8);
    AdvancedADC.setChannel(0);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer8[i]) + ", ");
    }
    Serial.println("...");
  
    Serial.print("single channel, 8-bit, normal mode, callback: ");
    AdvancedADC.setAutoTrigger(false);
    AdvancedADC.setChannel(0);
    AdvancedADC.setLeftAlignResult(true);
    AdvancedADC.registerCallback(&callback8);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer8[i]) + ", ");
    }
    Serial.println("...");
  
    Serial.print("single channel, 16-bit, normal mode: ");
    AdvancedADC.setAutoTrigger(false);
    AdvancedADC.setBuffer(buffer16);
    AdvancedADC.setChannel(0);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");
  
    Serial.print("single channel, 16-bit, normal mode, callback: ");
    AdvancedADC.setAutoTrigger(false);
    AdvancedADC.setChannel(0);
    AdvancedADC.registerCallback(&callback16);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");

    Serial.print("single channel, 16-bit, analogRead: ");
    t_start = micros();
    for (uint16_t i = 0; i < n_samples; i++) {
      buffer16[i] = analogRead(0);
    }
    t_end = micros();
    Serial.print(1e3*n_samples/(float)(t_end-t_start));
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");
  
    Serial.print("dual channel, 8-bit, interrupt-triggered: ");
    AdvancedADC.setSamplingRate(sampling_rate);
    AdvancedADC.setBuffer(buffer8);
    AdvancedADC.setChannels(channels);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer8[i]) + ", ");
    }
    Serial.println("...");

    Serial.print("dual channel, 16-bit, interrupt-triggered: ");
    AdvancedADC.setSamplingRate(sampling_rate);
    AdvancedADC.setBuffer(buffer16);
    AdvancedADC.setChannels(channels);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");
  
    Serial.print("dual channel, 8-bit, normal mode: ");
    AdvancedADC.setAutoTrigger(false);
    AdvancedADC.setTriggerSource(ADC_ATS_FREE_RUNNING_MODE);
    AdvancedADC.setBuffer(buffer8);
    AdvancedADC.setChannels(channels);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer8[i]) + ", ");
    }
    Serial.println("...");

    Serial.print("dual channel, 8-bit, normal mode, callback: ");
    AdvancedADC.setAutoTrigger(false);
    AdvancedADC.setChannels(channels);
    AdvancedADC.setLeftAlignResult(true);
    AdvancedADC.registerCallback(&callback8);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer8[i]) + ", ");
    }
    Serial.println("...");
    
    Serial.print("dual channel, 16-bit, normal mode: ");
    AdvancedADC.setAutoTrigger(false);
    AdvancedADC.setBuffer(buffer16);
    AdvancedADC.setChannels(channels);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");
  
    Serial.print("dual channel, 16-bit, normal mode, callback: ");
    AdvancedADC.setAutoTrigger(false);
    AdvancedADC.setChannels(channels);
    AdvancedADC.registerCallback(&callback16);
    AdvancedADC.begin();
    while(!AdvancedADC.finished());
    Serial.print(1e-3*AdvancedADC.measuredSamplingRate());
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");
  
    Serial.print("dual channel, 16-bit, analogRead: ");
    t_start = micros();
    for (uint16_t i = 0; i < n_samples; i += 2) {
      buffer16[i] = analogRead(0);
      buffer16[i+1] = analogRead(1);
    }
    t_end = micros();
    Serial.print(1e3*n_samples/(float)(t_end-t_start));
    Serial.print(" kHz: ");
    for (uint8_t i = 0; i < 10; i++) {
      Serial.print(String(buffer16[i]) + ", ");
    }
    Serial.println("...");
  }
}

void callback16(uint8_t channel_index, uint16_t value) {
  buffer16[AdvancedADC.currentIndex()] = value;
}

void callback8(uint8_t channel_index, uint16_t value) {
  buffer8[AdvancedADC.currentIndex()] = value;
}

void loop() {
}
