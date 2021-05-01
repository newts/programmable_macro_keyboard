/*

  HelloWorld.ino

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <RotaryEncoder.h>
#include "HID-Project.h"

#define ENCODER_IN1 1
#define ENCODER_IN2 0
#define ENCODER_BUTTON A3

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

const int num_layers = 16;
String layer_names[num_layers] = {"DISABLED", "Standard", "Fusion 360", "COD Warzone", "KiCAD", "Zoom", "OnShape", "Excel", "FreeCAD", "MCUXpresso", "WoWarships", "WoTanks", "Atom", "VSCode", "STM32 CUBE", "Filmora"};
int layer_number = 0;

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(ENCODER_IN1, ENCODER_IN2, RotaryEncoder::LatchMode::FOUR3);

void checkPosition()
{
  encoder.tick(); // just call tick() to check the state.
}

const int pressed_state = LOW;
bool is_pressed = false;

const int keycode_history_size = 4;
char keycode_history[keycode_history_size] = {0x00, 0x00, 0x00, 0x00};

bool do_display_update = true;
void update_keycode_history(char keycode)
{
  do_display_update = true;
  for(int i = keycode_history_size-1; i > 0; i--)
  {
    keycode_history[i] = keycode_history[i-1];
  }
  keycode_history[0] = keycode;
}

void draw_display()
{
  if (do_display_update)
  {
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_helvB08_tf);
    u8g2.drawStr(0,10,"Layout:");  // write something to the internal memory
    u8g2.drawRFrame(76, 0, 52, 12, 3);
    u8g2.setFont(u8g2_font_courR08_tf); 
    char key_history_disp[32];
    sprintf(key_history_disp, "%02X%02X%02X%02X", (uint8_t)keycode_history[3], (uint8_t)keycode_history[2], (uint8_t)keycode_history[1], (uint8_t)keycode_history[0]);
    u8g2.drawStr(78,9,key_history_disp);
    u8g2.setFont(u8g2_font_helvB14_tf); // choose a suitable font
    u8g2.drawStr(0,32,layer_names[layer_number].c_str());  // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the display  
    do_display_update = false;
  }
}

long last_encoder_position = 0;
int read_and_interpret_encoder()
{
  int retval = 0;
  long encoder_position = encoder.getPosition();
  long encoder_delta = encoder_position - last_encoder_position;
  int encoder_button_state = digitalRead(ENCODER_BUTTON);

  if (encoder_button_state == false) //pressed
  {
    layer_number += encoder_delta;
    do_display_update = true;

    if (layer_number >= num_layers)
    {
      layer_number = num_layers-1;
    }
    if (layer_number < 0)
    {
      layer_number = 0;
    } 
    retval = 0;
  }
  else
  {
    retval = encoder_delta;
  }
  last_encoder_position = encoder_position;
  return retval;
}

void setup(void) {
  u8g2.begin();
  pinMode(ENCODER_IN1, INPUT_PULLUP);
  pinMode(ENCODER_IN2, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN2), checkPosition, CHANGE);
  Consumer.begin();
  Serial.begin(115200);
}

void loop(void) {
  int encoder_delta = read_and_interpret_encoder();

  //make the volume knob work
  if (layer_number > 0)
  {
    if (encoder_delta > 0)
    {
      for (int i = 0; i < encoder_delta; i++)
      {
        Consumer.write(MEDIA_VOLUME_UP);
        update_keycode_history(MEDIA_VOLUME_UP);
      }
    }
    else if (encoder_delta < 0)
    {
      for (int i = encoder_delta; i < 0; i++)
      {
        Consumer.write(MEDIA_VOLUME_DOWN);
        update_keycode_history(MEDIA_VOLUME_DOWN);
      }
    }
  }
  
  draw_display();
  delay(1);  
}
