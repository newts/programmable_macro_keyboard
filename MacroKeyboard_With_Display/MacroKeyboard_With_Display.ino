
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

#define LED_PIN 10
int led_brightness = 0;

#define ROW_0_PIN 5
#define ROW_1_PIN 6
#define ROW_2_PIN 7
#define ROW_3_PIN 8

#define COL_0_PIN 9
#define COL_1_PIN A0
#define COL_2_PIN 16
#define COL_3_PIN 14
#define COL_4_PIN 15

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(ENCODER_IN1, ENCODER_IN2, RotaryEncoder::LatchMode::FOUR3);

#define NUM_ROWS 4
#define NUM_COLS 5
const int switch_rows[NUM_ROWS] = {ROW_0_PIN, ROW_1_PIN, ROW_2_PIN, ROW_3_PIN};
const int switch_cols[NUM_COLS] = {COL_4_PIN, COL_3_PIN, COL_2_PIN, COL_1_PIN, COL_0_PIN};

#define NUM_SWITCHES NUM_ROWS*NUM_COLS
bool switch_states[NUM_SWITCHES] = {false};
bool previous_switch_states[NUM_SWITCHES] = {false};

#define NUM_LAYERS 4
const String layer_names[NUM_LAYERS] = {"Disabled", "Nav Cluster", "Numpad", "Lietuvių"};
int layer_number = 0;

#define KEYCODE_MASK 0xFF
#define IS_MACRO     0x1000
const uint16_t PROGMEM layer_mappings[NUM_LAYERS][NUM_SWITCHES] = 
{ 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

{0,  KEY_INSERT,     KEY_HOME,       KEY_PAGE_UP,      KEY_PRINT,
 0,  KEY_DELETE,     KEY_END,        KEY_PAGE_DOWN,        0, 
 0,           0,     KEY_UP_ARROW,                 0,      0, 
 0,  KEY_LEFT_ARROW, KEY_DOWN_ARROW, KEY_RIGHT_ARROW,  KEYPAD_ENTER},

{0,         KEY_NUM_LOCK,   KEYPAD_DIVIDE,    KEYPAD_MULTIPLY,   KEYPAD_SUBTRACT,
 0,         KEYPAD_7,       KEYPAD_8,         KEYPAD_9,          KEYPAD_ADD, 
 0,         KEYPAD_4,       KEYPAD_5,         KEYPAD_6,          KEYPAD_DOT, 
 KEYPAD_0,  KEYPAD_1,       KEYPAD_2,         KEYPAD_3,          KEYPAD_ENTER},


 // Ą                Ė                     Ę                    Į                     Ų         
{IS_MACRO|0,        IS_MACRO|1,          IS_MACRO|2,          IS_MACRO|3,           IS_MACRO|4, 
//  ą                ė                     ę                    į                     ų
 IS_MACRO|9,        IS_MACRO|10,         IS_MACRO|11,         IS_MACRO|12,          IS_MACRO|13,
//  Ū                Č                     Š                    Ž
 IS_MACRO|5,        IS_MACRO|6,          IS_MACRO|7,          IS_MACRO|8,           0,
//  ū                č                    š                     ž                   
 IS_MACRO|14,       IS_MACRO|15,         IS_MACRO|16,         IS_MACRO|17,          0  },
};

#define NUM_MACROS 18
#define MAX_MACRO_LENGTH 32
#define MACRO_PRESS 0x100
#define MACRO_RELEASE 0x200
const uint16_t PROGMEM macros[NUM_MACROS][MAX_MACRO_LENGTH+1] =
{
/* 0 Ą */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_0, KEY_4, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/* 1 Ė */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_1, KEY_6, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/* 2 Ę */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_1, KEY_8, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/* 3 Į */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_2, KEY_E, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/* 4 Ų */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_7, KEY_2, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/* 5 Ū */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_6, KEY_A, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/* 6 Č */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_0, KEY_C, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/* 7 Š */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_6, KEY_0, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/* 8 Ž */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_7, KEY_D, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},

/* 9 ą */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_0, KEY_5, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/*10 ė */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_1, KEY_7, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/*11 ę */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_1, KEY_9, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/*12 į */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_2, KEY_F, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/*13 ų */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_7, KEY_3, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/*14 ū */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_6, KEY_B, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/*15 č */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_0, KEY_D, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/*16 š */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_6, KEY_1, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
/*17 ž */{10, MACRO_PRESS|KEY_LEFT_CTRL,MACRO_PRESS|KEY_LEFT_SHIFT, KEY_U, MACRO_RELEASE|KEY_LEFT_SHIFT,MACRO_RELEASE|KEY_LEFT_CTRL, KEY_0, KEY_1, KEY_7, KEY_E, KEY_SPACE, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},

};

void checkPosition()
{
  encoder.tick(); // just call tick() to check the state.
}

const int pressed_state = LOW;
bool is_pressed = false;

const int keycode_history_size = 6;
char keycode_history[keycode_history_size] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

bool do_display_update = true;
int previous_led_state = 0;
bool num_lock_state = false;
bool scroll_lock_state = false;
bool caps_lock_state = false;
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
    u8g2.setFont(u8g2_font_helvB08_tr);

    //print lock lights
    if (num_lock_state)
    {
      u8g2.drawStr(0,10,"NL");  // write something to the internal memory
    }

    if (scroll_lock_state)
    {
      u8g2.drawStr(16,10,"SL");  // write something to the internal memory
    }

    if (caps_lock_state)
    {
      u8g2.drawStr(32,10,"CL");  // write something to the internal memory
    }
    
    u8g2.drawRFrame(52, 0, 76, 12, 3);
    u8g2.setFont(u8g2_font_courR08_tr); 
    char key_history_disp[32];
    sprintf(key_history_disp, "%02X%02X%02X%02X%02X%02X", (uint8_t)keycode_history[5], (uint8_t)keycode_history[4], (uint8_t)keycode_history[3], (uint8_t)keycode_history[2], (uint8_t)keycode_history[1], (uint8_t)keycode_history[0]);
    u8g2.drawStr(54,9,key_history_disp);
    u8g2.setFont(u8g2_font_helvB12_tr); // choose a suitable font
    u8g2.drawStr(0,28,layer_names[layer_number].c_str());  // write something to the internal memory
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
    layer_number -= encoder_delta;
    do_display_update = true;

    if (layer_number >= NUM_LAYERS)
    {
      layer_number = NUM_LAYERS-1;
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

void scan_switches(void)
{
  for (int r = 0; r < NUM_ROWS; r++)
  {
      digitalWrite(switch_rows[r], true);
      delayMicroseconds(1);
      for (int c = 0; c < NUM_COLS; c++)
      {
        previous_switch_states[(r * (NUM_ROWS+1)) + c] = switch_states[(r * (NUM_ROWS+1)) + c];
        switch_states[(r * (NUM_ROWS+1)) + c] = digitalRead(switch_cols[c]);
      }
      digitalWrite(switch_rows[r], false);
      delayMicroseconds(1);
  }
}

void setup(void) {
  u8g2.begin();
  pinMode(ENCODER_IN1, INPUT_PULLUP);
  pinMode(ENCODER_IN2, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  for (int i = 0; i < NUM_ROWS; i++)
  {
    pinMode(switch_rows[i], OUTPUT);
    digitalWrite(switch_rows[1], false);
  }

  for (int i = 0; i < NUM_COLS; i++)
  {
    pinMode(switch_cols, INPUT);
  }
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN2), checkPosition, CHANGE);
  analogWrite(LED_PIN, led_brightness);
  Consumer.begin();
  BootKeyboard.begin();
  NKROKeyboard.begin();
  Serial.begin(115200);
}

void loop(void) {
  int encoder_delta = read_and_interpret_encoder();

  scan_switches();

  //make the volume knob work
  if (encoder_delta > 0)
  {
    for (int i = 0; i < encoder_delta; i++)
    {
      if (layer_number > 0)
      {
        Consumer.write(MEDIA_VOLUME_DOWN);
        update_keycode_history(MEDIA_VOLUME_DOWN);
      }
      else
      {
        led_brightness--;
        if(led_brightness < 0)
        {
          led_brightness = 0;
        }
        analogWrite(LED_PIN, led_brightness);
      }
    }
  }
  else if (encoder_delta < 0)
  {
    for (int i = encoder_delta; i < 0; i++)
    {
      if (layer_number > 0)
      {
        Consumer.write(MEDIA_VOLUME_UP);
        update_keycode_history(MEDIA_VOLUME_UP);
      }
      else
      {
        led_brightness++;
        if(led_brightness > 255)
        {
          led_brightness = 255;
        }
        analogWrite(LED_PIN, led_brightness);
      }
    }
  }

  //output keycodes
  bool do_nkro_update = false;
  for (int i = 0; i < NUM_SWITCHES; i++)
  {
    if(switch_states[i] != previous_switch_states[i])
    {
      KeyboardKeycode keycode = (KEYCODE_MASK & pgm_read_byte_near(&layer_mappings[layer_number][i]));
      bool is_macro = (IS_MACRO & pgm_read_word_near(&layer_mappings[layer_number][i])) ? true : false;
      if(switch_states[i] == true)
      {
        if(!is_macro)
        {
          NKROKeyboard.add(keycode);
          update_keycode_history(keycode);
          do_nkro_update = true;
        }
        else
        {
          //send a macro
          if(keycode < NUM_MACROS)
          {
            uint16_t macro_length = pgm_read_word_near(&macros[keycode][0]);
            if(macro_length < MAX_MACRO_LENGTH)
            {
              for(int i = 0; i < macro_length; i++)
              {
                uint16_t macro_keycode = pgm_read_word_near(&macros[keycode][i+1]);
                if(macro_keycode & MACRO_PRESS)
                {
                  NKROKeyboard.add((KeyboardKeycode)(KEYCODE_MASK & macro_keycode));  
                  NKROKeyboard.send();
                  update_keycode_history((KeyboardKeycode)(KEYCODE_MASK & macro_keycode));
                }
                else if(macro_keycode & MACRO_RELEASE)
                {
                  NKROKeyboard.remove((KeyboardKeycode)(KEYCODE_MASK & macro_keycode)); 
                  NKROKeyboard.send(); 
                }
                else
                {
                  NKROKeyboard.write((KeyboardKeycode)(KEYCODE_MASK & macro_keycode));
                  update_keycode_history((KeyboardKeycode)(KEYCODE_MASK & macro_keycode));
                }
              } 
            }
          }
        }
      }
      else
      {
        if(!is_macro)
        {
          NKROKeyboard.remove(keycode);
          do_nkro_update = true;
        }
      }
    }
  }

  if(do_nkro_update)
  {
    NKROKeyboard.send();
  }

  int ledStates = BootKeyboard.getLeds();
  if (ledStates != previous_led_state)
  {
    previous_led_state = ledStates;
    do_display_update = true;
  }
  
  caps_lock_state = ledStates & LED_CAPS_LOCK;
  num_lock_state = ledStates & LED_NUM_LOCK;
  scroll_lock_state = ledStates & LED_SCROLL_LOCK;
  
  draw_display();
  delay(1);  
}
