#include <Joystick.h>
//#include <mylib.h>


#include <stddef.h>

namespace std
{
#if defined(__GNUC__)
// Copyright (C) 2008-2020 Free Software Foundation, Inc.
// Copyright (C) 2020 Daniel Rossinsky <danielrossinsky@gmail.com>
//u
// This file is part of GCC.
//
// GCC is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3, or (at your option)
// any later version.
//
// GCC is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// Under Section 7 of GPL version 3, you are granted additional
// permissions described in the GCC Runtime Library Exception, version
// 3.1, as published by the Free Software Foundation.

// You should have received a copy of the GNU General Public License and
// a copy of the GCC Runtime Library Exception along with this program;
// see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see
// <http://www.gnu.org/licenses/>.

    template<typename T>
    class initializer_list
    {
    public:
        using value_type = T;
        using reference = const T&;
        using const_reference = const T&;
        using size_type = size_t;
        using iterator = const T*;
        using const_iterator = const T*;

    private:
        iterator  m_array;
        size_type m_len;

        // The compiler can call a private constructor.
        constexpr initializer_list(const_iterator itr, size_type st)
            : m_array(itr), m_len(st) { }

    public:
        constexpr initializer_list() noexcept : m_array(0), m_len(0) { }

        // Number of elements.
        constexpr size_type size() const noexcept { return m_len; }

        // First element.
        constexpr const_iterator begin() const noexcept { return m_array; }

        // One past the last element.
        constexpr const_iterator end() const noexcept { return begin() + size(); }
    };
#elif defined(__clang__)
// Copyright (c) 2019 Chandler Carruth <https://github.com/chandlerc>
// Copyright (c) 2018 Louis Dionne <https://github.com/ldionne>
// Copyright (c) 2017 Eric <https://github.com/EricWF>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---- LLVM Exceptions to the Apache 2.0 License ----
//
// As an exception, if, as a result of your compiling your source code, portions
// of this Software are embedded into an Object form of such source code, you
// may redistribute such embedded portions in such Object form without complying
// with the conditions of Sections 4(a), 4(b) and 4(d) of the License.
//
// In addition, if you combine or link compiled forms of this Software with
// software that is licensed under the GPLv2 ("Combined Software") and if a
// court of competent jurisdiction determines that the patent provision (Section
// 3), the indemnity provision (Section 9) or other Section of the License
// conflicts with the conditions of the GPLv2, you may retroactively and
// prospectively choose to deem waived or otherwise exclude such Section(s) of
// the License, but only in their entirety and only with respect to the Combined
// Software.

    template<typename T>
    class initializer_list
    {
    private:
        const T* m_first;
        const T* m_last;

    public:
        using value_type      = T;
        using reference       = const T&;
        using const_reference = const T&;
        using size_type       = size_t;
        using iterator        = const T*;
        using const_iterator  = const T*;

        initializer_list() noexcept : m_first(nullptr), m_last(nullptr) {}

        // Number of elements.
        size_t size() const noexcept { return m_last - m_first; }

        // First element.
        const T* begin() const noexcept { return m_first; }

        // One past the last element.
        const T* end() const noexcept { return m_last; }
    };
#elif defined(_MSC_VER)
// Copyright (c) Microsoft Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---- LLVM Exceptions to the Apache 2.0 License ----
//
// As an exception, if, as a result of your compiling your source code, portions
// of this Software are embedded into an Object form of such source code, you
// may redistribute such embedded portions in such Object form without complying
// with the conditions of Sections 4(a), 4(b) and 4(d) of the License.
//
// In addition, if you combine or link compiled forms of this Software with
// software that is licensed under the GPLv2 ("Combined Software") and if a
// court of competent jurisdiction determines that the patent provision (Section
// 3), the indemnity provision (Section 9) or other Section of the License
// conflicts with the conditions of the GPLv2, you may retroactively and
// prospectively choose to deem waived or otherwise exclude such Section(s) of
// the License, but only in their entirety and only with respect to the Combined
// Software.

    template<typename T>
    class initializer_list
    {
    public:
        using value_type = T;
        using reference = const T&;
        using const_reference = const T&;
        using size_type = size_t;
        using iterator = const T*;
        using const_iterator = const T*;

        constexpr initializer_list() noexcept : m_first(nullptr), m_last(nullptr) {}

        constexpr initializer_list(const T* first, const T* last) noexcept
            : m_first(first), m_last(last) {}

        // First element.
        constexpr const T* begin() const noexcept { return m_first; }

        // One past the last element.
        constexpr const T* end() const noexcept { return m_last; }

        // Number of elements.
        constexpr size_t size() const noexcept
        {
            return static_cast<size_t>(m_last - m_first);
        }

    private:
        const T* m_first;
        const T* m_last;
    };
#else
    #error "Initializer_list is not supported for this compiler"
#endif

    template<typename T>
    constexpr const T* begin(initializer_list<T> il) noexcept 
    {
        return il.begin();
    }

    template<typename T>
    constexpr const T* end(initializer_list<T> il) noexcept
    {
        return il.end();
    }
}

// Create the Joystick
Joystick_ Joystick;

class List {
public:
    byte length;
    byte data[16];
    void append(byte item) {
        if (length < 16) data[length++] = item;
    }
    void remove(byte index) {
        if (index >= length) return;
        memmove(&data[index], &data[index+1], length - index - 1);
        length--;
    }
};

template<typename T>
class List2 {
public:
    byte length;
    T* data[255];
    void append(T* item) {
        if (length < 255) data[length++] = item;
    }
    // void remove(T index) {
    //     if (index >= length) return;
    //     memmove(&data[index], &data[index+1], length - index - 1);
    //     length--;
    // }
};

class PinHeader
{
  private:
    List pins;

  public:
    PinHeader(std::initializer_list<int>  pins);
    int pin(int);
};

class Button
{
  public:
    //Button();

    int a();
    virtual void read()=0;
    virtual void init()=0;

};

int Button::a()
{
  return 1;
}

class DigitalButton: public Button {
  private:
    int pin=NULL;
    int value=NULL;

  public:
    DigitalButton(int pin);

    void read();
    void init();
};

// Button::Button()
// {
  
// }

DigitalButton::DigitalButton(int pin)
{
  this->pin = pin;
}

void DigitalButton::init()
{
  Serial.println("DigitalButton::init()");
  pinMode(this->pin, INPUT_PULLUP);
}

void DigitalButton::read()
{
  char buf[50];

  Serial.println("reading");
  this->value = digitalRead(this->pin);

  Serial.println("read");

  sprintf(buf, "Digital button %d: %d", this->pin, this->value);
  Serial.println(buf);
}

class Buttons
{
  public:
    int a = 1;
    List2<Button> buttons;

    void init();
    void read();
};

void Buttons::init()
{
  for (int i = 0; i < this->buttons.length; i++)
  {
    Serial.println("Buttons::Init() - Button " + String(i));
    this->buttons.data[i]->init();
  }
}

void Buttons::read()
{
  for (int i = 0; i < this->buttons.length; i++)
  {
    Serial.println("Buttons::Read() - Button " + String(i));
    this->buttons.data[i]->read();
  }
}

PinHeader::PinHeader(std::initializer_list<int>  pins)
{
  for (int element : pins)
  {
    Serial.println("Adding: " + String(element));
    this->pins.append(element);
  }
  // for (int i=0; i< sizeof(pins) / sizeof(int); i++)
  // {
  //   Serial.print('Adding: ' + String(pins[i]));
  //   this->pins.append(pins[i]);
  // }
}

int PinHeader::pin(int id)
{
  if ( id < pins.length)
  {
    return this->pins.data[id];
  } else {
    return -1;
  }
}

PinHeader header_1({A3, A2, A1, A0});
PinHeader header_2({14, 16, 15, 10});
PinHeader header_3({6, 7, 8, 9});
PinHeader header_4({2, 3, 4, 5});

Buttons buttons;

long lastLoop;
int loopa = 0;
bool first=true;

void setup() 
{  
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Init start"); 
  lastLoop = millis();
  Serial.println("Init End");
}


    

void loop() 
{
  char buf[50];
  ;
  
  if ((millis() - lastLoop) > 1000)
  {
    loopa += 1;
    lastLoop = millis();
    Serial.println("loop " + String(loopa));

    if (loopa >= 5)
    {
      static Button* button;
      
      if (first)
      {
        first = false;
        //buttons.buttons.append(&DigitalButton(header_2.pin(0)));
      // buttons.buttons.append(&DigitalButton(header_2.pin(1)));
      // buttons.buttons.append(&DigitalButton(header_2.pin(2)));
      // buttons.buttons.append(&DigitalButton(header_2.pin(3)));
      
        Serial.println("Button create");
        static DigitalButton b(header_2.pin(0));
        button = &b;
        Serial.println("Button Init");
        button->init();       
        Serial.println("Button Init Done");
      } 
      else
      {
        Serial.println("read");
        button->read();
      }

      Serial.println("a");
    }
  
    // buttons.read();
  }
}


// const long debounceDelay = 30;
// const int numInputDigital = 4;

// #define CLK 15
// #define DATA 10


// // 3, 2 - Toggle
// // 4, 5, 6 - Rotary press - encoder
// int pinInputDigital[numInputDigital] = {14};

// const int numInputAnaloguePin = 4;
// const int numInputAnalogueBracket = 5;

// const int numInputRotary=1;


// // 4 pin red (left to right)
// // - - switch right
// // - 

// const int numHeaders = 4;
// const int numPinsPerHeader = 4;

// const bool headerDigital[numHeaders] = {false, true, true, true};
// const int  headerPins[numHeaders] [numPinsPerHeader] = {{A3, A2, A1, A0}, {14, 16, 15, 10}, {6, 7, 8, 9}, {2, 3, 4, 5}};

// int pinInputAnalogue[numInputAnaloguePin] = {A3,A2,A1,A0};

// const int analogueBracket[numInputAnalogueBracket] = {800, 767, 681, 510, 0};

// const int numInputAnalogue = numInputAnaloguePin * numInputAnalogueBracket;
// const int numInput = numInputAnalogue + numInputDigital + numInputRotary*2;



// // Digital: 2, 1          // Toggle Switch - Left/Right
// // Rotary Encoder: 2      // PRess

// int inputState[numInput];
// int lastInputState[numInput];
// long lastDebounceTime[numInput];

// int val = 0; 


// static uint8_t prevNextCode = 0;
// static uint16_t store=0;

// void setup2() {

//   for (int iHeader = 0; iHeader < numHeaders; iHeader++)
//   {
//     for (int iOffset = 0;  iOffset < numPinsPerHeader; iOffset++)
//     {
//       int value;
      
//       if (headerDigital[iHeader])
//       {
//         pinMode(headerPins[iHeader][iOffset], INPUT_PULLUP);
//       }
//     }
//   }

//   pinMode(CLK, INPUT_PULLUP);
//   pinMode(DATA, INPUT_PULLUP);

//   // Initialize Joystick Library
//   Joystick.begin();
  
//   Serial.begin(9600);
// }


// void display_headers()
// {  
//   static long lastLoop;
//   char buf[50];
//   sprintf(buf, "%02d (%01d) ", headerPins[iHeader][iOffset], value);

//   if ((millis() - lastLoop) > 2000)
//   {
//     lastLoop = millis();


    
//     for (int iHeader = 0; iHeader < numHeaders; iHeader++)
//     {
//       Serial.print("H" + String(iHeader));
//       if (headerDigital[iHeader])
//       {
//         Serial.print("D");
//       } 
//       else
//       {
//         Serial.print("A");
//       }
      
//       Serial.print(": ");
      
//       for (int iOffset = 0;  iOffset < numPinsPerHeader; iOffset++)
//       {
//         int value;
        
//         if (headerDigital[iHeader])
//         {
//           value = digitalRead(headerPins[iHeader][iOffset]);
//           sprintf(buf, "%02d (%01d) ", headerPins[iHeader][iOffset], value);
//         }
//         else
//         {
//           value = analogRead(headerPins[iHeader][iOffset]);          
//           sprintf(buf, "%02d (%04d) ", headerPins[iHeader][iOffset], value);
//         }

//         Serial.print(buf);
//       }

//       Serial.print(" === ");
//     }
    
//     Serial.println("");
//   }
// }


// void loop2() 
// {
//   display_headers();
    
//   setInputFlags();
//   resolveInputFlags();
 
//   static int8_t c,val;

//  if( val=read_rotary() ) {
//     c +=val;
//     Serial.print(c);Serial.print(" ");

//     if ( prevNextCode==0x0b) {
//        Serial.print("eleven ");
//        Serial.println(store,HEX);
//     }

//     if ( prevNextCode==0x07) {
//        Serial.print("seven ");
//        Serial.println(store,HEX);
//     }
//  }
// }


// // A vald CW or  CCW move returns 1, invalid returns 0.
// int8_t read_rotary() {
//   static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

//   prevNextCode <<= 2;
//   if (digitalRead(DATA)) prevNextCode |= 0x02;
//   if (digitalRead(CLK)) prevNextCode |= 0x01;
//   prevNextCode &= 0x0f;

//    // If valid then store as 16 bit data.
//    if  (rot_enc_table[prevNextCode] ) {
//       store <<= 4;
//       store |= prevNextCode;
//       //if (store==0xd42b) return 1;
//       //if (store==0xe817) return -1;
//       if ((store&0xff)==0x2b) return -1;
//       if ((store&0xff)==0x17) return 1;
//    }
//    return 0;
// }

// void setInputFlags()
// {
//   for (int i = 0; i < numInputDigital + numInputAnalogue; i++)
//   {
//     int reading = LOW;
    
//     if (i < numInputDigital)
//     {
//       int pin = pinInputDigital[i];
//       if (pin == NULL)
//       {
//         reading = LOW;
//       } 
//       else
//       {
//         reading = !digitalRead(pinInputDigital[i]);
//       }
//     } 
//     else
//     {
//       int analoguePin = ((i - numInputDigital) / numInputAnalogueBracket);
//       int analogueOffset = (i - numInputDigital) % numInputAnalogueBracket;
   
//       int rawReading = analogRead(pinInputAnalogue[analoguePin]);

//       int minRange = max(analogueBracket[analogueOffset] - 5, 0);
//       int maxRange = min(analogueBracket[analogueOffset] + 5, 1024);

//       if ((rawReading >= minRange) and (rawReading <= maxRange))
//       {
//         reading = HIGH;
//       }

//       //Serial.print(" - " + String(analoguePin * numInputAnalogueBracket + analogueOffset) + " is " + String(reading));
//       //Serial.print("- Reading A." + String(analoguePin) + "." + String(analogueOffset) + " (" + String(reading) + ":" + String(rawReading) + ") [" + String(minRange) + " to " + String(maxRange) + "]");
//     }

//     if (reading != lastInputState[i])
//     {
//       lastDebounceTime[i] = millis();
//     }

//     // State must last "debounce" ms before being acknowledged
//     if ((millis() - lastDebounceTime[i]) > debounceDelay)
//     {
//       //Serial.print(" - Debounced (" + String(reading) + " - " + String(inputState[i]));
//       if (reading != inputState[i])
//       {
//         Joystick.setButton(i, reading);
//         if (reading == HIGH)
//         {
//           Serial.println("- Button " + String(i) + " pressed");
//         }
//         else
//         {
//           Serial.println("- Button " + String(i) + " released");
//         }
        
//         inputState[i] = reading;
//       }      
//     }
    
//     lastInputState[i] = reading;

//     //Serial.println();
//   }

//   // -- build the rotary into the list.
//   // have a delay to turn off button safter x ms
// //  uint8_t x = Rotary_1.read();
// //  if (x)
// //  {
// //    if (x == DIR_CW)
// //    {
// ////      Joystick.setButton(3, HIGH);
// //      Serial.println("- Rotary: Right");
// //      //delay(5);
// //      Joystick.setButton(3, LOW);
// //    }
// //    else
// //    {
// //      Joystick.setButton(4, HIGH);
// //      Serial.println("- Rotary: Left");
// //      //delay(5);
// //      Joystick.setButton(4, LOW);
// //    }
// //    //Serial.print(x == DIR_CW ? "\n+1" : "\n-1");
// //  }
  
//   //delay(1000);
// }

// void resolveInputFlags()
// {
  
// }



// // int lastButtonState = 0;
// // Read pin values
// //  int currentButtonState = !digitalRead(pinToButtonMap);
// //  if (currentButtonState != lastButtonState)
// //  {
// //    Joystick.setButton(0, currentButtonState);
// //    lastButtonState = currentButtonState;
// //  }
// //  
// //  val = analogRead(A0);
// //  if (val<=1020){
// //    Serial.println(val);
// //  }
// //  
// //  val = analogRead(A1);
// //  if (val<=1020){
// //    Serial.println(val);
// //  }
// //  
// //
// //  delay(30);
