//This code is placed under the MIT license
//Copyright (c) 2020 Albert Barber
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in
//all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

//code intended to run on esp8266, Wemos D1 mini
//requires lastest version of adafruit neopixel library (use the library manager)
#include <PixelStrip.h>
#include "segmentDefs.h"
#include <bluefruit.h>
#include <SPI.h>

//pin connections
#define DATAPIN 12
#define CLOCKPIN 13

// BLE Service
BLEDfu bledfu;
BLEDis bledis;
BLEUart bleuart;

// Packet buffer
extern uint8_t packetbuffer[];

//EEPROM Addresses for settings
//we want to store the brightness, current effect index, and the effect rotation toggle
//so they can be recalled after the mask has been turned off
#define BRIGHTNESS_ADDR 2 //brightness address
#define CUR_EFFECT_ADDR 0 //index of current effect address
#define EFFECT_ROT_ADDR 1 //effect rotaion bool address

//effects control vars
byte effectIndex = 0; //number of effect that's currently active (will be read from EEPROM later)
const byte numEffects = 41; //number of current effects - 1
boolean effectRota = true; //effects rotation on / off flag
boolean effectsStop = false; //stop all effects flag
boolean direct = true; //use for setting direction of effects
boolean breakCurrentEffect = false; //flag for breaking out of effects that use multiple sub effects / loops

//macro for implementing break for effects with multiple sub effects
#define breakEffectCheck() ({ \
    if( (breakCurrentEffect) ){ \
      (breakCurrentEffect) = false; \
      break; \
    } \
  })

//brightness vars
byte brightnessIndex = 1; //initial brightness, index of brightnessLevels array
//brightness levels array, max is 255, but 100 should be bright enough for amost all cases
//!!WARNING brightness is directly tied to power consumption, the max current per led is 60ma, this is for white at 255 brightness
//if you actually run all the leds at max, the glasses will draw 4.75 amps, this is beyond the rating of the jst connectors
const byte brightnessLevels[] = { 10, 30, 140, 230 };
const byte numBrightnessLevels = SIZE( brightnessLevels );

//Strip definitions
const uint16_t stripLength = 104;
const uint8_t stripType = NEO_RGB + NEO_KHZ800;

#define NUM_PATTERNS 2
enum pattern {
  RAINBOW_CYCLE,
  PATTERN_1,
  PATTERN_2,
  PATTERN_3,
  PATTERN_4,
  PATTERN_5,
  PATTERN_6,
  PATTERN_7,
  PATTERN_8,
  PATTERN_9,
  PATTERN_10,
  PATTERN_11,
  PATTERN_12,
  PATTERN_13,
  PATTERN_14,
  PATTERN_15,
  PATTERN_16,
  PATTERN_17,
  PATTERN_18,
  PATTERN_19,
  PATTERN_20,
  PATTERN_21,
  PATTERN_22,
  PATTERN_23,
  PATTERN_24,
  PATTERN_25,
  PATTERN_26,
  PATTERN_27,
  PATTERN_28,
  PATTERN_29,
  PATTERN_30,
  PATTERN_31,
  PATTERN_32,
  PATTERN_33,
  PATTERN_34,
  PATTERN_35,
  PATTERN_36,
  PATTERN_37,
  PATTERN_38,
  PATTERN_39,
  PATTERN_40
};
enum direction { FORWARD, REVERSE };
uint8_t patternIndex = 0;

//rowCenSegments
//two color sine wave pattern
byte spinPatternWave[(7 + 2) * 10] = {
  0, 1,   1, 2, 0, 0, 0, 0, 0,
  1, 2,   3, 1, 2, 0, 0, 0, 0,
  2, 3,   0, 3, 1, 2, 0, 0, 0,
  3, 4,   0, 0, 3, 1, 2, 0, 0,
  4, 5,   0, 0, 0, 3, 1, 2, 0,
  5, 6,   0, 0, 0, 0, 3, 1, 2,
  6, 7,   0, 0, 0, 3, 1, 2, 0,
  7, 8,   0, 0, 3, 1, 2, 0, 0,
  8, 9,   0, 3, 1, 2, 0, 0, 0,
  9, 10,  3, 1, 2, 0, 0, 0, 0,
};

//rowCenSegments
//two color sine wave pattern
byte spinPatternWave2[(7 + 2) * 8] = {
  0, 1,   0, 1, 2, 1, 0, 0, 0,
  1, 2,   0, 0, 1, 2, 1, 0, 0,
  2, 3,   0, 0, 0, 1, 2, 1, 0,
  3, 4,   0, 0, 0, 0, 1, 2, 1,
  4, 5,   0, 0, 0, 0, 1, 2, 1,
  5, 6,   0, 0, 0, 1, 2, 1, 0,
  6, 7,   0, 0, 1, 2, 1, 0, 0,
  7, 8,   0, 1, 2, 1, 0, 0, 0,
};

//rowCenSegments
//two color sine wave pattern
byte spinPatternHeart[(7 + 2) * 7] = {
  0, 1,   0, 0, 0, 0, 0, 0, 0,
  1, 2,   0, 0, 1, 1, 1, 0, 0,
  2, 3,   0, 1, 2, 2, 2, 1, 0,
  3, 4,   0, 0, 1, 2, 2, 2, 1,
  4, 5,   0, 1, 2, 2, 2, 1, 0,
  5, 6,   0, 0, 1, 1, 1, 0, 0,
  6, 7,   0, 0, 0, 0, 0, 0, 0,
};

//rowCenSegments
//two color sine wave pattern
byte spinPatternHelix[(7 + 2) * 10] = {
  0, 1,   0, 1, 0, 0, 0, 2, 0,
  1, 2,   0, 0, 1, 0, 2, 0, 0,
  2, 3,   0, 0, 0, 1, 0, 0, 0,
  3, 4,   0, 0, 2, 0, 1, 0, 0,
  4, 5,   0, 2, 0, 0, 0, 1, 0,
  5, 6,   0, 2, 0, 0, 0, 1, 0,
  6, 7,   0, 0, 2, 0, 1, 0, 0,
  7, 8,   0, 0, 0, 2, 0, 0, 0,
  8, 9,   0, 0, 1, 0, 2, 0, 0,
  9, 10,  0, 1, 0, 0, 0, 2, 0,
};

//for rowHalfCenSegments, we want rows seperated, so we can move the arrows in opposite directions
//two color arrows, moving horizontally
//if you want vertical arrows, you have to use columnSegments
byte spinPatternArrows[(14 + 2) * 5] = {
  0, 1,  1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1,
  1, 2,  2, 1, 0, 0, 0, 1, 2, 2, 1, 0, 0, 0, 1, 2,
  2, 3,  0, 2, 1, 0, 1, 2, 0, 0, 2, 1, 0, 1, 2, 0,
  3, 4,  0, 0, 2, 1, 2, 0, 0, 0, 0, 2, 1, 2, 0, 0,
  4, 5,  0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
};

byte spinPatternArrows2[(14 + 2) * 5] = {
  0, 1,  1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1,
  1, 2,  2, 1, 1, 1, 1, 1, 2, 2, 1, 1, 0, 1, 1, 2,
  2, 3,  2, 2, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 2, 2,
  3, 4,  0, 2, 2, 1, 2, 2, 0, 0, 2, 2, 1, 2, 2, 0,
  4, 5,  1, 0, 2, 2, 2, 0, 1, 1, 0, 2, 2, 2, 0, 1,
};

//for rowHalfCenSegments, we want rows seperated, so we can move the arrows in opposite directions
byte spinPatternCircles[(14 + 2) * 4] = {
  0, 1,  2, 1, 0, 0, 0, 1, 2, 2, 1, 0, 0, 0, 1, 2,
  1, 2,  0, 2, 1, 1, 1, 2, 0, 0, 2, 1, 1, 1, 2, 0,
  2, 3,  0, 0, 2, 2, 2, 0, 0, 0, 0, 2, 2, 2, 0, 0,
  3, 4,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

//for rowHalfCenSegments, we want rows seperated, so we can move the arrows in opposite directions
byte spinPatternX[(14 + 2) * 4] = {
  0, 1,  0, 0, 1, 2, 1, 0, 0, 0, 0, 1, 2, 1, 0, 0,
  1, 2,  0, 1, 2, 0, 2, 1, 0, 0, 1, 2, 0, 2, 1, 0,
  2, 3,  1, 2, 0, 0, 0, 2, 1, 1, 2, 0, 0, 0, 2, 1,
  3, 4,  2, 0, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 2,
};

//for colCenSegments, vertical arrows
byte spinPatternArrowsVert[(12 + 2) * 4] = {
  0, 1,   3, 4, 1, 2, 3, 4, 4, 3, 2, 1, 4, 3,
  1, 2,   2, 3, 4, 1, 2, 3, 3, 2, 1, 4, 3, 2,
  2, 3,   1, 2, 3, 4, 1, 2, 2, 1, 4, 3, 2, 1,
  3, 4,   4, 1, 2, 3, 4, 1, 1, 4, 3, 2, 1, 4,
};

//for colCenSegments, vertical arrows
byte spinPatternArrowsVert2[(12 + 2) * 6] = {
  0, 1,   0, 2, 2, 3, 3, 0, 0, 3, 3, 2, 2, 0,
  1, 2,   0, 0, 2, 2, 3, 3, 3, 3, 2, 2, 0, 0,
  2, 3,   3, 0, 0, 2, 2, 3, 3, 2, 2, 0, 0, 3,
  3, 4,   3, 3, 0, 0, 2, 2, 2, 2, 0, 0, 3, 3,
  4, 5,   2, 3, 3, 0, 0, 2, 2, 0, 0, 3, 3, 2,
  5, 6,   2, 2, 3, 3, 0, 0, 0, 0, 3, 3, 2, 2,
};

//for  rowCenSegments, USA flag
byte spinPatternUSflag[(7 + 2) * 5] = {
  0, 1,   4, 1, 4, 1, 6, 1, 6,
  1, 2,   1, 4, 1, 4, 6, 1, 6,
  2, 3,   4, 1, 4, 1, 6, 1, 6,
  3, 4,   1, 4, 1, 4, 6, 1, 6,
  4, 8,   6, 1, 6, 1, 6, 1, 6
};

//for  rowCenSegments, St George's cross (can't get blue right for Union Jack)
byte spinPatternStGeorgeCross[(7 + 2) * 3] = {
  0, 3,   1, 1, 1, 6, 1, 1, 1,
  3, 4,   6, 6, 6, 6, 6, 6, 6,
  4, 7,   1, 1, 1, 6, 1, 1, 1,
};

//storage for pallets we'll generate on the fly
uint32_t tempRandPallet[5]; //pallet for random colors (length sets number of colors used by randomColors effect)
uint32_t tempTwinklePallet[2]; //used for twinkle effect, we are only twinkling two colors at a time

//class DotStarPatterns : public Adafruit_DotStar
class PixelStripPatterns : public PixelStrip
{
  //Define some colors we'll use frequently
  const uint32_t white =    this->Color(255, 255, 255);
  const uint32_t UCLAGold = this->Color(254, 187, 54);
  const uint32_t UCLABlue = this->Color(83, 104, 149);
  const uint32_t off =      this->Color( 0, 0, 0 );
  const uint32_t red =      this->Color(255, 0, 0);
  const uint32_t orange =   this->Color(255, 43, 0);
  const uint32_t ltOrange = this->Color(255, 143, 0);
  const uint32_t yellow =   this->Color(255, 255, 0);
  const uint32_t ltYellow = this->Color(255, 255, 100);
  const uint32_t green =    this->Color(0, 128, 0);
  const uint32_t blue =     this->Color(0, 0, 255);
  const uint32_t indigo =   this->Color( 75, 0, 130);
  const uint32_t violet =   this->Color(238, 130, 238);
  const uint32_t purple =   this->Color(123, 7, 197);
  const uint32_t pink =     this->Color(225, 0, 127);

  const uint32_t pastelRainbow = this->Color(130, 185, 226); //178,231,254,
  const uint32_t pastelRainbow1 = this->Color(110, 46, 145); //purple
  const uint32_t pastelRainbow2 = this->Color(54, 174, 218); //teal
  const uint32_t pastelRainbow3 = this->Color(120, 212, 96); //green
  const uint32_t pastelRainbow4 = this->Color(255, 254, 188); //yellow
  const uint32_t pastelRainbow5 = this->Color(236, 116, 70); //orange
  const uint32_t pastelRainbow6 = this->Color(229, 61, 84); //pink red

  //define pallet array, contains 32bit representations of all colors used in patterns
  uint32_t pallet[9] = { off, white, UCLAGold, UCLABlue, blue, yellow, red, green, purple };
  //                   { -0-, --1--, ---2----, ----3---, -4--, ---5--, -6-, --7--, --8-- }

  //pallet to match typical fairy light colors
  uint32_t christmasPallet[5] = { red, blue, green, yellow, purple };
  
  uint32_t pastelRainbowPallet[7] = { pastelRainbow, pastelRainbow1 , pastelRainbow2, pastelRainbow3, pastelRainbow4, pastelRainbow5, pastelRainbow6 };
  byte pastelRainbowPattern[14] = {  6, 6, 1, 1, 2, 2, 5, 5, 4, 4, 3, 3, 0, 0 };
  
  uint32_t firePallet[3] = { red, ltOrange, ltYellow };
  
  uint32_t firePallet2[3] = { purple, pink, white };
  
  public:
    // Member variables
    pattern ActivePattern; // Running pattern
    direction Direction; // direction to run the pattern

    unsigned long Interval; // ms between updates
    unsigned long lastUpdate; // Last update of position

    uint32_t Color1, Color2; // What colors are in use
    uint16_t TotalSteps; // Total number of steps in the pattern
    uint16_t Index; // current step within the pattern

    void (*OnComplete)(); // Callback on completion of pattern

    // Constructor
    PixelStripPatterns(uint16_t pixels, uint8_t datapin, uint8_t clockpin, uint8_t type, void (*callback)()) : PixelStrip(pixels, datapin, clockpin, type)
    {
      OnComplete = callback;
    }
    
    // Update the pattern
    void Update()
    {
      if ((millis() - lastUpdate) > Interval) // time to update
      {
        lastUpdate = millis();
        switch (ActivePattern)
        {
          case RAINBOW_CYCLE:
            RainbowCycleUpdate();
            break;
          case PATTERN_1:
            Pattern1Update();
            break;
          case PATTERN_2:
            Pattern2Update();
            break;
          case PATTERN_3:
            Pattern3Update();
            break;
          case PATTERN_4:
            Pattern4Update();
            break;
          case PATTERN_5:
            Pattern5Update();
            break;
          case PATTERN_6:
            Pattern6Update();
            break;
          case PATTERN_7:
            Pattern7Update();
            break;
          case PATTERN_8:
            Pattern8Update();
            break;
          case PATTERN_9:
            Pattern9Update();
            break;
          case PATTERN_10:
            Pattern10Update();
            break;
          case PATTERN_11:
            Pattern11Update();
            break;
          case PATTERN_12:
            Pattern12Update();
            break;
          case PATTERN_13:
            Pattern13Update();
            break;
          case PATTERN_14:
            Pattern14Update();
            break;
          case PATTERN_15:
            Pattern15Update();
            break;
          case PATTERN_16:
            Pattern16Update();
            break;
          case PATTERN_17:
            Pattern17Update();
            break;
          case PATTERN_18:
            Pattern18Update();
            break;
          case PATTERN_19:
            Pattern19Update();
            break;
          case PATTERN_20:
            Pattern20Update();
            break;
          case PATTERN_21:
            Pattern21Update();
            break;
          case PATTERN_22:
            Pattern22Update();
            break;
          case PATTERN_23:
            Pattern23Update();
            break;
          case PATTERN_24:
            Pattern24Update();
            break;
          case PATTERN_25:
            Pattern25Update();
            break;
          case PATTERN_26:
            Pattern26Update();
            break;
          case PATTERN_27:
            Pattern27Update();
            break;
          case PATTERN_28:
            Pattern28Update();
            break;
          case PATTERN_29:
            Pattern29Update();
            break;
          case PATTERN_30:
            Pattern30Update();
            break;
          case PATTERN_31:
            Pattern31Update();
            break;
          case PATTERN_32:
            Pattern32Update();
            break;
          case PATTERN_33:
            Pattern33Update();
            break;
          case PATTERN_34:
            Pattern34Update();
            break;
          case PATTERN_35:
            Pattern35Update();
            break;
          case PATTERN_36:
            Pattern36Update();
            break;
          case PATTERN_37:
            Pattern37Update();
            break;
          case PATTERN_38:
            Pattern38Update();
            break;
          case PATTERN_39:
            Pattern39Update();
            break;
          case PATTERN_40:
            Pattern40Update();
            break;
          default:
            break;
        }
      }
    }

    // Increment the Index and reset at the end
    void Increment()
    {
        if (Direction == FORWARD)
        {
           Index++;
           if (Index >= TotalSteps)
            {
                Index = 0;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
        else // Direction == REVERSE
        {
            --Index;
            if (Index <= 0)
            {
                Index = TotalSteps-1;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
    }

    // Reverse pattern direction
    void Reverse()
    {
        if (Direction == FORWARD)
        {
            Direction = REVERSE;
            Index = TotalSteps-1;
        }
        else
        {
            Direction = FORWARD;
            Index = 0;
        }
    }

    //a quick shortening of the random color function, just to reduce the pattern function calls more readable
    uint32_t RC() {
      return this->randColor();
    }

    // Initialize for a RainbowCycle
    void RainbowCycle(uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = RAINBOW_CYCLE;
        Interval = interval;
        TotalSteps = 255;
        Index = 0;
        Direction = dir;
    }

    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate()
    {
        for(int i=0; i< numPixels(); i++)
        {
            setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
        }
        show();
        Increment();
    }

    void Pattern1Update()
    {
      this->shooterSeg(colSegments, pallet, 3, 0, true, 12, 1, 1, 3, 4, false, false, 10, 170);
    }

    void Pattern2Update()
    {
      this->shooterSeg( colSegments, pallet, 5, 0, true, 12, 1, 1, 3, 2, false, false, 10, 170); //rainbow
    }

    void Pattern3Update()
    {
      this->shooterSeg( rowHalfSegments, pallet, 3, 0, true, 10, 1, 1, 3, 4, true, false, 10, 170);
    }

    void Pattern4Update()
    {
      this->shooterSeg( rowHalfSegments, pallet, 5, 0, true, 10, 1, 1, 3, 2, true, false, 10, 170); //rainbow
    }

    void Pattern5Update()
    {
      this->shooterSeg( rowHalfSegments, pallet, 5, 0, true, 8, 1, 3, 3, 4, true, false, 10, 150);
    }

    void Pattern6Update()
    {
      this->shooterSeg( colSegments, pallet, 5, 0, true, 8, 1, 3, 3, 4, false, false, 10, 170);
    }

    void Pattern7Update()
    {
      this->shooterSeg( colHalfFlipSegments, pallet, 4, 0, true, 5, 6, 1, 6, 4, true, false, 10, 180);
    }

    void Pattern8Update()
    {
      this->genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
      this->patternSweepSetRand( 9, tempRandPallet,  SIZE(tempRandPallet), 0, 1, 4, true, 0, 1, 10, 200);
    }

    void Pattern9Update()
    {
      this->doFireV2Seg( colSegments, firePallet, SIZE(firePallet), 25, 200, true, 10, 70);
    }

    void Pattern10Update()
    {
      for (int i = 0; i < 3; i++) {
          breakEffectCheck();
          this->patternSweepRepeatRand(4, 0, 0, 2, 4, false, false, 0, 0, 1, 10, 120 );
        }
    }

    void Pattern11Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          this->gradientCycleRand( 5, 6, 150, direct, 100);
          direct = !direct;
        }
    }

    void Pattern12Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          this->genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          this->fillStrip(tempRandPallet[2], false);
          rowCenSegments.flipSegDirectionEvery(1, true);
          this->colorSpin( rowCenSegments, spinPatternWave2, SIZE(spinPatternWave2), tempRandPallet, 0, 1, true, 100, 10 );
        }
    }

    void Pattern13Update()
    {
      this->sonarWavesRand( colSegments, 6, 0, 4, 8, false, false, true, true, 120, 30);
    }

    void Pattern14Update()
    {
      this->sonarWavesRand( rowSegments, 2, 0, 4, 8, false, true, true, false, 110, 10);
    }

    void Pattern15Update()
    {
      this->rainbowWave( colSegments, 80, true, 10, 25);
    }

    void Pattern16Update()
    {
      this->rainbowWave( rowSegments, 80, false, 8, 20);
    }

    void Pattern17Update()
    {
      this->waves( colSegments, pastelRainbowPallet, SIZE(pastelRainbowPallet),  pastelRainbowPattern, SIZE(pastelRainbowPattern), 50, false, 5, 45);
    }

    void Pattern18Update()
    {
      for (int i = 0; i < 10; i++) {
          breakEffectCheck();
          this->colorWipeRandomSeg( colSegments, 2, 2, 7, 10, true, true, true);
          breakEffectCheck();
          this->colorWipeSeg(colSegments, 0, 7, 10, false, true, true);
        }
    }

    void Pattern19Update()
    {
      for (int i = 0; i < 6; i++) {
          breakEffectCheck();
          uint32_t color = RC();
          this->crossFadeColor(0, color, 30, 30);
        }
    }

    void Pattern20Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          this->randomWaves( colSegments, 5, 3, 2, 50, direct, 7, 20);
          direct = !direct;
        }
    }

    void Pattern21Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          this->randomWaves( rowSegments, 4, 2, 2, 50, direct, 7, 20);
          direct = !direct;
        }
    }

    void Pattern22Update()
    {
      for (int i = 0; i < 6; i++) {
          breakEffectCheck();
          this->colorWipeRandomSeg( rowHalfSegments, 2, 2, 0, 20, direct, false, true);
          breakEffectCheck();
          this->colorWipeSeg(rowHalfSegments, 0, 0, 20, !direct, false, true);
          direct = !direct;
        }
    }

    void Pattern23Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          colCenSegments.flipSegDirectionEvery(1, true);
          this->genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          this->fillStrip(tempRandPallet[2], false);
          this->colorSpin( colCenSegments, spinPatternArrowsVert2, SIZE(spinPatternArrowsVert2), tempRandPallet, 0, 1, true, 100, 10 );
        }
    }

    void Pattern24Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          this->genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          this->fillStrip(tempRandPallet[1], false);
          rowHalfCenSegments.flipSegDirectionEvery(1, true); //reverse the direction of each segment of rowLensesHalvesSegments
          this->colorSpin( rowHalfCenSegments, spinPatternArrows2, SIZE(spinPatternArrows2), tempRandPallet, 0, 1, true, 100, 30 );
        }
    }

    void Pattern25Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          this->genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          this->fillStrip(tempRandPallet[1], false);
          this->colorSpin( rowHalfCenSegments, spinPatternX, SIZE(spinPatternX), tempRandPallet, 0, 1, true, 100, 30 );
          rowHalfCenSegments.flipSegDirectionEvery(1, true); //reverse the direction of each segment of rowLensesHalvesSegments
        }
    }

    void Pattern26Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          this->genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          this->fillStrip(tempRandPallet[1], false);
          rowHalfCenSegments.flipSegDirectionEvery(1, true); //reverse the direction of each segment of rowLensesHalvesSegments
          this->colorSpin( rowHalfCenSegments, spinPatternArrows, SIZE(spinPatternArrows), tempRandPallet, 0, 1, true, 100, 10 );
        }
    }

    void Pattern27Update()
    {
      this->colorSpinSimple( rowHalfSegments, 5, 0, 0, 3, -1, 3, 0, 1, 170, 10 );
    }

    void Pattern28Update()
    {
      this->setRainbowOffsetCycle(40, false);
      this->runRainbowOffsetCycle(true);
      this->colorSpinSimple( rowSegments, 1, 0, 0, 5, 1, 0, 0, 2, 170, 10 );
    }

    void Pattern29Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          colHalfSegments.flipSegDirectionEvery(1, true);
          this->colorSpinSimple( colHalfSegments, 5, 0, 0, 3, -1, 3, 0, 1, 120, 10 );
        }
    }

    void Pattern30Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          colHalfSegments.flipSegDirectionEvery(1, true);
          this->colorSpinSimple( colHalfSegments, 3, 0, 0, 5, -1, 5, 0, 1, 120, 10 );
        }
    }

    void Pattern31Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          colHalfSegments.flipSegDirectionEvery(1, true);
          this->colorSpinSimple( colHalfSegments, 1, 0, 0, 5, -1, 5, 0, 2, 120, 10 );
        }
    }

    void Pattern32Update()
    {
      this->simpleStreamerRand( 5, 0, 7, 0, 0, true, 160, 10);
    }

    void Pattern33Update()
    {
      this->setRainbowOffsetCycle(40, true);
      this->runRainbowOffsetCycle(true);
      this->patternSweepRand( 12, white, -1, 0, 0, false, 0, 1, 10, 140 );
    }

    void Pattern34Update()
    {
      this->genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
      this->randomColorSet(off, true, tempRandPallet, SIZE(tempRandPallet), 100, 5, 20000);
    }

    void Pattern35Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          this->segGradientCycleRand(rowSegments, 3, 7, 150, direct, 1, 10);
          direct = !direct;
        }
    }

    void Pattern36Update()
    {
      for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          this->segGradientCycleRand(colSegments, 3, 7, 150, direct, 1, 10);
          direct = !direct;
        }
    }

    void Pattern37Update()
    {
      this->genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
        tempRandPallet[0] = 0; //background color for spin patterns (off)
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          rowCenSegments.flipSegDirectionEvery(1, true);
          this->colorSpin( rowCenSegments, spinPatternHelix, SIZE(spinPatternHelix), tempRandPallet, 0, 1, true, 100, 10 );
        }
    }

    void Pattern38Update()
    {
      this->colorSpin( rowCenSegments, spinPatternUSflag, SIZE(spinPatternUSflag), pallet, 0, 1, false, 130, 10 );
    }

    void Pattern39Update()
    {
      tempTwinklePallet[0] = RC();
      tempTwinklePallet[1] = RC();
      this->twinkleSet(0, tempTwinklePallet, SIZE(tempTwinklePallet), 2, 60, 35, 12000);
    }

    void Pattern40Update()
    {
      //rainbow() only does one full cycle, we'll do 3 to extend its duration
      for (int i = 0; i < 3; i++) {
        breakEffectCheck();
        this->rainbow(35);
      }
    }

    // Calculate 50% dimmed version of a color (used by ScannerUpdate)
    uint32_t DimColor(uint32_t color)
    {
        // Shift R, G and B components one bit to the right
        uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        return dimColor;
    }

    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
        }
        show();
    }

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }
    
    // Input a value 0 to 255 to get a color value.
    // The colours are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if(WheelPos < 85)
        {
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        else if(WheelPos < 170)
        {
            WheelPos -= 85;
            return Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        else
        {
            WheelPos -= 170;
            return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
        }
    }
};

void StripComplete();

// Define DotStarPatterns for the strip.
PixelStripPatterns strip(stripLength, DATAPIN, CLOCKPIN, NEO_GRB + NEO_KHZ800, &StripComplete);

//byte wavepattern[]  = { 6, 1 };
//byte wavepattern2[] = { 5, 4 };
//byte wavepattern3[] = { 6, 7 };
//byte wavepattern4[] = { 8, 7 };
//byte christmasWavePattern[5] = { 0, 1, 2, 3, 4};

//below are spin patterns
//these are for use with the colorSpin function, which draws a pattern across segments (using the longest segment as the pattern basis)
//and then moves it along the longest segment
//you can make your own, their structure is the following
//(note that you'll have to understand how segments work in my library, you can find notes on segments in segmentSet.h)
//a spin pattern is comprised of two parts combined into one 1d array
// { 0, 1,   1, 2, 0, 0, 0, }
//  part 1       part 2
//part 1 indicates the leds on the longest segment that part 2 is to be drawn on (doesn't inculde the last index, so 1 above wouldn't be drawn)
//part 2 is the pattern of colors for each segment, made up of indeces for at pallet (ie 1 represents the second color in the pallet)

//this is confusing so I'll give an example
//for the glasses, using lenseRowSegments (each row of the glasses is a segment, excluding the center 6 leds so that all the rows are the same length)
//part 1 above indicates that, from the 0th led of the first segment, up to (not including) the 1st led, the color pattern of the segments is part 2
//the result is that the first led of each row are colored (using pallet above) row 0 -> white, row 1 -> uclaGold, row 2 -> off,  row 3 -> off, row 4 -> off
//note that leds indexs in part 1 are always defined using the longest segment, because colorSpin will map the pattern (part 1) onto shorter segments
//this works well on most led arrangments, but not so much on the glasses
//which is why I'm using lenseRowSegments, so that all segments are equal length, and the mapping is one to one

//as below, spin patterns can have multiple pattern sections
//you define the matrix as: spinPatternName[(5 + 2) * 8] -> [ (number of segments + 2) * number of rows in pattern ]
//you can also make patterns repeat using colorSpin, so you just have to define a single complete pattern (like I do with spinPatternWave)

//triggered by button 1, stops the current pattern, and switches to the next one, wrapping if needed
//also stores effect index in eeprom
//if button 2 is also being held, turn effects on / off
void nextEffect() {
  patternIndex = (patternIndex + 1) % NUM_PATTERNS;
  strip.ActivePattern = (pattern)patternIndex;
}

//triggered by button 2, turns effect rotation on / off
//also stores the state in eeprom
//(if rotation is off, the current effect will be repeated continuously)
//if button 1 is also being held, turn effects on / off
void effectRotaToggle() {
  effectRota = !effectRota;
}

//triggered by button 3, sets the strip brightness to the next
//also stores the brighness index in eeprom
//brightness level in the brightnessLevels array (wrapping to the start if needed)
void brightnessAdjust() {
  brightnessIndex = (brightnessIndex + 1) % numBrightnessLevels;
  strip.setBrightness( brightnessLevels[brightnessIndex] );
  sendResponse("Changed brightness.");
}

//increments the effect index (wrapping if needed)
void incrementEffectIndex() {
  resetSegDirections();
  strip.runRainbowOffsetCycle(false);
  strip.setRainbowOffset(0);
  effectIndex = (effectIndex + 1) % numEffects;
}

//resets all the segments to their default directions (as set in segmentDefs)
void resetSegDirections() {
  colSegments.setsegDirectionEvery(2, true, true);
  colSegments.setsegDirectionEvery(2, false, false);

  colHalfSegments.setsegDirectionEvery(2, false, true);
  colHalfSegments.setsegDirectionEvery(2, true, false);

  colHalfFlipSegments.setsegDirectionEvery(1, true, true);

  colCenSegments.setsegDirectionEvery(2, true, true);
  colCenSegments.setsegDirectionEvery(2, false, false);

  rowSegments.setsegDirectionEvery(1, true, true);

  rowHalfSegments.setsegDirectionEvery(2, false, true);
  rowHalfSegments.setsegDirectionEvery(2, true, false);
}

//-------------SETUP-----------------------------------------------SETUP
void setup() {
  delay(3000); // power-up safety delay
  
  //initalize the led strip, and set the starting brightness
  strip.begin();

  //pinMode(BUTTON_1, INPUT_PULLUP);
  //pinMode(BUTTON_2, INPUT_PULLUP);
  //pinMode(BUTTON_3, INPUT_PULLUP);
  //because of the way my library currently works, effects occupy the processor until they end
  //so to break out of an effect, or change sytem values, we need to use interrupts
  //attachInterrupt(digitalPinToInterrupt(BUTTON_1), nextEffect, FALLING);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_2), effectRotaToggle, FALLING);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_3), brightnessAdjust, FALLING);

  //read EEPROM values for current effect, brightness, and effect rotation
  effectRota = 0; // Should be read from EEPROM
  brightnessIndex = 0; // Should be read from EEPROM
  effectIndex = 0; // Should be read from EEPROM

  strip.setBrightness( brightnessLevels[brightnessIndex] );
  strip.show();

  // Start a pattern
  strip.RainbowCycle(3);

  // Bluefruit setup
  Bluefruit.begin();
  Bluefruit.setName("Mitch_LED_Mask");
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  Bluefruit.Periph.setConnectCallback(connect_callback);
  
  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();  

  // Configure and start BLE UART service
  bleuart.begin();

  // Set up and start advertising
  startAdv();
}
//-------END SETUP---------------------------------------------END SETUP

void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
}

//!! If you want to change the main loop code, please read all the comments below and in the loop !!
//To remove an effect, simply change its case # to anything greater than the total number of effects (999 for ex)
//if you want to know about certain effects, please see comments in the library for said effect
//if you want to know how segments work, please see comments in segmentSet.h

//The main loop of the program, works as follows:
//if effectsStop is true, effects are "off", so we won't try to start the next effect
//otherwise we jump to the effect matching the effectIndex value using a switch statment
//we also "clean up" a bit by reseting direct to true and the breakCurrentEffect flag to false
//if we don't find an effect with the effectIndex, we increment the effectIndex until we do
//while an effect is running, button inputs can either lock the effect or skip to the next effect
//if we lock the effect (set effectRota to false), we do not increment effectIndex when the effect ends, essentially restarting the effect (with new colors if they're randomly choosen)
//if the effect is skipped, we set strip.pixelStripStopPattern and breakCurrentEffect to true
//strip.pixelStripStopPattern will stop the current effect, and breakCurrentEffect will break out of the current switch statement if needed (the switch case has more than one effect)
//once the effect ends (either naturally or from a button press), we incremented effectIndex (as long as effectRota is set true)
//and jump to the top of the main loop

// Send a, b, c, or d over bleuart to enter that mode, then
// Send 0-9 over bleuart to select a pattern.
// Sending 0 while in mode a will not be the same pattern as 0 in mode b, etc.
uint8_t mode = 'a';
char const* response;

//---------MAIN LOOP-------------------------------------------MAIN LOOP
void loop() {  
  if ( Bluefruit.connected() && bleuart.notifyEnabled() )
  {
    int command = bleuart.read();
  
    switch (command) {
      case 'n': { // Next effect
        nextEffect();
        break;
      }
      case 'r': { // Effect rotation toggle
        effectRotaToggle();
        break;
      }
      case 'x': { // Brightness
        brightnessAdjust();
        break;
      }
      case 'a': { // Change mode to a
        mode = 'a';
        sendResponse("Changed to mode A.");
        break;
      }
      case 'b': { // Change mode to b
        mode = 'b';
        sendResponse("Changed to mode B.");
        break;
      }
      case 'c': { // Change mode to c
        mode = 'c';
        sendResponse("Changed to mode C.");
        break;
      }
      case 'd': { // Change mode to d
        mode = 'd';
        sendResponse("Changed to mode D.");
        break;
      }
      case 'e': { // Change mode to e
        mode = 'e';
        sendResponse("Changed to mode E.");
        break;
      }
      case '0': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 0; sendResponse("Changed to pattern 0."); break; }
          case 'b': { n = 10; sendResponse("Changed to pattern 10."); break; }
          case 'c': { n = 20; sendResponse("Changed to pattern 20."); break; }
          case 'd': { n = 30; sendResponse("Changed to pattern 30."); break; }
          case 'e': { n = 40; sendResponse("Changed to pattern 40."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
      case '1': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 1; sendResponse("Changed to pattern 1."); break; }
          case 'b': { n = 11; sendResponse("Changed to pattern 11."); break; }
          case 'c': { n = 21; sendResponse("Changed to pattern 21."); break; }
          case 'd': { n = 31; sendResponse("Changed to pattern 31."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
      case '2': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 2; sendResponse("Changed to pattern 2."); break; }
          case 'b': { n = 12; sendResponse("Changed to pattern 12."); break; }
          case 'c': { n = 22; sendResponse("Changed to pattern 22."); break; }
          case 'd': { n = 32; sendResponse("Changed to pattern 32."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
      case '3': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 3; sendResponse("Changed to pattern 3."); break; }
          case 'b': { n = 13; sendResponse("Changed to pattern 13."); break; }
          case 'c': { n = 23; sendResponse("Changed to pattern 23."); break; }
          case 'd': { n = 33; sendResponse("Changed to pattern 33."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
      case '4': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 4; sendResponse("Changed to pattern 4."); break; }
          case 'b': { n = 14; sendResponse("Changed to pattern 14."); break; }
          case 'c': { n = 24; sendResponse("Changed to pattern 24."); break; }
          case 'd': { n = 34; sendResponse("Changed to pattern 34."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
      case '5': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 5; sendResponse("Changed to pattern 5."); break; }
          case 'b': { n = 15; sendResponse("Changed to pattern 15."); break; }
          case 'c': { n = 25; sendResponse("Changed to pattern 25."); break; }
          case 'd': { n = 35; sendResponse("Changed to pattern 35."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
      case '6': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 6; sendResponse("Changed to pattern 6."); break; }
          case 'b': { n = 16; sendResponse("Changed to pattern 16."); break; }
          case 'c': { n = 26; sendResponse("Changed to pattern 26."); break; }
          case 'd': { n = 36; sendResponse("Changed to pattern 36."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
      case '7': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 7; sendResponse("Changed to pattern 7."); break; }
          case 'b': { n = 17; sendResponse("Changed to pattern 17."); break; }
          case 'c': { n = 27; sendResponse("Changed to pattern 27."); break; }
          case 'd': { n = 37; sendResponse("Changed to pattern 37."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
      case '8': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 8; sendResponse("Changed to pattern 8."); break; }
          case 'b': { n = 18; sendResponse("Changed to pattern 18."); break; }
          case 'c': { n = 28; sendResponse("Changed to pattern 28."); break; }
          case 'd': { n = 38; sendResponse("Changed to pattern 38."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
      case '9': {
        uint8_t n;
        switch (mode) {
          case 'a': { n = 9; sendResponse("Changed to pattern 9."); break; }
          case 'b': { n = 19; sendResponse("Changed to pattern 19."); break; }
          case 'c': { n = 29; sendResponse("Changed to pattern 29."); break; }
          case 'd': { n = 39; sendResponse("Changed to pattern 39."); break; }
        }
        strip.ActivePattern = (pattern)n;
        break;
      }
    }
  }
  
  // Update the strip
  strip.Update();

  /*if (!effectsStop) { //if effectsStop is true, we won't display any effect
    direct = true;
    breakCurrentEffect = false;
    //switch statment contains all effects
    //I'm not going to comment each one, as they're hard to describe
    //if an case has a loop, it generally means the effect will by run multiple times in diff directions
    //these will contain breakEffectCheck(); which will breakout of the case if the effect is skipped by button input
    //segmentName.flipSegDirectionEvery(1, true) means that the direction of the names segments will be reversed
    //this is used to change the direction of colorSpin effects

    switch (effectIndex) { //select the next effect based on the effectIndex
      case 0:
        strip.setRainbowOffsetCycle(40, false);
        strip.runRainbowOffsetCycle(true);
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          rowCenSegments.flipSegDirectionEvery(1, true);
          strip.colorSpin( rowCenSegments, spinPatternHeart, SIZE(spinPatternHeart), tempRandPallet, 0, 2, true, 100, 110 );
        }
        break;
      case 1:
        strip.shooterSeg( colSegments, pallet, 3, 0, true, 12, 1, 1, 3, 4, false, false, 60, 170);
        break;
      case 2:
        strip.shooterSeg( colSegments, pallet, 5, 0, true, 12, 1, 1, 3, 2, false, false, 60, 170); //rainbow
        break;
      case 3:
        strip.shooterSeg( rowHalfSegments, pallet, 3, 0, true, 10, 1, 1, 3, 4, true, false, 70, 170);
        break;
      case 4:
        strip.shooterSeg( rowHalfSegments, pallet, 5, 0, true, 10, 1, 1, 3, 2, true, false, 70, 170); //rainbow
        break;
      case 5:
        strip.shooterSeg( rowHalfSegments, pallet, 5, 0, true, 8, 1, 3, 3, 4, true, false, 70, 150);
        break;
      case 6:
        strip.shooterSeg( colSegments, pallet, 5, 0, true, 8, 1, 3, 3, 4, false, false, 70, 150);
        break;
      case 7:
        strip.shooterSeg( colHalfFlipSegments, pallet, 4, 0, true, 5, 6, 1, 6, 4, true, false, 70, 180);
        break;
      case 8:
        strip.genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
        strip.patternSweepSetRand( 9, tempRandPallet,  SIZE(tempRandPallet), 0, 1, 4, true, 0, 1, 60, 200);
        break;
      case 9:
        strip.doFireV2Seg( colSegments, firePallet, SIZE(firePallet), 25, 200, true, 300, 70);
        break;
      case 10:
        for (int i = 0; i < 3; i++) {
          breakEffectCheck();
          strip.patternSweepRepeatRand(4, 0, 0, 2, 4, false, false, 0, 0, 1, 55, 120 );
        }
        break;
      case 11:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          strip.gradientCycleRand( 5, 6, 150, direct, 100);
          direct = !direct;
        }
        break;
      case 12:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          strip.genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          strip.fillStrip(tempRandPallet[2], false);
          rowCenSegments.flipSegDirectionEvery(1, true);
          strip.colorSpin( rowCenSegments, spinPatternWave2, SIZE(spinPatternWave2), tempRandPallet, 0, 1, true, 100, 100 );
        }
        break;
      case 13:
        strip.sonarWavesRand( colSegments, 6, 0, 4, 8, false, false, true, true, 120, 110);
        break;
      case 14:
        strip.sonarWavesRand( rowSegments, 2, 0, 4, 8, false, true, true, false, 100, 110);
        break;
      case 15:
        strip.rainbowWave( colSegments, 80, true, 10, 25);
        break;
      case 16:
        strip.rainbowWave( rowSegments, 80, false, 8, 20);
        break;
      case 17:
        strip.waves( colSegments, pastelRainbowPallet, SIZE(pastelRainbowPallet),  pastelRainbowPattern, SIZE(pastelRainbowPattern), 50, false, 5, 45);
        break;
      case 18:
        for (int i = 0; i < 10; i++) {
          breakEffectCheck();
          strip.colorWipeRandomSeg( colSegments, 2, 2, 7, 90, true, true, true);
          breakEffectCheck();
          strip.colorWipeSeg(colSegments, 0, 7, 90, false, true, true);
        }
        break;
      case 19:
        for (int i = 0; i < 6; i++) {
          breakEffectCheck();
          uint32_t color = RC();
          strip.crossFadeColor(0, color, 30, 30);
          breakEffectCheck();
          strip.crossFadeColor(color, 0, 30, 30);
        }
        break;
      case 20:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          strip.randomWaves( colSegments, 5, 3, 2, 50, direct, 7, 20);
          direct = !direct;
        }
        break;
      case 21:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          strip.randomWaves( rowSegments, 4, 2, 2, 50, direct, 7, 20);
          direct = !direct;
        }
        break;
      case 22:
        for (int i = 0; i < 6; i++) {
          breakEffectCheck();
          strip.colorWipeRandomSeg( rowHalfSegments, 2, 2, 0, 130, direct, false, true);
          breakEffectCheck();
          strip.colorWipeSeg(rowHalfSegments, 0, 0, 130, !direct, false, true);
          direct = !direct;
        }
        break;
      case 23:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          colCenSegments.flipSegDirectionEvery(1, true);
          strip.genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          strip.fillStrip(tempRandPallet[2], false);
          strip.colorSpin( colCenSegments, spinPatternArrowsVert2, SIZE(spinPatternArrowsVert2), tempRandPallet, 0, 1, true, 100, 110 );
        }
        break;
      case 24:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          strip.genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          strip.fillStrip(tempRandPallet[1], false);
          rowHalfCenSegments.flipSegDirectionEvery(1, true); //reverse the direction of each segment of rowLensesHalvesSegments
          strip.colorSpin( rowHalfCenSegments, spinPatternArrows2, SIZE(spinPatternArrows2), tempRandPallet, 0, 1, true, 100, 100 );
        }
        break;
      case 25:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          strip.genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          strip.fillStrip(tempRandPallet[1], false);
          strip.colorSpin( rowHalfCenSegments, spinPatternX, SIZE(spinPatternX), tempRandPallet, 0, 1, true, 100, 110 );
          rowHalfCenSegments.flipSegDirectionEvery(1, true); //reverse the direction of each segment of rowLensesHalvesSegments
        }
        break;
      case 26:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          strip.genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
          tempRandPallet[0] = 0; //background color for spin patterns (off)
          strip.fillStrip(tempRandPallet[1], false);
          rowHalfCenSegments.flipSegDirectionEvery(1, true); //reverse the direction of each segment of rowLensesHalvesSegments
          strip.colorSpin( rowHalfCenSegments, spinPatternArrows, SIZE(spinPatternArrows), tempRandPallet, 0, 1, true, 100, 110 );
        }
        break;
      case 27:
        strip.colorSpinSimple( rowHalfSegments, 5, 0, 0, 3, -1, 3, 0, 1, 170, 120 );
        break;
      case 28:
        strip.setRainbowOffsetCycle(40, false);
        strip.runRainbowOffsetCycle(true);
        strip.colorSpinSimple( rowSegments, 1, 0, 0, 5, 1, 0, 0, 2, 170, 100 ); //rainbow half
        break;
      case 29:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          colHalfSegments.flipSegDirectionEvery(1, true);
          strip.colorSpinSimple( colHalfSegments, 5, 0, 0, 3, -1, 3, 0, 1, 120, 80 );
        }
        break;
      case 30:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          colHalfSegments.flipSegDirectionEvery(1, true);
          strip.colorSpinSimple( colHalfSegments, 3, 0, 0, 5, -1, 5, 0, 1, 120, 80 );
        }
        break;
      case 31:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          colHalfSegments.flipSegDirectionEvery(1, true);
          strip.colorSpinSimple( colHalfSegments, 1, 0, 0, 5, -1, 5, 0, 2, 120, 80 );
        }
        break;
      case 32:
        strip.simpleStreamerRand( 5, 0, 7, 0, 0, true, 160, 80);
        break;
      case 33:
        strip.setRainbowOffsetCycle(40, true);
        strip.runRainbowOffsetCycle(true);
        strip.patternSweepRand( 12, white, -1, 0, 0, false, 0, 1, 60, 280 );
        break;
      case 34:
        strip.genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
        strip.randomColorSet(off, true, tempRandPallet, SIZE(tempRandPallet), 100, 5, 20000);
        break;
      case 35:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          strip.segGradientCycleRand(rowSegments, 3, 7, 150, direct, 1, 100);
          direct = !direct;
        }
        break;
      case 36:
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          strip.segGradientCycleRand(colSegments, 3, 7, 150, direct, 1, 100);
          direct = !direct;
        }
        break;
      case 37:
        strip.genRandPallet( tempRandPallet, SIZE(tempRandPallet) );
        tempRandPallet[0] = 0; //background color for spin patterns (off)
        for (int i = 0; i < 2; i++) {
          breakEffectCheck();
          rowCenSegments.flipSegDirectionEvery(1, true);
          strip.colorSpin( rowCenSegments, spinPatternHelix, SIZE(spinPatternHelix), tempRandPallet, 0, 1, true, 100, 100 );
        }
        break;
      case 38:
        strip.colorSpin( rowCenSegments, spinPatternUSflag, SIZE(spinPatternUSflag), pallet, 0, 1, false, 130, 100 );
        break;
      case 39:
        tempTwinklePallet[0] = RC();
        tempTwinklePallet[1] = RC();
        strip.twinkleSet(0, tempTwinklePallet, SIZE(tempTwinklePallet), 2, 60, 35, 12000);
        break;
      case 40:
        //rainbow() only does one full cycle, we'll do 3 to extend its duration
        for (int i = 0; i < 3; i++) {
          breakEffectCheck();
          strip.rainbow(35);
        }
        break;
      default:
        //if we don't find an effect for the current effectIndex, we'll move to the next effect
        //this is for when an effect is set to always be skipped
        //incrementEffectIndex();
        break;
    }
    //if effectRota is true we'll advance to the effect index
    if (effectRota) {
      incrementEffectIndex();
    }
    strip.stripOff(); //clear the strip for the next effect
  }

  //strip.crossFadeCycleRand(5, 30, 30, 10);*/
}
//------END MAIN LOOP-----------------------------------END MAIN LOOP

//------------------------------------------------------------
//Completion Routines - get called on completion of a pattern
//------------------------------------------------------------

// Strip completion callback
void StripComplete()
{
  strip.Reverse();
}

//a quick shortening of the random color function, just to reduce the pattern function calls more readable
uint32_t RC() {
  return strip.randColor();
}

void sendResponse(char const *response) {
    bleuart.write(response, strlen(response)*sizeof(char));
    bleuart.write("\n", strlen("\n")*sizeof(char));
}
