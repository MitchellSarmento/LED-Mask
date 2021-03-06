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

//effects control vars
byte effectIndex = 0; //number of effect that's currently active (will be read from EEPROM later)
const byte numEffects = 41; //number of current effects - 1
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
byte brightnessIndex = 0; //initial brightness, index of brightnessLevels array
//brightness levels array, max is 255, but 100 should be bright enough for amost all cases
//!!WARNING brightness is directly tied to power consumption, the max current per led is 60ma, this is for white at 255 brightness
//if you actually run all the leds at max, the glasses will draw 4.75 amps, this is beyond the rating of the jst connectors
const byte brightnessLevels[] = { 10, 30, 140, 230 };
const byte numBrightnessLevels = SIZE( brightnessLevels );

// Speed vars
byte pulseSpeedIndex = 0;
const byte pulseSpeeds[] = {0.5, 1, 1.5, 2};
const byte numPulseSpeeds = SIZE(pulseSpeeds);

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
  //define pallet array, contains 32bit representations of all colors used in patterns
  uint32_t pallet[9] = { off, white, UCLAGold, UCLABlue, blue, yellow, red, green, purple };
  //                   { -0-, --1--, ---2----, ----3---, -4--, ---5--, -6-, --7--, --8-- }

  //pallet to match typical fairy light colors
  uint32_t christmasPallet[5] = { red, blue, green, yellow, purple };
  
  uint32_t pastelRainbowPallet[7] = { pastelRainbow, pastelRainbow1 , pastelRainbow2, pastelRainbow3, pastelRainbow4, pastelRainbow5, pastelRainbow6 };
  byte pastelRainbowPattern[14] = {  6, 6, 1, 1, 2, 2, 5, 5, 4, 4, 3, 3, 0, 0 };
  
  uint32_t firePallet[3] = { red, ltOrange, ltYellow };

  uint32_t kindredPallet[3] = {violet, blue, indigo};
  
  uint32_t firePallet2[3] = { purple, pink, white };
  
  public:
    //-------------COLORS-----------------------------------------------COLORS
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
    //-------------COLORS-----------------------------------------------COLORS

    int col1[5] = {0, 1, 2, 3, 4};
    int col2[7] = {11, 10, 9, 8, 7, 6, 5};
    int col3[7] = {12, 13, 14, 15, 16, 17, 18};
    int col4[9] = {27, 26, 25, 24, 23, 22, 21, 20, 19};
    int col5[11] = {28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38};
    int col6[13] = {51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39};
    int col7[13] = {52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64};
    int col8[11] = {75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65};
    int col9[9] = {76, 77, 78, 79, 80, 81, 82, 83, 84};
    int col10[7] = {91, 90, 89, 88, 87, 86, 85};
    int col11[7] = {92, 93, 94, 95, 96, 97, 98};
    int col12[5] = {103, 102, 101, 100, 99};
    int* columns[12] = {col1, col2, col3, col4, col5, col6, col7, col8, col9, col10, col11, col12};
  

    /*int columns[][] =
    {
      {0, 1, 2, 3, 4},
      {11, 10, 9, 8, 7, 6, 5},
      {12, 13, 14, 15, 16, 17, 18},
      {27, 26, 25, 24, 23, 22, 21, 20, 19},
      {28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38},
      {51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39},
      {52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64},
      {75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65},
      {76, 77, 78, 79, 80, 81, 82, 83, 84},
      {91, 90, 89, 88, 87, 86, 85},
      {92, 93, 94, 95, 96, 97, 98},
      {103, 102, 101, 100, 99}
    }*/
    int columnsSize[12] = {5, 7, 7, 9, 11, 13, 13, 11, 9, 7, 7, 5};
    
    // Member variables
    pattern ActivePattern; // Running pattern
    direction Direction; // direction to run the pattern

    unsigned long Interval; // ms between updates
    unsigned long lastUpdate; // Last update of position

    uint32_t Color1, Color2; // What colors are in use
    uint16_t TotalSteps; // Total number of steps in the pattern
    uint16_t Index; // current step within the pattern

    bool ReverseWhenComplete;
    unsigned long PulseSpeed = pulseSpeeds[pulseSpeedIndex];

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

    // Initialize Pattern 5 (Rainbow Pulse).
    void Pattern5(uint8_t interval, direction dir = FORWARD)
    {
      ActivePattern = PATTERN_5;
      Interval = interval;
      TotalSteps = 208;
      Index = 0;
      Direction = dir;
      ReverseWhenComplete = false;
    }

    // Update Pattern 5 (Rainbow Pulse).
    void Pattern5Update()
    {
      for (int i = 0; i < 12; i++)
      {
        int* col = columns[i];

        for (int j = 0; j < columnsSize[i]; j++)
        {
          uint32_t color = Wheel(((i * 256 / numPixels()) + Index) & 255);
          int pixel = col[j];

          if (j == Index || j == Index + 1) {
            setPixelColor(pixel, color);
          } else {
            setPixelColor(pixel, DimColor(DimColor(color)));
          }
          delay(PulseSpeed);
        }
      }
      show();
      Increment();
    }

    // Initialize Pattern 6 (Photon).
    void Pattern6(uint8_t interval, direction dir = FORWARD)
    {
      ActivePattern = PATTERN_5;
      Interval = interval;
      TotalSteps = 13;
      Index = 0;
      Direction = dir;
      ReverseWhenComplete = false;
      Color1 = yellow;
    }

    // Update Pattern 6 (Photon).
    void Pattern6Update()
    {
      for (int i = 0; i < 12; i++)
      {
        int* col = columns[i];

        for (int j = 0; j < columnsSize[i]; j++)
        {
          int pixel = col[j];

          if (pixel == Index) {
            setPixelColor(pixel, Color1);
          } else if (pixel == Index + 1) {
            setPixelColor(pixel, DimColor(Color1));
          } else if (pixel == Index + 2) {
            setPixelColor(pixel, DimColor(DimColor(Color1)));
          } else {
            setPixelColor(pixel, 0);
          }
          delay(PulseSpeed);
        }
      }
      show();
      Increment();
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

    // Initialize Pattern13.
    void Pattern13(uint8_t interval, direction dir = FORWARD)
    {
      ActivePattern = PATTERN_13;
      Interval = interval;
      TotalSteps = 20;
      Index = 0;
      Direction = dir;
      ReverseWhenComplete = false;
      Color1 = yellow;

      fill(0, 0, numPixels());
    }

    void Pattern13Update()
    {
      uint8_t onPixels[72] =
      {
        0, 1, 2, 3,
        5, 6, 7, 8, 9,
        12, 13, 14, 15, 16,
        19, 20, 21, 22, 23, 24,
        28, 29, 30, 31, 32, 33, 34,
        41, 42, 43, 44, 45, 46, 47,
        56, 57, 58, 59, 60, 61, 62,
        69, 70, 71, 72, 73, 74, 75,
        79, 80, 81, 82, 83, 84,
        87, 88, 89, 90, 91,
        94, 95, 96, 97, 98,
        100, 101, 102, 103
      };

      for (int i = 0; i < sizeof onPixels / sizeof onPixels[0]; i++)
      {
        setPixelColor(onPixels[i], Color1);
      }
      show();
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
      for (int i = 0; i < 10; i++) {
          breakEffectCheck();
          this->colorWipeSeg( colSegments, blue, 20, 20, true, true, true);
          breakEffectCheck();
          this->colorWipeSeg(colSegments, 0, 7, 10, false, true, true);
        }
    }

    void Pattern18Update()
    {
      for (int i = 0; i < 10; i++) {
          breakEffectCheck();
          this->colorWipeRandomSeg( colSegments, 2, 1, 7, 10, true, true, true);
          breakEffectCheck();
          this->colorWipeSeg(colSegments, 0, 7, 10, false, true, true);
        }
    }

    void Pattern19Update()
    {
      for (int i = 0; i < 10; i++) {
          breakEffectCheck();
          this->colorWipeRainbowSeg( colSegments, 10, 7, true, true, true, true);
          breakEffectCheck();
          this->colorWipeSeg(colSegments, 0, 7, 10, false, true, true);
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
      this->colorSpinSimple( rowHalfSegments, 5, 0, 0, 3, -1, 3, 0, 1, 170, 40 );
      // numColors, prefColor, BgColor, sweepLength, numSweeps, sweepSpacing, patternMode, colorMode, numCycles, wait
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

    // Initialize Pattern 33 (Pulse).
    void Pattern33(uint8_t interval, direction dir = FORWARD)
    {
      ActivePattern = PATTERN_33;
      Interval = interval;
      TotalSteps = 13;
      Index = 0;
      Direction = dir;
      ReverseWhenComplete = false;
      Color1 = yellow;
    }

    // Update Pattern 33 (Pulse).
    void Pattern33Update()
    {
      for (int i = 0; i < 12; i++)
      {
        int* col = columns[i];

        for (int j = 0; j < columnsSize[i]; j++)
        {
          int pixel = col[j];

          if (j == Index || j == Index + 1) {
            setPixelColor(pixel, Color1);
          } else {
            setPixelColor(pixel, DimColor(DimColor(Color1)));
          }
          delay(PulseSpeed);
        }
      }
      show();
      Increment();
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
      this->randomColorSet(off, true, kindredPallet, SIZE(kindredPallet), 100, 5, 20000);
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
PixelStripPatterns strip(stripLength, DATAPIN, CLOCKPIN, stripType, &StripComplete);

//triggered by button 3, sets the strip brightness to the next
//also stores the brighness index in eeprom
//brightness level in the brightnessLevels array (wrapping to the start if needed)
void brightnessAdjust() {
  brightnessIndex = (brightnessIndex + 1) % numBrightnessLevels;
  strip.setBrightness( brightnessLevels[brightnessIndex] );
  sendResponse("Changed brightness.");
}

void pulseSpeedAdjust() {
  pulseSpeedIndex = (pulseSpeedIndex + 1) % numPulseSpeeds;
  strip.PulseSpeed = pulseSpeeds[pulseSpeedIndex];
  sendResponse("Changed pulse speed.");
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

// Send a, b, c, or d over bleuart to enter that mode, then
// Send 0-9 over bleuart to select a pattern.
// Sending 0 while in mode a will not be the same pattern as 0 in mode b, etc.
uint8_t mode = 'a';
char const* response;

bool adjustingColor = false;
bool adjustingMainColor = false;

//---------MAIN LOOP-------------------------------------------MAIN LOOP
void loop() {  
  if ( Bluefruit.connected() && bleuart.notifyEnabled() )
  {
    int command = bleuart.read();

    if (adjustingColor) {
      switch (command) {
        case 'r': { // Set color red
          if (adjustingMainColor) {
            strip.Color1 = strip.red;
          } else {
            strip.Color2 = strip.red;
          }
          adjustingColor = false;
          sendResponse("Set color red.");
          break;
        }
        case 'o': { // Set color orange
          if (adjustingMainColor) {
            strip.Color1 = strip.orange;
          } else {
            strip.Color2 = strip.orange;
          }
          adjustingColor = false;
          sendResponse("Set color orange.");
          break;
        }
        case 'y': { // Set color yellow
          if (adjustingMainColor) {
            strip.Color1 = strip.yellow;
          } else {
            strip.Color2 = strip.yellow;
          }
          adjustingColor = false;
          sendResponse("Set color yellow.");
          break;
        }
        case 'g': { // Set color green
          if (adjustingMainColor) {
            strip.Color1 = strip.green;
          } else {
            strip.Color2 = strip.green;
          }
          adjustingColor = false;
          sendResponse("Set color green.");
          break;
        }
        case 'b': { // Set color blue
          if (adjustingMainColor) {
            strip.Color1 = strip.blue;
          } else {
            strip.Color2 = strip.blue;
          }
          adjustingColor = false;
          sendResponse("Set color blue.");
          break;
        }
        case 'i': { // Set color indigo
          if (adjustingMainColor) {
            strip.Color1 = strip.indigo;
          } else {
            strip.Color2 = strip.indigo;
          }
          adjustingColor = false;
          sendResponse("Set color indigo.");
          break;
        }
        case 'v': { // Set color violet
          if (adjustingMainColor) {
            strip.Color1 = strip.violet;
          } else {
            strip.Color2 = strip.violet;
          }
          adjustingColor = false;
          sendResponse("Set color violet.");
          break;
        }
        case 'x': { // Cancel color change
          adjustingColor = false;
          sendResponse("Cancelled.");
          break;
        }
      }
    } else {
      switch (command) {
        case 'x': { // Brightness
          brightnessAdjust();
          break;
        }
        case 'p': { // Pulse speed
          pulseSpeedAdjust();
          break;
        }
        case 'm': { // Main color (Color1)
          adjustingColor = true;
          adjustingMainColor = true;
          sendResponse("Adj Color1: ROYGBIV.");
          break;
        }
        case 's': { // Secondary color (Color2)
          adjustingColor = true;
          adjustingMainColor = false;
          sendResponse("Adj Color2: ROYGBIV.");
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
            case 'b': { n = 13; strip.Pattern13(3); sendResponse("Changed to pattern 13."); break; }
            case 'c': { n = 23; sendResponse("Changed to pattern 23."); break; }
            case 'd': { n = 33; strip.Pattern33(3); sendResponse("Changed to pattern 33."); break; }
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
            case 'a': { n = 5; strip.Pattern5(3); sendResponse("Changed to pattern 5."); break; }
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
            case 'a': { n = 6; strip.Pattern6(3); sendResponse("Changed to pattern 6."); break; }
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
  }
  
  // Update the strip
  strip.Update();
}
//------END MAIN LOOP-----------------------------------END MAIN LOOP

//------------------------------------------------------------
//Completion Routines - get called on completion of a pattern
//------------------------------------------------------------

// Strip completion callback
void StripComplete()
{
  if (strip.ReverseWhenComplete) {
    strip.Reverse();
  }
}

//a quick shortening of the random color function, just to reduce the pattern function calls more readable
uint32_t RC() {
  return strip.randColor();
}

void sendResponse(char const *response) {
    bleuart.write(response, strlen(response)*sizeof(char));
    bleuart.write("\n", strlen("\n")*sizeof(char));
}
