#define DEBUG 0 // 1 true / 0 false

#define byte uint8_t

#define MAX_VELOCITY 2 // initial speed of the ball. 
#define DISAPPEAR_THRESHOLD 0.01
#define NUM_OF_X 41
#define NUM_OF_Y 8
#define X_AXIS_OFFSET 4
#define Y_AXIS_OFFSET 4
#define MODERATION_RATIO 0.96

#define MAX_BALL_COUNT 8
#define MAX_BALL_AGE 1000
#define MAX_BALL_BRIGHTNESS 50

#define MAX_BRIGHT_AGE 200
#define MAX_BRIGHT_BRIGHTNESS 63 // depending on RefleshAll() 
#define MAX_BRIGHT_BRIGHTNESS_SLEEP 32
#define MAX_BRIGHT_COUNT 8

#define NUM_OF_FRAMES 6
#define NUM_OF_SENSOR 9

#define SCREEN_SAVER_START_MSEC 25000  // screen saver will be shown if there is no activity after specified seconds later.

Peggy2 frames[NUM_OF_FRAMES];     // Make a frame buffer object, called frames[0] 

unsigned short repNumber = 1;
bool isSleepMode = false;
bool shouldFadeInLoad = true;
unsigned long sleepStartMillis = 0;
  
struct Bright {
      bool isActive;
      uint8_t yp;
      uint8_t xp;
      int age;
    };

struct Ball {
      float y;
      float x;
      float vy;
      float vx;
      bool isActive;
      uint8_t yp;
      uint8_t xp;
      int age;
    };

Ball balls [MAX_BALL_COUNT] = { 
  {0,0,0,0,false,0,0,0},
  {0,0,0,0,false,0,0,0},
  {0,0,0,0,false,0,0,0},
  {0,0,0,0,false,0,0,0},
  {0,0,0,0,false,0,0,0},
  {0,0,0,0,false,0,0,0},
  {0,0,0,0,false,0,0,0},
  {0,0,0,0,false,0,0,0}
};

Bright brights [MAX_BRIGHT_COUNT] = { 
  {false,0,0,0},
  {false,0,0,0},
  {false,0,0,0},
  {false,0,0,0},
  {false,0,0,0},
  {false,0,0,0},
  {false,0,0,0},
  {false,0,0,0}
};

struct Sensor {
  byte id;
  uint8_t x;
  uint8_t y;
  bool isRequested;
};

// TODO x and y should be changed according to actual sensor position.
Sensor sensors[NUM_OF_SENSOR] = {
  {0x01,  7,  0, false}, // い
  {0x02,  0,  8, false}, // ろ
  {0x03,  7, 15, false}, // は
  {0x04, 15,  8, false}, // に
  {0x05,  7,  4, false}, // ほ
  {0x06,  3,  8, false}, // へ
  {0x07,  8, 12, false}, // と
  {0x08, 11,  8, false}, // ち
  {0x09,  7,  8, false}  // り
};

