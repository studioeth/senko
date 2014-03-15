#define DEBUG 0 // 1 true / 0 false
#define NORMALMODE 0 // NORMALMODE uses RefreshAll() 
#define byte uint8_t

#define MAX_VELOCITY 2 // initial speed of the ball. 
#define DISAPPEAR_THRESHOLD 0.01
#define NUM_OF_X 41
#define NUM_OF_Y 8
#define X_AXIS_OFFSET 4
#define Y_AXIS_OFFSET 4

#define MAX_BALL_COUNT 12
#define MODERATION_RATIO 0.96 // TODO
#define MAX_BALL_AGE 500 // TODO
#define DEFAULT_BRIGHT_AGE 25 //TODO
#define MAX_BALL_BRIGHTNESS 50

#define MAX_BRIGHT_BRIGHTNESS 63 // depending on RefleshAll() 
#define MAX_BRIGHT_BRIGHTNESS_SLEEP 32
#define MAX_BRIGHT_COUNT 8

#define NUM_OF_FRAMES 6
#define NUM_OF_SENSOR 9

#define SCREEN_SAVER_START_MSEC 5000  // screen saver will be shown if there is no activity after specified seconds later.

Peggy2 frames[NUM_OF_FRAMES];     // Make a frame buffer object, called frames[0] 

unsigned char repNumber = 2;
bool isSleepMode = false;
bool shouldFadeInLoad = true;
unsigned long sleepStartMillis = 0;
  
struct Bright {
      bool isActive;
      uint8_t yp;
      uint8_t xp;
      int age;
      int spreadMax;
      bool isOneWay;
    };

struct Ball {
      float y;
      float x;
      float vy;
      float vx;
      bool isActive;
      uint8_t yp[3];
      uint8_t xp[3];
      int age;
    };

Ball balls [MAX_BALL_COUNT] = { 
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0},
  {0,0,0,0,false,{-1,-1,-1},{-1,-1,-1},0}
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

Sensor sensors[NUM_OF_SENSOR] = {
  //      y    x
  {0x01,  28,  3, false}, // い
  {0x02,  20,  3, false}, // ろ
  {0x03,  22, 4, false}, // は
  {0x04, 36,  3, false}, // に
  {0x05,  29,  4, false}, // ほ
  {0x06,  24,  3, false}, // へ
  {0x07,  32, 3, false}, // と
  {0x08, 34,  4, false}, // ち
  {0x09,  26,  4, false}  // り
};

