/* 
  oxoxo [zero by zero] Senko for Peggy 2.0, using the Peggy2 library, version 0.2. 
  http://oxoxo.me
*/

#include <Peggy2.h>
#include <math.h> 
#include <Wire.h>
#include "params.h"
#include "axis.h"

void setup()
{
  frames[0].HardwareInit();

//  DDRB = 254;	//All outputs except B0, the "OFF/SELECT" button.
//  PORTB |= 1;   // Turn on PortB Pin 0 pull up!

  Wire.begin(2);        // join i2c bus (address optional for master)
  Wire.onReceive(receiveEvent); // register event

   if(DEBUG){
     //Serial.begin(19200);
   }
}

void receiveEvent(int howMany)
{
  while(Wire.available()){
    byte sensorId = Wire.read();

    for(int i=0; i < NUM_OF_SENSOR; i++){
      if(sensors[i].id == sensorId){
        sensors[i].isRequested = true;
        break;
      }
    }
  }
}

void loop()
{ 
  clearFrames();
  
  for(int i=0; i < NUM_OF_SENSOR; i++){
    if(sensors[i].isRequested){
      Ball* pBall = createBall(sensors[i].x, sensors[i].y); // might be ignored if no ball is available.
      if(pBall != NULL){
        fireBright(pBall);
      }
      sensors[i].isRequested = false;
    }
  } 

  for(int i=0; i < MAX_BALL_COUNT; i++){
    updateBall(&balls[i]);
  }
/*
  if(isSleep()){
  //if(true){
    if(!isSleepMode){
      // start Sleeping
      isSleepMode = true;
      shouldFadeInLoad = true;
      sleepStartMillis = millis();
    }
    else {
      drawScreenSaver();
    }
  } else 
  */
  {
    // active //
    isSleepMode = false;
    //frames[0].Clear();
    
    for(int i=0; i < MAX_BALL_COUNT; i++){
      drawBall(&balls[i]);
    }
    
    
    for(int i=0; i < MAX_BALL_COUNT; i++){
      for(int j=i+1; j < MAX_BALL_COUNT; j++){
        if(balls[i].xp == balls[j].xp && balls[i].yp == balls[j].yp
          && balls[i].isActive && balls[j].isActive ){
          fireBright(&balls[i]);
          break;
        }
      }
    }
    drawBright();
  }
  refreshAll();
}

/*
boolean isSleep(){
  for(int i=0; i < MAX_BALL_COUNT; i++){
    if(balls[i].isActive){
      return false;
    }
  }
  for(int i=0; i < MAX_BRIGHT_COUNT; i++){
    if(brights[i].isActive){
      return false;
    }
  }
  return true;
}

void drawScreenSaver(){
  unsigned long sleepDuration = millis() - sleepStartMillis;
  if(sleepDuration > SCREEN_SAVER_START_MSEC){
    // TODO show screen saver.
    int brightness = MAX_BRIGHT_BRIGHTNESS_SLEEP * 0.5 * ((-1 * cos((sleepDuration - SCREEN_SAVER_START_MSEC) / (double)5000)) + 1);
    if(brightness >= MAX_BRIGHT_BRIGHTNESS_SLEEP - 1){
       shouldFadeInLoad = false;
    }
    for(unsigned short x=0; x < NUM_OF_X; x++){
      for(unsigned short y=0; y < NUM_OF_Y; y++){
        if(!shouldFadeInLoad && (x == 7 || y == 7 || x == 8 || y == 8)){
          setPointWithBrightness(x,y,MAX_BRIGHT_BRIGHTNESS_SLEEP);
        }else{
          setPointWithBrightness(x,y,MAX_BRIGHT_BRIGHTNESS_SLEEP);
        }
      }
    }
  }
}
  */

void drawBright(){
  for(int i= 0; i < MAX_BRIGHT_COUNT; i++){
    if(brights[i].isActive){
      int brightness = calcBallBrightnessFromAge(brights[i].age, MAX_BRIGHT_BRIGHTNESS, MAX_BRIGHT_AGE);
      setPointWithBrightness(brights[i].xp, brights[i].yp, brightness); 
      brights[i].age++;
      if(brights[i].age < NUM_OF_X){
        drawCircle(brights[i].xp, brights[i].yp, brights[i].age, brightness);
      }
    }
    if(brights[i].age > MAX_BRIGHT_AGE){
      // delete Bright.
      brights[i].isActive = false;
      brights[i].age = 0;
    }
  }
}

struct Ball* createBall(uint8_t x, uint8_t y){
  
    Ball* pBall = findUnusedBall(); // find unused ball.
    if(pBall == NULL){
      return NULL;
    }
    
    //Set first velocity
    pBall->vx = ( MAX_VELOCITY * random(-100, 100)) / 100.0;
    int vec = random(0, 1);
    if(vec == 0){
      vec = -1;
    }
    pBall->vy = sqrt(sq(MAX_VELOCITY) - sq(pBall->vx)) * vec;

    //pBall->x = random(NUM_OF_X);   // Initial position: up to NUM_OF_X - 1.
    //pBall->y = random(NUM_OF_X);   // Initial position: up to NUM_OF_X - 1.
    pBall->x = x;
    pBall->y = y;
    pBall->xp = (uint8_t) round(pBall->x);
    pBall->yp = (uint8_t) round(pBall->y);
  
    pBall->isActive = true;  
    return pBall;
}

struct Ball* findUnusedBall(){
  for(int i= 0; i < MAX_BALL_COUNT; i++){
    if(!balls[i].isActive){
      return &balls[i];
    }
  }
  return NULL;
}

void deleteBall(struct Ball* pBall){
  pBall->isActive = false;
  pBall->age = 0;
}

void drawBall(struct Ball* pBall){
  if(!pBall->isActive) return;

  //Next, figure out which point we're going to draw. 
  pBall->xp = (uint8_t) round(pBall->x);
  pBall->yp = (uint8_t) round(pBall->y);
  
  // Write the point to the buffer
  int brightness = calcBallBrightnessFromAge(pBall->age, MAX_BALL_BRIGHTNESS, MAX_BALL_AGE);
  setPointWithBrightness(pBall->xp , pBall->yp, brightness);  
}

int calcBallBrightnessFromAge(int age, int maxBrightness, int maxAge){
  float power = maxAge - age;
  int brightness = floor(( maxBrightness * power ) / float(maxAge));
  if(brightness > maxBrightness){
    brightness = maxBrightness;
  }
  return brightness;
}

void updateBall(struct Ball* pBall){
    float Xnew, Ynew, VxNew, VyNew;

  if(!pBall->isActive) return;
  
  Xnew = pBall->x + pBall->vx;
  Ynew = pBall->y + pBall->vy;

  VyNew = pBall->vy;
  VxNew = pBall->vx;

  // Bounce at walls

  if (Xnew < 0){
    VxNew = -1 * VxNew * MODERATION_RATIO;
    Xnew = 0;
  }

  if (Xnew >= NUM_OF_X - 1){
    VxNew *= -MODERATION_RATIO;
    Xnew = NUM_OF_X - 1;
  }

  if (Ynew < 0) {
    VyNew = -1 * VyNew * MODERATION_RATIO;
    Ynew = 0;
  }

  if (Ynew >= NUM_OF_Y - 1){
    VyNew = -1 * VyNew * MODERATION_RATIO;
    Ynew = NUM_OF_Y - 1;
  }

//  if(sq(VyNew) < DISAPPEAR_THRESHOLD && sq(VxNew) < DISAPPEAR_THRESHOLD){
  if(pBall->age > MAX_BALL_AGE){
    deleteBall(pBall);
  }

  //Age variables for the next iteration
  pBall->vx = VxNew;
  pBall->vy = VyNew;

  pBall->x = Xnew;
  pBall->y = Ynew; 
  
  pBall->age++;

}

void fireBright(struct Ball* pBall){
  for(int i= 0; i < MAX_BRIGHT_COUNT; i++){
    if(!brights[i].isActive){
       brights[i].xp = pBall->xp;
       brights[i].yp = pBall->yp;
       brights[i].isActive = true;
       brights[i].age = 0;
       break;
    }
  }
   // TODO it is better to replace with oldest one when the array is full.
}

void clearFrames(){
  for(int i=0; i < NUM_OF_FRAMES; i++){
    frames[i].Clear();
  }
}

void setPointWithBrightness(unsigned short xIn, unsigned short yIn, unsigned short brightness){
 
  unsigned short x,y;
  if(xIn >= NUM_OF_X || yIn >= NUM_OF_Y){
    return;
  }
  // transform the point to roppongi format.
  Point pt;
  if( yIn < 8 && xIn < 41 ){
    pt = rMatrix[yIn][xIn];
  }else {
    return;
  }
  y = pt.x;
  x = pt.y;

  if(DEBUG){
    Serial.print("\n xIn: ");
    Serial.print(xIn);
    Serial.print(" yIn: ");
    Serial.print(yIn);
    Serial.print("\n   x: ");
    Serial.print(x);
    Serial.print("   y: ");
    Serial.print(y);
  }

  x += X_AXIS_OFFSET;
  y += Y_AXIS_OFFSET;


  if(NORMALMODE){
    if(brightness > 0){
      frames[0].WritePoint( x, y, 1);
    }else{
      frames[0].WritePoint( x, y, 0);
    }    
  }else{
    frames[0].WritePoint( x, y, brightness & 1);
    frames[1].WritePoint( x, y, brightness & 2);
    frames[2].WritePoint( x, y, brightness & 4);
    frames[3].WritePoint( x, y, brightness & 8);
    frames[4].WritePoint( x, y, brightness & 16);
    frames[5].WritePoint( x, y, brightness & 32);
    // frames[6].WritePoint( x, y, brightness & 64);
  }
}

void refreshAll(){
  
  unsigned short reps = 0;  
  while (reps < repNumber)
  {
    if(NORMALMODE){
      frames[0].RefreshAll(50); //Draw frame buffer 1 time
    }
    else{
      frames[0].RefreshAllFast(1); //Draw frame buffer 1 time
      frames[1].RefreshAllFast(2); //Draw frame buffer 2 times
      frames[2].RefreshAllFast(4); //Draw frame buffer 4 times
      frames[3].RefreshAllFast(8); //Draw frame buffer 8 times 
      frames[4].RefreshAllFast(16); //Draw frame buffer 4 times 
      frames[5].RefreshAllFast(32); //Draw frame buffer 4 times 
     // frames[6].RefreshAllFast(64); //Draw frame buffer 4 times 
    }
    reps++;
  }
}

void drawCircle(int8_t xCenter, int8_t yCenter, int8_t radius, int brightness){
  int8_t x = 0;
  int8_t y = radius;
  int8_t p = (5 - radius*4)/4;
  drawCirclePoint(xCenter, yCenter, x, y, brightness);

  while (x < y){
    x++;
    if (p < 0){
      p += 2*x+1;
    }else{
      y--;
      p += 2*(x-y)+1;
    }
    drawCirclePoint(xCenter, yCenter, x, y, brightness);
  }
}

void drawCirclePoint(int8_t cx, int8_t cy, int8_t x, int8_t y, int brightness){
  if (x == 0){
    setPointWithBrightness(cx, cy + y, brightness);
    setPointWithBrightness(cx, cy - y, brightness);
    setPointWithBrightness(cx + y, cy, brightness);
    setPointWithBrightness(cx - y, cy, brightness);
  }else if (x == y){
    setPointWithBrightness(cx + x, cy + y, brightness);
    setPointWithBrightness(cx - x, cy + y, brightness);
    setPointWithBrightness(cx + x, cy - y, brightness);
    setPointWithBrightness(cx - x, cy - y, brightness);
  }else if (x < y){
    setPointWithBrightness(cx + x, cy + y, brightness);
    setPointWithBrightness(cx - x, cy + y, brightness);
    setPointWithBrightness(cx + x, cy - y, brightness);
    setPointWithBrightness(cx - x, cy - y, brightness);
    setPointWithBrightness(cx + y, cy + x, brightness);
    setPointWithBrightness(cx - y, cy + x, brightness);
    setPointWithBrightness(cx + y, cy - x, brightness);
    setPointWithBrightness(cx - y, cy - x, brightness);
  }
}

