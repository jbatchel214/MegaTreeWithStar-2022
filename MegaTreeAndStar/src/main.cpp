#include <Arduino.h>
#include <FastLED.h>
#include "driver/adc.h"

void clamp(int &val, int min, int max);
CRGB blendRGB(CRGB & startRGB, CRGB & endRGB, float pcnt);
bool angleInRange(float val, float minT, float maxT);
void SetupStar();
void SetupXMASTree();
void SetStarHSV(CHSV hsvLoc);
void SetTreeHSV(CHSV hsvLoc);
void TreeColorDrops(CHSV hsvBase);
//void TreeSpiralSolid(CHSV hsvBase, float theta, float dTheta, float omega, CHSV spiral);
void TreeSpiralRainbow(CHSV hsvBase, float theta, float dTheta, float omega, uint8_t spiralType, CHSV spiralStart, float spiraldTheta);
void TreeMeteors(CHSV hsvBase);
void CyclePixelColors(uint8_t deltaVal);
void CyclePointColors(uint8_t shiftVal, uint8_t deltaVal);

#define BRIGHTNESS 255
#define starValue 255
#define treeValue 155

#define STARPIN 27
#define NUMSTARLEDS 113

#define STRANDPIN00 16
#define STRANDPIN01 17
#define STRANDPIN02 18
#define STRANDPIN03 19
#define STRANDPIN04 21
#define STRANDPIN05 22
#define STRANDPIN06 23
#define STRANDPIN07 25
#define STRANDPIN08 26
#define NUMSTRANDLEDS 65
#define NUMSTRANDS 27
#define NUMSTRANDSPERPIN 3
#define NUMSTRANDPINS NUMSTRANDS/NUMSTRANDSPERPIN
#define NUMSTRANDLEDSPERPIN NUMSTRANDSPERPIN * NUMSTRANDLEDS
#define NUMTREELEDS NUMSTRANDS*NUMSTRANDLEDS

/* LED Arrays */
CRGB starLEDs[NUMSTARLEDS];
CRGB strandLEDsWired[NUMSTRANDPINS][NUMSTRANDLEDSPERPIN]; // these are as-wired, so bottom to top, top to bottom, bottom to top

/* universal constants */
float pi = 2.0*acosf(0.0);
float d2r = pi/180;
float r2d = 180/pi;


/* define geometries */

// define tree geometry
float firstAngle_rad = -130.0*d2r;
float lastAngle_rad = 130.0*d2r;
float baseDiam_in = 120.0;
float topDiam_in = 12.0;
float spaceInit_in = 3.0;
float spaceFinal_in = 3.0;
float gapPerLights_in = 3.0;
float vertexHeight_in = 0.0;
float treeX[NUMSTRANDPINS][NUMSTRANDLEDSPERPIN]; // x is towards road
float treeY[NUMSTRANDPINS][NUMSTRANDLEDSPERPIN]; // y is towards driveway
float treeZ[NUMSTRANDPINS][NUMSTRANDLEDSPERPIN]; // z is vertical up
float treeR[NUMSTRANDPINS][NUMSTRANDLEDSPERPIN]; // radially from center
float treeL[NUMSTRANDPINS][NUMSTRANDLEDSPERPIN]; // Length along strand
float treeT[NUMSTRANDPINS][NUMSTRANDLEDSPERPIN]; // Theta, measured from +X towards +Y (about +Z)
int treeLEDMap[NUMSTRANDS][NUMSTRANDLEDS][2]; // index map of each strand, sorted bottom to top per strand - useful for following something down a strand



// define star geoemtry
int starPoint[NUMSTARLEDS] = {0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 17, 17, 17, 24, 24, 24, 24, 24, 23, 23, 23, 22, 22, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 20, 19, 19, 19, 18, 18, 18, 18, 18, 25, 25, 25, 25};
int starPointType[NUMSTARLEDS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int starLocation[NUMSTARLEDS] = {0, 4, 1, 2, 3, 0, 2, 1, 4, 3, 10, 2, 1, 0, 1, 2, 3, 4, 0, 2, 1, 0, 4, 3, 2, 1, 2, 1, 0, 0, 4, 3, 2, 1, 2, 1, 0, 0, 4, 3, 2, 1, 0, 2, 1, 4, 3, 0, 1, 2, 3, 4, 0, 3, 2, 1, 4, 0, 1, 4, 3, 2, 0, 3, 2, 1, 4, 0, 4, 3, 2, 1, 0, 2, 3, 4, 1, 2, 1, 0, 0, 2, 3, 4, 1, 0, 2, 1, 0, 4, 3, 2, 1, 0, 2, 1, 0, 2, 3, 4, 1, 2, 0, 1, 0, 4, 3, 2, 1, 4, 3, 2, 1};
float starRadius[NUMSTARLEDS] = {8.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 8.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000, 5.000};
float starAzimuth[NUMSTARLEDS] = {0.000, 270.000, 0.000, 90.000, 180.000, 315.000, 326.250, 303.750, 300.000, 330.000, 270.000, 281.250, 258.750, 225.000, 213.750, 236.250, 240.000, 210.000, 180.000, 191.250, 168.750, 135.000, 120.000, 150.000, 146.250, 123.750, 101.250, 78.750, 90.000, 45.000, 30.000, 60.000, 56.250, 33.750, 11.250, 348.750, 0.000, 315.000, 303.750, 326.250, 326.250, 303.750, 270.000, 281.250, 258.750, 258.750, 281.250, 225.000, 213.750, 236.250, 236.250, 213.750, 180.000, 191.250, 191.250, 168.750, 168.750, 135.000, 123.750, 123.750, 146.250, 146.250, 90.000, 101.250, 101.250, 78.750, 78.750, 45.000, 33.750, 56.250, 56.250, 33.750, 0.000, 11.250, 11.250, 348.750, 348.750, 11.250, 0.000, 348.750, 315.000, 330.000, 326.250, 303.750, 300.000, 258.750, 281.250, 270.000, 225.000, 213.750, 236.250, 240.000, 210.000, 168.750, 191.250, 180.000, 135.000, 150.000, 146.250, 123.750, 120.000, 101.250, 78.750, 90.000, 45.000, 33.750, 56.250, 60.000, 30.000, 180.000, 270.000, 0.000, 90.000};
float starElevation[NUMSTARLEDS] = {0.000, 15.000, 15.000, 15.000, 15.000, 45.000, 60.000, 60.000, 30.000, 30.000, 30.000, 60.000, 60.000, 45.000, 60.000, 60.000, 30.000, 30.000, 30.000, 60.000, 60.000, 45.000, 30.000, 30.000, 60.000, 60.000, 60.000, 60.000, 30.000, 45.000, 30.000, 30.000, 60.000, 60.000, 60.000, 60.000, 30.000, 90.000, 75.000, 75.000, 105.000, 105.000, 90.000, 105.000, 105.000, 75.000, 75.000, 90.000, 105.000, 105.000, 75.000, 75.000, 90.000, 75.000, 105.000, 105.000, 75.000, 90.000, 105.000, 75.000, 75.000, 105.000, 90.000, 75.000, 105.000, 105.000, 75.000, 90.000, 75.000, 75.000, 105.000, 105.000, 90.000, 105.000, 75.000, 75.000, 105.000, 120.000, 150.000, 120.000, 135.000, 150.000, 120.000, 120.000, 150.000, 120.000, 120.000, 150.000, 135.000, 120.000, 120.000, 150.000, 150.000, 120.000, 120.000, 150.000, 135.000, 150.000, 120.000, 120.000, 150.000, 120.000, 120.000, 150.000, 135.000, 120.000, 120.000, 150.000, 150.000, 165.000, 165.000, 165.000, 165.000};
float starX[NUMSTARLEDS] = {0.000, 0.000, 1.294, 0.000, -1.294, 4.000, 3.600, 2.406, 1.250, 2.165, 0.000, 0.845, -0.845, -4.000, -3.600, -2.406, -1.250, -2.165, -2.500, -4.247, -4.247, -4.000, -1.250, -2.165, -3.600, -2.406, -0.845, 0.845, 0.000, 4.000, 2.165, 1.250, 2.406, 3.600, 4.247, 4.247, 2.500, 5.657, 2.683, 4.016, 4.016, 2.683, 0.000, 0.942, -0.942, -0.942, 0.942, -5.657, -4.016, -2.683, -2.683, -4.016, -8.000, -4.737, -4.737, -4.737, -4.737, -5.657, -2.683, -2.683, -4.016, -4.016, 0.000, -0.942, -0.942, 0.942, 0.942, 5.657, 4.016, 2.683, 2.683, 4.016, 8.000, 4.737, 4.737, 4.737, 4.737, 4.247, 2.500, 4.247, 4.000, 2.165, 3.600, 2.406, 1.250, -0.845, 0.845, 0.000, -4.000, -3.600, -2.406, -1.250, -2.165, -4.247, -4.247, -2.500, -4.000, -2.165, -3.600, -2.406, -1.250, -0.845, 0.845, 0.000, 4.000, 3.600, 2.406, 1.250, 2.165, -1.294, 0.000, 1.294, 0.000};
float starY[NUMSTARLEDS] = {0.000, -1.294, 0.000, 1.294, 0.000, -4.000, -2.406, -3.600, -2.165, -1.250, -2.500, -4.247, -4.247, -4.000, -2.406, -3.600, -2.165, -1.250, 0.000, -0.845, 0.845, 4.000, 2.165, 1.250, 2.406, 3.600, 4.247, 4.247, 2.500, 4.000, 1.250, 2.165, 3.600, 2.406, 0.845, -0.845, 0.000, -5.657, -4.016, -2.683, -2.683, -4.016, -8.000, -4.737, -4.737, -4.737, -4.737, -5.657, -2.683, -4.016, -4.016, -2.683, 0.000, -0.942, -0.942, 0.942, 0.942, 5.657, 4.016, 4.016, 2.683, 2.683, 8.000, 4.737, 4.737, 4.737, 4.737, 5.657, 2.683, 4.016, 4.016, 2.683, 0.000, 0.942, 0.942, -0.942, -0.942, 0.845, 0.000, -0.845, -4.000, -1.250, -2.406, -3.600, -2.165, -4.247, -4.247, -2.500, -4.000, -2.406, -3.600, -2.165, -1.250, 0.845, -0.845, 0.000, 4.000, 1.250, 2.406, 3.600, 2.165, 4.247, 4.247, 2.500, 4.000, 2.406, 3.600, 2.165, 1.250, 0.000, -1.294, 0.000, 1.294};
float starZ[NUMSTARLEDS] = {8.000, 4.830, 4.830, 4.830, 4.830, 5.657, 2.500, 2.500, 4.330, 4.330, 4.330, 2.500, 2.500, 5.657, 2.500, 2.500, 4.330, 4.330, 4.330, 2.500, 2.500, 5.657, 4.330, 4.330, 2.500, 2.500, 2.500, 2.500, 4.330, 5.657, 4.330, 4.330, 2.500, 2.500, 2.500, 2.500, 4.330, 0.000, 1.294, 1.294, -1.294, -1.294, 0.000, -1.294, -1.294, 1.294, 1.294, 0.000, -1.294, -1.294, 1.294, 1.294, 0.000, 1.294, -1.294, -1.294, 1.294, 0.000, -1.294, 1.294, 1.294, -1.294, 0.000, 1.294, -1.294, -1.294, 1.294, 0.000, 1.294, 1.294, -1.294, -1.294, 0.000, -1.294, 1.294, 1.294, -1.294, -2.500, -4.330, -2.500, -5.657, -4.330, -2.500, -2.500, -4.330, -2.500, -2.500, -4.330, -5.657, -2.500, -2.500, -4.330, -4.330, -2.500, -2.500, -4.330, -5.657, -4.330, -2.500, -2.500, -4.330, -2.500, -2.500, -4.330, -5.657, -2.500, -2.500, -4.330, -4.330, -4.830, -4.830, -4.830, -4.830};



int starPattern = 1;
int treePattern = 3;
bool treeTransition = false;

void setup() {
  // put your setup code here, to run once:
  adc_power_acquire(); // this was done because of an interrupt issue causing flickering

  //Serial.begin(115200);

  // setup star geometry
  //Serial.println("Starting Tree Setup");
  // setup tree geometry
  SetupXMASTree();
  //Serial.println("Finished Tree Setup");

  FastLED.addLeds<WS2811, STARPIN, RGB>(starLEDs, NUMSTARLEDS);
  //FastLED.addLeds<WS2811, STRANDPIN01, RGB>(strandLEDsIdeal[0], NUMSTRANDLEDS);
  FastLED.addLeds<WS2811, STRANDPIN00, RGB>(strandLEDsWired[0], NUMSTRANDLEDSPERPIN);
  FastLED.addLeds<WS2811, STRANDPIN01, RGB>(strandLEDsWired[1], NUMSTRANDLEDSPERPIN);
  FastLED.addLeds<WS2811, STRANDPIN02, RGB>(strandLEDsWired[2], NUMSTRANDLEDSPERPIN);
  FastLED.addLeds<WS2811, STRANDPIN03, RGB>(strandLEDsWired[3], NUMSTRANDLEDSPERPIN);
  FastLED.addLeds<WS2811, STRANDPIN04, RGB>(strandLEDsWired[4], NUMSTRANDLEDSPERPIN);
  FastLED.addLeds<WS2811, STRANDPIN05, RGB>(strandLEDsWired[5], NUMSTRANDLEDSPERPIN);
  FastLED.addLeds<WS2811, STRANDPIN06, RGB>(strandLEDsWired[6], NUMSTRANDLEDSPERPIN);
  FastLED.addLeds<WS2811, STRANDPIN07, RGB>(strandLEDsWired[7], NUMSTRANDLEDSPERPIN);
  FastLED.addLeds<WS2811, STRANDPIN08, RGB>(strandLEDsWired[8], NUMSTRANDLEDSPERPIN);
  
  // set the global brightness parameter
  FastLED.setBrightness(BRIGHTNESS);

}

void loop() {
  static unsigned long dTime = 20;
  static unsigned long frms = 0;
  const static unsigned long maxFrms = 1000;
  const static int transitionFrms = 40;
  const static int transitionRate = BRIGHTNESS/(transitionFrms-1);
  //static int transitionFrameCount = -transitionFrms;

  switch (starPattern) {
    case 0: {// solid star, vary hue vs time
      static uint8_t frameCnt = 0;
      static CHSV hsvLoc = {0, 255, starValue};
      SetStarHSV(hsvLoc);
      frameCnt++;
      if(frameCnt > 2) {
        hsvLoc.h++;
        frameCnt = 0;
      }
      break;
    }
    case 1: {// individual points cycle their own colors
      static uint8_t shiftVal = 10; // shift from time step to time step
      static uint8_t deltaVal = 70; // delta from point to point
      CyclePointColors(shiftVal, deltaVal);
      break;
    }
    case 2: { // cycle all pixels uniquely
      static uint8_t deltaVal = 70; // delta from point to point
      CyclePixelColors(deltaVal);
      break;
    }
    default: {
      starPattern = 0;
      break;
    }
  }
  //FastLED.show();
  
  switch (treePattern) {
    case 0: {// solid tree, vary hue vs time
      static CHSV hsvLoc = {0, 255, treeValue};
      SetTreeHSV(hsvLoc);
      hsvLoc.h++;
      break;
    }
    case 1: { // solid tree, but "drops" of color
      static CHSV hsvBase = rgb2hsv_approximate(CRGB::DarkGreen);
      TreeColorDrops(hsvBase);
      break;
    }
    case 2: { // tree spiral
      static CHSV hsvBase = rgb2hsv_approximate(CRGB::DarkGreen);
      static CHSV spiral = rgb2hsv_approximate(CRGB::Purple);
      static float theta_rad = 0.0*d2r;
      static float deltaTheta_rad = 180.0*d2r;
      static float omega_rps = 10.0*d2r;
      //theta_rad = fmodf(theta_rad+pi, 2*pi)-pi;
      TreeSpiralRainbow(hsvBase, theta_rad, deltaTheta_rad, omega_rps, 0, spiral, 0.0);
      theta_rad += 3.0*d2r;
      break;
    }//*/
    case 3: { // rainbow spiral
      static CHSV hsvBase = rgb2hsv_approximate(CRGB::DarkGreen);
      static CHSV spiral = {0,255,treeValue};
      static float theta_rad = 0.0*d2r;
      static float deltaTheta_rad = 180.0*d2r;
      static float omega_rps = 10.0*d2r;
      //theta_rad = fmodf(theta_rad+pi, 2*pi)-pi;
      TreeSpiralRainbow(hsvBase, theta_rad, deltaTheta_rad, omega_rps, 1, spiral, 4.0);
      theta_rad += 3.0*d2r;
      spiral.h+=10;
      break;
    }//*/
    default: {
      treePattern = 0;
      break;
    }
  }

  frms++;
  int tmpBrightness = BRIGHTNESS;
  if (frms<transitionFrms){ // initial ramp up in brightness
    tmpBrightness = transitionRate * frms;
    clamp(tmpBrightness, 0, 255);
  }
  else if (frms<transitionFrms + maxFrms) { // constant brightness
    tmpBrightness = BRIGHTNESS;
  }
  else if (frms<transitionFrms + maxFrms + transitionFrms) { // rampdown to transition
    int transitionCnt = transitionFrms - (frms - transitionFrms - maxFrms);
    tmpBrightness = transitionRate * transitionCnt;
    clamp(tmpBrightness, 0, 255);
    FastLED.setBrightness(tmpBrightness);
  } else {
    tmpBrightness = 0;
    treePattern++;
    frms = 0;
    //Serial.printf(" New Tree Pattern:  %d\n", treePattern);
  }
  //Serial.printf("Frame:  %d     Brightness:  %d\n", frms, tmpBrightness);
  FastLED.setBrightness(tmpBrightness);

  FastLED.show();
  FastLED.delay(dTime);

}


/* Utility Functions */
void clamp(int &val, int min, int max) {
  if (val < min) {
    val = min;
  }
  else if (val > max) {
    val = max;
  }
}

CRGB blendRGB(CRGB & startRGB, CRGB & endRGB, float pcnt) {
  CRGB output;
  // Linearly interpolates in R, G, B color space between 2 colors
  int tmpRed   = int(float(endRGB.r - startRGB.r)*pcnt + float(startRGB.r) + 0.5);
  int tmpGreen = int(float(endRGB.g - startRGB.g)*pcnt + float(startRGB.g) + 0.5);
  int tmpBlue  = int(float(endRGB.b - startRGB.b)*pcnt + float(startRGB.b) + 0.5);
  clamp(tmpRed, 0, 255);
  clamp(tmpGreen, 0, 255);
  clamp(tmpBlue, 0, 255);
  output.r = tmpRed;
  output.g = tmpGreen;
  output.b = tmpBlue;
  return output;
}

bool angleInRange(float val, float minT, float maxT) {
  // checks if an angle is between min and max angle (around a 360deg circle)
  val = fmodf(val + pi, 2*pi) - pi;
  minT = fmodf(minT + pi, 2*pi) - pi;
  maxT = fmodf(maxT + pi, 2*pi) - pi;
  if (maxT > minT) {
    if ((minT <= val) & (val <= maxT)) { return true; }
    else { return false; }
  }
  else {
    if ((maxT <  val) & (val <  minT)) { return false; }
    else { return true; }
  }
}

float smallestAngle(float val, float ref) {
  // finds the smallest angle between the val and the ref, non directional
  float diff = fmodf(val - ref + pi, 2*pi) - pi;
  diff = (diff < -pi) ? diff + 2*pi : diff;
  return (diff < 0) ? -diff : diff;
}

/* Setup Functions */

void SetupStar() {}

void SetupXMASTree() {
  //Serial.println("Detailed Tree Setup");
  // strands are wired in pairs of 3, going bottom to top, top to bottom, bottom to top
  float strandLength_in = float(NUMSTRANDLEDS - 1)*gapPerLights_in + spaceInit_in + spaceFinal_in;
  float verticalLength_in = sqrt(strandLength_in*strandLength_in - (0.5*(baseDiam_in - topDiam_in))*(0.5*(baseDiam_in - topDiam_in)));
  vertexHeight_in = verticalLength_in * baseDiam_in / (baseDiam_in - topDiam_in);
  float vertexLength_in = sqrt(vertexHeight_in*vertexHeight_in + baseDiam_in*baseDiam_in);
  float strandTiltAngle_rad = atan2f(baseDiam_in - topDiam_in, verticalLength_in);
  
  int iStrand = 0;
  for (int iP = 0; iP<NUMSTRANDPINS; iP++) { // pin, there's 9 pins and there's 3 strands per pin
    bool bottomUp = true;
    int iLwired = 0; // index of the light as it's wired on the pin
    for (int iS = 0; iS<NUMSTRANDSPERPIN; iS++) { // strand, there's 3 strands per pin
    float tmpAng = firstAngle_rad + ((lastAngle_rad - firstAngle_rad)/(float(NUMSTRANDS)-1.0))*float(iStrand);
      for (int iLcnt = 0; iLcnt<NUMSTRANDLEDS; iLcnt++) {
        // determines if you count up or down:
        int iL = (bottomUp) ? iLcnt : (NUMSTRANDLEDS-1) - iLcnt;

        // map the LEDs from "ideal" to wired:
        treeLEDMap[iStrand][iL][0] = iP;
        treeLEDMap[iStrand][iL][1] = iLwired;

        // create geometric map:
        treeT[iP][iLwired] = tmpAng;
        treeL[iP][iLwired] = spaceInit_in + float(iL)*gapPerLights_in;
        treeZ[iP][iLwired] = treeL[iP][iLwired]*cosf(strandTiltAngle_rad);
        treeR[iP][iLwired] = 0.5*baseDiam_in - treeL[iP][iL]*sinf(strandTiltAngle_rad);
        treeX[iP][iLwired] = treeR[iP][iLwired]*cosf(treeT[iP][iLwired]);
        treeY[iP][iLwired] = treeR[iP][iLwired]*sinf(treeT[iP][iLwired]);
        
        iLwired++;
      }
      bottomUp = !bottomUp;
      iStrand++;
    }
  }
}


/* Global Color Control */

void SetStarHSV(CHSV hsvLocal) {
  /* set the Star to a constant color */
  CRGB rgbLocal;
  hsv2rgb_rainbow(hsvLocal, rgbLocal);
  for (int i = 0; i<NUMSTARLEDS; i++) {
    starLEDs[i] = rgbLocal;
  }
}

void SetTreeHSV(CHSV hsvLocal) {
  /* set the Tree to a constant color */
  CRGB rgbLocal;
  hsv2rgb_rainbow(hsvLocal, rgbLocal);
  for (int iP = 0; iP< NUMSTRANDPINS; iP++) {
    for (int i = 0; i<NUMSTRANDLEDSPERPIN; i++) {
      strandLEDsWired[iP][i] = rgbLocal;
    }
  }
}


/* Color Patterns for the Tree */

void TreeColorDrops(CHSV hsvBase) {
  /* creates a drop effect where a random color will
     fall from the top of the tree down 
     Inputs:  baseHue - this is the color of the tree w/o drops
     */
  static uint8_t numDrops = 0;
  static uint8_t dropsStrand[100];
  static float dropsHeight[100];
  static CRGB dropsColor[100];
  const static float dropVelocity = -2.0; // movement per frame... so at dTime msec delay, a setting of 1 is really 100in/sec @ .01sec dTime
  const static float dropStartLocation = treeZ[0][NUMSTRANDLEDS-1] + 2; // vertexHeight_in;
  const static float flashLength = 4.0;
  const static float colorLength = 20.0;
  const static float colorTrail = 80.0;
  const static uint8_t oddsNewDrop = 60; // 1 = 1/256... 2 = 2/256, etc.
  const static float maxLength = float(NUMSTRANDLEDS)*gapPerLights_in + spaceInit_in + spaceFinal_in + flashLength + colorLength + colorTrail + 10.0; // 10 adds margin
  static bool firstLoop = true;
  
  CRGB rgbFlash = {255,255,255}; //CRGB::White;
  CRGB rgbBase;
  hsv2rgb_rainbow(hsvBase, rgbBase);

  // set the tree to the base color
  SetTreeHSV(hsvBase);
  
  // force a new drop
  if (firstLoop) {
    numDrops = 1;
    dropsHeight[0] = dropStartLocation;
    dropsColor[0] = CRGB::Red;
    dropsStrand[0] = 0;
    firstLoop = false;
  }

  // update existing drops
  for (int i = 0; i<numDrops; i++) {
    dropsHeight[i] += dropVelocity;
  }

  // check if any drops need to be deleted
  int i = 0;
  while (true) {
    if (dropsHeight[i] + flashLength + colorLength + colorTrail < 0.0) {
      for (int i2 = i+1; i2<numDrops; i2++) {
        dropsStrand[i2-1] = dropsStrand[i2];
        dropsHeight[i2-1] = dropsHeight[i2];
        dropsColor[i2-1] = dropsColor[i2];
      }
      numDrops--;
      //Serial.println("Removed old drop");
    }
    else {
      i++;
    }
    if (i>=numDrops) {break;}
  }

  // decide if a new drop is added
  if (random8() <= oddsNewDrop) {
    dropsStrand[numDrops] = random8(NUMSTRANDS);
    CHSV hsvLocal = {random8(), 255, treeValue}; // random draw on hue prevents adding a bunch of grey drops
    hsv2rgb_rainbow(hsvLocal, dropsColor[numDrops]);
    dropsHeight[numDrops] = dropStartLocation;
    //Serial.printf("Added new drop:  %d\n", dropsStrand[numDrops]);
    numDrops++;
  }
  //Serial.printf("Num Drops:  %d\n", numDrops);

  // loop through the LEDs and update the color
  for (int iD = 0; iD<numDrops; iD++) {
    //Serial.printf("Drop:  %4d  Height:  %f\n", iD, dropsHeight[iD]);
    uint8_t iS = dropsStrand[iD];
    float dropStartFlashZ = dropsHeight[iD];
    float dropStartColorZ = dropStartFlashZ + flashLength;
    float dropEndColorZ = dropStartColorZ + colorLength;
    float dropEndTrailZ = dropEndColorZ + colorTrail;
    for (int iL = 0; iL<NUMSTRANDLEDS; iL++) {
      int iP = treeLEDMap[iS][iL][0];
      int iLW = treeLEDMap[iS][iL][1];
      float ledZ = treeZ[iP][iLW];
      if (ledZ < dropStartFlashZ) {
        //break;
      }
      else if (ledZ > dropEndTrailZ) {
        //break;
      }
      else if (ledZ < dropStartColorZ) {
        float ledFrac = (ledZ - dropStartFlashZ)/flashLength;
        strandLEDsWired[iP][iLW] = blendRGB(rgbFlash, dropsColor[iD], ledFrac);
        /*int tmpRed = int(float(dropsColor[iD].r - rgbFlash.r)*ledFrac + float(rgbFlash.r) + 0.5);
        int tmpGreen = int(float(dropsColor[iD].g - rgbFlash.g)*ledFrac + float(rgbFlash.g) + 0.5);
        int tmpBlue = int(float(dropsColor[iD].b - rgbFlash.b)*ledFrac + float(rgbFlash.b) + 0.5);
        clamp(tmpRed, 0, 255);
        clamp(tmpGreen, 0, 255);
        clamp(tmpBlue, 0, 255);
        strandLEDsWired[iP][iLW].r = tmpRed;
        strandLEDsWired[iP][iLW].g = tmpGreen;
        strandLEDsWired[iP][iLW].b = tmpBlue;*/
      }
      else if (ledZ < dropEndColorZ) {
        strandLEDsWired[iP][iLW] = dropsColor[iD];
      }
      else if (ledZ <= dropEndTrailZ) {
        float ledFrac = (ledZ - dropEndColorZ)/colorTrail;
        strandLEDsWired[iP][iLW] = blendRGB(dropsColor[iD], rgbBase, ledFrac);
        /*int tmpRed = int(float(rgbBase.r - dropsColor[iD].r)*ledFrac + float(dropsColor[iD].r) + 0.5);
        int tmpGreen = int(float(rgbBase.g - dropsColor[iD].g)*ledFrac + float(dropsColor[iD].g) + 0.5);
        int tmpBlue = int(float(rgbBase.b - dropsColor[iD].b)*ledFrac + float(dropsColor[iD].b) + 0.5);
        clamp(tmpRed, 0, 255);
        clamp(tmpGreen, 0, 255);
        clamp(tmpBlue, 0, 255);
        strandLEDsWired[iP][iLW].r = tmpRed;
        strandLEDsWired[iP][iLW].g = tmpGreen;
        strandLEDsWired[iP][iLW].b = tmpBlue;*/
      }
      else {
        //Serial.printf("UNKNOWN Z - this is an issue:  %d, %d, %d, %d, %d, %f, %f\n", iD, iS, iL, iP, iLW, dropStartFlashZ, ledZ);
      }
      //Serial.printf("iL = %4d, iS = %4d, iP = %4d, iLW = %4d, ledZ = %10.4f  :  %4d  %4d  %4d\n", iL, iS, iP, iLW, ledZ, strandLEDsWired[iP][iLW].r, strandLEDsWired[iP][iLW].g, strandLEDsWired[iP][iLW].b);
    }
  }
  //delay(500);
}

void TreeSpiralRainbow(CHSV hsvBase, float theta, float dTheta, float omega, uint8_t spiralType, CHSV spiralColor, float spiraldTheta) {
  /* Creates a spiral around the tree.  Can be solid or rainbow
    Inputs:
      hsvBase         this is the color of the bulk of the tree
      theta           this is the angle that the spiral starts at the bottom of the tree
      dTheta          this is the width of the spiral in degrees
      omega           this is the rotation rate around the tree vs. height (wrap rate)
      spiralType      this is an int associated with the type of the spiral
      spiralColor     this is the starting color of the rainbow spiral
      spiraldTheta    controls how fast the color changes along the ribbon - should be even numbers for consistent colors
    Internal Settings:
      whiteBorder     this is the width of a white border measured in radians (fades from minT/maxT across border width)
  */    
  static float whiteBorder = 20.0*d2r; // width of the border
  static CRGB borderColor = {255, 255, 255}; // color of the border

  // set the full tree to solid
  SetTreeHSV(hsvBase);
  // loop through all the LEDs and determine if it's in the stripe or not
  //Serial.printf("Theta = %10.4f, dTheta = %10.4f, omega = %10.4f\n", theta, dTheta, omega);
  for (int iP = 0; iP < NUMSTRANDPINS; iP++) {
    for (int iLW = 0; iLW < NUMSTRANDLEDSPERPIN; iLW++) {
      float tmpZ = treeZ[iP][iLW];
      float tmpT = treeT[iP][iLW];
      float tmpR = treeR[iP][iLW];
      // calculate the min/max thetas associated with the width of the spiral
      float minT = theta+omega*tmpZ;
      float maxT = minT+dTheta;
      //Serial.printf("iP = %3d, iLW = %3d, Z = %10.4f, T = %10.4f, minT = %10.4f, maxT = %10.4f\n", iP, iLW, tmpZ, tmpT, minT, maxT);
      if (angleInRange(tmpT, minT, maxT)) { //((tmpT >= minT) & (tmpT <= maxT)) {
        switch (spiralType){
          case 0: {
            hsv2rgb_rainbow(spiralColor, strandLEDsWired[iP][iLW]);
            break;
          }
          case 1: {
            CHSV spiral = spiralColor;
            spiral.h += uint8_t(float(spiraldTheta) * (tmpT+pi) / (2.0*pi)*255.0 + 0.5);
            hsv2rgb_rainbow(spiral, strandLEDsWired[iP][iLW]);
            break;
          }
          default: {
            hsv2rgb_rainbow(spiralColor, strandLEDsWired[iP][iLW]);
            break;
          }
        }
      }
      
      // accent the edges of the spiral in white
      if (smallestAngle(tmpT, minT)<=whiteBorder) {
        float ledFrac = abs(smallestAngle(tmpT, minT)) / whiteBorder;
        strandLEDsWired[iP][iLW] = blendRGB(borderColor, strandLEDsWired[iP][iLW], ledFrac);
      }
      else if (smallestAngle(tmpT, maxT)<=whiteBorder) {
        float ledFrac = abs(smallestAngle(tmpT, maxT)) / whiteBorder;
        strandLEDsWired[iP][iLW] = blendRGB(borderColor, strandLEDsWired[iP][iLW], ledFrac);
      }//*/
    }
  }
  /*for (int iS = 0; iS<NUMSTRANDS; iS++) {
    int iL = 0;
    int iP = treeLEDMap[iS][iL][0];
    int iLW = treeLEDMap[iS][iL][1];
    //Serial.printf("iS = %3d, T=%10.4f,   r=%3d, g=%3d, b=%3d\n", iS, treeT[iP][iLW], strandLEDsWired[iP][iLW].r, strandLEDsWired[iP][iLW].g, strandLEDsWired[iP][iLW].b);
  }//*/
}

void TreeLightSparkles(CHSV hsvBase) {
  /* Create random colorful sparkles
    Inputs:
      hsvBase       This is the color of the tree
    Internal Variables:
      spawnRate     Odds of a new spot showing up
  */
  static uint8_t spawnRate = 100; // 0 = never, 255 = every frame
  static float diamRate = 2; // growth rate of the circle
  static float diamSpot = 6; // size of the spot before fading begins

  static const int maxSpots = 200;
  static int numSpots = 0;
  static float spotX[maxSpots];
  static float spotY[maxSpots];
  static float spotZ[maxSpots];
  static float spotR[maxSpots];

  if (random8() < spawnRate) {

  }
  
  for (int iP = 0; iP<NUMSTRANDPINS; iP++) {
    for (int iLW = 0; iLW<NUMSTRANDLEDSPERPIN; iLW++) {
      for (int iSpot = 0; iSpot<numSpots; iSpot++) {

      }
    }
  }
}

void TreeMeteors(CHSV hsvBase) {
  /* Create random meteors that shoot at random directions and varying speed
    Inputs:
      hsvBase         this is the color of the bulk of the tree
    Internal Settings:
      meteorSpawnRate random8()<meteroSpawnRate will generate a new meteor that frame
      meteorVelMin    min velocity of a meteor
      meteorVelMax    max velocity of a meteor
      meteorHeadMin   min size of meteor head
      meteorHeadMax   max size of meteor head
      meteorLength    nominal meteor length (@ mid Vel, mid Head size)
    Tail length is a function of the velocity, head size, and length scalar:
      meteorScalar = meteorLength / (0.5*(velMin+velMax) * 0.5*(headMin+headMax));
      meteortail = meteorVel * meteorHead * meteorScaler;
  */
  const static uint8_t meteorSpawnRate = 25; // 0 = never, 255 = every frame.
  const static float meteorVelMin = 1.0;
  const static float meteorVelMax = 3.0;

  const static uint8_t maxMeteors = 255;
  static uint8_t numMeteors = 0;
  static float meteorLoc[maxMeteors];
  static float meteorHead[maxMeteors];
  static float meteorVel[maxMeteors];
  static CRGB meteorColor[maxMeteors];

  SetTreeHSV(hsvBase);

}


/* Color Patterns for the Star */

void CyclePixelColors(uint8_t deltaVal) {
  static uint8_t startHue = 0;
  for (int ip = 0; ip<NUMSTARLEDS; ip++) {
      startHue += deltaVal;
    CHSV hsvLocal = {startHue, 255, starValue};
    CRGB rgbLocal;
    hsv2rgb_rainbow(hsvLocal,rgbLocal);
    starLEDs[ip] = rgbLocal;
  }
}

void CyclePointColors(uint8_t shiftVal, uint8_t deltaVal) {
  static uint8_t pointHue[26] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  for (int i = 0; i<26; i++) {
    if (i==0) {
      pointHue[i] += shiftVal;
    }
    else {
      pointHue[i] = pointHue[i-1]+deltaVal;
    }
  }
  uint8_t oldPoint = 255;
  CRGB rgbLocal;
  for (int ip = 0; ip<NUMSTARLEDS; ip++) {
    if (oldPoint != starPoint[ip]) {
      CHSV hsvLocal = {pointHue[starPoint[ip]], 255, starValue};
      hsv2rgb_rainbow(hsvLocal,rgbLocal);
      oldPoint = starPoint[ip];
    }
    starLEDs[ip] = rgbLocal;
  }
}