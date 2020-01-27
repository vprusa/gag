#include <TFT_eSPI.h>
#include <SPI.h>
#include "WiFi.h"
#include <Wire.h>
#include <Button2.h>
#include "esp_adc_cal.h"
#include "bmp.h"

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define TFT_MOSI            19
#define TFT_SCLK            18
#define TFT_CS              5
#define TFT_DC              16
#define TFT_RST             23

#define TFT_BL          4  // Display backlight control pin
#define ADC_EN          14
#define ADC_PIN         34
#define BUTTON_1        35
#define BUTTON_2        0

#define MENU_SCAN_WIFI 1
#define MENU_VOLTAGE 2
#define MENU_CUBE 3
#define MENU_CLOCK_SKETCH 4
#define MENU_CLOCK 5

#define MENU_ITEM_MAX 5


#define BLACK 0x0000
#define WHITE 0xFFFF

//# include <SPI.h>

//# include <TFT_eSPI.h> // Hardware-specific library


/*
 An example analogue clock using a TFT LCD screen to show the time
 use of some of the drawing commands with the ST7735 library.

 For a more accurate clock, it would be better to use the RTClib library.
 But this is just a demo. 

 Uses compile time to set the time so a reset will start with the compile time again
 
 Gilchrist 6/2/2014 1.0
 Updated by Bodmer
 */

// #include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
// #include <SPI.h>

//TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

#define TFT_GREY 0xBDF7
TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library


float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg=0, mdeg=0, hdeg=0;
uint16_t osx=64, osy=64, omx=64, omy=64, ohx=64, ohy=64;  // Saved H, M, S x & y coords
uint16_t x0=0, x1=0, yy0=0, yy1=0;
uint32_t targetTime = 0;                    // for next 1 second timeout

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

uint8_t hh=conv2d(__TIME__), mm=conv2d(__TIME__+3), ss=conv2d(__TIME__+6);  // Get H, M, S from compile time

boolean initial = 1;

Button2 btnR(BUTTON_1);
Button2 btnL(BUTTON_2);

//TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

int16_t h;
int16_t w;

int inc = -2;

float xx, xy, xz;
float yx, yy, yz;
float zx, zy, zz;

float fact;

int Xan, Yan;

int Xoff;
int Yoff;
int Zoff;

struct Point3d
{
  int x;
  int y;
  int z;
};

struct Point2d
{
  int x;
  int y;
};

int LinestoRender; // lines to render.
int OldLinestoRender; // lines to render just in case it changes. this makes sure the old lines all get erased.

struct Line3d
{
  Point3d p0;
  Point3d p1;
};

struct Line2d
{
  Point2d p0;
  Point2d p1;
};

Line3d Lines[20];
Line2d Render[20];
Line2d ORender[20];



char buff[512];
int vref = 1100;
int btnRClick = false;
int btnLClick = false;

boolean inMenu = false;
int menuItem = 1;
bool menuItemChanged = false;
bool cubePlaying = false;
bool clockPlaying = false;



//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms) {   
    /*esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();*/
    delay(ms);
}

void showVoltage() {
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        String voltage = "Voltage :" + String(battery_voltage) + "V";
        Serial.println(voltage);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(voltage,  tft.width() / 2, tft.height() / 2 );
    }
}

void showMenu() {
    inMenu = true;
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000 || menuItemChanged) {
        menuItemChanged = false;
        timeStamp = millis();
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setTextSize(1);
        tft.setCursor(0, 0);

        tft.drawString("Menu", 10, 0);
        //TODO fix selector prefix from space to padding
        tft.drawString(" Scan", 15, 15);
        tft.drawString(" Voltage", 15, 26);
        tft.drawString(" Cube", 15, 37);
        tft.drawString(" Clock (sketch)", 15, 48);
        tft.drawString(" Clock", 15, 48);

        tft.setCursor(0, 0);
        tft.drawString(">", 0, (menuItem*11)+5);
    }
}

void button_init() {
    btnR.setLongClickHandler([](Button2 & b) {
        //btnRClick = false;

        /*
        if(inMenu){
            int r = digitalRead(TFT_BL);
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
            //espDelay(6000);
            espDelay(1000);
            digitalWrite(TFT_BL, !r);
    
            tft.writecommand(TFT_DISPOFF);
            tft.writecommand(TFT_SLPIN);
            esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
            esp_deep_sleep_start();
        }
        */
    });
    btnR.setPressedHandler([](Button2 & b) {
       
        btnRClick = true;
        btnLClick = false;
        cubePlaying = false;
        clockPlaying = false;

        if(inMenu){
            inMenu = false;
            switch(menuItem) {
                case MENU_SCAN_WIFI:
                    Serial.println("wifi scan");
                    wifi_scan();
                    break;
                case MENU_VOLTAGE:
                    Serial.println("Detect Voltage..");
                    showVoltage();
                    break;
                case MENU_CUBE:
                    Serial.println("Cube..");
                    btnRClick = false;
                    btnLClick = false;
                    cubePlaying = true;
                    cube(); // reset cube geometry
                    break;
                case MENU_CLOCK_SKETCH:
                    Serial.println("Clock(sketch)..");
                    btnRClick = false;
                    btnLClick = false;
                    clockPlaying = true;
                    play_clock(); // reset cube geometry
                    break;
                case MENU_CLOCK:
                    Serial.println("Clock..");
                    btnRClick = false;
                    btnLClick = false;
                    clockPlaying = true;
                    play_clock(); // reset cube geometry
                    break;
                default:
                    inMenu = true;
                    break;
            }
        }
    });

    btnL.setPressedHandler([](Button2 & b) {
        btnRClick = false;
        btnLClick = true;
      
        if(inMenu){
            menuItemChanged=true;
            menuItem++;
            if(menuItem>MENU_ITEM_MAX){
                menuItem=1;
            }
        } else {
          inMenu = true;
        }
    });
}

void button_loop() {
    btnR.loop();
    btnL.loop();
}

void loop() {
        espDelay(1);
}

void wifi_scan() {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);

    tft.drawString("Scan Network", tft.width() / 2, tft.height() / 2);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    espDelay(100);

    int16_t n = WiFi.scanNetworks();
    tft.fillScreen(TFT_BLACK);
    if (n == 0) {
        tft.drawString("no networks found", tft.width() / 2, tft.height() / 2);
    } else {
        // TODO add scroll down loop on right button press event
        tft.setTextDatum(TL_DATUM);
        tft.setCursor(0, 0);
        Serial.printf("Found %d net\n", n);
        for (int i = 0; i < n; ++i) {
            sprintf(buff,
                    "[%d]:%s(%d)",
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i));
            tft.println(buff);
        }
    }
    WiFi.mode(WIFI_OFF);
}

void loop_cube() {

    // Rotate around x and y axes in 1 degree increments
    Xan++;
    Yan++;

    Yan = Yan % 360;
    Xan = Xan % 360; // prevents overflow.

    SetVars(); //sets up the global vars to do the 3D conversion.

    // Zoom in and out on Z axis within limits
    // the cube intersects with the screen for values < 160
    Zoff += inc; 
    if (Zoff > 500) inc = -1;     // Switch to zoom in
    else if (Zoff < 160) inc = 1; // Switch to zoom out

    for (int i = 0; i < LinestoRender ; i++)
    {
        ORender[i] = Render[i]; // stores the old line segment so we can delete it later.
        ProcessLine(&Render[i], Lines[i]); // converts the 3d line segments to 2d.
    }
    RenderImage(); // go draw it!

    //delay(14); // Delay to reduce loop rate (reduces flicker caused by aliasing with TFT screen refresh rate)
    espDelay(14);
}


/***********************************************************************************************************************************/
void RenderImage( void) {
  // renders all the lines after erasing the old ones.
  // in here is the only code actually interfacing with the OLED. so if you use a different lib, this is where to change it.

  for (int i = 0; i < OldLinestoRender; i++ ) {
    tft.drawLine(ORender[i].p0.x, ORender[i].p0.y, ORender[i].p1.x, ORender[i].p1.y, BLACK); // erase the old lines.
  }


  for (int i = 0; i < LinestoRender; i++ ) {
    uint16_t color = TFT_BLUE;
    if (i < 4) color = TFT_RED;
    if (i > 7) color = TFT_GREEN;
    tft.drawLine(Render[i].p0.x, Render[i].p0.y, Render[i].p1.x, Render[i].p1.y, color);
  }
  OldLinestoRender = LinestoRender;
}

/***********************************************************************************************************************************/
// Sets the global vars for the 3d transform. Any points sent through "process" will be transformed using these figures.
// only needs to be called if Xan or Yan are changed.
void SetVars(void) {
  float Xan2, Yan2, Zan2;
  float s1, s2, s3, c1, c2, c3;

  Xan2 = Xan / fact; // convert degrees to radians.
  Yan2 = Yan / fact;

  // Zan is assumed to be zero

  s1 = sin(Yan2);
  s2 = sin(Xan2);

  c1 = cos(Yan2);
  c2 = cos(Xan2);

  xx = c1;
  xy = 0;
  xz = -s1;

  yx = (s1 * s2);
  yy = c2;
  yz = (c1 * s2);

  zx = (s1 * c2);
  zy = -s2;
  zz = (c1 * c2);
}

/***********************************************************************************************************************************/
// processes x1,y1,z1 and returns rx1,ry1 transformed by the variables set in SetVars()
// fairly heavy on floating point here.
// uses a bunch of global vars. Could be rewritten with a struct but not worth the effort.
void ProcessLine(struct Line2d *ret, struct Line3d vec) {
  float zvt1;
  int xv1, yv1, zv1;

  float zvt2;
  int xv2, yv2, zv2;

  int rx1, ry1;
  int rx2, ry2;

  int x1;
  int y1;
  int z1;

  int x2;
  int y2;
  int z2;

  int Ok;

  x1 = vec.p0.x;
  y1 = vec.p0.y;
  z1 = vec.p0.z;

  x2 = vec.p1.x;
  y2 = vec.p1.y;
  z2 = vec.p1.z;

  Ok = 0; // defaults to not OK

  xv1 = (x1 * xx) + (y1 * xy) + (z1 * xz);
  yv1 = (x1 * yx) + (y1 * yy) + (z1 * yz);
  zv1 = (x1 * zx) + (y1 * zy) + (z1 * zz);

  zvt1 = zv1 - Zoff;

  if ( zvt1 < -5) {
    rx1 = 256 * (xv1 / zvt1) + Xoff;
    ry1 = 256 * (yv1 / zvt1) + Yoff;
    Ok = 1; // ok we are alright for point 1.
  }

  xv2 = (x2 * xx) + (y2 * xy) + (z2 * xz);
  yv2 = (x2 * yx) + (y2 * yy) + (z2 * yz);
  zv2 = (x2 * zx) + (y2 * zy) + (z2 * zz);

  zvt2 = zv2 - Zoff;

  if ( zvt2 < -5) {
    rx2 = 256 * (xv2 / zvt2) + Xoff;
    ry2 = 256 * (yv2 / zvt2) + Yoff;
  } else {
    Ok = 0;
  }

  if (Ok == 1) {

    ret->p0.x = rx1;
    ret->p0.y = ry1;

    ret->p1.x = rx2;
    ret->p1.y = ry2;
  }
  // The ifs here are checks for out of bounds. needs a bit more code here to "safe" lines that will be way out of whack, so they dont get drawn and cause screen garbage.

}

void play_clock(){
    setup_clock();
}

/***********************************************************************************************************************************/
// line segments to draw a cube. basically p0 to p1. p1 to p2. p2 to p3 so on.
void cube(void) {
  tft.fillScreen(TFT_BLACK);

  // Front Face.
  Lines[0].p0.x = -50;
  Lines[0].p0.y = -50;
  Lines[0].p0.z = 50;
  Lines[0].p1.x = 50;
  Lines[0].p1.y = -50;
  Lines[0].p1.z = 50;

  Lines[1].p0.x = 50;
  Lines[1].p0.y = -50;
  Lines[1].p0.z = 50;
  Lines[1].p1.x = 50;
  Lines[1].p1.y = 50;
  Lines[1].p1.z = 50;

  Lines[2].p0.x = 50;
  Lines[2].p0.y = 50;
  Lines[2].p0.z = 50;
  Lines[2].p1.x = -50;
  Lines[2].p1.y = 50;
  Lines[2].p1.z = 50;

  Lines[3].p0.x = -50;
  Lines[3].p0.y = 50;
  Lines[3].p0.z = 50;
  Lines[3].p1.x = -50;
  Lines[3].p1.y = -50;
  Lines[3].p1.z = 50;


  //back face.

  Lines[4].p0.x = -50;
  Lines[4].p0.y = -50;
  Lines[4].p0.z = -50;
  Lines[4].p1.x = 50;
  Lines[4].p1.y = -50;
  Lines[4].p1.z = -50;

  Lines[5].p0.x = 50;
  Lines[5].p0.y = -50;
  Lines[5].p0.z = -50;
  Lines[5].p1.x = 50;
  Lines[5].p1.y = 50;
  Lines[5].p1.z = -50;

  Lines[6].p0.x = 50;
  Lines[6].p0.y = 50;
  Lines[6].p0.z = -50;
  Lines[6].p1.x = -50;
  Lines[6].p1.y = 50;
  Lines[6].p1.z = -50;

  Lines[7].p0.x = -50;
  Lines[7].p0.y = 50;
  Lines[7].p0.z = -50;
  Lines[7].p1.x = -50;
  Lines[7].p1.y = -50;
  Lines[7].p1.z = -50;


  // now the 4 edge lines.

  Lines[8].p0.x = -50;
  Lines[8].p0.y = -50;
  Lines[8].p0.z = 50;
  Lines[8].p1.x = -50;
  Lines[8].p1.y = -50;
  Lines[8].p1.z = -50;

  Lines[9].p0.x = 50;
  Lines[9].p0.y = -50;
  Lines[9].p0.z = 50;
  Lines[9].p1.x = 50;
  Lines[9].p1.y = -50;
  Lines[9].p1.z = -50;

  Lines[10].p0.x = -50;
  Lines[10].p0.y = 50;
  Lines[10].p0.z = 50;
  Lines[10].p1.x = -50;
  Lines[10].p1.y = 50;
  Lines[10].p1.z = -50;

  Lines[11].p0.x = 50;
  Lines[11].p0.y = 50;
  Lines[11].p0.z = 50;
  Lines[11].p1.x = 50;
  Lines[11].p1.y = 50;
  Lines[11].p1.z = -50;

  LinestoRender = 12;
  OldLinestoRender = LinestoRender;

}


void setup_clock(){
    //  tft.init();
//   tft.setRotation(0);
   tft.fillScreen(TFT_BLACK);
   tft.setTextColor(TFT_GREEN, TFT_BLACK);  // Adding a black background colour erases previous text automatically
  
  // Draw clock face
  tft.fillCircle(64, 64, 61, TFT_BLUE);
  tft.fillCircle(64, 64, 57, TFT_BLACK);

  // Draw 12 lines
  for(int i = 0; i<360; i+= 30) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*57+64;
    yy0 = sy*57+64;
    x1 = sx*50+64;
    yy1 = sy*50+64;

    tft.drawLine(x0, yy0, x1, yy1, TFT_BLUE);
  }

  // Draw 60 dots
  for(int i = 0; i<360; i+= 6) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*53+64;
    yy0 = sy*53+64;
    
    tft.drawPixel(x0, yy0, TFT_BLUE);
    if(i==0 || i==180) tft.fillCircle(x0, yy0, 1, TFT_CYAN);
    if(i==0 || i==180) tft.fillCircle(x0+1, yy0, 1, TFT_CYAN);
    if(i==90 || i==270) tft.fillCircle(x0, yy0, 1, TFT_CYAN);
    if(i==90 || i==270) tft.fillCircle(x0+1, yy0, 1, TFT_CYAN);
  }

  tft.fillCircle(65, 65, 3, TFT_RED);

  // Draw text at position 64,125 using fonts 4
  // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . a p m
  // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
  tft.drawCentreString("Time flies",64,130,4);

  targetTime = millis() + 1000; 
}


TaskHandle_t Task1;
TaskHandle_t Task2;

// LED pins
const int led1 = 2;
const int led2 = 4;

void Task2code(void * pvParameters);
void Task1code(void * pvParameters);

void setup() {
    Serial.begin(115200);
    Serial.println("Start");
    Serial.println(xPortGetCoreID());

    tft.init();

    h = tft.height();
    w = tft.width();

    tft.setRotation(1);

    tft.fillScreen(TFT_BLACK);

    cube();

    fact = 180 / 3.14159259; // conversion from degrees to radians.

    Xoff = 67; // Position the center of the 3d conversion space into the center of the TFT screen.
    Yoff = 120;
    Zoff = 1100; // Z offset in 3D space (smaller = closer and bigger rendering)


    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(0, 0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    
    showMenu();

    if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
         pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
         digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    }

    tft.setSwapBytes(true);
    //tft.pushImage(0, 0,  240, 135, ttgo);
    espDelay(1000);

    tft.setRotation(0);
    int i = 5;
    while (i--) {
        tft.fillScreen(TFT_RED);
        //espDelay(1000);
        tft.fillScreen(TFT_BLUE);
        //espDelay(1000);
        tft.fillScreen(TFT_GREEN);
        //espDelay(1000);
    }

    button_init();

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV");
    }

    setup_clock();


    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    
    xTaskCreatePinnedToCore(
                        Task1code,   // Task function. /
                        "Task1",     //name of task. /
                        10000,       // Stack size of task /
                        NULL,        // parameter of the task /
                        1,           // priority of the task /
                        &Task1,      // Task handle to keep track of created task /
                        0);          // pin task to core 0 /   
                
    espDelay(500); 

    //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(
                        Task2code,   /* Task function. */
                        "Task2",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        1,           /* priority of the task */
                        &Task2,      /* Task handle to keep track of created task */
                        1);          /* pin task to core 1 */
    //espDelay(500); 
}


void loop_clock() {
  if (targetTime < millis()) {
    targetTime = millis()+1000;
    ss++;              // Advance second
    if (ss==60) {
      ss=0;
      mm++;            // Advance minute
      if(mm>59) {
        mm=0;
        hh++;          // Advance hour
        if (hh>23) {
          hh=0;
        }
      }
    }

    // Pre-compute hand degrees, x & y coords for a fast screen update
    sdeg = ss*6;                  // 0-59 -> 0-354
    mdeg = mm*6+sdeg*0.01666667;  // 0-59 -> 0-360 - includes seconds
    hdeg = hh*30+mdeg*0.0833333;  // 0-11 -> 0-360 - includes minutes and seconds
    hx = cos((hdeg-90)*0.0174532925);    
    hy = sin((hdeg-90)*0.0174532925);
    mx = cos((mdeg-90)*0.0174532925);    
    my = sin((mdeg-90)*0.0174532925);
    sx = cos((sdeg-90)*0.0174532925);    
    sy = sin((sdeg-90)*0.0174532925);

    if (ss==0 || initial) {
      initial = 0;
      // Erase hour and minute hand positions every minute
      tft.drawLine(ohx, ohy, 65, 65, TFT_BLACK);
      ohx = hx*33+65;    
      ohy = hy*33+65;
      tft.drawLine(omx, omy, 65, 65, TFT_BLACK);
      omx = mx*44+65;    
      omy = my*44+65;
    }

      // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
      tft.drawLine(osx, osy, 65, 65, TFT_BLACK);
      tft.drawLine(ohx, ohy, 65, 65, TFT_WHITE);
      tft.drawLine(omx, omy, 65, 65, TFT_WHITE);
      osx = sx*47+65;    
      osy = sy*47+65;
      tft.drawLine(osx, osy, 65, 65, TFT_RED);

    tft.fillCircle(65, 65, 3, TFT_RED);
  }
}

void Task1code( void * pvParameters ){
    static uint64_t timeStamp = 0;
    for(;;){
        if (millis() - timeStamp > 1000) {
            Serial.print("Task1 running on core ");
            Serial.println(xPortGetCoreID());
            // Serial.print(" delay ");
            // Serial.println((int)(millis() - timeStamp));

            timeStamp = millis();
        }
        espDelay(1000);
        //for(;;){
        //digitalWrite(led2, HIGH);
        //digitalWrite(led2, LOW);
        //espDelay(1000);
        //}
    }
    vTaskDelete( NULL );
}

//void loop()
//void Task1code()
void Task2code( void * pvParameters ) {
   
    static uint64_t timeStamp = 0;
    for(;;){
        if (millis() - timeStamp > 1000) {
            Serial.print("Task2 running on core ");
            Serial.println(xPortGetCoreID());
            // Serial.print(" delay ");
            // Serial.println((int)(millis() - timeStamp));
            timeStamp = millis();
            
        }

        if(inMenu){
            showMenu();
        } else {
            if(btnLClick && cubePlaying){
                cubePlaying = false;
                inMenu = true;
            }else if(cubePlaying) {
                loop_cube();
            }else if(clockPlaying) {
                loop_clock();
            }
        }
        button_loop();
        
        // just in case .. to avoid https://github.com/espressif/arduino-esp32/issues/922
        espDelay(1); 
    }
    vTaskDelete( NULL );
}

