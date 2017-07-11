//cases times and {}
//#define PINS

#define NAtt 50
#define Nd 32 //payload
#define connectDelay 1000    //max delay for receiving
#define attachDelay 500     //delay to do nothing after electrodes displacement 
#define fallLevel 80        //is specific for amplifier  
#define N1 10

#define pl 4       //payload size
#define protokolBound 9



#include "MYO_lib.h"
#include <nRF24L01.h>
//#include "decoder.h"
#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


#define Nb 15
//int16_t h2 = 0xFFFF;
#define transm_N1 1
#define transm_N2 3
int pinO[4] = {4, 5, 2, 1};//D(4) D(5) D(6) B(9)
long moniL[2];
int moni[2];
uint8_t helpI[2];
int16_t plG[2][16];
int16_t *plGp[2];
byte plS[32];
byte st[2];
byte sp[2];
int8_t d[2];
RF24 radio(8, 7);
int c1;
const uint64_t pipes[5] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xF0F0F0F0D3LL, 0xF0F0F0F0D4LL, 0xF0F0F0F0D5LL  };




typedef enum {slow = 1, fast} modeType;
typedef enum {floatMode = 1,
              monitorMode, FEETMode, PuzleBoxMode,
              watchFEET, LabView, RandomMode
             } connectMode;


connectMode connectModeArg =
  //_________________________CHOOSE_THE_MODE______________
  monitorMode;
//floatMode;
//watchFEET;
//FEETMode;
//PuzleBoxMode;
//RandomMode;
//LabView;

modeType speedMode =
  //_________________________CHOOSE_THE_MODE______________
  slow;
//fast;
//_________________________CHOOSE_THE_FLEXURE_PARAM______________
boolean flexureTRUE = true;

class sendBuf
{
  public:
    byte ch1[10], ch2[10], ch1S, ch2S, ch1lv, ch2lv;
    sendBuf()
    {
      //ch1 = 0; ch2 = 0;
      ch1S = 0; ch2S = 0;
      ch1lv = 0; ch2lv = 0;
    }
};




byte minMax(byte a, byte b)
{
  if (a < b)return (a);
  else
    return (b);
}


int8_t fr[3];

byte a[2] = {105, 150};
byte b[2] = {140, 150};
byte anByte[2];
byte ch, del = 400;
int16_t otkl[3];
int16_t kt;

int16_t thRange = 90;
int16_t thUp = 120;
int16_t thDn = 30;
int16_t othRange = 50;
int16_t othUp = 120;
int16_t othDn = 70;



byte sa, sw12[4];
byte contr[2] = {0, 0};
byte c2 = 0;
int gl;
float help[2], help2;
bool mode = false;
uint8_t help3;
//int h[8];
//int16_t x[N], y[N], z[N];
int nCh[3], m;
float xfB[2][3],
      xfW[2][2],
      yfB[2][3],
      yfW[2][2];

Servo myServo0, myServo1;


//__________________________SET_OUTPUT_NONLINEARITY
byte curve(byte x, byte a[2], byte b[2])
{
  if (x < a[0]) return (    (((double)(b[0]) / (double)(a[0]))) * x    );
  else if (x < a[1])    return (    b[0] + (double)(b[1] - b[0]) / (double)(a[1] - a[0]) * (x - a[0])      );
  else return b[1];
}



byte getCircle(byte x)
{
  return (x < Nb) ? (x) : (x - Nb);
}


class absMot   //abstract motor class
{
  public:
    double sp;
    int ep;
    byte count;
    double tPar;
    double x;
    int realX;
    unsigned long t;
    void getDelay(int y)
    {
      tPar = y;
    }
    void extractX(int e)
    {
      if (count == tPar)
      {
        ep = realX;
        realX = e;
        count = 0;
      }
    }
    void xCalcSl(int e, double k)
    {
      count++;
      extractX(e);
      sp = (realX - x) * k;
      x += sp;
    }
    void xCalc(int e, double k)
    {
      count++;
      extractX(e);
      sp = (realX - ep) / tPar;
      x += sp;
    }
};



int unsWindow(int y, int x)
{
  if (x < 0)
    return 0;
  else if (x < y)
    return x;
  else
    return y;
}



int reject1(int a, int thr)
{
  if ((a > 0) && (a > thr))
    return (a - thr);
  else if (a > 0)
    return (0);
  else if ((a <= 0) && (a < -thr))
    return (a + thr);
  else if (a <= 0)
    return (0);

}

int thresholdRange(int down, int up, float x)
{
  if ((x < up) && (x >= down)) return (x);
  else if (x >= up) return (up);
  else return (down);
}



int threshold(int thr, double x)
{
  if ((x < thr) && (x >= 0)) return (x);
  else if (x < 0) return (0);
  else return (thr);
}





void changeNom(byte & b)
{
  if (b) b = 0;
  else b = 1;
}





void setup() {
#ifdef PINS
  pinMode(pinO[0], OUTPUT);
  pinMode(pinO[1], OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
#endif

  pinMode(3, OUTPUT);

  integrator IR[2];
  matchedFr MF[2];
  slowFilt BeHp[2];
  lastStage LS[2];
  FilterBuHp2 BuHp2[2];
  //slowFilt BuHp2[2];
  calibr cr[2];
  LowPassF LPF[2];
  median mdn[2];
  RMSM rmsM;
  double sens = 26;

  radio.begin();
  radio.setRetries(0, 0);
  radio.setDataRate(RF24_2MBPS);
  radio.setPayloadSize(32);
  radio.setChannel(20);

  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[1]);
  radio.openReadingPipe(2, pipes[2]);
  radio.openReadingPipe(3, pipes[3]);


  Serial.begin(115200);
  //Serial.println(frBtw1.w);

  plS[0] = 1;
  plS[1] = 500 / 8;
  plS[2] = 1400 / 8;//1600
  plS[3] = 1400 / 8; //1100
  //interface


  delay(2);
  myServo0.attach(6);
  myServo1.attach(3);

  myServo0.write(thDn);
  myServo1.write(othUp);




  help2 = 1;
  // DECR.sd0.k = .5;  //___________ADJUST_BUTTERWORTH_FILTER
  // DECR.sd1.k = .5;


  //____________________________________________________________________

  //______________SET_ADC_SETTINGS
  analogRead(A0);


  while (help2)
  {

    static unsigned long tConnect[2] = {0, 0};
    static unsigned long TimeAtt[2] = {0, 0}; //attach/detach of electrodes


    //__________________ADJUST_SENSITIVITY____
    anByte[0] = ADCL;
    anByte[1] = ADCH;
    ADCSRA |= (1 << ADSC);




    kt = (int16_t)anByte[0] + ((int16_t)anByte[1] << 8);
    //kt=40;
    //___________________RECEIVE_DATA_________


    radio.write( plS, 32 );
    radio.startListening();



    for (int i = 0; i < 2; i++)
    {
      switch ((uint8_t)((plG[i][15] & 0xFF00) >> 8))
      {
        case transm_N1: plGp[0] = (int16_t*)plG[i]; break;
        case transm_N2: plGp[1] = (int16_t*)plG[i]; break;
      }
    }

    //____________________FIRST_TRANSMITER_______________
    if ((uint8_t)((plGp[0][15] & 0xFF00) >> 8) == transm_N1)
    {
      sp[0] = plGp[0][15] & 0xFF;
      d[0] = sp[0] - st[0];
      d[0] = (d[0] >= 0) ? d[0] : (d[0] + Nb);
      static int ff;

      for (int i = 0; i < minMax(d[0], protokolBound); i++)
      {
        otkl[0] = plGp[0][getCircle(st[0] + i)];

        if ((connectModeArg == monitorMode) || (connectModeArg == watchFEET))
        {

          Serial.write(1);
          Serial.write(thresh(fr[0]*.5,-125,125));
          Serial.write((uint8_t)help[0]);

        }

        tConnect[0] = millis();

        /*
                fr[0] = help[0] = BuHp2[0](otkl[0]);
                help[0] = killRange(-abs(fr[0]) / 2, 1); //12 15//15 15+hot -cool
                //help[0] = killRange(MF[0](fr[0]) / 15, 15); //12 15//15 15
                moni[0] = help[0] = threshI(help[0]);
                moniL[0] = IR[0]((int8_t)(help[0] / 2.)) / 3.;
                help[0] = (int8_t)(threshI(BeHp[0](moniL[0])));
                cr[0].getM(help[0], millis());
        */
        int chan = 0;
         help[chan] = BuHp2[chan](otkl[chan])*.5;
        fr[chan] =thresh(help[chan],-126,126);
        help[chan] = rmsM(help[chan]) * .7;
        //        help[chan]=killRange(abs(help[chan]),3);
        //        help[chan]=LPF[chan](sens*help[chan]);
        help[chan] = thresh(LS[chan](help[chan]), 0, 254);
        //        help[chan]=mdn[chan](help[chan]);
        //______________________________________________
      } st[0] = sp[0]; plGp[0][15] &= 0xFF;
    }




    //____________________SECOND_TRANSMITER_______________
    if ((uint8_t)((plGp[1][15] & 0xFF00) >> 8) == (transm_N2))
    {
      tConnect[1] = millis();

      sp[1] = plGp[1][15] & 0xFF;
      d[1] = sp[1] - st[1];
      d[1] = (d[1] >= 0) ? d[1] : (d[1] + Nb);

      for (int i = 0; i < minMax(d[1], protokolBound); i++)
      {
        otkl[1] = plGp[1][getCircle(st[1] + i)];

        if ((connectModeArg == monitorMode) || (connectModeArg == watchFEET))
        {
          Serial.write(2);
          Serial.write(thresh(fr[1]*.5,-125,125));

          // Serial.write(helpI[1] = thresh(cr[1].setM(help[1]), -120, 120) + 128);

          // Serial.write((int8_t)(threshI(BeHp[1](moniL[1]))) + 128);
          //Serial.write(LS[1]((int)moniL[1]));
          Serial.write((int)help[1]);
        }
        tConnect[1] = millis();
        /*
                fr[1] = BuHp2[1](otkl[1]);
                help[1] = killRange(MF[1](fr[1]) / 15, 15); //12 15
                help[1] = threshI(help[1]);
                moni[1] = moniL[1] = IR[1]((int8_t)(help[1] / 2.)) / 3.;
        */

        /*
          int chan=1;
                fr[chan] = help[chan] = BuHp2[chan](otkl[chan]);
                help[chan] = killRange(-abs(fr[chan]) / 2, 1); //12 15//15 15+hot -cool
                //help[0] = killRange(MF[0](fr[0]) / 15, 15); //12 15//15 15
                moni[chan] = help[chan] = threshI(help[chan]);
                moniL[chan] = IR[chan]((int8_t)(help[chan] / 2.)) / 3.;
                help[chan] = (int8_t)(threshI(BeHp[chan](moniL[chan])));
                cr[chan].getM(help[chan], millis());
        */

        int chan = 1;
        fr[chan] = help[chan] = BuHp2[chan](otkl[chan])*.5;
        help[chan] = killRange(abs(help[chan]), 3);
        help[chan] = LPF[chan](sens * help[chan]);
        help[chan] = thresh(LS[chan](help[chan]), 0, 254);
        //        help[chan]=mdn[chan](help[chan]);

        //______________________________________________
      } st[1] = sp[1]; plGp[1][15] &= 0xFF;
    }


    {
      static int cnt;
      cnt++;
      if (cnt == 1)
      {
        cnt = 0;
        //             if(contr>10)
        analogWrite(6, (uint8_t)help[0]);
        //             else
        //             digitalWrite(6,0);
      }
    }




    //______________________________________________KULER
#ifdef PINS
    if (millis() < 3500)
    {
      static bool h;
      static long T;
      if (millis() - T > 500)
      {
        T = millis();
        h = !h;
        if (h)
        {
          PORTB &= ~(1 << pinO[2]);
          PORTB |= (1 << pinO[3]);
        }
        else
        {
          PORTB |= (1 << pinO[2]);
          PORTB &= ~(1 << pinO[3]);
        }
      }
    }
    else
    {
      if (helpI[0] == 8)
      {
        PORTD |= (1 << pinO[1]);
        PORTB |= (1 << pinO[3]);
      }
      else
      {
        PORTD &= ~((1 << pinO[1]));
        PORTB &= ~(1 << pinO[3]);
      }


      if (helpI[0] == 248)
      {
        PORTD |= (1 << pinO[0]);
        PORTB |= (1 << pinO[2]);
      }
      else
      {
        PORTD &= ~((1 << pinO[0]));
        PORTB &= ~(1 << pinO[2]);
      }
    }
#endif




    delayMicroseconds(2400);
    if (radio.available())
      radio.read( plG[0], 32 );
    if (radio.available())
      radio.read( plG[1], 32 );

    //radio.flush_rx();
    radio.stopListening();


  }
}








void loop()
{

}

