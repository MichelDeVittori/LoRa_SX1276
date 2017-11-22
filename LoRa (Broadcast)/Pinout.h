#ifndef PINOUT_H    
#define PINOUT_H

/* Defines pinout
 * A -> 0x0----
 * B -> 0x1----
 * C -> 0x2----
 * D -> 0x3----
 * E -> 0x4----
 * F -> 0x5----
 * G -> 0x6----*/
#define PULS_2      0x40020U    //RE5
#define PULS_1      0x40040U    //RE6
#define TDC_CS1     0x40080U    //RE7
#define TDC_CS2     0x60080U    //RG7
#define TDC_CS3     0x60100U    //RG8
#define D0          0x60200U    //RG9
#define D1          0x10020U    //RB5
#define D2          0x10010U    //RB4
#define TDC_INTB1   0x10004U    //RB2

#define TDC_TRG1    0x10040U    //RB6
#define TDC_INTB2   0x10080U    //RB7
#define TDC_TRG2    0x10100U    //RB8
#define TDC_INTB3   0x10200U    //RB9
#define GPS_PPS     0x10400U    //RB10
#define TDC_TRG3    0x10800U    //RB11
#define TDC_STR     0x11000U    //RB12, Schematic name: "RB12"
#define GPS_RST     0x12000U    //RB13

#define TDC_EN      0x50020U    //RF5
#define RFM95W_DIO0 0x30001U    //RD0
#define FT232_RST   0x22000U    //RC13

#define RFM95W_DIO4 0x30008U    //RD3
#define RFM95W_CS   0x30010U    //RD4
#define RFM95W_DIO1 0x30020U    //RD5
#define RFM95W_DIO5 0x50001U    //RF0
#define RFM95W_DIO3 0x50002U    //RF1
#define RFM95W_RST  0x40002U    //RE1
#define LED_1       0x40004U    //RE2
#define LED_2       0x40008U    //RE3
#define LED_3       0x40010U    //RE4

#endif