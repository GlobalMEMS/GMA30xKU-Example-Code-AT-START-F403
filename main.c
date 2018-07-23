/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : main.c
 *
 * Usage: main function
 *
 ****************************************************************************
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

/*! @file main.c
 *  @brief main program
 *  @author Joseph FC Tseng
 */

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "stdbool.h"
#include "math.h"
#include "at32f4xx.h"
#include "at32f4xx_dbgmcu.h"
#include "i2c_gmems.h"
#include "bus_support.h"
#include "gma30xku.h"
#include "gSensor_autoNil.h"
#include "Lcd_Driver.h"
#include "GUI.h"
#include "usart.h"
#include "delay.h"
#include "key.h"
#include "string.h"
#include "math.h"

/* Private macro -------------------------------------------------------------*/
#define RADIAN_TO_DEGREE            (180. / 3.14159265358979323846) //1 radian = 180/pi degree

/* global variables ---------------------------------------------------------*/
u8 ui8StartAutoNilFlag = 0;

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
    {}
}

#endif

const u16 RESOLUTION_X = 128;
const u16 RESOLUTION_Y = 120;
const u16 FONT_HEIGHT = 16;
const u16 LINE_HEIGHT = FONT_HEIGHT + 2;
const u16 MAX_DISPLAY_ITEM = 9;
void showMsg(u16 x, u16 line, u8* str, u16 color, u8 reDraw){

  int i;
  char* subStr;

  if(reDraw) Lcd_Clear(GRAY0);

  subStr = strtok((char*)str, "\n");

  for(i = line; subStr; ++i){
    Gui_DrawFont_GBK16(x, LINE_HEIGHT * i, color, GRAY0, (u8*)subStr);
    subStr = strtok(NULL, "\n");
  }
}

void floatCatToSciStr(float fIn, u8 precision, u8* outStr){

  s32 i = 0;
  float fTmp;
  s32 s32Dec, s32Dig, s32Exp;

  if(fIn == 0){
    s32Dec = s32Dig = s32Exp = 0;
  }
  else{

    s32Exp = 0;
    fTmp = fIn;

    if(fabs(fTmp) < 1){
      do{
	s32Exp -= 1;
	fTmp *= 10.0f;
      }while(fabs(fTmp) < 1);
    }
    else if(fabs(fTmp) >= 10){
      do{
	s32Exp += 1;
	fTmp /= 10.0f;
      }while(fabs(fTmp) >= 10);
    }

    s32Dec = (s32)fTmp;
    fTmp -= s32Dec;

    //precision
    for(i = 0; i < precision; ++i)
      fTmp *= 10;

    s32Dig = (s32)(fabs(fTmp));

  }

  itoa(s32Dec, &outStr[strlen((const char*)outStr)]);
  strcat((char*)outStr, ".");

  fTmp = 1;
  for(i = 0; i < precision; ++i)
    fTmp *= 10;
  for(i = 0; i < precision; ++i){
    fTmp /= 10;
    if(s32Dig < fTmp){
      strcat((char*)outStr, "0");
    }
    else{
      itoa(s32Dig, &outStr[strlen((const char*)outStr)]);
      break;
    }
  }
  if(s32Exp != 0){
    strcat((char*)outStr, "e");
    itoa(s32Exp, &outStr[strlen((const char*)outStr)]);
  }
}

/**
 * @brief   Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  bus_support_t gma30xku_bus;
  raw_data_xyzt_t rawData;
  raw_data_xyzt_t offsetData;
  raw_data_xyzt_t calibData;
  float fTilt_degree;
  float gVal[3];
  int i;
  u8 str[64];

  /* System Initialization */
  SystemInit();

  /* I2C1 initialization */
  I2C1_Init();

  /* Init Key */
  KEY_Init();

  /* Initialize the LCD */
  uart_init(19200);
  delay_init();
  Lcd_Init();

  /* GMA30xKU I2C bus setup */
  bus_init_I2C1(&gma30xku_bus, GMA30xKU_8BIT_I2C_ADDR);  //Initialize bus support to I2C1
  gma30xku_bus_init(&gma30xku_bus);  //Initailze GMA30xKU bus to I2C1

  /* GMA30xKU soft reset */
  gma30xku_soft_reset();

  /* GMA30xKU initialization */
  gma30xku_initialization();

  /* User message: press Key1 to start offset AutoNil */
  strcpy((char*)str, "Hold g-sensor in\nlevel for offset\nAutoNil.");
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "Press Key1 when\nready.");
  showMsg(0, 4, str, RED, 0);

  do{
    delay_ms(10);
  }while(KEY_Scan() != KEY1_PRES);

  //Conduct g-sensor AutoNil, gravity is along the positive Z-axis
  gSensorAutoNil(gma30xku_read_data_xyz, AUTONIL_POSITIVE + AUTONIL_Z, GMA30xKU_RAW_DATA_SENSITIVITY, &offsetData);

  /* User message: show offset */
  strcpy((char*)str, "Offset(code):\nX= ");
  itoa(offsetData.u.x, &str[strlen((const char*)str)]);
  strcat((char*)str, "\nY= ");
  itoa(offsetData.u.y, &str[strlen((const char*)str)]);
  strcat((char*)str, "\nZ= ");
  itoa(offsetData.u.z, &str[strlen((const char*)str)]);
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "Press Key1 to\ncontinue");
  showMsg(0, 5, str, RED, 0);

  do{
    delay_ms(10);
  }while(KEY_Scan() != KEY1_PRES);

  strcpy((char*)str, "Raw XYZT(code):");
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "Calib XYZ(code):");
  showMsg(0, 2, str, BLACK, 0);
  strcpy((char*)str, "G-value (g):");
  showMsg(0, 4, str, BLACK, 0);
  strcpy((char*)str, "Tilt:       deg");
  showMsg(0, 8, str, BLACK, 0);

  while (1){

    /* Read XYZT data */
    gma30xku_read_data_xyzt(&rawData);

    //Offset compensation
    calibData.u.x = rawData.u.x - offsetData.u.x;
    calibData.u.y = rawData.u.y - offsetData.u.y;
    calibData.u.z = rawData.u.z - offsetData.u.z;

    //Raw data to g value
    for(i = 0; i < 3; ++i)
      gVal[i] = (float)calibData.v[i] / GMA30xKU_RAW_DATA_SENSITIVITY;

    //Tilt angle
    fTilt_degree = acos(calibData.u.z
			/ sqrt(calibData.u.x*calibData.u.x + calibData.u.y*calibData.u.y + calibData.u.z*calibData.u.z)
			) * RADIAN_TO_DEGREE;

    /* User message: Raw data*/
    strcpy((char*)str, "");
    itoa(rawData.u.x, &str[strlen((const char*)str)]);
    strcat((char*)str, ",");
    itoa(rawData.u.y, &str[strlen((const char*)str)]);
    strcat((char*)str, ",");
    itoa(rawData.u.z, &str[strlen((const char*)str)]);
    strcat((char*)str, ",");
    itoa(rawData.u.t, &str[strlen((const char*)str)]);
    strcat((char*)str, "       ");
    showMsg(0, 1, str, BLUE, 0);

    /* User message: Calibrated data*/
    strcpy((char*)str, "");
    itoa(calibData.u.x, &str[strlen((const char*)str)]);
    strcat((char*)str, ",");
    itoa(calibData.u.y, &str[strlen((const char*)str)]);
    strcat((char*)str, ",");
    itoa(calibData.u.z, &str[strlen((const char*)str)]);
    strcat((char*)str, "       ");
    showMsg(0, 3, str, BLUE, 0);

    /* User message: g-value */
    strcpy((char*)str, "");
    strcat((char*)str, "gx= ");
    floatCatToSciStr(gVal[0], 3, str);
    strcat((char*)str, "       \ngy= ");
    floatCatToSciStr(gVal[1], 3, str);
    strcat((char*)str, "       \ngz= ");
    floatCatToSciStr(gVal[2], 3, str);
    strcat((char*)str, "       ");
    showMsg(0, 5, str, BLUE, 0);

    /* User message: Tilt angle */
    strcpy((char*)str, "");
    itoa((s32)(fTilt_degree > 0 ? (fTilt_degree+0.5f):(fTilt_degree-0.5f)), &str[strlen((const char*)str)]);
    strcat((char*)str, "  ");
    showMsg(50, 8, str, BLUE, 0);

    /* Delay 1 sec */
    delay_ms(1000);
  }
}

