/**
  ******************************************************************************
  * @file    rc5_decode.h
  * @brief   RC5 infrared decoder - header file
  *          Aangepast voor gebruik met STM32L432KC + TIM2 PWM Input mode
  *          Gebaseerd op ST Application Note AN4834
  ******************************************************************************
  */

#ifndef __RC5_DECODE_H
#define __RC5_DECODE_H

#include <stdint.h>

/* StatusYesOrNo type definitie */
typedef enum
{
  NO  = 0,
  YES = 1
} StatusYesOrNo_t;

/**
  * @brief  RC5 frame structure (14-bits RC5 protocol)
  *         S | F | Toggle | Address (5 bits) | Command (6 bits)
  */
typedef struct
{
  __IO uint8_t FieldBit;    /*!< Field bit (F): selecteert boven/onder commando veld */
  __IO uint8_t ToggleBit;   /*!< Toggle bit: wisselt bij elke nieuwe toetsdruk */
  __IO uint8_t Address;     /*!< Adres veld (5 bits: 0-31, apparaatselectie) */
  __IO uint8_t Command;     /*!< Commando veld (6 bits: 0-63) */
} RC5_Frame_t;

/**
  * @brief  RC5 intern pakket (tijdelijke opslag tijdens ontvangst)
  */
typedef struct
{
  __IO uint16_t data;       /*!< Ontvangen RC5 data bits */
  __IO uint8_t  status;     /*!< Pakket status (leeg of in ontvangst) */
  __IO uint8_t  lastBit;    /*!< Laatste geldig ontvangen bit */
  __IO uint8_t  bitCount;   /*!< Teller voor ontvangen bits */
} RC5_Packet_t;

/**
  * @brief  Vorige bit status type (voor Manchester decodering)
  */
typedef enum
{
  RC5_ZER = 0,  /*!< Logische '0' */
  RC5_ONE,      /*!< Logische '1' */
  RC5_NAN,      /*!< Geen nieuw bit (overgang naar zelfde niveau) */
  RC5_INV       /*!< Ongeldig bit (fout in frame) */
} RC5_lastBit_t;

/* Globale variabelen (extern, gedefinieerd in rc5_decode.c) */
extern __IO StatusYesOrNo_t RC5FrameReceived;  /*!< YES als volledig RC5 frame ontvangen */
extern __IO RC5_Packet_t    RC5TmpPacket;       /*!< Tijdelijk pakket tijdens ontvangst */
extern RC5_Frame_t          RC5_FRAME;          /*!< Laatste volledig ontvangen RC5 frame */

/* Publieke functies */
void RC5_ResetPacket(void);
void RC5_DataSampling(uint32_t rawPulseLength, uint32_t edge);
void RC5_Decode(RC5_Frame_t *pIRFrame);

#endif /* __RC5_DECODE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
