/**
  ******************************************************************************
  * @file    rc5_decode.c
  * @brief   RC5 infrared decoder implementatie
  *          Aangepast voor gebruik met STM32L432KC + TIM2 PWM Input mode
  *          Gebaseerd op ST Application Note AN4834
  *
  * Werking:
  *   TIM2 is geconfigureerd in PWM Input mode op PA0 (TSOP4838 uitgang):
  *   - CH1 (PA0, dalende flank): meet de volledige periode (1T of 2T)
  *   - CH2 (indirect, stijgende flank): meet de duur van de lage puls
  *   - Timer overflow (3.6 ms geen flank): reset het huidige pakket
  *
  *   Elke interrupt roept RC5_DataSampling() aan met de pulslengte en flankrichting.
  *   Zodra 13 bits correct ontvangen zijn, wordt RC5FrameReceived op YES gezet.
  *   De hoofdlus pollt RC5FrameReceived en roept RC5_Decode() aan om het frame
  *   te verwerken.
  *
  * Timertiming (vereist prescaler = 31 bij 32 MHz -> 1 µs per tick):
  *   - Halve bitperiode (1T) : 889 µs (tolerantie: 600 - 1200 µs)
  *   - Volledige bitperiode (2T): 1778 µs (tolerantie: 1500 - 2100 µs)
  *   - Timeout: 3600 µs (geen flank -> frame reset)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rc5_decode.h"
#include "tim.h"

/* Private constanten --------------------------------------------------------*/

/* Tijdswaarden in timer ticks (1 tick = 1 µs bij prescaler=31 en 32MHz klok) */
#define RC5_MIN_1T          600U    /*!< Minimale 1T pulslengte (µs) */
#define RC5_MAX_1T          1200U   /*!< Maximale 1T pulslengte (µs) */
#define RC5_MIN_2T          1500U   /*!< Minimale 2T pulslengte (µs) */
#define RC5_MAX_2T          2100U   /*!< Maximale 2T pulslengte (µs) */

#define RC5_1T_TIME         ((uint8_t)0x00)   /*!< Puls = 1T */
#define RC5_2T_TIME         ((uint8_t)0x01)   /*!< Puls = 2T */
#define RC5_WRONG_TIME      ((uint8_t)0xFF)   /*!< Puls buiten geldige range */

#define RC5_PACKET_BIT_COUNT        13U       /*!< Totaal aantal RC5 bits */
#define RC5_PACKET_STATUS_EMPTY     ((uint8_t)(1 << 0))

/* Logische overgangs-tabellen voor Manchester decodering --------------------*/

/* Stijgende flank tabel: [vorig bit][pulstype] -> nieuw bit */
static const RC5_lastBit_t RC5_logicTableRisingEdge[2][2] =
{
  { RC5_ZER, RC5_INV },  /* vorig bit = RC5_ZER */
  { RC5_NAN, RC5_ZER },  /* vorig bit = RC5_ONE */
};

/* Dalende flank tabel: [vorig bit][pulstype] -> nieuw bit */
static const RC5_lastBit_t RC5_logicTableFallingEdge[2][2] =
{
  { RC5_NAN, RC5_ONE },  /* vorig bit = RC5_ZER */
  { RC5_ONE, RC5_INV },  /* vorig bit = RC5_ONE */
};

/* Globale variabelen --------------------------------------------------------*/
__IO StatusYesOrNo_t RC5FrameReceived = NO;  /*!< YES als volledig frame ontvangen */
__IO RC5_Packet_t    RC5TmpPacket;            /*!< Tijdelijk pakket tijdens ontvangst */
RC5_Frame_t          RC5_FRAME;              /*!< Laatste volledig ontvangen frame */

/* Private functie prototypes ------------------------------------------------*/
static uint8_t RC5_GetPulseLength(uint32_t pulseLength);
static void    RC5_modifyLastBit(RC5_lastBit_t bit);
static void    RC5_WriteBit(uint8_t bitVal);

/* Functies ------------------------------------------------------------------*/

/**
  * @brief  Reset het tijdelijke RC5 pakket naar de beginstaat.
  *         Wordt aangeroepen bij opstart, na ontvangst en bij timeout.
  */
void RC5_ResetPacket(void)
{
  RC5TmpPacket.data     = 0;
  RC5TmpPacket.bitCount = RC5_PACKET_BIT_COUNT - 1;
  RC5TmpPacket.lastBit  = RC5_ONE;
  RC5TmpPacket.status   = RC5_PACKET_STATUS_EMPTY;
}

/**
  * @brief  Decodeer het ontvangen RC5 frame naar adres, commando en toggle bit.
  *         Roep deze functie aan wanneer RC5FrameReceived == YES.
  * @param  pIRFrame: pointer naar RC5_Frame_t structuur voor het resultaat
  *
  * Voorbeeld:
  *   if (RC5FrameReceived == YES) {
  *       RC5_Decode(&RC5_FRAME);
  *       printf("Adres: %d | Commando: %d | Toggle: %d\r\n",
  *              RC5_FRAME.Address, RC5_FRAME.Command, RC5_FRAME.ToggleBit);
  *   }
  */
void RC5_Decode(RC5_Frame_t *pIRFrame)
{
  if (RC5FrameReceived != NO)
  {
    /* Splits de 13-bit data op in de RC5 velden */
    pIRFrame->FieldBit  = (RC5TmpPacket.data >> 12) & 0x01;
    pIRFrame->ToggleBit = (RC5TmpPacket.data >> 11) & 0x01;
    pIRFrame->Address   = (RC5TmpPacket.data >> 6)  & 0x1F;
    pIRFrame->Command   = (uint8_t)(RC5TmpPacket.data & 0x3F);

    /* Als field bit = 0 -> commando ligt in het bovenste veld (64-127) */
    if (pIRFrame->FieldBit == 0)
    {
      pIRFrame->Command |= (1 << 6);
    }

    /* Klaar: reset voor volgend frame */
    RC5FrameReceived = NO;
    RC5_ResetPacket();
  }
}

/**
  * @brief  Verwerk een gemeten pulslengte en flankrichting.
  *         Wordt aangeroepen vanuit de TIM2 interrupt callback.
  * @param  rawPulseLength: pulslengte in timer ticks (1 tick = 1 µs)
  * @param  edge: 1 = stijgende flank (CH2), 0 = dalende flank (CH1)
  */
void RC5_DataSampling(uint32_t rawPulseLength, uint32_t edge)
{
  uint8_t        pulse;
  RC5_lastBit_t  tmp_bit;

  /* Zet de pulslengte om naar 1T of 2T */
  pulse = RC5_GetPulseLength(rawPulseLength);

  if (edge == 1)  /* Stijgende flank */
  {
    if (pulse <= RC5_2T_TIME)
    {
      tmp_bit = RC5_logicTableRisingEdge[RC5TmpPacket.lastBit][pulse];
      RC5_modifyLastBit(tmp_bit);
    }
    else
    {
      RC5_ResetPacket();
    }
  }
  else  /* Dalende flank */
  {
    /* Eerste dalende flank: markeer pakket als gestart, geen bitberekening */
    if (RC5TmpPacket.status & RC5_PACKET_STATUS_EMPTY)
    {
      RC5TmpPacket.status &= (uint8_t)~RC5_PACKET_STATUS_EMPTY;
    }
    else
    {
      if (pulse <= RC5_2T_TIME)
      {
        tmp_bit = RC5_logicTableFallingEdge[RC5TmpPacket.lastBit][pulse];
        RC5_modifyLastBit(tmp_bit);
      }
      else
      {
        RC5_ResetPacket();
      }
    }
  }
}

/* Private hulpfuncties ------------------------------------------------------*/

/**
  * @brief  Categoriseer de pulslengte als 1T, 2T of ongeldig.
  * @param  pulseLength: gemeten pulslengte in µs (timer ticks)
  * @retval RC5_1T_TIME, RC5_2T_TIME of RC5_WRONG_TIME
  */
static uint8_t RC5_GetPulseLength(uint32_t pulseLength)
{
  if ((pulseLength > RC5_MIN_1T) && (pulseLength < RC5_MAX_1T))
  {
    return RC5_1T_TIME;
  }
  else if ((pulseLength > RC5_MIN_2T) && (pulseLength < RC5_MAX_2T))
  {
    return RC5_2T_TIME;
  }
  return RC5_WRONG_TIME;
}

/**
  * @brief  Verwerk het nieuwe bit en sla het op in het pakket.
  */
static void RC5_modifyLastBit(RC5_lastBit_t bit)
{
  if (bit != RC5_NAN)
  {
    if (RC5TmpPacket.lastBit != RC5_INV)
    {
      RC5TmpPacket.lastBit = bit;
      RC5_WriteBit(RC5TmpPacket.lastBit);
    }
    else
    {
      RC5_ResetPacket();
    }
  }
}

/**
  * @brief  Schrijf een bit (0 of 1) naar het data veld van het pakket.
  *         Als alle 13 bits ontvangen zijn, zet RC5FrameReceived op YES.
  */
static void RC5_WriteBit(uint8_t bitVal)
{
  if (bitVal == RC5_ONE)
  {
    bitVal = 1;
  }
  else if (bitVal == RC5_ZER)
  {
    bitVal = 0;
  }
  else
  {
    RC5_ResetPacket();
    return;
  }

  RC5TmpPacket.data |= bitVal;

  if (RC5TmpPacket.bitCount != 0)
  {
    RC5TmpPacket.data = RC5TmpPacket.data << 1;
    RC5TmpPacket.bitCount--;
  }
  else
  {
    /* Alle 13 bits ontvangen -> frame is compleet */
    RC5FrameReceived = YES;
  }
}

/* TIM2 Interrupt callbacks --------------------------------------------------*/

/**
  * @brief  HAL callback voor Input Capture events van TIM2.
  *         Wordt automatisch aangeroepen door de HAL bij elke flank op PA0.
  *
  *         CH1 (dalende flank): leest CCR1 = totale periode (1T of 2T)
  *         CH2 (stijgende flank): leest CCR2 = lage pulsduur
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      /* Dalende flank: CCR1 bevat de periode tussen twee dalende flanken */
      uint32_t period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      RC5_DataSampling(period, 0);  /* edge = 0: dalende flank */
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      /* Stijgende flank: CCR2 bevat de duur van de lage puls */
      uint32_t pulseWidth = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      RC5_DataSampling(pulseWidth, 1);  /* edge = 1: stijgende flank */
    }
  }
}

/**
  * @brief  HAL callback voor timer overflow (period elapsed) van TIM2.
  *         Als er 3.6 ms geen flank ontvangen werd, reset het pakket.
  *         Dit voorkomt dat een onvolledig frame als geldig beschouwd wordt.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    RC5_ResetPacket();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
