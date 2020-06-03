#include "platform.h"
#include "stm32f3xx.h"

void log_float(uint8_t channel, float val)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
        ((ITM->TER & (1UL<<channel)) != 0UL))                  /* ITM Port #channel enabled */
    {
        while (ITM->PORT[channel].u32 == 0UL)
        {
            __NOP();
        }
        ITM->PORT[channel].u32 = *(uint32_t*)&val;
    }
}

void log_uint8 (uint8_t channel, uint8_t val)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
        ((ITM->TER & (1UL<<channel)) != 0UL))                  /* ITM Port #channel enabled */
    {
        while (ITM->PORT[channel].u32 == 0UL)
        {
            __NOP();
        }
        ITM->PORT[channel].u8 = val;
    }
}

void log_int8  (uint8_t channel, int8_t val)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
        ((ITM->TER & (1UL<<channel)) != 0UL))                  /* ITM Port #channel enabled */
    {
        while (ITM->PORT[channel].u32 == 0UL)
        {
            __NOP();
        }
        ITM->PORT[channel].u8 = (uint8_t)val;
    }
}

void log_uint16(uint8_t channel, uint16_t val)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
        ((ITM->TER & (1UL<<channel)) != 0UL))                  /* ITM Port #channel enabled */
    {
        while (ITM->PORT[channel].u32 == 0UL)
        {
            __NOP();
        }
        ITM->PORT[channel].u16 = val;
    }

}

void log_int16 (uint8_t channel, int16_t val)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
        ((ITM->TER & (1UL<<channel)) != 0UL))                  /* ITM Port #channel enabled */
    {
        while (ITM->PORT[channel].u32 == 0UL)
        {
            __NOP();
        }
        ITM->PORT[channel].u16 = (uint16_t)val;
    }
}

void log_uint32(uint8_t channel, uint32_t val)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
        ((ITM->TER & (1UL<<channel)) != 0UL))                  /* ITM Port #channel enabled */
    {
        while (ITM->PORT[channel].u32 == 0UL)
        {
            __NOP();
        }
        ITM->PORT[channel].u32 = val;
    }
}

void log_int32 (uint8_t channel, int32_t val)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
        ((ITM->TER & (1UL<<channel)) != 0UL))                  /* ITM Port #channel enabled */
    {
        while (ITM->PORT[channel].u32 == 0UL)
        {
            __NOP();
        }
        ITM->PORT[channel].u32 = (uint32_t)val;
    }
}

void log_str   (uint8_t channel, const char* str)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
        ((ITM->TER & (1UL<<channel)) != 0UL))                  /* ITM Port #channel enabled */
    {
        while(*str)
        {

            while (ITM->PORT[channel].u32 == 0UL)
            {
                __NOP();
            }
            ITM->PORT[channel].u8 = (uint8_t) *str++;
        }
    }
}
