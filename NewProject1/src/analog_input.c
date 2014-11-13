/* GPL code for Wiznet Challenge 2014 */

#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include "task.h"
#include "analog_input.h"


void adc_configure(void)
{
    // zet de ADC van de A0 input voor de keys goed,
    // de ADC gaat 8 bit werken, geen 16 of 12, nergens voor nodig, veel te veel ruis
    //

    ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    GPIO_InitTypeDef GPIO_initStructure; //Structure for analog input pin
//Clock configuration RCC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOAEN,ENABLE);//Clock for the ADC port!! Do not forget about this one ;)
//Analog pin configuration
    GPIO_StructInit(&GPIO_initStructure); // set some sane defaults in the struct
    //
    GPIO_initStructure.GPIO_Pin = GPIO_Pin_0;//The channel 0 is connected to PA0
    GPIO_initStructure.GPIO_Mode = GPIO_Mode_AN; //The pin is configured in analog mode
    GPIO_initStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
    GPIO_Init(GPIOA,&GPIO_initStructure);//Affecting the port with the initialization structure configuration
//ADC structure configuration
    // ADC_DeInit();
    ADC_StructInit(&ADC_init_structure); // sane defaults in struct
    /* ADC Common Init */
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // iets langzamer
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    //
    ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
    ADC_init_structure.ADC_Resolution = ADC_Resolution_8b;//Input voltage is converted into a 12bit number giving a maximum value of 4096
    ADC_init_structure.ADC_ContinuousConvMode = DISABLE; // ENABLE; // enkele conversie per keer op aanvraag
    ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;// conversion is synchronous with T2 ??
    ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;// ADC_ExternalTrigConvEdge_None;//no trigger for conversion
    ADC_init_structure.ADC_NbrOfConversion = 1;

    ADC_init_structure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
    ADC_Init(ADC1,&ADC_init_structure);//Initialize ADC with the previous configuration

//Select the channel to be read from 0 dus in dit geval
    ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_144Cycles);
//Enable ADC conversion

    ADC_Cmd(ADC1,ENABLE);

    // enable the interrupts
    // NVIC_InitTypeDef    NVIC_adc;

    // NVIC_adc.NVIC_IRQChannel = ADC_IRQn;
    //  NVIC_adc.NVIC_IRQChannelCmd = ENABLE;
    //  NVIC_adc.NVIC_IRQChannelPreemptionPriority = 0x0F;
    //  NVIC_adc.NVIC_IRQChannelSubPriority = 0x01;
//   NVIC_Init(&NVIC_adc);

}

uint16_t adc_convert()
{
    ADC_SoftwareStartConv(ADC1);//Start the conversion
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
    return ADC_GetConversionValue(ADC1); //Return the converted data
}


uint8_t val2key(uint16_t key)
{
    // xprintf("%d\r\n",key);
    // gemeten waarden
    // 227 is SELECT key
    // 188 is LEFT key
    // 55 is UP key
    // 124 is DOWN key
    // 2 of 1 of 0 is RIGHT key

    if (key < 15) return 'R'; // right
    if (key < 65 && key > 45) return 'U'; // up
    if (key < 134 && key > 114) return 'D'; // down
    if (key < 198 && key > 178) return 'L'; // left
    if (key < 237 && key > 217) return 'S'; // select
    return '-'; // geen waarde, no key
}

#if 0
int main(void)
{
    adc_configure();//Start configuration

    int ConvertedValue = 0; //Converted value read from ADC
    while(1) { //loop while the board is working
        ConvertedValue = adc_convert();//Read the ADC converted value
    }
}
#endif
