/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "project_config.h"


/*
	Target : 250 Freq
	DUTY : 50%
	
	SYS_CLK : 72M
	PSC : 1

	72 000 000/30 000 = PSC * (CNR + 1)
	CNR = (SYS_CLK/FREQ)/PSC - 1 = 57600 - 1

	DUTY ratio = CMR / (CNR + 1)
	50% = CMR / (CNR + 1)
	
*/
#define SYS_CLOCK       							(FREQ_72MHZ)

#define PWM_CHANNEL                                 (0)
#define PWM_CHANNEL_MASK                            (BPWM_CH_0_MASK)
#define PWM_PSC 								    (5)	
#define PWM_FREQ 								    (2048)	

#define PWM_RESOLUTION                        	    (100)
#define PWM_TARGET_DUTY(d)					        ((PWM_RESOLUTION*(100-d))/100)	//((PWM_RESOLUTION*d)/100)
#define PWM_DUTY                              	    (PWM_TARGET_DUTY(20))		//percent

// #define USE_PWM_API
#define USE_PWM_DISCRETE


//16 bit
/*
	Up-Down Counter Type : 
	BPWM period time =(2*PERIOD) * (CLKPSC+1) * BPWMx_CLK. 

	Up Counter Type  or Down Counter Type 
	BPWM period time = (PERIOD+1) *(CLKPSC+1)* BPWMx_CLK. 
	
*/

// #define PWM_CNR 								    ((SYS_CLOCK/PWM_FREQ)/PWM_PSC - 1)			//Up Counter Type  or Down Counter Type
#define PWM_CNR 								    (((SYS_CLOCK/PWM_FREQ)/PWM_PSC - 1)>>1 )	//Up-Down Counter Type
#define PWM_CMR 								    (PWM_DUTY * (PWM_CNR + 1)/PWM_RESOLUTION)

#define CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)	    (u32DutyCycle * (BPWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution)
#define CalNewDuty(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)		(BPWM_SET_CMR(pwm, u32ChannelNum, CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)))

uint16_t pwm_out_duty = 0;
uint32_t pwm_out_freq = PWM_FREQ;



/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;
volatile uint32_t counter_systick = 0;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void systick_counter(void)
{
	counter_systick++;
}

uint32_t get_systick(void)
{
	return (counter_systick);
}

void set_systick(uint32_t t)
{
	counter_systick = t;
}

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}


//PF15 , BPWM1_CH0 : PWM output
void PWM_Init(void)
{
    SYS_ResetModule(BPWM1_RST);
    
    pwm_out_duty = 20;//default

    #if defined (USE_PWM_DISCRETE)

	BPWM_SET_ALIGNED_TYPE(BPWM1, PWM_CHANNEL_MASK, BPWM_CENTER_ALIGNED);

    /* Set PWM0 timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM1, PWM_CHANNEL, PWM_PSC - 1);
	
    /* Set PWM0 timer period */
    BPWM_SET_CNR(BPWM1, PWM_CHANNEL, PWM_CNR);
	
    /* Set PWM0 timer duty */
    BPWM_SET_CMR(BPWM1, PWM_CHANNEL, PWM_CMR);	
	
    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    // BPWM_SET_OUTPUT_LEVEL(BPWM1, PWM_CHANNEL_MASK, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    BPWM_SET_OUTPUT_LEVEL(BPWM1, PWM_CHANNEL_MASK, BPWM_OUTPUT_LOW, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_LOW);

    #elif defined (USE_PWM_API)
    /* set BPWM1 channel 0 output configuration */
    BPWM_ConfigOutputChannel(BPWM1, PWM_CHANNEL, PWM_FREQ, pwm_out_duty);

    #endif

    /* Enable BPWM Output path for BPWM1 channel 0 */
    BPWM_EnableOutput(BPWM1, PWM_CHANNEL_MASK);

    /* Enable Timer for BPWM1 channel 0 */
    BPWM_Start(BPWM1, PWM_CHANNEL_MASK);    

    printf("%s\r\n",__FUNCTION__);
}

void loop(void)
{
	static uint32_t LOG = 0;

    if (is_flag_set(flag_duty_change))
    {
        set_flag(flag_duty_change ,DISABLE);

        /* set BPWM1 channel 0 output configuration */
        #if defined (USE_PWM_DISCRETE)
		CalNewDuty(BPWM1, PWM_CHANNEL, PWM_TARGET_DUTY(pwm_out_duty), PWM_RESOLUTION);
        #elif defined (USE_PWM_API)
        BPWM_ConfigOutputChannel(BPWM1, PWM_CHANNEL, pwm_out_freq, pwm_out_duty);
        #endif
    }

    if (is_flag_set(flag_PWM_turn_on))
    {
        set_flag(flag_PWM_turn_on ,DISABLE);

        PWM_Init();    
    }

    if (is_flag_set(flag_PWM_turn_off))
    {
        set_flag(flag_PWM_turn_off ,DISABLE);

        BPWM_MASK_OUTPUT(BPWM1,PWM_CHANNEL_MASK, 0x00);
        BPWM_Stop(BPWM1, PWM_CHANNEL_MASK);
        // BPWM_DisableOutput(BPWM1, PWM_CHANNEL_MASK);	
        // BPWM_ForceStop(BPWM1, PWM_CHANNEL_MASK);
       
    }   


    if (is_flag_set(flag_PWM_MODULE_on))
    {
        set_flag(flag_PWM_MODULE_on ,DISABLE);

        SYS_UnlockReg();
        CLK_EnableModuleClock(BPWM1_MODULE);
        SYS->GPF_MFPH &= ~(SYS_GPF_MFPH_PF15MFP_Msk);
        SYS->GPF_MFPH |= SYS_GPF_MFPH_PF15MFP_BPWM1_CH0;
        /* Lock protected registers */
        SYS_LockReg();
        
        PWM_Init();
    }


    if (is_flag_set(flag_PWM_MODULE_off))
    {
        set_flag(flag_PWM_MODULE_off ,DISABLE);

		pwm_out_duty = 0;	
        #if defined (USE_PWM_DISCRETE)
		CalNewDuty(BPWM1, PWM_CHANNEL, PWM_TARGET_DUTY(pwm_out_duty), PWM_RESOLUTION);
        #elif defined (USE_PWM_API)
        BPWM_ConfigOutputChannel(BPWM1, PWM_CHANNEL, pwm_out_freq, pwm_out_duty);
        #endif

        BPWM_Stop(BPWM1, PWM_CHANNEL_MASK);
        BPWM_DisableOutput(BPWM1, PWM_CHANNEL_MASK);		

        SYS_UnlockReg();
        CLK_DisableModuleClock(BPWM1_MODULE);
        SYS->GPF_MFPH &= ~(SYS_GPF_MFPH_PF15MFP_Msk);
        SYS->GPF_MFPH |= SYS_GPF_MFPH_PF15MFP_GPIO;
        /* Lock protected registers */
        SYS_LockReg();
        PF15 = 0;
    }    

    if (is_flag_set(flag_period_1000ms))
    {
        set_flag(flag_period_1000ms ,DISABLE);

        printf("%s : %4d\r\n",__FUNCTION__,LOG++);
        PB14 ^= 1;        
    }
  

}

void GPIO_Init (void)
{
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | (SYS_GPB_MFPH_PB15MFP_GPIO);
	
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	
}


void TMR1_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

        if ((get_tick() % 1000) == 0)
        {
            set_flag(flag_period_1000ms ,ENABLE);
        }  

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
        printf("digit :%c\r\n",res);
		switch(res)
		{
			case 'a':
			case 'A':
                set_flag(flag_duty_change ,ENABLE);
                pwm_out_duty += 5;
                if (pwm_out_duty >= 100)
                {
                    pwm_out_duty = 100;
                }
                printf("pwm_out_duty = %2d\r\n",pwm_out_duty);
				break;

			case 'd':
			case 'D':    
                set_flag(flag_duty_change ,ENABLE);
                pwm_out_duty -= 5;
                if (pwm_out_duty <= 5)
                {
                    pwm_out_duty = 5;
                }      
                printf("pwm_out_duty = %2d\r\n",pwm_out_duty);              
				break;    

			case '1':
                // TURN ON
                set_flag(flag_PWM_turn_on ,ENABLE);
				break;                            

			case '2':    
                // TURN OFF
                set_flag(flag_PWM_turn_off ,ENABLE);
				break;   

			case '3':
                set_flag(flag_PWM_MODULE_on ,ENABLE);
				break;                            

			case '4':    
                set_flag(flag_PWM_MODULE_off ,ENABLE);
				break;                 

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void UART0_IRQHandler(void)
{

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART0_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());		
	#endif	

}

void SYS_Init(void)
{
    SYS_UnlockReg();	

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
//    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
//    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /* Enable GPIO Port F module clock */
//    CLK_EnableModuleClock(GPIOF_MODULE);

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) as input mode to use HXT */
//    GPIO_SetMode(PF, BIT2|BIT3, GPIO_MODE_INPUT);


//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);		
	
    /* Set core clock to 72MHz */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
//    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC_DIV2, 0);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

//    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | (UART0_RXD_PB12 | UART0_TXD_PB13);

    // PF15 , BPWM1_CH0 : PWM output 
    CLK_EnableModuleClock(BPWM1_MODULE);    
    SYS->GPF_MFPH &= ~(SYS_GPF_MFPH_PF15MFP_Msk);
    SYS->GPF_MFPH |= SYS_GPF_MFPH_PF15MFP_BPWM1_CH0;

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();

    PWM_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
