# NUC1262BSP_BPWM_output_low
 NUC1262BSP_BPWM_output_low

update @ 2022/07/12

1. use BPWM1 CH0 (PF.15) , to generate PWM freq and duty ( default : freq 2048 , duty 20)
 
2. check define : USE_PWM_API , define : USE_PWM_DISCRETE , for PWM output init

3. by using UART terminal , to control PWM operation

	- digit a/A : increase DUTY
	
	- digit c/C : decrease DUTY
	
	- digit 1 : turn ON PWM ( PWM re-initial )
	
	- digit 2 : turn OFF PWM ( PWM output low with MASK)
	
	- digit 3 : PWM module turn on and PWM initial
	
	- digit 4 : PWM module turn off and set as GPIO LOW

4. Below is log capture , 

![image](https://github.com/released/NUC1262BSP_BPWM_output_low/blob/main/log.jpg)

below is scope capture when adjust duty

![image](https://github.com/released/NUC1262BSP_BPWM_output_low/blob/main/waveform.jpg)


