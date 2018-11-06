unit atmega3208;

{$goto on}

interface

implementation

{$i avrcommon.inc}

procedure CRCSCAN_NMI_ISR; external name 'CRCSCAN_NMI'; // Interrupt 1 CRCSCAN
procedure BOD_VLM_ISR; external name 'BOD_VLM'; // Interrupt 2 BOD
procedure RTC_CNT_ISR; external name 'RTC_CNT'; // Interrupt 3 RTC
procedure RTC_PIT_ISR; external name 'RTC_PIT'; // Interrupt 4 RTC
procedure CCL_CCL_ISR; external name 'CCL_CCL'; // Interrupt 5 CCL
procedure PORTA_PORT_ISR; external name 'PORTA_PORT'; // Interrupt 6 PORTA
procedure TCA0_LUNF_ISR; external name 'TCA0_LUNF'; // Interrupt 7 TCA0

procedure TCA0_HUNF_ISR; external name 'TCA0_HUNF'; // Interrupt 8 TCA0
procedure TCA0_LCMP0_ISR; external name 'TCA0_LCMP0'; // Interrupt 9 TCA0

procedure TCA0_CMP1_ISR; external name 'TCA0_CMP1'; // Interrupt 10 TCA0

procedure TCA0_CMP2_ISR; external name 'TCA0_CMP2'; // Interrupt 11 TCA0

procedure TCB0_INT_ISR; external name 'TCB0_INT'; // Interrupt 12 TCB0
procedure TCB1_INT_ISR; external name 'TCB1_INT'; // Interrupt 13 TCB1
procedure TWI0_TWIS_ISR; external name 'TWI0_TWIS'; // Interrupt 14 TWI0
procedure TWI0_TWIM_ISR; external name 'TWI0_TWIM'; // Interrupt 15 TWI0
procedure SPI0_INT_ISR; external name 'SPI0_INT'; // Interrupt 16 SPI0
procedure USART0_RXC_ISR; external name 'USART0_RXC'; // Interrupt 17 USART0
procedure USART0_DRE_ISR; external name 'USART0_DRE'; // Interrupt 18 USART0
procedure USART0_TXC_ISR; external name 'USART0_TXC'; // Interrupt 19 USART0
procedure PORTD_PORT_ISR; external name 'PORTD_PORT'; // Interrupt 20 PORTD
procedure AC0_AC_ISR; external name 'AC0_AC'; // Interrupt 21 AC0
procedure ADC0_RESRDY_ISR; external name 'ADC0_RESRDY'; // Interrupt 22 ADC0
procedure ADC0_WCOMP_ISR; external name 'ADC0_WCOMP'; // Interrupt 23 ADC0
procedure PORTC_PORT_ISR; external name 'PORTC_PORT'; // Interrupt 24 PORTC
procedure TCB2_INT_ISR; external name 'TCB2_INT'; // Interrupt 25 TCB2
procedure USART1_RXC_ISR; external name 'USART1_RXC'; // Interrupt 26 USART1
procedure USART1_DRE_ISR; external name 'USART1_DRE'; // Interrupt 27 USART1
procedure USART1_TXC_ISR; external name 'USART1_TXC'; // Interrupt 28 USART1
procedure PORTF_PORT_ISR; external name 'PORTF_PORT'; // Interrupt 29 PORTF
procedure NVMCTRL_EE_ISR; external name 'NVMCTRL_EE'; // Interrupt 30 NVMCTRL
procedure USART2_RXC_ISR; external name 'USART2_RXC'; // Interrupt 31 USART2
procedure USART2_DRE_ISR; external name 'USART2_DRE'; // Interrupt 32 USART2
procedure USART2_TXC_ISR; external name 'USART2_TXC'; // Interrupt 33 USART2
procedure PORTB_PORT_ISR; external name 'PORTB_PORT'; // Interrupt 34 PORTB
procedure PORTE_PORT_ISR; external name 'PORTE_PORT'; // Interrupt 35 PORTE

procedure _FPC_start; assembler; nostackframe;
label
   _start;
 asm
   .init
   .globl _start

   jmp _start
   jmp CRCSCAN_NMI_ISR
   jmp BOD_VLM_ISR
   jmp RTC_CNT_ISR
   jmp RTC_PIT_ISR
   jmp CCL_CCL_ISR
   jmp PORTA_PORT_ISR
   jmp TCA0_LUNF_ISR

   jmp TCA0_HUNF_ISR
   jmp TCA0_LCMP0_ISR

   jmp TCA0_CMP1_ISR

   jmp TCA0_CMP2_ISR

   jmp TCB0_INT_ISR
   jmp TCB1_INT_ISR
   jmp TWI0_TWIS_ISR
   jmp TWI0_TWIM_ISR
   jmp SPI0_INT_ISR
   jmp USART0_RXC_ISR
   jmp USART0_DRE_ISR
   jmp USART0_TXC_ISR
   jmp PORTD_PORT_ISR
   jmp AC0_AC_ISR
   jmp ADC0_RESRDY_ISR
   jmp ADC0_WCOMP_ISR
   jmp PORTC_PORT_ISR
   jmp TCB2_INT_ISR
   jmp USART1_RXC_ISR
   jmp USART1_DRE_ISR
   jmp USART1_TXC_ISR
   jmp PORTF_PORT_ISR
   jmp NVMCTRL_EE_ISR
   jmp USART2_RXC_ISR
   jmp USART2_DRE_ISR
   jmp USART2_TXC_ISR
   jmp PORTB_PORT_ISR
   jmp PORTE_PORT_ISR

   {$i start.inc}

   .weak CRCSCAN_NMI_ISR
   .weak BOD_VLM_ISR
   .weak RTC_CNT_ISR
   .weak RTC_PIT_ISR
   .weak CCL_CCL_ISR
   .weak PORTA_PORT_ISR
   .weak TCA0_LUNF_ISR

   .weak TCA0_HUNF_ISR
   .weak TCA0_LCMP0_ISR

   .weak TCA0_CMP1_ISR

   .weak TCA0_CMP2_ISR

   .weak TCB0_INT_ISR
   .weak TCB1_INT_ISR
   .weak TWI0_TWIS_ISR
   .weak TWI0_TWIM_ISR
   .weak SPI0_INT_ISR
   .weak USART0_RXC_ISR
   .weak USART0_DRE_ISR
   .weak USART0_TXC_ISR
   .weak PORTD_PORT_ISR
   .weak AC0_AC_ISR
   .weak ADC0_RESRDY_ISR
   .weak ADC0_WCOMP_ISR
   .weak PORTC_PORT_ISR
   .weak TCB2_INT_ISR
   .weak USART1_RXC_ISR
   .weak USART1_DRE_ISR
   .weak USART1_TXC_ISR
   .weak PORTF_PORT_ISR
   .weak NVMCTRL_EE_ISR
   .weak USART2_RXC_ISR
   .weak USART2_DRE_ISR
   .weak USART2_TXC_ISR
   .weak PORTB_PORT_ISR
   .weak PORTE_PORT_ISR

   .set CRCSCAN_NMI_ISR, Default_IRQ_handler
   .set BOD_VLM_ISR, Default_IRQ_handler
   .set RTC_CNT_ISR, Default_IRQ_handler
   .set RTC_PIT_ISR, Default_IRQ_handler
   .set CCL_CCL_ISR, Default_IRQ_handler
   .set PORTA_PORT_ISR, Default_IRQ_handler
   .set TCA0_LUNF_ISR, Default_IRQ_handler

   .set TCA0_HUNF_ISR, Default_IRQ_handler
   .set TCA0_LCMP0_ISR, Default_IRQ_handler

   .set TCA0_CMP1_ISR, Default_IRQ_handler

   .set TCA0_CMP2_ISR, Default_IRQ_handler

   .set TCB0_INT_ISR, Default_IRQ_handler
   .set TCB1_INT_ISR, Default_IRQ_handler
   .set TWI0_TWIS_ISR, Default_IRQ_handler
   .set TWI0_TWIM_ISR, Default_IRQ_handler
   .set SPI0_INT_ISR, Default_IRQ_handler
   .set USART0_RXC_ISR, Default_IRQ_handler
   .set USART0_DRE_ISR, Default_IRQ_handler
   .set USART0_TXC_ISR, Default_IRQ_handler
   .set PORTD_PORT_ISR, Default_IRQ_handler
   .set AC0_AC_ISR, Default_IRQ_handler
   .set ADC0_RESRDY_ISR, Default_IRQ_handler
   .set ADC0_WCOMP_ISR, Default_IRQ_handler
   .set PORTC_PORT_ISR, Default_IRQ_handler
   .set TCB2_INT_ISR, Default_IRQ_handler
   .set USART1_RXC_ISR, Default_IRQ_handler
   .set USART1_DRE_ISR, Default_IRQ_handler
   .set USART1_TXC_ISR, Default_IRQ_handler
   .set PORTF_PORT_ISR, Default_IRQ_handler
   .set NVMCTRL_EE_ISR, Default_IRQ_handler
   .set USART2_RXC_ISR, Default_IRQ_handler
   .set USART2_DRE_ISR, Default_IRQ_handler
   .set USART2_TXC_ISR, Default_IRQ_handler
   .set PORTB_PORT_ISR, Default_IRQ_handler
   .set PORTE_PORT_ISR, Default_IRQ_handler
end;

end.
