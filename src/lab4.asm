;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 2.9.0 #5416 (Mar 22 2009) (MINGW32)
; This file was generated Wed Dec 13 17:04:57 2017
;--------------------------------------------------------
	.module lab4
	.optsdcc -mmcs51 --model-small
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _EOL
	.globl _main
	.globl _print_result
	.globl _print_num
	.globl _print_error
	.globl _init_kb_timer
	.globl _timer_kb
	.globl _scan_row
	.globl _invalid_input
	.globl _get_input
	.globl _capture_input
	.globl _is_queue_empty
	.globl _leds
	.globl _led
	.globl _readdip
	.globl _read_max
	.globl _write_max
	.globl _SetVector
	.globl _SPR0
	.globl _SPR1
	.globl _CPHA
	.globl _CPOL
	.globl _SPIM
	.globl _SPE
	.globl _WCOL
	.globl _ISPI
	.globl _I2CI
	.globl _I2CTX
	.globl _I2CRS
	.globl _I2CM
	.globl _MDI
	.globl _MCO
	.globl _MDE
	.globl _MDO
	.globl _CS0
	.globl _CS1
	.globl _CS2
	.globl _CS3
	.globl _SCONV
	.globl _CCONV
	.globl _DMA
	.globl _ADCI
	.globl _P
	.globl _F1
	.globl _OV
	.globl _RS0
	.globl _RS1
	.globl _F0
	.globl _AC
	.globl _CY
	.globl _CAP2
	.globl _CNT2
	.globl _TR2
	.globl _XEN
	.globl _TCLK
	.globl _RCLK
	.globl _EXF2
	.globl _TF2
	.globl _WDE
	.globl _WDS
	.globl _WDR2
	.globl _WDR1
	.globl _PRE0
	.globl _PRE1
	.globl _PRE2
	.globl _PX0
	.globl _PT0
	.globl _PX1
	.globl _PT1
	.globl _PS
	.globl _PT2
	.globl _PADC
	.globl _PSI
	.globl _RXD
	.globl _TXD
	.globl _INT0
	.globl _INT1
	.globl _T0
	.globl _T1
	.globl _WR
	.globl _RD
	.globl _EX0
	.globl _ET0
	.globl _EX1
	.globl _ET1
	.globl _ES
	.globl _ET2
	.globl _EADC
	.globl _EA
	.globl _RI
	.globl _TI
	.globl _RB8
	.globl _TB8
	.globl _REN
	.globl _SM2
	.globl _SM1
	.globl _SM0
	.globl _T2
	.globl _T2EX
	.globl _IT0
	.globl _IE0
	.globl _IT1
	.globl _IE1
	.globl _TR0
	.globl _TF0
	.globl _TR1
	.globl _TF1
	.globl _DACCON
	.globl _DAC1H
	.globl _DAC1L
	.globl _DAC0H
	.globl _DAC0L
	.globl _SPICON
	.globl _SPIDAT
	.globl _ADCCON3
	.globl _ADCGAINH
	.globl _ADCGAINL
	.globl _ADCOFSH
	.globl _ADCOFSL
	.globl _B
	.globl _ADCCON1
	.globl _I2CCON
	.globl _ACC
	.globl _PSMCON
	.globl _ADCDATAH
	.globl _ADCDATAL
	.globl _ADCCON2
	.globl _DMAP
	.globl _DMAH
	.globl _DMAL
	.globl _PSW
	.globl _TH2
	.globl _TL2
	.globl _RCAP2H
	.globl _RCAP2L
	.globl _T2CON
	.globl _EADRL
	.globl _WDCON
	.globl _EDATA4
	.globl _EDATA3
	.globl _EDATA2
	.globl _EDATA1
	.globl _ETIM3
	.globl _ETIM2
	.globl _ETIM1
	.globl _ECON
	.globl _IP
	.globl _P3
	.globl _IE2
	.globl _IE
	.globl _P2
	.globl _I2CADD
	.globl _I2CDAT
	.globl _SBUF
	.globl _SCON
	.globl _P1
	.globl _TH1
	.globl _TH0
	.globl _TL1
	.globl _TL0
	.globl _TMOD
	.globl _TCON
	.globl _PCON
	.globl _DPP
	.globl _DPH
	.globl _DPL
	.globl _SP
	.globl _P0
	.globl _state
	.globl _second_num
	.globl _first_num
	.globl _second_size
	.globl _first_size
	.globl _ir
	.globl _READ_FIFO
	.globl _keyboard
	.globl _end_queue
	.globl _start_queue
	.globl _queue
	.globl _time
	.globl _bounce
	.globl _column
	.globl _uart_s_init
	.globl _uart_s_read_ready
	.globl _uart_s_write
	.globl _uart_s_read
	.globl _type
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
	.area RSEG    (DATA)
_P0	=	0x0080
_SP	=	0x0081
_DPL	=	0x0082
_DPH	=	0x0083
_DPP	=	0x0084
_PCON	=	0x0087
_TCON	=	0x0088
_TMOD	=	0x0089
_TL0	=	0x008a
_TL1	=	0x008b
_TH0	=	0x008c
_TH1	=	0x008d
_P1	=	0x0090
_SCON	=	0x0098
_SBUF	=	0x0099
_I2CDAT	=	0x009a
_I2CADD	=	0x009b
_P2	=	0x00a0
_IE	=	0x00a8
_IE2	=	0x00a9
_P3	=	0x00b0
_IP	=	0x00b8
_ECON	=	0x00b9
_ETIM1	=	0x00ba
_ETIM2	=	0x00bb
_ETIM3	=	0x00c4
_EDATA1	=	0x00bc
_EDATA2	=	0x00bd
_EDATA3	=	0x00be
_EDATA4	=	0x00bf
_WDCON	=	0x00c0
_EADRL	=	0x00c6
_T2CON	=	0x00c8
_RCAP2L	=	0x00ca
_RCAP2H	=	0x00cb
_TL2	=	0x00cc
_TH2	=	0x00cd
_PSW	=	0x00d0
_DMAL	=	0x00d2
_DMAH	=	0x00d3
_DMAP	=	0x00d4
_ADCCON2	=	0x00d8
_ADCDATAL	=	0x00d9
_ADCDATAH	=	0x00da
_PSMCON	=	0x00df
_ACC	=	0x00e0
_I2CCON	=	0x00e8
_ADCCON1	=	0x00ef
_B	=	0x00f0
_ADCOFSL	=	0x00f1
_ADCOFSH	=	0x00f2
_ADCGAINL	=	0x00f3
_ADCGAINH	=	0x00f4
_ADCCON3	=	0x00f5
_SPIDAT	=	0x00f7
_SPICON	=	0x00f8
_DAC0L	=	0x00f9
_DAC0H	=	0x00fa
_DAC1L	=	0x00fb
_DAC1H	=	0x00fc
_DACCON	=	0x00fd
;--------------------------------------------------------
; special function bits
;--------------------------------------------------------
	.area RSEG    (DATA)
_TF1	=	0x008f
_TR1	=	0x008e
_TF0	=	0x008d
_TR0	=	0x008c
_IE1	=	0x008b
_IT1	=	0x008a
_IE0	=	0x0089
_IT0	=	0x0088
_T2EX	=	0x0091
_T2	=	0x0090
_SM0	=	0x009f
_SM1	=	0x009e
_SM2	=	0x009d
_REN	=	0x009c
_TB8	=	0x009b
_RB8	=	0x009a
_TI	=	0x0099
_RI	=	0x0098
_EA	=	0x00af
_EADC	=	0x00ae
_ET2	=	0x00ad
_ES	=	0x00ac
_ET1	=	0x00ab
_EX1	=	0x00aa
_ET0	=	0x00a9
_EX0	=	0x00a8
_RD	=	0x00b7
_WR	=	0x00b6
_T1	=	0x00b5
_T0	=	0x00b4
_INT1	=	0x00b3
_INT0	=	0x00b2
_TXD	=	0x00b1
_RXD	=	0x00b0
_PSI	=	0x00bf
_PADC	=	0x00be
_PT2	=	0x00bd
_PS	=	0x00bc
_PT1	=	0x00bb
_PX1	=	0x00ba
_PT0	=	0x00b9
_PX0	=	0x00b8
_PRE2	=	0x00c7
_PRE1	=	0x00c6
_PRE0	=	0x00c5
_WDR1	=	0x00c3
_WDR2	=	0x00c2
_WDS	=	0x00c1
_WDE	=	0x00c0
_TF2	=	0x00cf
_EXF2	=	0x00ce
_RCLK	=	0x00cd
_TCLK	=	0x00cc
_XEN	=	0x00cb
_TR2	=	0x00ca
_CNT2	=	0x00c9
_CAP2	=	0x00c8
_CY	=	0x00d7
_AC	=	0x00d6
_F0	=	0x00d5
_RS1	=	0x00d4
_RS0	=	0x00d3
_OV	=	0x00d2
_F1	=	0x00d1
_P	=	0x00d0
_ADCI	=	0x00df
_DMA	=	0x00de
_CCONV	=	0x00dd
_SCONV	=	0x00dc
_CS3	=	0x00db
_CS2	=	0x00da
_CS1	=	0x00d9
_CS0	=	0x00d8
_MDO	=	0x00ef
_MDE	=	0x00ee
_MCO	=	0x00ed
_MDI	=	0x00ec
_I2CM	=	0x00eb
_I2CRS	=	0x00ea
_I2CTX	=	0x00e9
_I2CI	=	0x00e8
_ISPI	=	0x00ff
_WCOL	=	0x00fe
_SPE	=	0x00fd
_SPIM	=	0x00fc
_CPOL	=	0x00fb
_CPHA	=	0x00fa
_SPR1	=	0x00f9
_SPR0	=	0x00f8
;--------------------------------------------------------
; overlayable register banks
;--------------------------------------------------------
	.area REG_BANK_0	(REL,OVR,DATA)
	.ds 8
;--------------------------------------------------------
; overlayable bit register bank
;--------------------------------------------------------
	.area BIT_BANK	(REL,OVR,DATA)
bits:
	.ds 1
	b0 = bits[0]
	b1 = bits[1]
	b2 = bits[2]
	b3 = bits[3]
	b4 = bits[4]
	b5 = bits[5]
	b6 = bits[6]
	b7 = bits[7]
;--------------------------------------------------------
; internal ram data
;--------------------------------------------------------
	.area DSEG    (DATA)
_old_led:
	.ds 1
_column::
	.ds 1
_bounce::
	.ds 16
_time::
	.ds 32
_queue::
	.ds 10
_start_queue::
	.ds 1
_end_queue::
	.ds 1
_keyboard::
	.ds 16
_READ_FIFO::
	.ds 8
_ir::
	.ds 1
_first_size::
	.ds 1
_second_size::
	.ds 1
_first_num::
	.ds 1
_second_num::
	.ds 1
_state::
	.ds 1
;--------------------------------------------------------
; overlayable items in internal ram 
;--------------------------------------------------------
	.area OSEG    (OVR,DATA)
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG	(DATA)
__start__stack:
	.ds	1

;--------------------------------------------------------
; indirectly addressable internal ram data
;--------------------------------------------------------
	.area ISEG    (DATA)
;--------------------------------------------------------
; absolute internal ram data
;--------------------------------------------------------
	.area IABS    (ABS,DATA)
	.area IABS    (ABS,DATA)
;--------------------------------------------------------
; bit data
;--------------------------------------------------------
	.area BSEG    (BIT)
;--------------------------------------------------------
; paged external ram data
;--------------------------------------------------------
	.area PSEG    (PAG,XDATA)
;--------------------------------------------------------
; external ram data
;--------------------------------------------------------
	.area XSEG    (XDATA)
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area XABS    (ABS,XDATA)
;--------------------------------------------------------
; external initialized ram data
;--------------------------------------------------------
	.area XISEG   (XDATA)
	.area HOME    (CODE)
	.area GSINIT0 (CODE)
	.area GSINIT1 (CODE)
	.area GSINIT2 (CODE)
	.area GSINIT3 (CODE)
	.area GSINIT4 (CODE)
	.area GSINIT5 (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area CSEG    (CODE)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME    (CODE)
__interrupt_vect:
	ljmp	__sdcc_gsinit_startup
	reti
	.ds	7
	ljmp	_timer_kb
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME    (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area GSINIT  (CODE)
	.globl __sdcc_gsinit_startup
	.globl __sdcc_program_startup
	.globl __start__stack
	.globl __mcs51_genXINIT
	.globl __mcs51_genXRAMCLEAR
	.globl __mcs51_genRAMCLEAR
;	./include/led.h:6: static unsigned char old_led = 0;   // "Видеопамять" линейки светодиодов
	mov	_old_led,#0x00
;	./include/kb.h:11: unsigned char column = 3;
	mov	_column,#0x03
;	./include/kb.h:12: unsigned char bounce[4][4] = {{0}};
	mov	_bounce,#0x00
;	./include/kb.h:13: unsigned short time[4][4] = {{0}};
	mov	_time,#0x00
	mov	(_time + 1),#0x00
;	./include/kb.h:14: unsigned char queue[QUEUE_LENGTH] = {0};
	mov	_queue,#0x00
;	./include/kb.h:15: char start_queue = 0, end_queue = 0;
	mov	_start_queue,#0x00
;	./include/kb.h:15: unsigned char keyboard[4][4] = {
	mov	_end_queue,#0x00
;	./include/kb.h:16: {'1', '2', '3', 'A'},
	mov	_keyboard,#0x31
	mov	(_keyboard + 0x0001),#0x32
	mov	(_keyboard + 0x0002),#0x33
	mov	(_keyboard + 0x0003),#0x41
	mov	(_keyboard + 0x0004),#0x34
	mov	(_keyboard + 0x0005),#0x35
	mov	(_keyboard + 0x0006),#0x36
	mov	(_keyboard + 0x0007),#0x42
	mov	(_keyboard + 0x0008),#0x37
	mov	(_keyboard + 0x0009),#0x38
	mov	(_keyboard + 0x000a),#0x39
	mov	(_keyboard + 0x000b),#0x43
	mov	(_keyboard + 0x000c),#0x2A
	mov	(_keyboard + 0x000d),#0x30
	mov	(_keyboard + 0x000e),#0x23
	mov	(_keyboard + 0x000f),#0x44
;	src/lab4.c:19: unsigned char READ_FIFO[BUFFSZ] = {0};
	mov	_READ_FIFO,#0x00
;	src/lab4.c:20: unsigned char ir = 0;
	mov	_ir,#0x00
;	src/lab4.c:22: unsigned char first_size = 0;
	mov	_first_size,#0x00
;	src/lab4.c:23: unsigned char second_size = 0;
	mov	_second_size,#0x00
;	src/lab4.c:24: char first_num = -1;
	mov	_first_num,#0xFF
;	src/lab4.c:25: char second_num = -1;
	mov	_second_num,#0xFF
;	src/lab4.c:27: unsigned char state = 0;
	mov	_state,#0x00
	.area GSFINAL (CODE)
	ljmp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME    (CODE)
	.area HOME    (CODE)
__sdcc_program_startup:
	lcall	_main
;	return from main will lock up
	sjmp .
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CSEG    (CODE)
;------------------------------------------------------------
;Allocation info for local variables in function 'SetVector'
;------------------------------------------------------------
;Vector                    Allocated to stack - offset -5
;Address                   Allocated to registers r2 r3 
;TmpVector                 Allocated to registers r2 r3 
;------------------------------------------------------------
;	./include/interrupt.h:13: void SetVector(unsigned char xdata * Address, void * Vector) {
;	-----------------------------------------
;	 function SetVector
;	-----------------------------------------
_SetVector:
	ar2 = 0x02
	ar3 = 0x03
	ar4 = 0x04
	ar5 = 0x05
	ar6 = 0x06
	ar7 = 0x07
	ar0 = 0x00
	ar1 = 0x01
	push	_bp
	mov	_bp,sp
;	./include/interrupt.h:16: *Address = 0x02;
	mov	r2,dpl
	mov  r3,dph
	mov	a,#0x02
	movx	@dptr,a
;	./include/interrupt.h:18: TmpVector = (unsigned char xdata *) (Address + 1);
	inc	r2
	cjne	r2,#0x00,00103$
	inc	r3
00103$:
;	./include/interrupt.h:19: *TmpVector = (unsigned char) ((unsigned short)Vector >> 8);
	mov	a,_bp
	add	a,#0xfb
	mov	r0,a
	mov	ar4,@r0
	inc	r0
	mov	ar5,@r0
	mov	ar4,r5
	mov	dpl,r2
	mov	dph,r3
	mov	a,r4
	movx	@dptr,a
	inc	dptr
	mov	r2,dpl
	mov	r3,dph
;	./include/interrupt.h:20: ++TmpVector;
;	./include/interrupt.h:21: *TmpVector = (unsigned char) Vector;
	mov	a,_bp
	add	a,#0xfb
	mov	r0,a
	mov	ar4,@r0
	mov	dpl,r2
	mov	dph,r3
	mov	a,r4
	movx	@dptr,a
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'write_max'
;------------------------------------------------------------
;val                       Allocated to stack - offset -3
;regnum                    Allocated to registers r2 r3 
;oldDPP                    Allocated to registers r4 
;------------------------------------------------------------
;	./include/max.h:20: void write_max( unsigned char xdata *regnum, unsigned char val ) {
;	-----------------------------------------
;	 function write_max
;	-----------------------------------------
_write_max:
	push	_bp
	mov	_bp,sp
;	./include/max.h:21: unsigned char oldDPP = DPP;
;	./include/max.h:22: DPP     = MAXBASE;
;	./include/max.h:23: *regnum = val;
	mov	r4,_DPP
	mov	_DPP,#0x08
	mov	r0,_bp
	dec	r0
	dec	r0
	dec	r0
	mov	a,@r0
	movx	@dptr,a
;	./include/max.h:24: DPP     = oldDPP;
	mov	_DPP,r4
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'read_max'
;------------------------------------------------------------
;regnum                    Allocated to registers r2 r3 
;oldDPP                    Allocated to registers r4 
;val                       Allocated to registers r2 
;------------------------------------------------------------
;	./include/max.h:27: unsigned char read_max( unsigned char xdata *regnum ) {
;	-----------------------------------------
;	 function read_max
;	-----------------------------------------
_read_max:
;	./include/max.h:28: unsigned char oldDPP=DPP;
;	./include/max.h:31: DPP = MAXBASE;
;	./include/max.h:32: val = *regnum;
	mov	r4,_DPP
	mov	_DPP,#0x08
	movx	a,@dptr
	mov	r2,a
;	./include/max.h:33: DPP = oldDPP;
	mov	_DPP,r4
;	./include/max.h:35: return val;
	mov	dpl,r2
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'readdip'
;------------------------------------------------------------
;------------------------------------------------------------
;	./include/max.h:38: unsigned char readdip() {
;	-----------------------------------------
;	 function readdip
;	-----------------------------------------
_readdip:
;	./include/max.h:39: write_max(ENA, 0x00);
	clr	a
	push	acc
	mov	dptr,#0x0004
	lcall	_write_max
	dec	sp
;	./include/max.h:40: return read_max(EXT_LO);
	mov	dptr,#0x0002
	ljmp	_read_max
;------------------------------------------------------------
;Allocation info for local variables in function 'led'
;------------------------------------------------------------
;on                        Allocated to stack - offset -3
;n                         Allocated to registers r2 
;c                         Allocated to registers r3 
;mask                      Allocated to registers r2 
;------------------------------------------------------------
;	./include/led.h:9: void led( unsigned char n, unsigned char on )
;	-----------------------------------------
;	 function led
;	-----------------------------------------
_led:
	push	_bp
	mov	_bp,sp
;	./include/led.h:14: if( n > 7 ) return;
	mov	a,dpl
	mov	r2,a
	add	a,#0xff - 0x07
	jnc	00102$
	sjmp	00106$
00102$:
;	./include/led.h:16: c = old_led;
	mov	r3,_old_led
;	./include/led.h:18: mask <<= n;
	mov	b,r2
	inc	b
	mov	a,#0x01
	sjmp	00113$
00111$:
	add	a,acc
00113$:
	djnz	b,00111$
	mov	r2,a
;	./include/led.h:20: if( on )
	mov	r0,_bp
	dec	r0
	dec	r0
	dec	r0
	mov	a,@r0
	jz	00104$
;	./include/led.h:21: c |= mask;
	mov	a,r2
	orl	ar3,a
	sjmp	00105$
00104$:
;	./include/led.h:23: c &= ~mask;         
	mov	a,r2
	cpl	a
	mov	r2,a
	anl	ar3,a
00105$:
;	./include/led.h:25: write_max( SV, c );     
	push	ar3
	push	ar3
	mov	dptr,#0x0007
	lcall	_write_max
	dec	sp
	pop	ar3
;	./include/led.h:27: old_led = c;
	mov	_old_led,r3
00106$:
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'leds'
;------------------------------------------------------------
;on                        Allocated to registers r2 
;------------------------------------------------------------
;	./include/led.h:30: void leds( unsigned char on ) {
;	-----------------------------------------
;	 function leds
;	-----------------------------------------
_leds:
	mov	r2,dpl
;	./include/led.h:31: write_max( SV, on );     
	push	ar2
	push	ar2
	mov	dptr,#0x0007
	lcall	_write_max
	dec	sp
	pop	ar2
;	./include/led.h:32: old_led = on;
	mov	_old_led,r2
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'uart_s_init'
;------------------------------------------------------------
;speed                     Allocated to registers r2 r3 
;------------------------------------------------------------
;	./include/serial.h:21: void uart_s_init(int speed) {
;	-----------------------------------------
;	 function uart_s_init
;	-----------------------------------------
_uart_s_init:
	mov	r2,dpl
;	./include/serial.h:22: TH1 = speed;   
	mov	_TH1,r2
;	./include/serial.h:23: TMOD |= 0x20; // Таймер 1 будет работать в режиме autoreload
	orl	_TMOD,#0x20
;	./include/serial.h:24: TR1 = 1;      // start T1
	setb	_TR1
;	./include/serial.h:25: SCON = 0x50;  // REN = 1, UART mode 1
	mov	_SCON,#0x50
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'uart_s_read_ready'
;------------------------------------------------------------
;------------------------------------------------------------
;	./include/serial.h:28: unsigned char uart_s_read_ready() {
;	-----------------------------------------
;	 function uart_s_read_ready
;	-----------------------------------------
_uart_s_read_ready:
;	./include/serial.h:29: return RI;
	mov	c,_RI
	clr	a
	rlc	a
	mov	dpl,a
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'uart_s_write'
;------------------------------------------------------------
;c                         Allocated to registers 
;------------------------------------------------------------
;	./include/serial.h:32: void uart_s_write(unsigned char c) {
;	-----------------------------------------
;	 function uart_s_write
;	-----------------------------------------
_uart_s_write:
	mov	_SBUF,dpl
;	./include/serial.h:34: TI = 0;
	clr	_TI
;	./include/serial.h:35: while(!TI);
00101$:
	jnb	_TI,00101$
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'uart_s_read'
;------------------------------------------------------------
;------------------------------------------------------------
;	./include/serial.h:38: unsigned char uart_s_read() {
;	-----------------------------------------
;	 function uart_s_read
;	-----------------------------------------
_uart_s_read:
;	./include/serial.h:39: while(!RI);
00101$:
;	./include/serial.h:40: RI = 0;
	jbc	_RI,00108$
	sjmp	00101$
00108$:
;	./include/serial.h:41: return SBUF;
	mov	dpl,_SBUF
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'type'
;------------------------------------------------------------
;str                       Allocated to registers r2 r3 r4 
;------------------------------------------------------------
;	./include/serial.h:44: void type(char* str) {
;	-----------------------------------------
;	 function type
;	-----------------------------------------
_type:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	./include/serial.h:45: while(*str)
00101$:
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r5,a
	jz	00104$
;	./include/serial.h:46: uart_s_write(*str++);
	inc	r2
	cjne	r2,#0x00,00110$
	inc	r3
00110$:
	mov	dpl,r5
	push	ar2
	push	ar3
	push	ar4
	lcall	_uart_s_write
	pop	ar4
	pop	ar3
	pop	ar2
	sjmp	00101$
00104$:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'is_queue_empty'
;------------------------------------------------------------
;------------------------------------------------------------
;	./include/kb.h:23: unsigned char is_queue_empty(){
;	-----------------------------------------
;	 function is_queue_empty
;	-----------------------------------------
_is_queue_empty:
;	./include/kb.h:24: return start_queue == end_queue;
	mov	a,_end_queue
	cjne	a,_start_queue,00103$
	mov	a,#0x01
	sjmp	00104$
00103$:
	clr	a
00104$:
	mov	dpl,a
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'capture_input'
;------------------------------------------------------------
;c                         Allocated to registers r2 
;------------------------------------------------------------
;	./include/kb.h:27: void capture_input(unsigned char c) {
;	-----------------------------------------
;	 function capture_input
;	-----------------------------------------
_capture_input:
	mov	r2,dpl
;	./include/kb.h:28: if (start_queue == QUEUE_LENGTH)
	mov	a,#0x0A
	cjne	a,_start_queue,00102$
;	./include/kb.h:29: start_queue = 0;
	mov	_start_queue,#0x00
00102$:
;	./include/kb.h:30: queue[start_queue++] = c;
	mov	r3,_start_queue
	inc	_start_queue
	mov	a,r3
	add	a,#_queue
	mov	r0,a
	mov	@r0,ar2
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'get_input'
;------------------------------------------------------------
;------------------------------------------------------------
;	./include/kb.h:33: unsigned char get_input() {
;	-----------------------------------------
;	 function get_input
;	-----------------------------------------
_get_input:
;	./include/kb.h:34: if (end_queue == QUEUE_LENGTH)
	mov	a,#0x0A
	cjne	a,_end_queue,00102$
;	./include/kb.h:35: end_queue = 0;
	mov	_end_queue,#0x00
00102$:
;	./include/kb.h:36: return queue[end_queue++];
	mov	r2,_end_queue
	inc	_end_queue
	mov	a,r2
	add	a,#_queue
	mov	r0,a
	mov	dpl,@r0
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'invalid_input'
;------------------------------------------------------------
;i                         Allocated to registers r2 
;j                         Allocated to registers r5 
;------------------------------------------------------------
;	./include/kb.h:39: void invalid_input(){
;	-----------------------------------------
;	 function invalid_input
;	-----------------------------------------
_invalid_input:
;	./include/kb.h:41: EA = 0;
	clr	_EA
;	./include/kb.h:42: type("too many buttons\r\n");
	mov	dptr,#__str_0
	mov	b,#0x80
	lcall	_type
;	./include/kb.h:43: for(i = 0; i < 4; i++) {
	mov	r2,#0x00
00105$:
	cjne	r2,#0x04,00117$
00117$:
	jnc	00108$
;	./include/kb.h:44: for(j = 0; j < 4; j++){
	mov	a,r2
	add	a,r2
	add	a,acc
	add	a,#_bounce
	mov	r3,a
	mov	a,r2
	swap	a
	rr	a
	anl	a,#0xf8
	add	a,#_time
	mov	r4,a
	mov	r5,#0x00
00101$:
	cjne	r5,#0x04,00119$
00119$:
	jnc	00107$
;	./include/kb.h:45: bounce[i][j] = 0;
	mov	a,r5
	add	a,r3
	mov	r0,a
	mov	@r0,#0x00
;	./include/kb.h:46: time[i][j] = 0;
	mov	a,r5
	add	a,r5
	mov	r6,a
	add	a,r4
	mov	r0,a
	mov	@r0,#0x00
	inc	r0
	mov	@r0,#0x00
;	./include/kb.h:44: for(j = 0; j < 4; j++){
	inc	r5
	sjmp	00101$
00107$:
;	./include/kb.h:43: for(i = 0; i < 4; i++) {
	inc	r2
	sjmp	00105$
00108$:
;	./include/kb.h:49: EA = 1;
	setb	_EA
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'scan_row'
;------------------------------------------------------------
;col                       Allocated to registers r2 
;row                       Allocated to registers r2 
;------------------------------------------------------------
;	./include/kb.h:52: unsigned char scan_row() {
;	-----------------------------------------
;	 function scan_row
;	-----------------------------------------
_scan_row:
;	./include/kb.h:55: if (column == 3)
	mov	a,#0x03
	cjne	a,_column,00102$
;	./include/kb.h:56: column = 0;
	mov	_column,#0x00
	sjmp	00103$
00102$:
;	./include/kb.h:57: else column++;
	inc	_column
00103$:
;	./include/kb.h:59: col = 0x1 << column; //0001,0010,0100,1000,0001,...
	mov	b,_column
	inc	b
	mov	a,#0x01
	sjmp	00111$
00109$:
	add	a,acc
00111$:
	djnz	b,00109$
;	./include/kb.h:60: write_max(KB, ~col); //11111110,11111101,11111011,11110111,11111110,...
	cpl	a
	mov	r2,a
	push	ar2
	mov	dptr,#0x0000
	lcall	_write_max
	dec	sp
;	./include/kb.h:62: row = read_max(KB) & (0xF0);
	mov	dptr,#0x0000
	lcall	_read_max
;	./include/kb.h:63: row = (~(row >> 4)) & 0x0F;
	mov	a,dpl
	anl	a,#0xF0
	swap	a
	anl	a,#0x0f
	cpl	a
	mov	r3,a
	mov	a,#0x0F
	anl	a,r3
;	./include/kb.h:64: return row;
	mov	dpl,a
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'timer_kb'
;------------------------------------------------------------
;row                       Allocated to registers r4 
;scanned_row               Allocated to stack - offset 1
;key_pressed               Allocated to registers r3 
;i                         Allocated to registers r2 
;j                         Allocated to registers r7 
;------------------------------------------------------------
;	./include/kb.h:67: void timer_kb(void) __interrupt( 1 ) {
;	-----------------------------------------
;	 function timer_kb
;	-----------------------------------------
_timer_kb:
	push	bits
	push	acc
	push	b
	push	dpl
	push	dph
	push	(0+2)
	push	(0+3)
	push	(0+4)
	push	(0+5)
	push	(0+6)
	push	(0+7)
	push	(0+0)
	push	(0+1)
	push	psw
	mov	psw,#0x00
	push	_bp
	mov	_bp,sp
	inc	sp
;	./include/kb.h:71: scanned_row = scan_row();
	lcall	_scan_row
	mov	r2,dpl
	mov	r0,_bp
	inc	r0
	mov	@r0,ar2
;	./include/kb.h:72: key_pressed = 0;
	mov	r3,#0x00
;	./include/kb.h:73: for (row = 0; row < 4; row++) {
	mov	r4,#0x00
00131$:
	cjne	r4,#0x04,00155$
00155$:
	jc	00156$
	ljmp	00134$
00156$:
;	./include/kb.h:74: if (scanned_row & (0x01 << row)) {
	push	ar3
	mov	b,r4
	inc	b
	mov	r5,#0x01
	mov	r6,#0x00
	sjmp	00158$
00157$:
	mov	a,r5
	add	a,r5
	mov	r5,a
	mov	a,r6
	rlc	a
	mov	r6,a
00158$:
	djnz	b,00157$
	mov	r0,_bp
	inc	r0
	mov	ar7,@r0
	mov	r3,#0x00
	mov	a,r7
	anl	ar5,a
	mov	a,r3
	anl	ar6,a
	pop	ar3
	mov	a,r5
	orl	a,r6
	jz	00110$
;	./include/kb.h:75: if (bounce[row][column] < 3)
	mov	a,r4
	add	a,r4
	add	a,acc
	mov	r5,a
	add	a,#_bounce
	mov	r6,a
	mov	a,_column
	add	a,r6
	mov	r0,a
	mov	ar6,@r0
	cjne	r6,#0x03,00160$
00160$:
	jnc	00102$
;	./include/kb.h:76: bounce[row][column]++;
	mov	a,r5
	add	a,#_bounce
	mov	r5,a
	mov	a,_column
	add	a,r5
	mov	r0,a
	mov	a,@r0
	mov	r5,a
	inc	a
	mov	@r0,a
	sjmp	00146$
00102$:
;	./include/kb.h:78: time[row][column]++;
	mov	a,r4
	swap	a
	rr	a
	anl	a,#0xf8
	add	a,#_time
	mov	r5,a
	mov	a,_column
	add	a,_column
	add	a,r5
	mov	r0,a
	mov	ar5,@r0
	inc	r0
	mov	ar6,@r0
	dec	r0
	inc	r5
	cjne	r5,#0x00,00162$
	inc	r6
00162$:
	mov	@r0,ar5
	inc	r0
	mov	@r0,ar6
	dec	r0
	sjmp	00146$
00110$:
;	./include/kb.h:81: if (bounce[row][column] > 0)
	mov	a,r4
	add	a,r4
	add	a,acc
	mov	r5,a
	add	a,#_bounce
	mov	r6,a
	mov	a,_column
	add	a,r6
	mov	r0,a
	mov	a,@r0
	jz	00107$
;	./include/kb.h:82: bounce[row][column] = 0;
	mov	a,r5
	add	a,#_bounce
	mov	r5,a
	mov	a,_column
	add	a,r5
	mov	r0,a
	mov	@r0,#0x00
	sjmp	00146$
00107$:
;	./include/kb.h:83: else if (time[row][column] > 0)
	mov	a,r4
	swap	a
	rr	a
	anl	a,#0xf8
	mov	r5,a
	add	a,#_time
	mov	r6,a
	mov	a,_column
	add	a,_column
	mov	r7,a
	add	a,r6
	mov	r0,a
	mov	ar6,@r0
	inc	r0
	mov	ar2,@r0
	dec	r0
	mov	a,r6
	orl	a,r2
	jz	00146$
;	./include/kb.h:84: time[row][column] = 0;
	mov	a,r5
	add	a,#_time
	mov	r5,a
	mov	a,r7
	add	a,r5
	mov	r0,a
	mov	@r0,#0x00
	inc	r0
	mov	@r0,#0x00
;	./include/kb.h:87: for(i = 0; i < 4; i++) {
00146$:
	mov	r2,#0x00
00127$:
	cjne	r2,#0x04,00165$
00165$:
	jnc	00130$
;	./include/kb.h:88: for(j = 0; j < 4; j++) {
	mov	a,r2
	add	a,r2
	add	a,acc
	add	a,#_bounce
	mov	r5,a
	mov	ar6,r3
	mov	r7,#0x00
00123$:
	cjne	r7,#0x04,00167$
00167$:
	jnc	00152$
;	./include/kb.h:89: if(bounce[i][j] == 3)
	push	ar2
	mov	a,r7
	add	a,r5
	mov	r0,a
	mov	ar2,@r0
	cjne	r2,#0x03,00169$
	sjmp	00170$
00169$:
	pop	ar2
	sjmp	00125$
00170$:
	pop	ar2
;	./include/kb.h:90: key_pressed++;
	inc	r6
	mov	ar3,r6
00125$:
;	./include/kb.h:88: for(j = 0; j < 4; j++) {
	inc	r7
	sjmp	00123$
00152$:
	mov	ar3,r6
;	./include/kb.h:87: for(i = 0; i < 4; i++) {
	inc	r2
	sjmp	00127$
00130$:
;	./include/kb.h:94: leds(key_pressed);
	mov	dpl,r3
	push	ar3
	push	ar4
	lcall	_leds
	pop	ar4
	pop	ar3
;	./include/kb.h:95: if (key_pressed > MAX_KEYS_PRESSED)
	mov	a,r3
	add	a,#0xff - 0x02
	jnc	00121$
;	./include/kb.h:96: invalid_input();
	push	ar3
	push	ar4
	lcall	_invalid_input
	pop	ar4
	pop	ar3
	ljmp	00133$
00121$:
;	./include/kb.h:98: key_pressed = 0;
	mov	r3,#0x00
;	./include/kb.h:99: if (bounce[row][column] >= 3 && time[row][column] == 1) {
	mov	a,r4
	add	a,r4
	add	a,acc
	mov	r2,a
	add	a,#_bounce
	mov	r5,a
	mov	a,_column
	add	a,r5
	mov	r0,a
	mov	ar5,@r0
	cjne	r5,#0x03,00172$
00172$:
	jc	00117$
	mov	a,r4
	swap	a
	rr	a
	anl	a,#0xf8
	add	a,#_time
	mov	r5,a
	mov	a,_column
	add	a,_column
	add	a,r5
	mov	r0,a
	mov	ar5,@r0
	inc	r0
	mov	ar6,@r0
	dec	r0
	cjne	r5,#0x01,00117$
	cjne	r6,#0x00,00117$
;	./include/kb.h:100: capture_input(keyboard[row][column]);
	mov	a,r2
	add	a,#_keyboard
	mov	r5,a
	mov	a,_column
	add	a,r5
	mov	r0,a
	mov	dpl,@r0
	push	ar3
	push	ar4
	lcall	_capture_input
	pop	ar4
	pop	ar3
	sjmp	00133$
00117$:
;	./include/kb.h:102: else if (time[row][column] >= 15) {
	mov	a,r4
	swap	a
	rr	a
	anl	a,#0xf8
	mov	r5,a
	add	a,#_time
	mov	r6,a
	mov	a,_column
	add	a,_column
	add	a,r6
	mov	r0,a
	mov	ar6,@r0
	inc	r0
	mov	ar7,@r0
	dec	r0
	clr	c
	mov	a,r6
	subb	a,#0x0F
	mov	a,r7
	subb	a,#0x00
	jc	00133$
;	./include/kb.h:103: capture_input(keyboard[row][column]);
	mov	a,r2
	add	a,#_keyboard
	mov	r2,a
	mov	a,_column
	add	a,r2
	mov	r0,a
	mov	dpl,@r0
	push	ar3
	push	ar4
	push	ar5
	lcall	_capture_input
	pop	ar5
	pop	ar4
	pop	ar3
;	./include/kb.h:104: time[row][column] = 2;
	mov	a,r5
	add	a,#_time
	mov	r5,a
	mov	a,_column
	add	a,_column
	mov	r2,a
	add	a,r5
	mov	r0,a
	mov	@r0,#0x02
	inc	r0
	mov	@r0,#0x00
00133$:
;	./include/kb.h:73: for (row = 0; row < 4; row++) {
	inc	r4
	ljmp	00131$
00134$:
;	./include/kb.h:108: TH0 = 0xED;    // T0 1kHz
	mov	_TH0,#0xED
;	./include/kb.h:109: TL0 = 0xBB;
	mov	_TL0,#0xBB
	mov	sp,_bp
	pop	_bp
	pop	psw
	pop	(0+1)
	pop	(0+0)
	pop	(0+7)
	pop	(0+6)
	pop	(0+5)
	pop	(0+4)
	pop	(0+3)
	pop	(0+2)
	pop	dph
	pop	dpl
	pop	b
	pop	acc
	pop	bits
	reti
;------------------------------------------------------------
;Allocation info for local variables in function 'init_kb_timer'
;------------------------------------------------------------
;------------------------------------------------------------
;	./include/kb.h:112: void init_kb_timer(){
;	-----------------------------------------
;	 function init_kb_timer
;	-----------------------------------------
_init_kb_timer:
;	./include/kb.h:113: SetVector(0x200B, (void*) timer_kb); // T0 int prog
	mov	r2,#_timer_kb
	mov	r3,#(_timer_kb >> 8)
	mov	r4,#0x80
	push	ar2
	push	ar3
	push	ar4
	mov	dptr,#0x200B
	lcall	_SetVector
	dec	sp
	dec	sp
	dec	sp
;	./include/kb.h:114: TH0 = 0xED;    // T0 1kHz
	mov	_TH0,#0xED
;	./include/kb.h:115: TL0 = 0xBB;
	mov	_TL0,#0xBB
;	./include/kb.h:116: TMOD |= 0x01;  // T0 16 bit
	orl	_TMOD,#0x01
;	./include/kb.h:117: ET0 = 1;       // T0 int
	setb	_ET0
;	./include/kb.h:118: TR0 = 1;       // T0 run
	setb	_TR0
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'print_error'
;------------------------------------------------------------
;------------------------------------------------------------
;	src/lab4.c:29: void print_error(){
;	-----------------------------------------
;	 function print_error
;	-----------------------------------------
_print_error:
;	src/lab4.c:30: EA = 0;
	clr	_EA
;	src/lab4.c:31: type(EOL);
	mov	dptr,#_EOL
	mov	b,#0x80
	lcall	_type
;	src/lab4.c:32: type("Invalid arguments.");
	mov	dptr,#__str_1
	mov	b,#0x80
	lcall	_type
;	src/lab4.c:33: type(EOL);
	mov	dptr,#_EOL
	mov	b,#0x80
	lcall	_type
;	src/lab4.c:34: EA = 1;
	setb	_EA
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'print_num'
;------------------------------------------------------------
;num                       Allocated to registers r2 
;------------------------------------------------------------
;	src/lab4.c:37: void print_num(char num) {
;	-----------------------------------------
;	 function print_num
;	-----------------------------------------
_print_num:
;	src/lab4.c:38: if(num < 0) {
	mov	a,dpl
	mov	r2,a
	jnb	acc.7,00102$
;	src/lab4.c:39: uart_s_write('-');
	mov	dpl,#0x2D
	push	ar2
	lcall	_uart_s_write
	pop	ar2
;	src/lab4.c:40: num *= -1;
	clr	c
	clr	a
	subb	a,r2
	mov	r2,a
00102$:
;	src/lab4.c:42: if(num > 9) uart_s_write(num / 10 + '0');
	clr	c
	mov	a,#(0x09 ^ 0x80)
	mov	b,r2
	xrl	b,#0x80
	subb	a,b
	jnc	00104$
	clr	F0
	mov	b,#0x0a
	mov	a,r2
	jnb	acc.7,00111$
	cpl	F0
	cpl	a
	inc	a
00111$:
	div	ab
	jnb	F0,00112$
	cpl	a
	inc	a
00112$:
	add	a,#0x30
	mov	dpl,a
	push	ar2
	lcall	_uart_s_write
	pop	ar2
00104$:
;	src/lab4.c:43: uart_s_write(num % 10 + '0');
	mov	b,#0x0a
	mov	a,r2
	clr	F0
	jnb	acc.7,00113$
	setb	F0
	cpl	a
	inc	a
00113$:
	div	ab
	mov	a,b
	jnb	F0,00114$
	cpl	a
	inc	a
00114$:
	add	a,#0x30
	mov	dpl,a
	ljmp	_uart_s_write
;------------------------------------------------------------
;Allocation info for local variables in function 'print_result'
;------------------------------------------------------------
;array                     Allocated to stack - offset 1
;count                     Allocated to registers r2 r3 
;it                        Allocated to registers r2 r3 
;i                         Allocated to registers 
;------------------------------------------------------------
;	src/lab4.c:46: void print_result() {
;	-----------------------------------------
;	 function print_result
;	-----------------------------------------
_print_result:
	push	_bp
	mov	a,sp
	mov	_bp,a
	add	a,#0x10
	mov	sp,a
;	src/lab4.c:47: int array[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	mov	r0,_bp
	inc	r0
	mov	@r0,#0x00
	inc	r0
	mov	@r0,#0x00
	dec	r0
	mov	a,#0x02
	add	a,r0
	mov	r1,a
	mov	@r1,#0x00
	inc	r1
	mov	@r1,#0x00
	mov	a,#0x04
	add	a,r0
	mov	r1,a
	mov	@r1,#0x00
	inc	r1
	mov	@r1,#0x00
	mov	a,#0x06
	add	a,r0
	mov	r1,a
	mov	@r1,#0x00
	inc	r1
	mov	@r1,#0x00
	mov	a,#0x08
	add	a,r0
	mov	r1,a
	mov	@r1,#0x00
	inc	r1
	mov	@r1,#0x00
	mov	a,#0x0A
	add	a,r0
	mov	r1,a
	mov	@r1,#0x00
	inc	r1
	mov	@r1,#0x00
	mov	a,#0x0C
	add	a,r0
	mov	r1,a
	mov	@r1,#0x00
	inc	r1
	mov	@r1,#0x00
	mov	a,#0x0E
	add	a,r0
	mov	r1,a
	mov	@r1,#0x00
	inc	r1
	mov	@r1,#0x00
;	src/lab4.c:50: while (first_num > 0) {
	mov	r2,#0x00
	mov	r3,#0x00
00101$:
	clr	c
	mov	a,#(0x00 ^ 0x80)
	mov	b,_first_num
	xrl	b,#0x80
	subb	a,b
	jnc	00103$
;	src/lab4.c:51: array[count] = first_num % 2;
	mov	ar4,r2
	mov	a,r3
	xch	a,r4
	add	a,acc
	xch	a,r4
	rlc	a
	mov	r5,a
	mov	a,r4
	add	a,r0
	mov	r1,a
	mov	a,_first_num
	mov	c,acc.7
	anl	a,#0x01
	jz	00117$
	jnc	00117$
	orl	a,#0xfe
00117$:
	mov	r4,a
	rlc	a
	subb	a,acc
	mov	r5,a
	mov	@r1,ar4
	inc	r1
	mov	@r1,ar5
	dec	r1
;	src/lab4.c:52: first_num = first_num / 2;
	clr	F0
	mov	b,#0x02
	mov	a,_first_num
	jnb	acc.7,00118$
	cpl	F0
	cpl	a
	inc	a
00118$:
	div	ab
	jnb	F0,00119$
	cpl	a
	inc	a
00119$:
	mov	_first_num,a
;	src/lab4.c:53: count++;
	inc	r2
	cjne	r2,#0x00,00101$
	inc	r3
	sjmp	00101$
00103$:
;	src/lab4.c:55: for (it = 7; it >= 0; it--) {
	mov	r2,#0x07
	mov	r3,#0x00
00104$:
	mov	a,r3
	jb	acc.7,00107$
;	src/lab4.c:56: char i = array[it];
	mov	ar4,r2
	mov	a,r3
	xch	a,r4
	add	a,acc
	xch	a,r4
	rlc	a
	mov	a,r4
	add	a,r0
	mov	r1,a
	mov	ar4,@r1
	inc	r1
	mov	ar5,@r1
	dec	r1
	mov	dpl,r4
;	src/lab4.c:57: print_num(i);
	push	ar2
	push	ar3
	push	ar0
	lcall	_print_num
	pop	ar0
	pop	ar3
	pop	ar2
;	src/lab4.c:55: for (it = 7; it >= 0; it--) {
	dec	r2
	cjne	r2,#0xff,00104$
	dec	r3
	sjmp	00104$
00107$:
;	src/lab4.c:60: ir = 0;
	mov	_ir,#0x00
;	src/lab4.c:61: type(EOL);
	mov	dptr,#_EOL
	mov	b,#0x80
	lcall	_type
	mov	sp,_bp
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'to_num'
;------------------------------------------------------------
;size                      Allocated to stack - offset -3
;fifo_pos                  Allocated to stack - offset -5
;num                       Allocated to registers r2 r3 r4 
;------------------------------------------------------------
;	src/lab4.c:64: static int to_num(char *num, unsigned char size, int fifo_pos) {
;	-----------------------------------------
;	 function to_num
;	-----------------------------------------
_to_num:
	push	_bp
	mov	_bp,sp
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	src/lab4.c:65: if(size == 3) *num = (READ_FIFO[fifo_pos] - '0') * 100 + (READ_FIFO[fifo_pos+1] - '0') *10 + (READ_FIFO[fifo_pos+2] - '0');
	mov	r0,_bp
	dec	r0
	dec	r0
	dec	r0
	cjne	@r0,#0x03,00105$
	mov	a,_bp
	add	a,#0xfb
	mov	r1,a
	mov	a,@r1
	add	a,#_READ_FIFO
	mov	r0,a
	mov	a,@r0
	add	a,#0xd0
	mov	b,#0x64
	mul	ab
	mov	r5,a
	mov	a,_bp
	add	a,#0xfb
	mov	r0,a
	mov	a,@r0
	mov	r6,a
	inc	a
	add	a,#_READ_FIFO
	mov	r0,a
	mov	a,@r0
	mov	r7,a
	add	a,#0xd0
	mov	b,#0x0A
	mul	ab
	add	a,r5
	mov	r5,a
	mov	a,#0x02
	add	a,r6
	add	a,#_READ_FIFO
	mov	r0,a
	mov	a,@r0
	mov	r6,a
	add	a,#0xd0
	add	a,r5
	mov	r5,a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrput
	sjmp	00108$
00105$:
;	src/lab4.c:66: else if (size == 2) *num = (READ_FIFO[fifo_pos] - '0') * 10 + (READ_FIFO[fifo_pos+1] - '0');
	mov	r0,_bp
	dec	r0
	dec	r0
	dec	r0
	cjne	@r0,#0x02,00102$
	mov	a,_bp
	add	a,#0xfb
	mov	r1,a
	mov	a,@r1
	add	a,#_READ_FIFO
	mov	r0,a
	mov	a,@r0
	add	a,#0xd0
	mov	b,#0x0A
	mul	ab
	mov	r5,a
	mov	a,_bp
	add	a,#0xfb
	mov	r0,a
	mov	a,@r0
	inc	a
	add	a,#_READ_FIFO
	mov	r0,a
	mov	a,@r0
	mov	r6,a
	add	a,#0xd0
	add	a,r5
	mov	r5,a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrput
	sjmp	00108$
00102$:
;	src/lab4.c:67: else *num = READ_FIFO[fifo_pos] - '0';
	mov	a,_bp
	add	a,#0xfb
	mov	r1,a
	mov	a,@r1
	add	a,#_READ_FIFO
	mov	r0,a
	mov	a,@r0
	add	a,#0xd0
	mov	r5,a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrput
;	src/lab4.c:69: return -1;
00108$:
;	src/lab4.c:71: return 0;
	mov	dptr,#0x0000
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'add_char'
;------------------------------------------------------------
;button                    Allocated to registers r2 
;------------------------------------------------------------
;	src/lab4.c:74: static int add_char(unsigned char button) {
;	-----------------------------------------
;	 function add_char
;	-----------------------------------------
_add_char:
	mov	r2,dpl
;	src/lab4.c:75: READ_FIFO[ir++] = button;
	mov	r3,_ir
	inc	_ir
	mov	a,r3
	add	a,#_READ_FIFO
	mov	r0,a
	mov	@r0,ar2
;	src/lab4.c:76: if(READ_FIFO[ir - 1] == '*') {
	mov	a,_ir
	dec	a
	add	a,#_READ_FIFO
	mov	r0,a
	mov	ar2,@r0
	cjne	r2,#0x2A,00104$
;	src/lab4.c:77: if(first_size == 0) return -1;
	mov	a,_first_size
	jnz	00102$
	mov	dptr,#0xFFFF
	ret
00102$:
;	src/lab4.c:78: return 2;
	mov	dptr,#0x0002
	ret
00104$:
;	src/lab4.c:80: first_size++;
	inc	_first_size
;	src/lab4.c:81: return 0;
	mov	dptr,#0x0000
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'reset'
;------------------------------------------------------------
;------------------------------------------------------------
;	src/lab4.c:102: static void reset() {
;	-----------------------------------------
;	 function reset
;	-----------------------------------------
_reset:
;	src/lab4.c:103: state = ST_FIRST;
	mov	_state,#0x00
;	src/lab4.c:104: first_size = 0;
	mov	_first_size,#0x00
;	src/lab4.c:105: second_size = 0;
	mov	_second_size,#0x00
;	src/lab4.c:106: ir = 0;
	mov	_ir,#0x00
;	src/lab4.c:107: ET0 = 1;
	setb	_ET0
;	src/lab4.c:108: first_num = -1;
	mov	_first_num,#0xFF
;	src/lab4.c:109: second_num = -1;
	mov	_second_num,#0xFF
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'fail'
;------------------------------------------------------------
;------------------------------------------------------------
;	src/lab4.c:112: static void fail() {
;	-----------------------------------------
;	 function fail
;	-----------------------------------------
_fail:
;	src/lab4.c:113: reset();
	lcall	_reset
;	src/lab4.c:114: type(EOL);
	mov	dptr,#_EOL
	mov	b,#0x80
	lcall	_type
;	src/lab4.c:115: print_error();
	ljmp	_print_error
;------------------------------------------------------------
;Allocation info for local variables in function 'main'
;------------------------------------------------------------
;dip                       Allocated to registers r2 
;button                    Allocated to registers r3 
;j                         Allocated to registers 
;rc                        Allocated to registers r3 r4 
;------------------------------------------------------------
;	src/lab4.c:119: void main() {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
;	src/lab4.c:123: uart_s_init(S9600);
	mov	dptr,#0x00FD
	lcall	_uart_s_init
;	src/lab4.c:124: init_kb_timer();
	lcall	_init_kb_timer
;	src/lab4.c:126: EA = 1;
	setb	_EA
;	src/lab4.c:128: while (1) {
00119$:
;	src/lab4.c:129: dip = readdip();
	lcall	_readdip
	mov	r2,dpl
;	src/lab4.c:130: if (dip == NORMAL) {
	cjne	r2,#0xFF,00116$
;	src/lab4.c:131: if (!is_queue_empty()) {
	lcall	_is_queue_empty
	mov	a,dpl
	jnz	00119$
;	src/lab4.c:132: ET0 = 0;
	clr	_ET0
;	src/lab4.c:134: button = get_input();
	lcall	_get_input
	mov	r3,dpl
;	src/lab4.c:135: if(button=='*') uart_s_write('=');
	cjne	r3,#0x2A,00102$
	mov	dpl,#0x3D
	push	ar3
	lcall	_uart_s_write
	pop	ar3
	sjmp	00103$
00102$:
;	src/lab4.c:136: else uart_s_write(button);
	mov	dpl,r3
	push	ar3
	lcall	_uart_s_write
	pop	ar3
00103$:
;	src/lab4.c:138: rc = add_char(button);
	mov	dpl,r3
	lcall	_add_char
	mov	r3,dpl
;	src/lab4.c:139: if(rc < 0) {
	mov	a,dph
	mov	r4,a
	jnb	acc.7,00105$
;	src/lab4.c:140: fail();
	lcall	_fail
;	src/lab4.c:141: continue;
	sjmp	00119$
00105$:
;	src/lab4.c:151: if(rc==2) {
	cjne	r3,#0x02,00107$
	cjne	r4,#0x00,00107$
;	src/lab4.c:157: rc = to_num(&first_num, first_size, 0);
	clr	a
	push	acc
	push	acc
	push	_first_size
	mov	dptr,#_first_num
	mov	b,#0x40
	lcall	_to_num
	dec	sp
	dec	sp
	dec	sp
;	src/lab4.c:158: print_result();
	lcall	_print_result
;	src/lab4.c:159: reset();
	lcall	_reset
00107$:
;	src/lab4.c:186: ET0 = 1;
	setb	_ET0
	sjmp	00119$
00116$:
;	src/lab4.c:188: } else if (dip == DEBUG) {
	cjne	r2,#0xFE,00113$
;	src/lab4.c:189: if (!is_queue_empty()) {
	lcall	_is_queue_empty
	mov	a,dpl
	jz	00141$
	ljmp	00119$
00141$:
;	src/lab4.c:190: ET0 = 0;
	clr	_ET0
;	src/lab4.c:191: uart_s_write(get_input());
	lcall	_get_input
	lcall	_uart_s_write
;	src/lab4.c:192: type(EOL);
	mov	dptr,#_EOL
	mov	b,#0x80
	lcall	_type
;	src/lab4.c:193: ET0 = 1;
	setb	_ET0
	ljmp	00119$
00113$:
;	src/lab4.c:197: leds(0xAA);
	mov	dpl,#0xAA
	lcall	_leds
	ljmp	00119$
	.area CSEG    (CODE)
	.area CONST   (CODE)
_EOL:
	.db #0x0D
	.db #0x0A
	.db #0x00
__str_0:
	.ascii "too many buttons"
	.db 0x0D
	.db 0x0A
	.db 0x00
__str_1:
	.ascii "Invalid arguments."
	.db 0x00
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
