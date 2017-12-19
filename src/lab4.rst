                              1 ;--------------------------------------------------------
                              2 ; File Created by SDCC : free open source ANSI-C Compiler
                              3 ; Version 2.9.0 #5416 (Mar 22 2009) (MINGW32)
                              4 ; This file was generated Wed Dec 13 17:04:57 2017
                              5 ;--------------------------------------------------------
                              6 	.module lab4
                              7 	.optsdcc -mmcs51 --model-small
                              8 	
                              9 ;--------------------------------------------------------
                             10 ; Public variables in this module
                             11 ;--------------------------------------------------------
                             12 	.globl _EOL
                             13 	.globl _main
                             14 	.globl _print_result
                             15 	.globl _print_num
                             16 	.globl _print_error
                             17 	.globl _init_kb_timer
                             18 	.globl _timer_kb
                             19 	.globl _scan_row
                             20 	.globl _invalid_input
                             21 	.globl _get_input
                             22 	.globl _capture_input
                             23 	.globl _is_queue_empty
                             24 	.globl _leds
                             25 	.globl _led
                             26 	.globl _readdip
                             27 	.globl _read_max
                             28 	.globl _write_max
                             29 	.globl _SetVector
                             30 	.globl _SPR0
                             31 	.globl _SPR1
                             32 	.globl _CPHA
                             33 	.globl _CPOL
                             34 	.globl _SPIM
                             35 	.globl _SPE
                             36 	.globl _WCOL
                             37 	.globl _ISPI
                             38 	.globl _I2CI
                             39 	.globl _I2CTX
                             40 	.globl _I2CRS
                             41 	.globl _I2CM
                             42 	.globl _MDI
                             43 	.globl _MCO
                             44 	.globl _MDE
                             45 	.globl _MDO
                             46 	.globl _CS0
                             47 	.globl _CS1
                             48 	.globl _CS2
                             49 	.globl _CS3
                             50 	.globl _SCONV
                             51 	.globl _CCONV
                             52 	.globl _DMA
                             53 	.globl _ADCI
                             54 	.globl _P
                             55 	.globl _F1
                             56 	.globl _OV
                             57 	.globl _RS0
                             58 	.globl _RS1
                             59 	.globl _F0
                             60 	.globl _AC
                             61 	.globl _CY
                             62 	.globl _CAP2
                             63 	.globl _CNT2
                             64 	.globl _TR2
                             65 	.globl _XEN
                             66 	.globl _TCLK
                             67 	.globl _RCLK
                             68 	.globl _EXF2
                             69 	.globl _TF2
                             70 	.globl _WDE
                             71 	.globl _WDS
                             72 	.globl _WDR2
                             73 	.globl _WDR1
                             74 	.globl _PRE0
                             75 	.globl _PRE1
                             76 	.globl _PRE2
                             77 	.globl _PX0
                             78 	.globl _PT0
                             79 	.globl _PX1
                             80 	.globl _PT1
                             81 	.globl _PS
                             82 	.globl _PT2
                             83 	.globl _PADC
                             84 	.globl _PSI
                             85 	.globl _RXD
                             86 	.globl _TXD
                             87 	.globl _INT0
                             88 	.globl _INT1
                             89 	.globl _T0
                             90 	.globl _T1
                             91 	.globl _WR
                             92 	.globl _RD
                             93 	.globl _EX0
                             94 	.globl _ET0
                             95 	.globl _EX1
                             96 	.globl _ET1
                             97 	.globl _ES
                             98 	.globl _ET2
                             99 	.globl _EADC
                            100 	.globl _EA
                            101 	.globl _RI
                            102 	.globl _TI
                            103 	.globl _RB8
                            104 	.globl _TB8
                            105 	.globl _REN
                            106 	.globl _SM2
                            107 	.globl _SM1
                            108 	.globl _SM0
                            109 	.globl _T2
                            110 	.globl _T2EX
                            111 	.globl _IT0
                            112 	.globl _IE0
                            113 	.globl _IT1
                            114 	.globl _IE1
                            115 	.globl _TR0
                            116 	.globl _TF0
                            117 	.globl _TR1
                            118 	.globl _TF1
                            119 	.globl _DACCON
                            120 	.globl _DAC1H
                            121 	.globl _DAC1L
                            122 	.globl _DAC0H
                            123 	.globl _DAC0L
                            124 	.globl _SPICON
                            125 	.globl _SPIDAT
                            126 	.globl _ADCCON3
                            127 	.globl _ADCGAINH
                            128 	.globl _ADCGAINL
                            129 	.globl _ADCOFSH
                            130 	.globl _ADCOFSL
                            131 	.globl _B
                            132 	.globl _ADCCON1
                            133 	.globl _I2CCON
                            134 	.globl _ACC
                            135 	.globl _PSMCON
                            136 	.globl _ADCDATAH
                            137 	.globl _ADCDATAL
                            138 	.globl _ADCCON2
                            139 	.globl _DMAP
                            140 	.globl _DMAH
                            141 	.globl _DMAL
                            142 	.globl _PSW
                            143 	.globl _TH2
                            144 	.globl _TL2
                            145 	.globl _RCAP2H
                            146 	.globl _RCAP2L
                            147 	.globl _T2CON
                            148 	.globl _EADRL
                            149 	.globl _WDCON
                            150 	.globl _EDATA4
                            151 	.globl _EDATA3
                            152 	.globl _EDATA2
                            153 	.globl _EDATA1
                            154 	.globl _ETIM3
                            155 	.globl _ETIM2
                            156 	.globl _ETIM1
                            157 	.globl _ECON
                            158 	.globl _IP
                            159 	.globl _P3
                            160 	.globl _IE2
                            161 	.globl _IE
                            162 	.globl _P2
                            163 	.globl _I2CADD
                            164 	.globl _I2CDAT
                            165 	.globl _SBUF
                            166 	.globl _SCON
                            167 	.globl _P1
                            168 	.globl _TH1
                            169 	.globl _TH0
                            170 	.globl _TL1
                            171 	.globl _TL0
                            172 	.globl _TMOD
                            173 	.globl _TCON
                            174 	.globl _PCON
                            175 	.globl _DPP
                            176 	.globl _DPH
                            177 	.globl _DPL
                            178 	.globl _SP
                            179 	.globl _P0
                            180 	.globl _state
                            181 	.globl _second_num
                            182 	.globl _first_num
                            183 	.globl _second_size
                            184 	.globl _first_size
                            185 	.globl _ir
                            186 	.globl _READ_FIFO
                            187 	.globl _keyboard
                            188 	.globl _end_queue
                            189 	.globl _start_queue
                            190 	.globl _queue
                            191 	.globl _time
                            192 	.globl _bounce
                            193 	.globl _column
                            194 	.globl _uart_s_init
                            195 	.globl _uart_s_read_ready
                            196 	.globl _uart_s_write
                            197 	.globl _uart_s_read
                            198 	.globl _type
                            199 ;--------------------------------------------------------
                            200 ; special function registers
                            201 ;--------------------------------------------------------
                            202 	.area RSEG    (DATA)
                    0080    203 _P0	=	0x0080
                    0081    204 _SP	=	0x0081
                    0082    205 _DPL	=	0x0082
                    0083    206 _DPH	=	0x0083
                    0084    207 _DPP	=	0x0084
                    0087    208 _PCON	=	0x0087
                    0088    209 _TCON	=	0x0088
                    0089    210 _TMOD	=	0x0089
                    008A    211 _TL0	=	0x008a
                    008B    212 _TL1	=	0x008b
                    008C    213 _TH0	=	0x008c
                    008D    214 _TH1	=	0x008d
                    0090    215 _P1	=	0x0090
                    0098    216 _SCON	=	0x0098
                    0099    217 _SBUF	=	0x0099
                    009A    218 _I2CDAT	=	0x009a
                    009B    219 _I2CADD	=	0x009b
                    00A0    220 _P2	=	0x00a0
                    00A8    221 _IE	=	0x00a8
                    00A9    222 _IE2	=	0x00a9
                    00B0    223 _P3	=	0x00b0
                    00B8    224 _IP	=	0x00b8
                    00B9    225 _ECON	=	0x00b9
                    00BA    226 _ETIM1	=	0x00ba
                    00BB    227 _ETIM2	=	0x00bb
                    00C4    228 _ETIM3	=	0x00c4
                    00BC    229 _EDATA1	=	0x00bc
                    00BD    230 _EDATA2	=	0x00bd
                    00BE    231 _EDATA3	=	0x00be
                    00BF    232 _EDATA4	=	0x00bf
                    00C0    233 _WDCON	=	0x00c0
                    00C6    234 _EADRL	=	0x00c6
                    00C8    235 _T2CON	=	0x00c8
                    00CA    236 _RCAP2L	=	0x00ca
                    00CB    237 _RCAP2H	=	0x00cb
                    00CC    238 _TL2	=	0x00cc
                    00CD    239 _TH2	=	0x00cd
                    00D0    240 _PSW	=	0x00d0
                    00D2    241 _DMAL	=	0x00d2
                    00D3    242 _DMAH	=	0x00d3
                    00D4    243 _DMAP	=	0x00d4
                    00D8    244 _ADCCON2	=	0x00d8
                    00D9    245 _ADCDATAL	=	0x00d9
                    00DA    246 _ADCDATAH	=	0x00da
                    00DF    247 _PSMCON	=	0x00df
                    00E0    248 _ACC	=	0x00e0
                    00E8    249 _I2CCON	=	0x00e8
                    00EF    250 _ADCCON1	=	0x00ef
                    00F0    251 _B	=	0x00f0
                    00F1    252 _ADCOFSL	=	0x00f1
                    00F2    253 _ADCOFSH	=	0x00f2
                    00F3    254 _ADCGAINL	=	0x00f3
                    00F4    255 _ADCGAINH	=	0x00f4
                    00F5    256 _ADCCON3	=	0x00f5
                    00F7    257 _SPIDAT	=	0x00f7
                    00F8    258 _SPICON	=	0x00f8
                    00F9    259 _DAC0L	=	0x00f9
                    00FA    260 _DAC0H	=	0x00fa
                    00FB    261 _DAC1L	=	0x00fb
                    00FC    262 _DAC1H	=	0x00fc
                    00FD    263 _DACCON	=	0x00fd
                            264 ;--------------------------------------------------------
                            265 ; special function bits
                            266 ;--------------------------------------------------------
                            267 	.area RSEG    (DATA)
                    008F    268 _TF1	=	0x008f
                    008E    269 _TR1	=	0x008e
                    008D    270 _TF0	=	0x008d
                    008C    271 _TR0	=	0x008c
                    008B    272 _IE1	=	0x008b
                    008A    273 _IT1	=	0x008a
                    0089    274 _IE0	=	0x0089
                    0088    275 _IT0	=	0x0088
                    0091    276 _T2EX	=	0x0091
                    0090    277 _T2	=	0x0090
                    009F    278 _SM0	=	0x009f
                    009E    279 _SM1	=	0x009e
                    009D    280 _SM2	=	0x009d
                    009C    281 _REN	=	0x009c
                    009B    282 _TB8	=	0x009b
                    009A    283 _RB8	=	0x009a
                    0099    284 _TI	=	0x0099
                    0098    285 _RI	=	0x0098
                    00AF    286 _EA	=	0x00af
                    00AE    287 _EADC	=	0x00ae
                    00AD    288 _ET2	=	0x00ad
                    00AC    289 _ES	=	0x00ac
                    00AB    290 _ET1	=	0x00ab
                    00AA    291 _EX1	=	0x00aa
                    00A9    292 _ET0	=	0x00a9
                    00A8    293 _EX0	=	0x00a8
                    00B7    294 _RD	=	0x00b7
                    00B6    295 _WR	=	0x00b6
                    00B5    296 _T1	=	0x00b5
                    00B4    297 _T0	=	0x00b4
                    00B3    298 _INT1	=	0x00b3
                    00B2    299 _INT0	=	0x00b2
                    00B1    300 _TXD	=	0x00b1
                    00B0    301 _RXD	=	0x00b0
                    00BF    302 _PSI	=	0x00bf
                    00BE    303 _PADC	=	0x00be
                    00BD    304 _PT2	=	0x00bd
                    00BC    305 _PS	=	0x00bc
                    00BB    306 _PT1	=	0x00bb
                    00BA    307 _PX1	=	0x00ba
                    00B9    308 _PT0	=	0x00b9
                    00B8    309 _PX0	=	0x00b8
                    00C7    310 _PRE2	=	0x00c7
                    00C6    311 _PRE1	=	0x00c6
                    00C5    312 _PRE0	=	0x00c5
                    00C3    313 _WDR1	=	0x00c3
                    00C2    314 _WDR2	=	0x00c2
                    00C1    315 _WDS	=	0x00c1
                    00C0    316 _WDE	=	0x00c0
                    00CF    317 _TF2	=	0x00cf
                    00CE    318 _EXF2	=	0x00ce
                    00CD    319 _RCLK	=	0x00cd
                    00CC    320 _TCLK	=	0x00cc
                    00CB    321 _XEN	=	0x00cb
                    00CA    322 _TR2	=	0x00ca
                    00C9    323 _CNT2	=	0x00c9
                    00C8    324 _CAP2	=	0x00c8
                    00D7    325 _CY	=	0x00d7
                    00D6    326 _AC	=	0x00d6
                    00D5    327 _F0	=	0x00d5
                    00D4    328 _RS1	=	0x00d4
                    00D3    329 _RS0	=	0x00d3
                    00D2    330 _OV	=	0x00d2
                    00D1    331 _F1	=	0x00d1
                    00D0    332 _P	=	0x00d0
                    00DF    333 _ADCI	=	0x00df
                    00DE    334 _DMA	=	0x00de
                    00DD    335 _CCONV	=	0x00dd
                    00DC    336 _SCONV	=	0x00dc
                    00DB    337 _CS3	=	0x00db
                    00DA    338 _CS2	=	0x00da
                    00D9    339 _CS1	=	0x00d9
                    00D8    340 _CS0	=	0x00d8
                    00EF    341 _MDO	=	0x00ef
                    00EE    342 _MDE	=	0x00ee
                    00ED    343 _MCO	=	0x00ed
                    00EC    344 _MDI	=	0x00ec
                    00EB    345 _I2CM	=	0x00eb
                    00EA    346 _I2CRS	=	0x00ea
                    00E9    347 _I2CTX	=	0x00e9
                    00E8    348 _I2CI	=	0x00e8
                    00FF    349 _ISPI	=	0x00ff
                    00FE    350 _WCOL	=	0x00fe
                    00FD    351 _SPE	=	0x00fd
                    00FC    352 _SPIM	=	0x00fc
                    00FB    353 _CPOL	=	0x00fb
                    00FA    354 _CPHA	=	0x00fa
                    00F9    355 _SPR1	=	0x00f9
                    00F8    356 _SPR0	=	0x00f8
                            357 ;--------------------------------------------------------
                            358 ; overlayable register banks
                            359 ;--------------------------------------------------------
                            360 	.area REG_BANK_0	(REL,OVR,DATA)
   0000                     361 	.ds 8
                            362 ;--------------------------------------------------------
                            363 ; overlayable bit register bank
                            364 ;--------------------------------------------------------
                            365 	.area BIT_BANK	(REL,OVR,DATA)
   0020                     366 bits:
   0020                     367 	.ds 1
                    8000    368 	b0 = bits[0]
                    8100    369 	b1 = bits[1]
                    8200    370 	b2 = bits[2]
                    8300    371 	b3 = bits[3]
                    8400    372 	b4 = bits[4]
                    8500    373 	b5 = bits[5]
                    8600    374 	b6 = bits[6]
                    8700    375 	b7 = bits[7]
                            376 ;--------------------------------------------------------
                            377 ; internal ram data
                            378 ;--------------------------------------------------------
                            379 	.area DSEG    (DATA)
   0021                     380 _old_led:
   0021                     381 	.ds 1
   0022                     382 _column::
   0022                     383 	.ds 1
   0023                     384 _bounce::
   0023                     385 	.ds 16
   0033                     386 _time::
   0033                     387 	.ds 32
   0053                     388 _queue::
   0053                     389 	.ds 10
   005D                     390 _start_queue::
   005D                     391 	.ds 1
   005E                     392 _end_queue::
   005E                     393 	.ds 1
   005F                     394 _keyboard::
   005F                     395 	.ds 16
   006F                     396 _READ_FIFO::
   006F                     397 	.ds 8
   0077                     398 _ir::
   0077                     399 	.ds 1
   0078                     400 _first_size::
   0078                     401 	.ds 1
   0079                     402 _second_size::
   0079                     403 	.ds 1
   007A                     404 _first_num::
   007A                     405 	.ds 1
   007B                     406 _second_num::
   007B                     407 	.ds 1
   007C                     408 _state::
   007C                     409 	.ds 1
                            410 ;--------------------------------------------------------
                            411 ; overlayable items in internal ram 
                            412 ;--------------------------------------------------------
                            413 	.area OSEG    (OVR,DATA)
                            414 ;--------------------------------------------------------
                            415 ; Stack segment in internal ram 
                            416 ;--------------------------------------------------------
                            417 	.area	SSEG	(DATA)
   007D                     418 __start__stack:
   007D                     419 	.ds	1
                            420 
                            421 ;--------------------------------------------------------
                            422 ; indirectly addressable internal ram data
                            423 ;--------------------------------------------------------
                            424 	.area ISEG    (DATA)
                            425 ;--------------------------------------------------------
                            426 ; absolute internal ram data
                            427 ;--------------------------------------------------------
                            428 	.area IABS    (ABS,DATA)
                            429 	.area IABS    (ABS,DATA)
                            430 ;--------------------------------------------------------
                            431 ; bit data
                            432 ;--------------------------------------------------------
                            433 	.area BSEG    (BIT)
                            434 ;--------------------------------------------------------
                            435 ; paged external ram data
                            436 ;--------------------------------------------------------
                            437 	.area PSEG    (PAG,XDATA)
                            438 ;--------------------------------------------------------
                            439 ; external ram data
                            440 ;--------------------------------------------------------
                            441 	.area XSEG    (XDATA)
                            442 ;--------------------------------------------------------
                            443 ; absolute external ram data
                            444 ;--------------------------------------------------------
                            445 	.area XABS    (ABS,XDATA)
                            446 ;--------------------------------------------------------
                            447 ; external initialized ram data
                            448 ;--------------------------------------------------------
                            449 	.area XISEG   (XDATA)
                            450 	.area HOME    (CODE)
                            451 	.area GSINIT0 (CODE)
                            452 	.area GSINIT1 (CODE)
                            453 	.area GSINIT2 (CODE)
                            454 	.area GSINIT3 (CODE)
                            455 	.area GSINIT4 (CODE)
                            456 	.area GSINIT5 (CODE)
                            457 	.area GSINIT  (CODE)
                            458 	.area GSFINAL (CODE)
                            459 	.area CSEG    (CODE)
                            460 ;--------------------------------------------------------
                            461 ; interrupt vector 
                            462 ;--------------------------------------------------------
                            463 	.area HOME    (CODE)
   2100                     464 __interrupt_vect:
   2100 02 21 13            465 	ljmp	__sdcc_gsinit_startup
   2103 32                  466 	reti
   2104                     467 	.ds	7
   210B 02 23 8E            468 	ljmp	_timer_kb
                            469 ;--------------------------------------------------------
                            470 ; global & static initialisations
                            471 ;--------------------------------------------------------
                            472 	.area HOME    (CODE)
                            473 	.area GSINIT  (CODE)
                            474 	.area GSFINAL (CODE)
                            475 	.area GSINIT  (CODE)
                            476 	.globl __sdcc_gsinit_startup
                            477 	.globl __sdcc_program_startup
                            478 	.globl __start__stack
                            479 	.globl __mcs51_genXINIT
                            480 	.globl __mcs51_genXRAMCLEAR
                            481 	.globl __mcs51_genRAMCLEAR
                            482 ;	./include/led.h:6: static unsigned char old_led = 0;   // "Видеопамять" линейки светодиодов
   216C 75 21 00            483 	mov	_old_led,#0x00
                            484 ;	./include/kb.h:11: unsigned char column = 3;
   216F 75 22 03            485 	mov	_column,#0x03
                            486 ;	./include/kb.h:12: unsigned char bounce[4][4] = {{0}};
   2172 75 23 00            487 	mov	_bounce,#0x00
                            488 ;	./include/kb.h:13: unsigned short time[4][4] = {{0}};
   2175 75 33 00            489 	mov	_time,#0x00
   2178 75 34 00            490 	mov	(_time + 1),#0x00
                            491 ;	./include/kb.h:14: unsigned char queue[QUEUE_LENGTH] = {0};
   217B 75 53 00            492 	mov	_queue,#0x00
                            493 ;	./include/kb.h:15: char start_queue = 0, end_queue = 0;
   217E 75 5D 00            494 	mov	_start_queue,#0x00
                            495 ;	./include/kb.h:15: unsigned char keyboard[4][4] = {
   2181 75 5E 00            496 	mov	_end_queue,#0x00
                            497 ;	./include/kb.h:16: {'1', '2', '3', 'A'},
   2184 75 5F 31            498 	mov	_keyboard,#0x31
   2187 75 60 32            499 	mov	(_keyboard + 0x0001),#0x32
   218A 75 61 33            500 	mov	(_keyboard + 0x0002),#0x33
   218D 75 62 41            501 	mov	(_keyboard + 0x0003),#0x41
   2190 75 63 34            502 	mov	(_keyboard + 0x0004),#0x34
   2193 75 64 35            503 	mov	(_keyboard + 0x0005),#0x35
   2196 75 65 36            504 	mov	(_keyboard + 0x0006),#0x36
   2199 75 66 42            505 	mov	(_keyboard + 0x0007),#0x42
   219C 75 67 37            506 	mov	(_keyboard + 0x0008),#0x37
   219F 75 68 38            507 	mov	(_keyboard + 0x0009),#0x38
   21A2 75 69 39            508 	mov	(_keyboard + 0x000a),#0x39
   21A5 75 6A 43            509 	mov	(_keyboard + 0x000b),#0x43
   21A8 75 6B 2A            510 	mov	(_keyboard + 0x000c),#0x2A
   21AB 75 6C 30            511 	mov	(_keyboard + 0x000d),#0x30
   21AE 75 6D 23            512 	mov	(_keyboard + 0x000e),#0x23
   21B1 75 6E 44            513 	mov	(_keyboard + 0x000f),#0x44
                            514 ;	src/lab4.c:19: unsigned char READ_FIFO[BUFFSZ] = {0};
   21B4 75 6F 00            515 	mov	_READ_FIFO,#0x00
                            516 ;	src/lab4.c:20: unsigned char ir = 0;
   21B7 75 77 00            517 	mov	_ir,#0x00
                            518 ;	src/lab4.c:22: unsigned char first_size = 0;
   21BA 75 78 00            519 	mov	_first_size,#0x00
                            520 ;	src/lab4.c:23: unsigned char second_size = 0;
   21BD 75 79 00            521 	mov	_second_size,#0x00
                            522 ;	src/lab4.c:24: char first_num = -1;
   21C0 75 7A FF            523 	mov	_first_num,#0xFF
                            524 ;	src/lab4.c:25: char second_num = -1;
   21C3 75 7B FF            525 	mov	_second_num,#0xFF
                            526 ;	src/lab4.c:27: unsigned char state = 0;
   21C6 75 7C 00            527 	mov	_state,#0x00
                            528 	.area GSFINAL (CODE)
   21C9 02 21 0E            529 	ljmp	__sdcc_program_startup
                            530 ;--------------------------------------------------------
                            531 ; Home
                            532 ;--------------------------------------------------------
                            533 	.area HOME    (CODE)
                            534 	.area HOME    (CODE)
   210E                     535 __sdcc_program_startup:
   210E 12 27 F5            536 	lcall	_main
                            537 ;	return from main will lock up
   2111 80 FE               538 	sjmp .
                            539 ;--------------------------------------------------------
                            540 ; code
                            541 ;--------------------------------------------------------
                            542 	.area CSEG    (CODE)
                            543 ;------------------------------------------------------------
                            544 ;Allocation info for local variables in function 'SetVector'
                            545 ;------------------------------------------------------------
                            546 ;Vector                    Allocated to stack - offset -5
                            547 ;Address                   Allocated to registers r2 r3 
                            548 ;TmpVector                 Allocated to registers r2 r3 
                            549 ;------------------------------------------------------------
                            550 ;	./include/interrupt.h:13: void SetVector(unsigned char xdata * Address, void * Vector) {
                            551 ;	-----------------------------------------
                            552 ;	 function SetVector
                            553 ;	-----------------------------------------
   21CC                     554 _SetVector:
                    0002    555 	ar2 = 0x02
                    0003    556 	ar3 = 0x03
                    0004    557 	ar4 = 0x04
                    0005    558 	ar5 = 0x05
                    0006    559 	ar6 = 0x06
                    0007    560 	ar7 = 0x07
                    0000    561 	ar0 = 0x00
                    0001    562 	ar1 = 0x01
   21CC C0 08               563 	push	_bp
   21CE 85 81 08            564 	mov	_bp,sp
                            565 ;	./include/interrupt.h:16: *Address = 0x02;
   21D1 AA 82               566 	mov	r2,dpl
   21D3 AB 83               567 	mov  r3,dph
   21D5 74 02               568 	mov	a,#0x02
   21D7 F0                  569 	movx	@dptr,a
                            570 ;	./include/interrupt.h:18: TmpVector = (unsigned char xdata *) (Address + 1);
   21D8 0A                  571 	inc	r2
   21D9 BA 00 01            572 	cjne	r2,#0x00,00103$
   21DC 0B                  573 	inc	r3
   21DD                     574 00103$:
                            575 ;	./include/interrupt.h:19: *TmpVector = (unsigned char) ((unsigned short)Vector >> 8);
   21DD E5 08               576 	mov	a,_bp
   21DF 24 FB               577 	add	a,#0xfb
   21E1 F8                  578 	mov	r0,a
   21E2 86 04               579 	mov	ar4,@r0
   21E4 08                  580 	inc	r0
   21E5 86 05               581 	mov	ar5,@r0
   21E7 8D 04               582 	mov	ar4,r5
   21E9 8A 82               583 	mov	dpl,r2
   21EB 8B 83               584 	mov	dph,r3
   21ED EC                  585 	mov	a,r4
   21EE F0                  586 	movx	@dptr,a
   21EF A3                  587 	inc	dptr
   21F0 AA 82               588 	mov	r2,dpl
   21F2 AB 83               589 	mov	r3,dph
                            590 ;	./include/interrupt.h:20: ++TmpVector;
                            591 ;	./include/interrupt.h:21: *TmpVector = (unsigned char) Vector;
   21F4 E5 08               592 	mov	a,_bp
   21F6 24 FB               593 	add	a,#0xfb
   21F8 F8                  594 	mov	r0,a
   21F9 86 04               595 	mov	ar4,@r0
   21FB 8A 82               596 	mov	dpl,r2
   21FD 8B 83               597 	mov	dph,r3
   21FF EC                  598 	mov	a,r4
   2200 F0                  599 	movx	@dptr,a
   2201 D0 08               600 	pop	_bp
   2203 22                  601 	ret
                            602 ;------------------------------------------------------------
                            603 ;Allocation info for local variables in function 'write_max'
                            604 ;------------------------------------------------------------
                            605 ;val                       Allocated to stack - offset -3
                            606 ;regnum                    Allocated to registers r2 r3 
                            607 ;oldDPP                    Allocated to registers r4 
                            608 ;------------------------------------------------------------
                            609 ;	./include/max.h:20: void write_max( unsigned char xdata *regnum, unsigned char val ) {
                            610 ;	-----------------------------------------
                            611 ;	 function write_max
                            612 ;	-----------------------------------------
   2204                     613 _write_max:
   2204 C0 08               614 	push	_bp
   2206 85 81 08            615 	mov	_bp,sp
                            616 ;	./include/max.h:21: unsigned char oldDPP = DPP;
                            617 ;	./include/max.h:22: DPP     = MAXBASE;
                            618 ;	./include/max.h:23: *regnum = val;
   2209 AC 84               619 	mov	r4,_DPP
   220B 75 84 08            620 	mov	_DPP,#0x08
   220E A8 08               621 	mov	r0,_bp
   2210 18                  622 	dec	r0
   2211 18                  623 	dec	r0
   2212 18                  624 	dec	r0
   2213 E6                  625 	mov	a,@r0
   2214 F0                  626 	movx	@dptr,a
                            627 ;	./include/max.h:24: DPP     = oldDPP;
   2215 8C 84               628 	mov	_DPP,r4
   2217 D0 08               629 	pop	_bp
   2219 22                  630 	ret
                            631 ;------------------------------------------------------------
                            632 ;Allocation info for local variables in function 'read_max'
                            633 ;------------------------------------------------------------
                            634 ;regnum                    Allocated to registers r2 r3 
                            635 ;oldDPP                    Allocated to registers r4 
                            636 ;val                       Allocated to registers r2 
                            637 ;------------------------------------------------------------
                            638 ;	./include/max.h:27: unsigned char read_max( unsigned char xdata *regnum ) {
                            639 ;	-----------------------------------------
                            640 ;	 function read_max
                            641 ;	-----------------------------------------
   221A                     642 _read_max:
                            643 ;	./include/max.h:28: unsigned char oldDPP=DPP;
                            644 ;	./include/max.h:31: DPP = MAXBASE;
                            645 ;	./include/max.h:32: val = *regnum;
   221A AC 84               646 	mov	r4,_DPP
   221C 75 84 08            647 	mov	_DPP,#0x08
   221F E0                  648 	movx	a,@dptr
   2220 FA                  649 	mov	r2,a
                            650 ;	./include/max.h:33: DPP = oldDPP;
   2221 8C 84               651 	mov	_DPP,r4
                            652 ;	./include/max.h:35: return val;
   2223 8A 82               653 	mov	dpl,r2
   2225 22                  654 	ret
                            655 ;------------------------------------------------------------
                            656 ;Allocation info for local variables in function 'readdip'
                            657 ;------------------------------------------------------------
                            658 ;------------------------------------------------------------
                            659 ;	./include/max.h:38: unsigned char readdip() {
                            660 ;	-----------------------------------------
                            661 ;	 function readdip
                            662 ;	-----------------------------------------
   2226                     663 _readdip:
                            664 ;	./include/max.h:39: write_max(ENA, 0x00);
   2226 E4                  665 	clr	a
   2227 C0 E0               666 	push	acc
   2229 90 00 04            667 	mov	dptr,#0x0004
   222C 12 22 04            668 	lcall	_write_max
   222F 15 81               669 	dec	sp
                            670 ;	./include/max.h:40: return read_max(EXT_LO);
   2231 90 00 02            671 	mov	dptr,#0x0002
   2234 02 22 1A            672 	ljmp	_read_max
                            673 ;------------------------------------------------------------
                            674 ;Allocation info for local variables in function 'led'
                            675 ;------------------------------------------------------------
                            676 ;on                        Allocated to stack - offset -3
                            677 ;n                         Allocated to registers r2 
                            678 ;c                         Allocated to registers r3 
                            679 ;mask                      Allocated to registers r2 
                            680 ;------------------------------------------------------------
                            681 ;	./include/led.h:9: void led( unsigned char n, unsigned char on )
                            682 ;	-----------------------------------------
                            683 ;	 function led
                            684 ;	-----------------------------------------
   2237                     685 _led:
   2237 C0 08               686 	push	_bp
   2239 85 81 08            687 	mov	_bp,sp
                            688 ;	./include/led.h:14: if( n > 7 ) return;
   223C E5 82               689 	mov	a,dpl
   223E FA                  690 	mov	r2,a
   223F 24 F8               691 	add	a,#0xff - 0x07
   2241 50 02               692 	jnc	00102$
   2243 80 32               693 	sjmp	00106$
   2245                     694 00102$:
                            695 ;	./include/led.h:16: c = old_led;
   2245 AB 21               696 	mov	r3,_old_led
                            697 ;	./include/led.h:18: mask <<= n;
   2247 8A F0               698 	mov	b,r2
   2249 05 F0               699 	inc	b
   224B 74 01               700 	mov	a,#0x01
   224D 80 02               701 	sjmp	00113$
   224F                     702 00111$:
   224F 25 E0               703 	add	a,acc
   2251                     704 00113$:
   2251 D5 F0 FB            705 	djnz	b,00111$
   2254 FA                  706 	mov	r2,a
                            707 ;	./include/led.h:20: if( on )
   2255 A8 08               708 	mov	r0,_bp
   2257 18                  709 	dec	r0
   2258 18                  710 	dec	r0
   2259 18                  711 	dec	r0
   225A E6                  712 	mov	a,@r0
   225B 60 05               713 	jz	00104$
                            714 ;	./include/led.h:21: c |= mask;
   225D EA                  715 	mov	a,r2
   225E 42 03               716 	orl	ar3,a
   2260 80 05               717 	sjmp	00105$
   2262                     718 00104$:
                            719 ;	./include/led.h:23: c &= ~mask;         
   2262 EA                  720 	mov	a,r2
   2263 F4                  721 	cpl	a
   2264 FA                  722 	mov	r2,a
   2265 52 03               723 	anl	ar3,a
   2267                     724 00105$:
                            725 ;	./include/led.h:25: write_max( SV, c );     
   2267 C0 03               726 	push	ar3
   2269 C0 03               727 	push	ar3
   226B 90 00 07            728 	mov	dptr,#0x0007
   226E 12 22 04            729 	lcall	_write_max
   2271 15 81               730 	dec	sp
   2273 D0 03               731 	pop	ar3
                            732 ;	./include/led.h:27: old_led = c;
   2275 8B 21               733 	mov	_old_led,r3
   2277                     734 00106$:
   2277 D0 08               735 	pop	_bp
   2279 22                  736 	ret
                            737 ;------------------------------------------------------------
                            738 ;Allocation info for local variables in function 'leds'
                            739 ;------------------------------------------------------------
                            740 ;on                        Allocated to registers r2 
                            741 ;------------------------------------------------------------
                            742 ;	./include/led.h:30: void leds( unsigned char on ) {
                            743 ;	-----------------------------------------
                            744 ;	 function leds
                            745 ;	-----------------------------------------
   227A                     746 _leds:
   227A AA 82               747 	mov	r2,dpl
                            748 ;	./include/led.h:31: write_max( SV, on );     
   227C C0 02               749 	push	ar2
   227E C0 02               750 	push	ar2
   2280 90 00 07            751 	mov	dptr,#0x0007
   2283 12 22 04            752 	lcall	_write_max
   2286 15 81               753 	dec	sp
   2288 D0 02               754 	pop	ar2
                            755 ;	./include/led.h:32: old_led = on;
   228A 8A 21               756 	mov	_old_led,r2
   228C 22                  757 	ret
                            758 ;------------------------------------------------------------
                            759 ;Allocation info for local variables in function 'uart_s_init'
                            760 ;------------------------------------------------------------
                            761 ;speed                     Allocated to registers r2 r3 
                            762 ;------------------------------------------------------------
                            763 ;	./include/serial.h:21: void uart_s_init(int speed) {
                            764 ;	-----------------------------------------
                            765 ;	 function uart_s_init
                            766 ;	-----------------------------------------
   228D                     767 _uart_s_init:
   228D AA 82               768 	mov	r2,dpl
                            769 ;	./include/serial.h:22: TH1 = speed;   
   228F 8A 8D               770 	mov	_TH1,r2
                            771 ;	./include/serial.h:23: TMOD |= 0x20; // Таймер 1 будет работать в режиме autoreload
   2291 43 89 20            772 	orl	_TMOD,#0x20
                            773 ;	./include/serial.h:24: TR1 = 1;      // start T1
   2294 D2 8E               774 	setb	_TR1
                            775 ;	./include/serial.h:25: SCON = 0x50;  // REN = 1, UART mode 1
   2296 75 98 50            776 	mov	_SCON,#0x50
   2299 22                  777 	ret
                            778 ;------------------------------------------------------------
                            779 ;Allocation info for local variables in function 'uart_s_read_ready'
                            780 ;------------------------------------------------------------
                            781 ;------------------------------------------------------------
                            782 ;	./include/serial.h:28: unsigned char uart_s_read_ready() {
                            783 ;	-----------------------------------------
                            784 ;	 function uart_s_read_ready
                            785 ;	-----------------------------------------
   229A                     786 _uart_s_read_ready:
                            787 ;	./include/serial.h:29: return RI;
   229A A2 98               788 	mov	c,_RI
   229C E4                  789 	clr	a
   229D 33                  790 	rlc	a
   229E F5 82               791 	mov	dpl,a
   22A0 22                  792 	ret
                            793 ;------------------------------------------------------------
                            794 ;Allocation info for local variables in function 'uart_s_write'
                            795 ;------------------------------------------------------------
                            796 ;c                         Allocated to registers 
                            797 ;------------------------------------------------------------
                            798 ;	./include/serial.h:32: void uart_s_write(unsigned char c) {
                            799 ;	-----------------------------------------
                            800 ;	 function uart_s_write
                            801 ;	-----------------------------------------
   22A1                     802 _uart_s_write:
   22A1 85 82 99            803 	mov	_SBUF,dpl
                            804 ;	./include/serial.h:34: TI = 0;
   22A4 C2 99               805 	clr	_TI
                            806 ;	./include/serial.h:35: while(!TI);
   22A6                     807 00101$:
   22A6 30 99 FD            808 	jnb	_TI,00101$
   22A9 22                  809 	ret
                            810 ;------------------------------------------------------------
                            811 ;Allocation info for local variables in function 'uart_s_read'
                            812 ;------------------------------------------------------------
                            813 ;------------------------------------------------------------
                            814 ;	./include/serial.h:38: unsigned char uart_s_read() {
                            815 ;	-----------------------------------------
                            816 ;	 function uart_s_read
                            817 ;	-----------------------------------------
   22AA                     818 _uart_s_read:
                            819 ;	./include/serial.h:39: while(!RI);
   22AA                     820 00101$:
                            821 ;	./include/serial.h:40: RI = 0;
   22AA 10 98 02            822 	jbc	_RI,00108$
   22AD 80 FB               823 	sjmp	00101$
   22AF                     824 00108$:
                            825 ;	./include/serial.h:41: return SBUF;
   22AF 85 99 82            826 	mov	dpl,_SBUF
   22B2 22                  827 	ret
                            828 ;------------------------------------------------------------
                            829 ;Allocation info for local variables in function 'type'
                            830 ;------------------------------------------------------------
                            831 ;str                       Allocated to registers r2 r3 r4 
                            832 ;------------------------------------------------------------
                            833 ;	./include/serial.h:44: void type(char* str) {
                            834 ;	-----------------------------------------
                            835 ;	 function type
                            836 ;	-----------------------------------------
   22B3                     837 _type:
   22B3 AA 82               838 	mov	r2,dpl
   22B5 AB 83               839 	mov	r3,dph
   22B7 AC F0               840 	mov	r4,b
                            841 ;	./include/serial.h:45: while(*str)
   22B9                     842 00101$:
   22B9 8A 82               843 	mov	dpl,r2
   22BB 8B 83               844 	mov	dph,r3
   22BD 8C F0               845 	mov	b,r4
   22BF 12 28 AB            846 	lcall	__gptrget
   22C2 FD                  847 	mov	r5,a
   22C3 60 18               848 	jz	00104$
                            849 ;	./include/serial.h:46: uart_s_write(*str++);
   22C5 0A                  850 	inc	r2
   22C6 BA 00 01            851 	cjne	r2,#0x00,00110$
   22C9 0B                  852 	inc	r3
   22CA                     853 00110$:
   22CA 8D 82               854 	mov	dpl,r5
   22CC C0 02               855 	push	ar2
   22CE C0 03               856 	push	ar3
   22D0 C0 04               857 	push	ar4
   22D2 12 22 A1            858 	lcall	_uart_s_write
   22D5 D0 04               859 	pop	ar4
   22D7 D0 03               860 	pop	ar3
   22D9 D0 02               861 	pop	ar2
   22DB 80 DC               862 	sjmp	00101$
   22DD                     863 00104$:
   22DD 22                  864 	ret
                            865 ;------------------------------------------------------------
                            866 ;Allocation info for local variables in function 'is_queue_empty'
                            867 ;------------------------------------------------------------
                            868 ;------------------------------------------------------------
                            869 ;	./include/kb.h:23: unsigned char is_queue_empty(){
                            870 ;	-----------------------------------------
                            871 ;	 function is_queue_empty
                            872 ;	-----------------------------------------
   22DE                     873 _is_queue_empty:
                            874 ;	./include/kb.h:24: return start_queue == end_queue;
   22DE E5 5E               875 	mov	a,_end_queue
   22E0 B5 5D 04            876 	cjne	a,_start_queue,00103$
   22E3 74 01               877 	mov	a,#0x01
   22E5 80 01               878 	sjmp	00104$
   22E7                     879 00103$:
   22E7 E4                  880 	clr	a
   22E8                     881 00104$:
   22E8 F5 82               882 	mov	dpl,a
   22EA 22                  883 	ret
                            884 ;------------------------------------------------------------
                            885 ;Allocation info for local variables in function 'capture_input'
                            886 ;------------------------------------------------------------
                            887 ;c                         Allocated to registers r2 
                            888 ;------------------------------------------------------------
                            889 ;	./include/kb.h:27: void capture_input(unsigned char c) {
                            890 ;	-----------------------------------------
                            891 ;	 function capture_input
                            892 ;	-----------------------------------------
   22EB                     893 _capture_input:
   22EB AA 82               894 	mov	r2,dpl
                            895 ;	./include/kb.h:28: if (start_queue == QUEUE_LENGTH)
   22ED 74 0A               896 	mov	a,#0x0A
   22EF B5 5D 03            897 	cjne	a,_start_queue,00102$
                            898 ;	./include/kb.h:29: start_queue = 0;
   22F2 75 5D 00            899 	mov	_start_queue,#0x00
   22F5                     900 00102$:
                            901 ;	./include/kb.h:30: queue[start_queue++] = c;
   22F5 AB 5D               902 	mov	r3,_start_queue
   22F7 05 5D               903 	inc	_start_queue
   22F9 EB                  904 	mov	a,r3
   22FA 24 53               905 	add	a,#_queue
   22FC F8                  906 	mov	r0,a
   22FD A6 02               907 	mov	@r0,ar2
   22FF 22                  908 	ret
                            909 ;------------------------------------------------------------
                            910 ;Allocation info for local variables in function 'get_input'
                            911 ;------------------------------------------------------------
                            912 ;------------------------------------------------------------
                            913 ;	./include/kb.h:33: unsigned char get_input() {
                            914 ;	-----------------------------------------
                            915 ;	 function get_input
                            916 ;	-----------------------------------------
   2300                     917 _get_input:
                            918 ;	./include/kb.h:34: if (end_queue == QUEUE_LENGTH)
   2300 74 0A               919 	mov	a,#0x0A
   2302 B5 5E 03            920 	cjne	a,_end_queue,00102$
                            921 ;	./include/kb.h:35: end_queue = 0;
   2305 75 5E 00            922 	mov	_end_queue,#0x00
   2308                     923 00102$:
                            924 ;	./include/kb.h:36: return queue[end_queue++];
   2308 AA 5E               925 	mov	r2,_end_queue
   230A 05 5E               926 	inc	_end_queue
   230C EA                  927 	mov	a,r2
   230D 24 53               928 	add	a,#_queue
   230F F8                  929 	mov	r0,a
   2310 86 82               930 	mov	dpl,@r0
   2312 22                  931 	ret
                            932 ;------------------------------------------------------------
                            933 ;Allocation info for local variables in function 'invalid_input'
                            934 ;------------------------------------------------------------
                            935 ;i                         Allocated to registers r2 
                            936 ;j                         Allocated to registers r5 
                            937 ;------------------------------------------------------------
                            938 ;	./include/kb.h:39: void invalid_input(){
                            939 ;	-----------------------------------------
                            940 ;	 function invalid_input
                            941 ;	-----------------------------------------
   2313                     942 _invalid_input:
                            943 ;	./include/kb.h:41: EA = 0;
   2313 C2 AF               944 	clr	_EA
                            945 ;	./include/kb.h:42: type("too many buttons\r\n");
   2315 90 28 CE            946 	mov	dptr,#__str_0
   2318 75 F0 80            947 	mov	b,#0x80
   231B 12 22 B3            948 	lcall	_type
                            949 ;	./include/kb.h:43: for(i = 0; i < 4; i++) {
   231E 7A 00               950 	mov	r2,#0x00
   2320                     951 00105$:
   2320 BA 04 00            952 	cjne	r2,#0x04,00117$
   2323                     953 00117$:
   2323 50 2B               954 	jnc	00108$
                            955 ;	./include/kb.h:44: for(j = 0; j < 4; j++){
   2325 EA                  956 	mov	a,r2
   2326 2A                  957 	add	a,r2
   2327 25 E0               958 	add	a,acc
   2329 24 23               959 	add	a,#_bounce
   232B FB                  960 	mov	r3,a
   232C EA                  961 	mov	a,r2
   232D C4                  962 	swap	a
   232E 03                  963 	rr	a
   232F 54 F8               964 	anl	a,#0xf8
   2331 24 33               965 	add	a,#_time
   2333 FC                  966 	mov	r4,a
   2334 7D 00               967 	mov	r5,#0x00
   2336                     968 00101$:
   2336 BD 04 00            969 	cjne	r5,#0x04,00119$
   2339                     970 00119$:
   2339 50 12               971 	jnc	00107$
                            972 ;	./include/kb.h:45: bounce[i][j] = 0;
   233B ED                  973 	mov	a,r5
   233C 2B                  974 	add	a,r3
   233D F8                  975 	mov	r0,a
   233E 76 00               976 	mov	@r0,#0x00
                            977 ;	./include/kb.h:46: time[i][j] = 0;
   2340 ED                  978 	mov	a,r5
   2341 2D                  979 	add	a,r5
   2342 FE                  980 	mov	r6,a
   2343 2C                  981 	add	a,r4
   2344 F8                  982 	mov	r0,a
   2345 76 00               983 	mov	@r0,#0x00
   2347 08                  984 	inc	r0
   2348 76 00               985 	mov	@r0,#0x00
                            986 ;	./include/kb.h:44: for(j = 0; j < 4; j++){
   234A 0D                  987 	inc	r5
   234B 80 E9               988 	sjmp	00101$
   234D                     989 00107$:
                            990 ;	./include/kb.h:43: for(i = 0; i < 4; i++) {
   234D 0A                  991 	inc	r2
   234E 80 D0               992 	sjmp	00105$
   2350                     993 00108$:
                            994 ;	./include/kb.h:49: EA = 1;
   2350 D2 AF               995 	setb	_EA
   2352 22                  996 	ret
                            997 ;------------------------------------------------------------
                            998 ;Allocation info for local variables in function 'scan_row'
                            999 ;------------------------------------------------------------
                           1000 ;col                       Allocated to registers r2 
                           1001 ;row                       Allocated to registers r2 
                           1002 ;------------------------------------------------------------
                           1003 ;	./include/kb.h:52: unsigned char scan_row() {
                           1004 ;	-----------------------------------------
                           1005 ;	 function scan_row
                           1006 ;	-----------------------------------------
   2353                    1007 _scan_row:
                           1008 ;	./include/kb.h:55: if (column == 3)
   2353 74 03              1009 	mov	a,#0x03
   2355 B5 22 05           1010 	cjne	a,_column,00102$
                           1011 ;	./include/kb.h:56: column = 0;
   2358 75 22 00           1012 	mov	_column,#0x00
   235B 80 02              1013 	sjmp	00103$
   235D                    1014 00102$:
                           1015 ;	./include/kb.h:57: else column++;
   235D 05 22              1016 	inc	_column
   235F                    1017 00103$:
                           1018 ;	./include/kb.h:59: col = 0x1 << column; //0001,0010,0100,1000,0001,...
   235F 85 22 F0           1019 	mov	b,_column
   2362 05 F0              1020 	inc	b
   2364 74 01              1021 	mov	a,#0x01
   2366 80 02              1022 	sjmp	00111$
   2368                    1023 00109$:
   2368 25 E0              1024 	add	a,acc
   236A                    1025 00111$:
   236A D5 F0 FB           1026 	djnz	b,00109$
                           1027 ;	./include/kb.h:60: write_max(KB, ~col); //11111110,11111101,11111011,11110111,11111110,...
   236D F4                 1028 	cpl	a
   236E FA                 1029 	mov	r2,a
   236F C0 02              1030 	push	ar2
   2371 90 00 00           1031 	mov	dptr,#0x0000
   2374 12 22 04           1032 	lcall	_write_max
   2377 15 81              1033 	dec	sp
                           1034 ;	./include/kb.h:62: row = read_max(KB) & (0xF0);
   2379 90 00 00           1035 	mov	dptr,#0x0000
   237C 12 22 1A           1036 	lcall	_read_max
                           1037 ;	./include/kb.h:63: row = (~(row >> 4)) & 0x0F;
   237F E5 82              1038 	mov	a,dpl
   2381 54 F0              1039 	anl	a,#0xF0
   2383 C4                 1040 	swap	a
   2384 54 0F              1041 	anl	a,#0x0f
   2386 F4                 1042 	cpl	a
   2387 FB                 1043 	mov	r3,a
   2388 74 0F              1044 	mov	a,#0x0F
   238A 5B                 1045 	anl	a,r3
                           1046 ;	./include/kb.h:64: return row;
   238B F5 82              1047 	mov	dpl,a
   238D 22                 1048 	ret
                           1049 ;------------------------------------------------------------
                           1050 ;Allocation info for local variables in function 'timer_kb'
                           1051 ;------------------------------------------------------------
                           1052 ;row                       Allocated to registers r4 
                           1053 ;scanned_row               Allocated to stack - offset 1
                           1054 ;key_pressed               Allocated to registers r3 
                           1055 ;i                         Allocated to registers r2 
                           1056 ;j                         Allocated to registers r7 
                           1057 ;------------------------------------------------------------
                           1058 ;	./include/kb.h:67: void timer_kb(void) __interrupt( 1 ) {
                           1059 ;	-----------------------------------------
                           1060 ;	 function timer_kb
                           1061 ;	-----------------------------------------
   238E                    1062 _timer_kb:
   238E C0 20              1063 	push	bits
   2390 C0 E0              1064 	push	acc
   2392 C0 F0              1065 	push	b
   2394 C0 82              1066 	push	dpl
   2396 C0 83              1067 	push	dph
   2398 C0 02              1068 	push	(0+2)
   239A C0 03              1069 	push	(0+3)
   239C C0 04              1070 	push	(0+4)
   239E C0 05              1071 	push	(0+5)
   23A0 C0 06              1072 	push	(0+6)
   23A2 C0 07              1073 	push	(0+7)
   23A4 C0 00              1074 	push	(0+0)
   23A6 C0 01              1075 	push	(0+1)
   23A8 C0 D0              1076 	push	psw
   23AA 75 D0 00           1077 	mov	psw,#0x00
   23AD C0 08              1078 	push	_bp
   23AF 85 81 08           1079 	mov	_bp,sp
   23B2 05 81              1080 	inc	sp
                           1081 ;	./include/kb.h:71: scanned_row = scan_row();
   23B4 12 23 53           1082 	lcall	_scan_row
   23B7 AA 82              1083 	mov	r2,dpl
   23B9 A8 08              1084 	mov	r0,_bp
   23BB 08                 1085 	inc	r0
   23BC A6 02              1086 	mov	@r0,ar2
                           1087 ;	./include/kb.h:72: key_pressed = 0;
   23BE 7B 00              1088 	mov	r3,#0x00
                           1089 ;	./include/kb.h:73: for (row = 0; row < 4; row++) {
   23C0 7C 00              1090 	mov	r4,#0x00
   23C2                    1091 00131$:
   23C2 BC 04 00           1092 	cjne	r4,#0x04,00155$
   23C5                    1093 00155$:
   23C5 40 03              1094 	jc	00156$
   23C7 02 25 5A           1095 	ljmp	00134$
   23CA                    1096 00156$:
                           1097 ;	./include/kb.h:74: if (scanned_row & (0x01 << row)) {
   23CA C0 03              1098 	push	ar3
   23CC 8C F0              1099 	mov	b,r4
   23CE 05 F0              1100 	inc	b
   23D0 7D 01              1101 	mov	r5,#0x01
   23D2 7E 00              1102 	mov	r6,#0x00
   23D4 80 06              1103 	sjmp	00158$
   23D6                    1104 00157$:
   23D6 ED                 1105 	mov	a,r5
   23D7 2D                 1106 	add	a,r5
   23D8 FD                 1107 	mov	r5,a
   23D9 EE                 1108 	mov	a,r6
   23DA 33                 1109 	rlc	a
   23DB FE                 1110 	mov	r6,a
   23DC                    1111 00158$:
   23DC D5 F0 F7           1112 	djnz	b,00157$
   23DF A8 08              1113 	mov	r0,_bp
   23E1 08                 1114 	inc	r0
   23E2 86 07              1115 	mov	ar7,@r0
   23E4 7B 00              1116 	mov	r3,#0x00
   23E6 EF                 1117 	mov	a,r7
   23E7 52 05              1118 	anl	ar5,a
   23E9 EB                 1119 	mov	a,r3
   23EA 52 06              1120 	anl	ar6,a
   23EC D0 03              1121 	pop	ar3
   23EE ED                 1122 	mov	a,r5
   23EF 4E                 1123 	orl	a,r6
   23F0 60 42              1124 	jz	00110$
                           1125 ;	./include/kb.h:75: if (bounce[row][column] < 3)
   23F2 EC                 1126 	mov	a,r4
   23F3 2C                 1127 	add	a,r4
   23F4 25 E0              1128 	add	a,acc
   23F6 FD                 1129 	mov	r5,a
   23F7 24 23              1130 	add	a,#_bounce
   23F9 FE                 1131 	mov	r6,a
   23FA E5 22              1132 	mov	a,_column
   23FC 2E                 1133 	add	a,r6
   23FD F8                 1134 	mov	r0,a
   23FE 86 06              1135 	mov	ar6,@r0
   2400 BE 03 00           1136 	cjne	r6,#0x03,00160$
   2403                    1137 00160$:
   2403 50 0E              1138 	jnc	00102$
                           1139 ;	./include/kb.h:76: bounce[row][column]++;
   2405 ED                 1140 	mov	a,r5
   2406 24 23              1141 	add	a,#_bounce
   2408 FD                 1142 	mov	r5,a
   2409 E5 22              1143 	mov	a,_column
   240B 2D                 1144 	add	a,r5
   240C F8                 1145 	mov	r0,a
   240D E6                 1146 	mov	a,@r0
   240E FD                 1147 	mov	r5,a
   240F 04                 1148 	inc	a
   2410 F6                 1149 	mov	@r0,a
   2411 80 62              1150 	sjmp	00146$
   2413                    1151 00102$:
                           1152 ;	./include/kb.h:78: time[row][column]++;
   2413 EC                 1153 	mov	a,r4
   2414 C4                 1154 	swap	a
   2415 03                 1155 	rr	a
   2416 54 F8              1156 	anl	a,#0xf8
   2418 24 33              1157 	add	a,#_time
   241A FD                 1158 	mov	r5,a
   241B E5 22              1159 	mov	a,_column
   241D 25 22              1160 	add	a,_column
   241F 2D                 1161 	add	a,r5
   2420 F8                 1162 	mov	r0,a
   2421 86 05              1163 	mov	ar5,@r0
   2423 08                 1164 	inc	r0
   2424 86 06              1165 	mov	ar6,@r0
   2426 18                 1166 	dec	r0
   2427 0D                 1167 	inc	r5
   2428 BD 00 01           1168 	cjne	r5,#0x00,00162$
   242B 0E                 1169 	inc	r6
   242C                    1170 00162$:
   242C A6 05              1171 	mov	@r0,ar5
   242E 08                 1172 	inc	r0
   242F A6 06              1173 	mov	@r0,ar6
   2431 18                 1174 	dec	r0
   2432 80 41              1175 	sjmp	00146$
   2434                    1176 00110$:
                           1177 ;	./include/kb.h:81: if (bounce[row][column] > 0)
   2434 EC                 1178 	mov	a,r4
   2435 2C                 1179 	add	a,r4
   2436 25 E0              1180 	add	a,acc
   2438 FD                 1181 	mov	r5,a
   2439 24 23              1182 	add	a,#_bounce
   243B FE                 1183 	mov	r6,a
   243C E5 22              1184 	mov	a,_column
   243E 2E                 1185 	add	a,r6
   243F F8                 1186 	mov	r0,a
   2440 E6                 1187 	mov	a,@r0
   2441 60 0C              1188 	jz	00107$
                           1189 ;	./include/kb.h:82: bounce[row][column] = 0;
   2443 ED                 1190 	mov	a,r5
   2444 24 23              1191 	add	a,#_bounce
   2446 FD                 1192 	mov	r5,a
   2447 E5 22              1193 	mov	a,_column
   2449 2D                 1194 	add	a,r5
   244A F8                 1195 	mov	r0,a
   244B 76 00              1196 	mov	@r0,#0x00
   244D 80 26              1197 	sjmp	00146$
   244F                    1198 00107$:
                           1199 ;	./include/kb.h:83: else if (time[row][column] > 0)
   244F EC                 1200 	mov	a,r4
   2450 C4                 1201 	swap	a
   2451 03                 1202 	rr	a
   2452 54 F8              1203 	anl	a,#0xf8
   2454 FD                 1204 	mov	r5,a
   2455 24 33              1205 	add	a,#_time
   2457 FE                 1206 	mov	r6,a
   2458 E5 22              1207 	mov	a,_column
   245A 25 22              1208 	add	a,_column
   245C FF                 1209 	mov	r7,a
   245D 2E                 1210 	add	a,r6
   245E F8                 1211 	mov	r0,a
   245F 86 06              1212 	mov	ar6,@r0
   2461 08                 1213 	inc	r0
   2462 86 02              1214 	mov	ar2,@r0
   2464 18                 1215 	dec	r0
   2465 EE                 1216 	mov	a,r6
   2466 4A                 1217 	orl	a,r2
   2467 60 0C              1218 	jz	00146$
                           1219 ;	./include/kb.h:84: time[row][column] = 0;
   2469 ED                 1220 	mov	a,r5
   246A 24 33              1221 	add	a,#_time
   246C FD                 1222 	mov	r5,a
   246D EF                 1223 	mov	a,r7
   246E 2D                 1224 	add	a,r5
   246F F8                 1225 	mov	r0,a
   2470 76 00              1226 	mov	@r0,#0x00
   2472 08                 1227 	inc	r0
   2473 76 00              1228 	mov	@r0,#0x00
                           1229 ;	./include/kb.h:87: for(i = 0; i < 4; i++) {
   2475                    1230 00146$:
   2475 7A 00              1231 	mov	r2,#0x00
   2477                    1232 00127$:
   2477 BA 04 00           1233 	cjne	r2,#0x04,00165$
   247A                    1234 00165$:
   247A 50 2D              1235 	jnc	00130$
                           1236 ;	./include/kb.h:88: for(j = 0; j < 4; j++) {
   247C EA                 1237 	mov	a,r2
   247D 2A                 1238 	add	a,r2
   247E 25 E0              1239 	add	a,acc
   2480 24 23              1240 	add	a,#_bounce
   2482 FD                 1241 	mov	r5,a
   2483 8B 06              1242 	mov	ar6,r3
   2485 7F 00              1243 	mov	r7,#0x00
   2487                    1244 00123$:
   2487 BF 04 00           1245 	cjne	r7,#0x04,00167$
   248A                    1246 00167$:
   248A 50 18              1247 	jnc	00152$
                           1248 ;	./include/kb.h:89: if(bounce[i][j] == 3)
   248C C0 02              1249 	push	ar2
   248E EF                 1250 	mov	a,r7
   248F 2D                 1251 	add	a,r5
   2490 F8                 1252 	mov	r0,a
   2491 86 02              1253 	mov	ar2,@r0
   2493 BA 03 02           1254 	cjne	r2,#0x03,00169$
   2496 80 04              1255 	sjmp	00170$
   2498                    1256 00169$:
   2498 D0 02              1257 	pop	ar2
   249A 80 05              1258 	sjmp	00125$
   249C                    1259 00170$:
   249C D0 02              1260 	pop	ar2
                           1261 ;	./include/kb.h:90: key_pressed++;
   249E 0E                 1262 	inc	r6
   249F 8E 03              1263 	mov	ar3,r6
   24A1                    1264 00125$:
                           1265 ;	./include/kb.h:88: for(j = 0; j < 4; j++) {
   24A1 0F                 1266 	inc	r7
   24A2 80 E3              1267 	sjmp	00123$
   24A4                    1268 00152$:
   24A4 8E 03              1269 	mov	ar3,r6
                           1270 ;	./include/kb.h:87: for(i = 0; i < 4; i++) {
   24A6 0A                 1271 	inc	r2
   24A7 80 CE              1272 	sjmp	00127$
   24A9                    1273 00130$:
                           1274 ;	./include/kb.h:94: leds(key_pressed);
   24A9 8B 82              1275 	mov	dpl,r3
   24AB C0 03              1276 	push	ar3
   24AD C0 04              1277 	push	ar4
   24AF 12 22 7A           1278 	lcall	_leds
   24B2 D0 04              1279 	pop	ar4
   24B4 D0 03              1280 	pop	ar3
                           1281 ;	./include/kb.h:95: if (key_pressed > MAX_KEYS_PRESSED)
   24B6 EB                 1282 	mov	a,r3
   24B7 24 FD              1283 	add	a,#0xff - 0x02
   24B9 50 0E              1284 	jnc	00121$
                           1285 ;	./include/kb.h:96: invalid_input();
   24BB C0 03              1286 	push	ar3
   24BD C0 04              1287 	push	ar4
   24BF 12 23 13           1288 	lcall	_invalid_input
   24C2 D0 04              1289 	pop	ar4
   24C4 D0 03              1290 	pop	ar3
   24C6 02 25 56           1291 	ljmp	00133$
   24C9                    1292 00121$:
                           1293 ;	./include/kb.h:98: key_pressed = 0;
   24C9 7B 00              1294 	mov	r3,#0x00
                           1295 ;	./include/kb.h:99: if (bounce[row][column] >= 3 && time[row][column] == 1) {
   24CB EC                 1296 	mov	a,r4
   24CC 2C                 1297 	add	a,r4
   24CD 25 E0              1298 	add	a,acc
   24CF FA                 1299 	mov	r2,a
   24D0 24 23              1300 	add	a,#_bounce
   24D2 FD                 1301 	mov	r5,a
   24D3 E5 22              1302 	mov	a,_column
   24D5 2D                 1303 	add	a,r5
   24D6 F8                 1304 	mov	r0,a
   24D7 86 05              1305 	mov	ar5,@r0
   24D9 BD 03 00           1306 	cjne	r5,#0x03,00172$
   24DC                    1307 00172$:
   24DC 40 31              1308 	jc	00117$
   24DE EC                 1309 	mov	a,r4
   24DF C4                 1310 	swap	a
   24E0 03                 1311 	rr	a
   24E1 54 F8              1312 	anl	a,#0xf8
   24E3 24 33              1313 	add	a,#_time
   24E5 FD                 1314 	mov	r5,a
   24E6 E5 22              1315 	mov	a,_column
   24E8 25 22              1316 	add	a,_column
   24EA 2D                 1317 	add	a,r5
   24EB F8                 1318 	mov	r0,a
   24EC 86 05              1319 	mov	ar5,@r0
   24EE 08                 1320 	inc	r0
   24EF 86 06              1321 	mov	ar6,@r0
   24F1 18                 1322 	dec	r0
   24F2 BD 01 1A           1323 	cjne	r5,#0x01,00117$
   24F5 BE 00 17           1324 	cjne	r6,#0x00,00117$
                           1325 ;	./include/kb.h:100: capture_input(keyboard[row][column]);
   24F8 EA                 1326 	mov	a,r2
   24F9 24 5F              1327 	add	a,#_keyboard
   24FB FD                 1328 	mov	r5,a
   24FC E5 22              1329 	mov	a,_column
   24FE 2D                 1330 	add	a,r5
   24FF F8                 1331 	mov	r0,a
   2500 86 82              1332 	mov	dpl,@r0
   2502 C0 03              1333 	push	ar3
   2504 C0 04              1334 	push	ar4
   2506 12 22 EB           1335 	lcall	_capture_input
   2509 D0 04              1336 	pop	ar4
   250B D0 03              1337 	pop	ar3
   250D 80 47              1338 	sjmp	00133$
   250F                    1339 00117$:
                           1340 ;	./include/kb.h:102: else if (time[row][column] >= 15) {
   250F EC                 1341 	mov	a,r4
   2510 C4                 1342 	swap	a
   2511 03                 1343 	rr	a
   2512 54 F8              1344 	anl	a,#0xf8
   2514 FD                 1345 	mov	r5,a
   2515 24 33              1346 	add	a,#_time
   2517 FE                 1347 	mov	r6,a
   2518 E5 22              1348 	mov	a,_column
   251A 25 22              1349 	add	a,_column
   251C 2E                 1350 	add	a,r6
   251D F8                 1351 	mov	r0,a
   251E 86 06              1352 	mov	ar6,@r0
   2520 08                 1353 	inc	r0
   2521 86 07              1354 	mov	ar7,@r0
   2523 18                 1355 	dec	r0
   2524 C3                 1356 	clr	c
   2525 EE                 1357 	mov	a,r6
   2526 94 0F              1358 	subb	a,#0x0F
   2528 EF                 1359 	mov	a,r7
   2529 94 00              1360 	subb	a,#0x00
   252B 40 29              1361 	jc	00133$
                           1362 ;	./include/kb.h:103: capture_input(keyboard[row][column]);
   252D EA                 1363 	mov	a,r2
   252E 24 5F              1364 	add	a,#_keyboard
   2530 FA                 1365 	mov	r2,a
   2531 E5 22              1366 	mov	a,_column
   2533 2A                 1367 	add	a,r2
   2534 F8                 1368 	mov	r0,a
   2535 86 82              1369 	mov	dpl,@r0
   2537 C0 03              1370 	push	ar3
   2539 C0 04              1371 	push	ar4
   253B C0 05              1372 	push	ar5
   253D 12 22 EB           1373 	lcall	_capture_input
   2540 D0 05              1374 	pop	ar5
   2542 D0 04              1375 	pop	ar4
   2544 D0 03              1376 	pop	ar3
                           1377 ;	./include/kb.h:104: time[row][column] = 2;
   2546 ED                 1378 	mov	a,r5
   2547 24 33              1379 	add	a,#_time
   2549 FD                 1380 	mov	r5,a
   254A E5 22              1381 	mov	a,_column
   254C 25 22              1382 	add	a,_column
   254E FA                 1383 	mov	r2,a
   254F 2D                 1384 	add	a,r5
   2550 F8                 1385 	mov	r0,a
   2551 76 02              1386 	mov	@r0,#0x02
   2553 08                 1387 	inc	r0
   2554 76 00              1388 	mov	@r0,#0x00
   2556                    1389 00133$:
                           1390 ;	./include/kb.h:73: for (row = 0; row < 4; row++) {
   2556 0C                 1391 	inc	r4
   2557 02 23 C2           1392 	ljmp	00131$
   255A                    1393 00134$:
                           1394 ;	./include/kb.h:108: TH0 = 0xED;    // T0 1kHz
   255A 75 8C ED           1395 	mov	_TH0,#0xED
                           1396 ;	./include/kb.h:109: TL0 = 0xBB;
   255D 75 8A BB           1397 	mov	_TL0,#0xBB
   2560 85 08 81           1398 	mov	sp,_bp
   2563 D0 08              1399 	pop	_bp
   2565 D0 D0              1400 	pop	psw
   2567 D0 01              1401 	pop	(0+1)
   2569 D0 00              1402 	pop	(0+0)
   256B D0 07              1403 	pop	(0+7)
   256D D0 06              1404 	pop	(0+6)
   256F D0 05              1405 	pop	(0+5)
   2571 D0 04              1406 	pop	(0+4)
   2573 D0 03              1407 	pop	(0+3)
   2575 D0 02              1408 	pop	(0+2)
   2577 D0 83              1409 	pop	dph
   2579 D0 82              1410 	pop	dpl
   257B D0 F0              1411 	pop	b
   257D D0 E0              1412 	pop	acc
   257F D0 20              1413 	pop	bits
   2581 32                 1414 	reti
                           1415 ;------------------------------------------------------------
                           1416 ;Allocation info for local variables in function 'init_kb_timer'
                           1417 ;------------------------------------------------------------
                           1418 ;------------------------------------------------------------
                           1419 ;	./include/kb.h:112: void init_kb_timer(){
                           1420 ;	-----------------------------------------
                           1421 ;	 function init_kb_timer
                           1422 ;	-----------------------------------------
   2582                    1423 _init_kb_timer:
                           1424 ;	./include/kb.h:113: SetVector(0x200B, (void*) timer_kb); // T0 int prog
   2582 7A 8E              1425 	mov	r2,#_timer_kb
   2584 7B 23              1426 	mov	r3,#(_timer_kb >> 8)
   2586 7C 80              1427 	mov	r4,#0x80
   2588 C0 02              1428 	push	ar2
   258A C0 03              1429 	push	ar3
   258C C0 04              1430 	push	ar4
   258E 90 20 0B           1431 	mov	dptr,#0x200B
   2591 12 21 CC           1432 	lcall	_SetVector
   2594 15 81              1433 	dec	sp
   2596 15 81              1434 	dec	sp
   2598 15 81              1435 	dec	sp
                           1436 ;	./include/kb.h:114: TH0 = 0xED;    // T0 1kHz
   259A 75 8C ED           1437 	mov	_TH0,#0xED
                           1438 ;	./include/kb.h:115: TL0 = 0xBB;
   259D 75 8A BB           1439 	mov	_TL0,#0xBB
                           1440 ;	./include/kb.h:116: TMOD |= 0x01;  // T0 16 bit
   25A0 43 89 01           1441 	orl	_TMOD,#0x01
                           1442 ;	./include/kb.h:117: ET0 = 1;       // T0 int
   25A3 D2 A9              1443 	setb	_ET0
                           1444 ;	./include/kb.h:118: TR0 = 1;       // T0 run
   25A5 D2 8C              1445 	setb	_TR0
   25A7 22                 1446 	ret
                           1447 ;------------------------------------------------------------
                           1448 ;Allocation info for local variables in function 'print_error'
                           1449 ;------------------------------------------------------------
                           1450 ;------------------------------------------------------------
                           1451 ;	src/lab4.c:29: void print_error(){
                           1452 ;	-----------------------------------------
                           1453 ;	 function print_error
                           1454 ;	-----------------------------------------
   25A8                    1455 _print_error:
                           1456 ;	src/lab4.c:30: EA = 0;
   25A8 C2 AF              1457 	clr	_EA
                           1458 ;	src/lab4.c:31: type(EOL);
   25AA 90 28 CB           1459 	mov	dptr,#_EOL
   25AD 75 F0 80           1460 	mov	b,#0x80
   25B0 12 22 B3           1461 	lcall	_type
                           1462 ;	src/lab4.c:32: type("Invalid arguments.");
   25B3 90 28 E1           1463 	mov	dptr,#__str_1
   25B6 75 F0 80           1464 	mov	b,#0x80
   25B9 12 22 B3           1465 	lcall	_type
                           1466 ;	src/lab4.c:33: type(EOL);
   25BC 90 28 CB           1467 	mov	dptr,#_EOL
   25BF 75 F0 80           1468 	mov	b,#0x80
   25C2 12 22 B3           1469 	lcall	_type
                           1470 ;	src/lab4.c:34: EA = 1;
   25C5 D2 AF              1471 	setb	_EA
   25C7 22                 1472 	ret
                           1473 ;------------------------------------------------------------
                           1474 ;Allocation info for local variables in function 'print_num'
                           1475 ;------------------------------------------------------------
                           1476 ;num                       Allocated to registers r2 
                           1477 ;------------------------------------------------------------
                           1478 ;	src/lab4.c:37: void print_num(char num) {
                           1479 ;	-----------------------------------------
                           1480 ;	 function print_num
                           1481 ;	-----------------------------------------
   25C8                    1482 _print_num:
                           1483 ;	src/lab4.c:38: if(num < 0) {
   25C8 E5 82              1484 	mov	a,dpl
   25CA FA                 1485 	mov	r2,a
   25CB 30 E7 0E           1486 	jnb	acc.7,00102$
                           1487 ;	src/lab4.c:39: uart_s_write('-');
   25CE 75 82 2D           1488 	mov	dpl,#0x2D
   25D1 C0 02              1489 	push	ar2
   25D3 12 22 A1           1490 	lcall	_uart_s_write
   25D6 D0 02              1491 	pop	ar2
                           1492 ;	src/lab4.c:40: num *= -1;
   25D8 C3                 1493 	clr	c
   25D9 E4                 1494 	clr	a
   25DA 9A                 1495 	subb	a,r2
   25DB FA                 1496 	mov	r2,a
   25DC                    1497 00102$:
                           1498 ;	src/lab4.c:42: if(num > 9) uart_s_write(num / 10 + '0');
   25DC C3                 1499 	clr	c
   25DD 74 89              1500 	mov	a,#(0x09 ^ 0x80)
   25DF 8A F0              1501 	mov	b,r2
   25E1 63 F0 80           1502 	xrl	b,#0x80
   25E4 95 F0              1503 	subb	a,b
   25E6 50 1E              1504 	jnc	00104$
   25E8 C2 D5              1505 	clr	F0
   25EA 75 F0 0A           1506 	mov	b,#0x0a
   25ED EA                 1507 	mov	a,r2
   25EE 30 E7 04           1508 	jnb	acc.7,00111$
   25F1 B2 D5              1509 	cpl	F0
   25F3 F4                 1510 	cpl	a
   25F4 04                 1511 	inc	a
   25F5                    1512 00111$:
   25F5 84                 1513 	div	ab
   25F6 30 D5 02           1514 	jnb	F0,00112$
   25F9 F4                 1515 	cpl	a
   25FA 04                 1516 	inc	a
   25FB                    1517 00112$:
   25FB 24 30              1518 	add	a,#0x30
   25FD F5 82              1519 	mov	dpl,a
   25FF C0 02              1520 	push	ar2
   2601 12 22 A1           1521 	lcall	_uart_s_write
   2604 D0 02              1522 	pop	ar2
   2606                    1523 00104$:
                           1524 ;	src/lab4.c:43: uart_s_write(num % 10 + '0');
   2606 75 F0 0A           1525 	mov	b,#0x0a
   2609 EA                 1526 	mov	a,r2
   260A C2 D5              1527 	clr	F0
   260C 30 E7 04           1528 	jnb	acc.7,00113$
   260F D2 D5              1529 	setb	F0
   2611 F4                 1530 	cpl	a
   2612 04                 1531 	inc	a
   2613                    1532 00113$:
   2613 84                 1533 	div	ab
   2614 E5 F0              1534 	mov	a,b
   2616 30 D5 02           1535 	jnb	F0,00114$
   2619 F4                 1536 	cpl	a
   261A 04                 1537 	inc	a
   261B                    1538 00114$:
   261B 24 30              1539 	add	a,#0x30
   261D F5 82              1540 	mov	dpl,a
   261F 02 22 A1           1541 	ljmp	_uart_s_write
                           1542 ;------------------------------------------------------------
                           1543 ;Allocation info for local variables in function 'print_result'
                           1544 ;------------------------------------------------------------
                           1545 ;array                     Allocated to stack - offset 1
                           1546 ;count                     Allocated to registers r2 r3 
                           1547 ;it                        Allocated to registers r2 r3 
                           1548 ;i                         Allocated to registers 
                           1549 ;------------------------------------------------------------
                           1550 ;	src/lab4.c:46: void print_result() {
                           1551 ;	-----------------------------------------
                           1552 ;	 function print_result
                           1553 ;	-----------------------------------------
   2622                    1554 _print_result:
   2622 C0 08              1555 	push	_bp
   2624 E5 81              1556 	mov	a,sp
   2626 F5 08              1557 	mov	_bp,a
   2628 24 10              1558 	add	a,#0x10
   262A F5 81              1559 	mov	sp,a
                           1560 ;	src/lab4.c:47: int array[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   262C A8 08              1561 	mov	r0,_bp
   262E 08                 1562 	inc	r0
   262F 76 00              1563 	mov	@r0,#0x00
   2631 08                 1564 	inc	r0
   2632 76 00              1565 	mov	@r0,#0x00
   2634 18                 1566 	dec	r0
   2635 74 02              1567 	mov	a,#0x02
   2637 28                 1568 	add	a,r0
   2638 F9                 1569 	mov	r1,a
   2639 77 00              1570 	mov	@r1,#0x00
   263B 09                 1571 	inc	r1
   263C 77 00              1572 	mov	@r1,#0x00
   263E 74 04              1573 	mov	a,#0x04
   2640 28                 1574 	add	a,r0
   2641 F9                 1575 	mov	r1,a
   2642 77 00              1576 	mov	@r1,#0x00
   2644 09                 1577 	inc	r1
   2645 77 00              1578 	mov	@r1,#0x00
   2647 74 06              1579 	mov	a,#0x06
   2649 28                 1580 	add	a,r0
   264A F9                 1581 	mov	r1,a
   264B 77 00              1582 	mov	@r1,#0x00
   264D 09                 1583 	inc	r1
   264E 77 00              1584 	mov	@r1,#0x00
   2650 74 08              1585 	mov	a,#0x08
   2652 28                 1586 	add	a,r0
   2653 F9                 1587 	mov	r1,a
   2654 77 00              1588 	mov	@r1,#0x00
   2656 09                 1589 	inc	r1
   2657 77 00              1590 	mov	@r1,#0x00
   2659 74 0A              1591 	mov	a,#0x0A
   265B 28                 1592 	add	a,r0
   265C F9                 1593 	mov	r1,a
   265D 77 00              1594 	mov	@r1,#0x00
   265F 09                 1595 	inc	r1
   2660 77 00              1596 	mov	@r1,#0x00
   2662 74 0C              1597 	mov	a,#0x0C
   2664 28                 1598 	add	a,r0
   2665 F9                 1599 	mov	r1,a
   2666 77 00              1600 	mov	@r1,#0x00
   2668 09                 1601 	inc	r1
   2669 77 00              1602 	mov	@r1,#0x00
   266B 74 0E              1603 	mov	a,#0x0E
   266D 28                 1604 	add	a,r0
   266E F9                 1605 	mov	r1,a
   266F 77 00              1606 	mov	@r1,#0x00
   2671 09                 1607 	inc	r1
   2672 77 00              1608 	mov	@r1,#0x00
                           1609 ;	src/lab4.c:50: while (first_num > 0) {
   2674 7A 00              1610 	mov	r2,#0x00
   2676 7B 00              1611 	mov	r3,#0x00
   2678                    1612 00101$:
   2678 C3                 1613 	clr	c
   2679 74 80              1614 	mov	a,#(0x00 ^ 0x80)
   267B 85 7A F0           1615 	mov	b,_first_num
   267E 63 F0 80           1616 	xrl	b,#0x80
   2681 95 F0              1617 	subb	a,b
   2683 50 40              1618 	jnc	00103$
                           1619 ;	src/lab4.c:51: array[count] = first_num % 2;
   2685 8A 04              1620 	mov	ar4,r2
   2687 EB                 1621 	mov	a,r3
   2688 CC                 1622 	xch	a,r4
   2689 25 E0              1623 	add	a,acc
   268B CC                 1624 	xch	a,r4
   268C 33                 1625 	rlc	a
   268D FD                 1626 	mov	r5,a
   268E EC                 1627 	mov	a,r4
   268F 28                 1628 	add	a,r0
   2690 F9                 1629 	mov	r1,a
   2691 E5 7A              1630 	mov	a,_first_num
   2693 A2 E7              1631 	mov	c,acc.7
   2695 54 01              1632 	anl	a,#0x01
   2697 60 04              1633 	jz	00117$
   2699 50 02              1634 	jnc	00117$
   269B 44 FE              1635 	orl	a,#0xfe
   269D                    1636 00117$:
   269D FC                 1637 	mov	r4,a
   269E 33                 1638 	rlc	a
   269F 95 E0              1639 	subb	a,acc
   26A1 FD                 1640 	mov	r5,a
   26A2 A7 04              1641 	mov	@r1,ar4
   26A4 09                 1642 	inc	r1
   26A5 A7 05              1643 	mov	@r1,ar5
   26A7 19                 1644 	dec	r1
                           1645 ;	src/lab4.c:52: first_num = first_num / 2;
   26A8 C2 D5              1646 	clr	F0
   26AA 75 F0 02           1647 	mov	b,#0x02
   26AD E5 7A              1648 	mov	a,_first_num
   26AF 30 E7 04           1649 	jnb	acc.7,00118$
   26B2 B2 D5              1650 	cpl	F0
   26B4 F4                 1651 	cpl	a
   26B5 04                 1652 	inc	a
   26B6                    1653 00118$:
   26B6 84                 1654 	div	ab
   26B7 30 D5 02           1655 	jnb	F0,00119$
   26BA F4                 1656 	cpl	a
   26BB 04                 1657 	inc	a
   26BC                    1658 00119$:
   26BC F5 7A              1659 	mov	_first_num,a
                           1660 ;	src/lab4.c:53: count++;
   26BE 0A                 1661 	inc	r2
   26BF BA 00 B6           1662 	cjne	r2,#0x00,00101$
   26C2 0B                 1663 	inc	r3
   26C3 80 B3              1664 	sjmp	00101$
   26C5                    1665 00103$:
                           1666 ;	src/lab4.c:55: for (it = 7; it >= 0; it--) {
   26C5 7A 07              1667 	mov	r2,#0x07
   26C7 7B 00              1668 	mov	r3,#0x00
   26C9                    1669 00104$:
   26C9 EB                 1670 	mov	a,r3
   26CA 20 E7 29           1671 	jb	acc.7,00107$
                           1672 ;	src/lab4.c:56: char i = array[it];
   26CD 8A 04              1673 	mov	ar4,r2
   26CF EB                 1674 	mov	a,r3
   26D0 CC                 1675 	xch	a,r4
   26D1 25 E0              1676 	add	a,acc
   26D3 CC                 1677 	xch	a,r4
   26D4 33                 1678 	rlc	a
   26D5 EC                 1679 	mov	a,r4
   26D6 28                 1680 	add	a,r0
   26D7 F9                 1681 	mov	r1,a
   26D8 87 04              1682 	mov	ar4,@r1
   26DA 09                 1683 	inc	r1
   26DB 87 05              1684 	mov	ar5,@r1
   26DD 19                 1685 	dec	r1
   26DE 8C 82              1686 	mov	dpl,r4
                           1687 ;	src/lab4.c:57: print_num(i);
   26E0 C0 02              1688 	push	ar2
   26E2 C0 03              1689 	push	ar3
   26E4 C0 00              1690 	push	ar0
   26E6 12 25 C8           1691 	lcall	_print_num
   26E9 D0 00              1692 	pop	ar0
   26EB D0 03              1693 	pop	ar3
   26ED D0 02              1694 	pop	ar2
                           1695 ;	src/lab4.c:55: for (it = 7; it >= 0; it--) {
   26EF 1A                 1696 	dec	r2
   26F0 BA FF D6           1697 	cjne	r2,#0xff,00104$
   26F3 1B                 1698 	dec	r3
   26F4 80 D3              1699 	sjmp	00104$
   26F6                    1700 00107$:
                           1701 ;	src/lab4.c:60: ir = 0;
   26F6 75 77 00           1702 	mov	_ir,#0x00
                           1703 ;	src/lab4.c:61: type(EOL);
   26F9 90 28 CB           1704 	mov	dptr,#_EOL
   26FC 75 F0 80           1705 	mov	b,#0x80
   26FF 12 22 B3           1706 	lcall	_type
   2702 85 08 81           1707 	mov	sp,_bp
   2705 D0 08              1708 	pop	_bp
   2707 22                 1709 	ret
                           1710 ;------------------------------------------------------------
                           1711 ;Allocation info for local variables in function 'to_num'
                           1712 ;------------------------------------------------------------
                           1713 ;size                      Allocated to stack - offset -3
                           1714 ;fifo_pos                  Allocated to stack - offset -5
                           1715 ;num                       Allocated to registers r2 r3 r4 
                           1716 ;------------------------------------------------------------
                           1717 ;	src/lab4.c:64: static int to_num(char *num, unsigned char size, int fifo_pos) {
                           1718 ;	-----------------------------------------
                           1719 ;	 function to_num
                           1720 ;	-----------------------------------------
   2708                    1721 _to_num:
   2708 C0 08              1722 	push	_bp
   270A 85 81 08           1723 	mov	_bp,sp
   270D AA 82              1724 	mov	r2,dpl
   270F AB 83              1725 	mov	r3,dph
   2711 AC F0              1726 	mov	r4,b
                           1727 ;	src/lab4.c:65: if(size == 3) *num = (READ_FIFO[fifo_pos] - '0') * 100 + (READ_FIFO[fifo_pos+1] - '0') *10 + (READ_FIFO[fifo_pos+2] - '0');
   2713 A8 08              1728 	mov	r0,_bp
   2715 18                 1729 	dec	r0
   2716 18                 1730 	dec	r0
   2717 18                 1731 	dec	r0
   2718 B6 03 3D           1732 	cjne	@r0,#0x03,00105$
   271B E5 08              1733 	mov	a,_bp
   271D 24 FB              1734 	add	a,#0xfb
   271F F9                 1735 	mov	r1,a
   2720 E7                 1736 	mov	a,@r1
   2721 24 6F              1737 	add	a,#_READ_FIFO
   2723 F8                 1738 	mov	r0,a
   2724 E6                 1739 	mov	a,@r0
   2725 24 D0              1740 	add	a,#0xd0
   2727 75 F0 64           1741 	mov	b,#0x64
   272A A4                 1742 	mul	ab
   272B FD                 1743 	mov	r5,a
   272C E5 08              1744 	mov	a,_bp
   272E 24 FB              1745 	add	a,#0xfb
   2730 F8                 1746 	mov	r0,a
   2731 E6                 1747 	mov	a,@r0
   2732 FE                 1748 	mov	r6,a
   2733 04                 1749 	inc	a
   2734 24 6F              1750 	add	a,#_READ_FIFO
   2736 F8                 1751 	mov	r0,a
   2737 E6                 1752 	mov	a,@r0
   2738 FF                 1753 	mov	r7,a
   2739 24 D0              1754 	add	a,#0xd0
   273B 75 F0 0A           1755 	mov	b,#0x0A
   273E A4                 1756 	mul	ab
   273F 2D                 1757 	add	a,r5
   2740 FD                 1758 	mov	r5,a
   2741 74 02              1759 	mov	a,#0x02
   2743 2E                 1760 	add	a,r6
   2744 24 6F              1761 	add	a,#_READ_FIFO
   2746 F8                 1762 	mov	r0,a
   2747 E6                 1763 	mov	a,@r0
   2748 FE                 1764 	mov	r6,a
   2749 24 D0              1765 	add	a,#0xd0
   274B 2D                 1766 	add	a,r5
   274C FD                 1767 	mov	r5,a
   274D 8A 82              1768 	mov	dpl,r2
   274F 8B 83              1769 	mov	dph,r3
   2751 8C F0              1770 	mov	b,r4
   2753 12 28 92           1771 	lcall	__gptrput
   2756 80 4A              1772 	sjmp	00108$
   2758                    1773 00105$:
                           1774 ;	src/lab4.c:66: else if (size == 2) *num = (READ_FIFO[fifo_pos] - '0') * 10 + (READ_FIFO[fifo_pos+1] - '0');
   2758 A8 08              1775 	mov	r0,_bp
   275A 18                 1776 	dec	r0
   275B 18                 1777 	dec	r0
   275C 18                 1778 	dec	r0
   275D B6 02 2C           1779 	cjne	@r0,#0x02,00102$
   2760 E5 08              1780 	mov	a,_bp
   2762 24 FB              1781 	add	a,#0xfb
   2764 F9                 1782 	mov	r1,a
   2765 E7                 1783 	mov	a,@r1
   2766 24 6F              1784 	add	a,#_READ_FIFO
   2768 F8                 1785 	mov	r0,a
   2769 E6                 1786 	mov	a,@r0
   276A 24 D0              1787 	add	a,#0xd0
   276C 75 F0 0A           1788 	mov	b,#0x0A
   276F A4                 1789 	mul	ab
   2770 FD                 1790 	mov	r5,a
   2771 E5 08              1791 	mov	a,_bp
   2773 24 FB              1792 	add	a,#0xfb
   2775 F8                 1793 	mov	r0,a
   2776 E6                 1794 	mov	a,@r0
   2777 04                 1795 	inc	a
   2778 24 6F              1796 	add	a,#_READ_FIFO
   277A F8                 1797 	mov	r0,a
   277B E6                 1798 	mov	a,@r0
   277C FE                 1799 	mov	r6,a
   277D 24 D0              1800 	add	a,#0xd0
   277F 2D                 1801 	add	a,r5
   2780 FD                 1802 	mov	r5,a
   2781 8A 82              1803 	mov	dpl,r2
   2783 8B 83              1804 	mov	dph,r3
   2785 8C F0              1805 	mov	b,r4
   2787 12 28 92           1806 	lcall	__gptrput
   278A 80 16              1807 	sjmp	00108$
   278C                    1808 00102$:
                           1809 ;	src/lab4.c:67: else *num = READ_FIFO[fifo_pos] - '0';
   278C E5 08              1810 	mov	a,_bp
   278E 24 FB              1811 	add	a,#0xfb
   2790 F9                 1812 	mov	r1,a
   2791 E7                 1813 	mov	a,@r1
   2792 24 6F              1814 	add	a,#_READ_FIFO
   2794 F8                 1815 	mov	r0,a
   2795 E6                 1816 	mov	a,@r0
   2796 24 D0              1817 	add	a,#0xd0
   2798 FD                 1818 	mov	r5,a
   2799 8A 82              1819 	mov	dpl,r2
   279B 8B 83              1820 	mov	dph,r3
   279D 8C F0              1821 	mov	b,r4
   279F 12 28 92           1822 	lcall	__gptrput
                           1823 ;	src/lab4.c:69: return -1;
   27A2                    1824 00108$:
                           1825 ;	src/lab4.c:71: return 0;
   27A2 90 00 00           1826 	mov	dptr,#0x0000
   27A5 D0 08              1827 	pop	_bp
   27A7 22                 1828 	ret
                           1829 ;------------------------------------------------------------
                           1830 ;Allocation info for local variables in function 'add_char'
                           1831 ;------------------------------------------------------------
                           1832 ;button                    Allocated to registers r2 
                           1833 ;------------------------------------------------------------
                           1834 ;	src/lab4.c:74: static int add_char(unsigned char button) {
                           1835 ;	-----------------------------------------
                           1836 ;	 function add_char
                           1837 ;	-----------------------------------------
   27A8                    1838 _add_char:
   27A8 AA 82              1839 	mov	r2,dpl
                           1840 ;	src/lab4.c:75: READ_FIFO[ir++] = button;
   27AA AB 77              1841 	mov	r3,_ir
   27AC 05 77              1842 	inc	_ir
   27AE EB                 1843 	mov	a,r3
   27AF 24 6F              1844 	add	a,#_READ_FIFO
   27B1 F8                 1845 	mov	r0,a
   27B2 A6 02              1846 	mov	@r0,ar2
                           1847 ;	src/lab4.c:76: if(READ_FIFO[ir - 1] == '*') {
   27B4 E5 77              1848 	mov	a,_ir
   27B6 14                 1849 	dec	a
   27B7 24 6F              1850 	add	a,#_READ_FIFO
   27B9 F8                 1851 	mov	r0,a
   27BA 86 02              1852 	mov	ar2,@r0
   27BC BA 2A 0C           1853 	cjne	r2,#0x2A,00104$
                           1854 ;	src/lab4.c:77: if(first_size == 0) return -1;
   27BF E5 78              1855 	mov	a,_first_size
   27C1 70 04              1856 	jnz	00102$
   27C3 90 FF FF           1857 	mov	dptr,#0xFFFF
   27C6 22                 1858 	ret
   27C7                    1859 00102$:
                           1860 ;	src/lab4.c:78: return 2;
   27C7 90 00 02           1861 	mov	dptr,#0x0002
   27CA 22                 1862 	ret
   27CB                    1863 00104$:
                           1864 ;	src/lab4.c:80: first_size++;
   27CB 05 78              1865 	inc	_first_size
                           1866 ;	src/lab4.c:81: return 0;
   27CD 90 00 00           1867 	mov	dptr,#0x0000
   27D0 22                 1868 	ret
                           1869 ;------------------------------------------------------------
                           1870 ;Allocation info for local variables in function 'reset'
                           1871 ;------------------------------------------------------------
                           1872 ;------------------------------------------------------------
                           1873 ;	src/lab4.c:102: static void reset() {
                           1874 ;	-----------------------------------------
                           1875 ;	 function reset
                           1876 ;	-----------------------------------------
   27D1                    1877 _reset:
                           1878 ;	src/lab4.c:103: state = ST_FIRST;
   27D1 75 7C 00           1879 	mov	_state,#0x00
                           1880 ;	src/lab4.c:104: first_size = 0;
   27D4 75 78 00           1881 	mov	_first_size,#0x00
                           1882 ;	src/lab4.c:105: second_size = 0;
   27D7 75 79 00           1883 	mov	_second_size,#0x00
                           1884 ;	src/lab4.c:106: ir = 0;
   27DA 75 77 00           1885 	mov	_ir,#0x00
                           1886 ;	src/lab4.c:107: ET0 = 1;
   27DD D2 A9              1887 	setb	_ET0
                           1888 ;	src/lab4.c:108: first_num = -1;
   27DF 75 7A FF           1889 	mov	_first_num,#0xFF
                           1890 ;	src/lab4.c:109: second_num = -1;
   27E2 75 7B FF           1891 	mov	_second_num,#0xFF
   27E5 22                 1892 	ret
                           1893 ;------------------------------------------------------------
                           1894 ;Allocation info for local variables in function 'fail'
                           1895 ;------------------------------------------------------------
                           1896 ;------------------------------------------------------------
                           1897 ;	src/lab4.c:112: static void fail() {
                           1898 ;	-----------------------------------------
                           1899 ;	 function fail
                           1900 ;	-----------------------------------------
   27E6                    1901 _fail:
                           1902 ;	src/lab4.c:113: reset();
   27E6 12 27 D1           1903 	lcall	_reset
                           1904 ;	src/lab4.c:114: type(EOL);
   27E9 90 28 CB           1905 	mov	dptr,#_EOL
   27EC 75 F0 80           1906 	mov	b,#0x80
   27EF 12 22 B3           1907 	lcall	_type
                           1908 ;	src/lab4.c:115: print_error();
   27F2 02 25 A8           1909 	ljmp	_print_error
                           1910 ;------------------------------------------------------------
                           1911 ;Allocation info for local variables in function 'main'
                           1912 ;------------------------------------------------------------
                           1913 ;dip                       Allocated to registers r2 
                           1914 ;button                    Allocated to registers r3 
                           1915 ;j                         Allocated to registers 
                           1916 ;rc                        Allocated to registers r3 r4 
                           1917 ;------------------------------------------------------------
                           1918 ;	src/lab4.c:119: void main() {
                           1919 ;	-----------------------------------------
                           1920 ;	 function main
                           1921 ;	-----------------------------------------
   27F5                    1922 _main:
                           1923 ;	src/lab4.c:123: uart_s_init(S9600);
   27F5 90 00 FD           1924 	mov	dptr,#0x00FD
   27F8 12 22 8D           1925 	lcall	_uart_s_init
                           1926 ;	src/lab4.c:124: init_kb_timer();
   27FB 12 25 82           1927 	lcall	_init_kb_timer
                           1928 ;	src/lab4.c:126: EA = 1;
   27FE D2 AF              1929 	setb	_EA
                           1930 ;	src/lab4.c:128: while (1) {
   2800                    1931 00119$:
                           1932 ;	src/lab4.c:129: dip = readdip();
   2800 12 22 26           1933 	lcall	_readdip
   2803 AA 82              1934 	mov	r2,dpl
                           1935 ;	src/lab4.c:130: if (dip == NORMAL) {
   2805 BA FF 5E           1936 	cjne	r2,#0xFF,00116$
                           1937 ;	src/lab4.c:131: if (!is_queue_empty()) {
   2808 12 22 DE           1938 	lcall	_is_queue_empty
   280B E5 82              1939 	mov	a,dpl
   280D 70 F1              1940 	jnz	00119$
                           1941 ;	src/lab4.c:132: ET0 = 0;
   280F C2 A9              1942 	clr	_ET0
                           1943 ;	src/lab4.c:134: button = get_input();
   2811 12 23 00           1944 	lcall	_get_input
   2814 AB 82              1945 	mov	r3,dpl
                           1946 ;	src/lab4.c:135: if(button=='*') uart_s_write('=');
   2816 BB 2A 0C           1947 	cjne	r3,#0x2A,00102$
   2819 75 82 3D           1948 	mov	dpl,#0x3D
   281C C0 03              1949 	push	ar3
   281E 12 22 A1           1950 	lcall	_uart_s_write
   2821 D0 03              1951 	pop	ar3
   2823 80 09              1952 	sjmp	00103$
   2825                    1953 00102$:
                           1954 ;	src/lab4.c:136: else uart_s_write(button);
   2825 8B 82              1955 	mov	dpl,r3
   2827 C0 03              1956 	push	ar3
   2829 12 22 A1           1957 	lcall	_uart_s_write
   282C D0 03              1958 	pop	ar3
   282E                    1959 00103$:
                           1960 ;	src/lab4.c:138: rc = add_char(button);
   282E 8B 82              1961 	mov	dpl,r3
   2830 12 27 A8           1962 	lcall	_add_char
   2833 AB 82              1963 	mov	r3,dpl
                           1964 ;	src/lab4.c:139: if(rc < 0) {
   2835 E5 83              1965 	mov	a,dph
   2837 FC                 1966 	mov	r4,a
   2838 30 E7 05           1967 	jnb	acc.7,00105$
                           1968 ;	src/lab4.c:140: fail();
   283B 12 27 E6           1969 	lcall	_fail
                           1970 ;	src/lab4.c:141: continue;
   283E 80 C0              1971 	sjmp	00119$
   2840                    1972 00105$:
                           1973 ;	src/lab4.c:151: if(rc==2) {
   2840 BB 02 1F           1974 	cjne	r3,#0x02,00107$
   2843 BC 00 1C           1975 	cjne	r4,#0x00,00107$
                           1976 ;	src/lab4.c:157: rc = to_num(&first_num, first_size, 0);
   2846 E4                 1977 	clr	a
   2847 C0 E0              1978 	push	acc
   2849 C0 E0              1979 	push	acc
   284B C0 78              1980 	push	_first_size
   284D 90 00 7A           1981 	mov	dptr,#_first_num
   2850 75 F0 40           1982 	mov	b,#0x40
   2853 12 27 08           1983 	lcall	_to_num
   2856 15 81              1984 	dec	sp
   2858 15 81              1985 	dec	sp
   285A 15 81              1986 	dec	sp
                           1987 ;	src/lab4.c:158: print_result();
   285C 12 26 22           1988 	lcall	_print_result
                           1989 ;	src/lab4.c:159: reset();
   285F 12 27 D1           1990 	lcall	_reset
   2862                    1991 00107$:
                           1992 ;	src/lab4.c:186: ET0 = 1;
   2862 D2 A9              1993 	setb	_ET0
   2864 80 9A              1994 	sjmp	00119$
   2866                    1995 00116$:
                           1996 ;	src/lab4.c:188: } else if (dip == DEBUG) {
   2866 BA FE 20           1997 	cjne	r2,#0xFE,00113$
                           1998 ;	src/lab4.c:189: if (!is_queue_empty()) {
   2869 12 22 DE           1999 	lcall	_is_queue_empty
   286C E5 82              2000 	mov	a,dpl
   286E 60 03              2001 	jz	00141$
   2870 02 28 00           2002 	ljmp	00119$
   2873                    2003 00141$:
                           2004 ;	src/lab4.c:190: ET0 = 0;
   2873 C2 A9              2005 	clr	_ET0
                           2006 ;	src/lab4.c:191: uart_s_write(get_input());
   2875 12 23 00           2007 	lcall	_get_input
   2878 12 22 A1           2008 	lcall	_uart_s_write
                           2009 ;	src/lab4.c:192: type(EOL);
   287B 90 28 CB           2010 	mov	dptr,#_EOL
   287E 75 F0 80           2011 	mov	b,#0x80
   2881 12 22 B3           2012 	lcall	_type
                           2013 ;	src/lab4.c:193: ET0 = 1;
   2884 D2 A9              2014 	setb	_ET0
   2886 02 28 00           2015 	ljmp	00119$
   2889                    2016 00113$:
                           2017 ;	src/lab4.c:197: leds(0xAA);
   2889 75 82 AA           2018 	mov	dpl,#0xAA
   288C 12 22 7A           2019 	lcall	_leds
   288F 02 28 00           2020 	ljmp	00119$
                           2021 	.area CSEG    (CODE)
                           2022 	.area CONST   (CODE)
   28CB                    2023 _EOL:
   28CB 0D                 2024 	.db #0x0D
   28CC 0A                 2025 	.db #0x0A
   28CD 00                 2026 	.db #0x00
   28CE                    2027 __str_0:
   28CE 74 6F 6F 20 6D 61  2028 	.ascii "too many buttons"
        6E 79 20 62 75 74
        74 6F 6E 73
   28DE 0D                 2029 	.db 0x0D
   28DF 0A                 2030 	.db 0x0A
   28E0 00                 2031 	.db 0x00
   28E1                    2032 __str_1:
   28E1 49 6E 76 61 6C 69  2033 	.ascii "Invalid arguments."
        64 20 61 72 67 75
        6D 65 6E 74 73 2E
   28F3 00                 2034 	.db 0x00
                           2035 	.area XINIT   (CODE)
                           2036 	.area CABS    (ABS,CODE)
