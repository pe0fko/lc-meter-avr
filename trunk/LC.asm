;************************************************************************
;**
;** Project......: L & C Meter
;**
;** Platform.....: AT90S2313
;**
;** RCS-ID.......: $Id$
;**
;** Programmer...: F.W. Krom, K2-Electronics
;** 
;** Description..: Meet de L or C waarde zelf calibrerend door bekende C.
;**
;** History......: 
;**
;**************************************************************************/

.include "2313def.inc"
.list

;.equ	xtal	= 4433619
;.equ	xtal	= 8000000
.equ	xtal	= 10000000

; Bekende referentie C
;.equ	_Cref	= 100		; 100.0pF
.equ	_Cref	= 1000-46	; 944.0pF

.equ	prescale = 64		; timer0 prescaler count
;.equ	prescale = 256		; timer0 prescaler count

; Maximale tijd die we meten voor pulsen.
; Bij 10MHz en prescaler van 64 (timer0) is dat minimaal 1.22 kHz (0,82ms)
;.equ	portmax	= 0x7f		; 128 * 64 / 10MHz = 1.22 kHz


.macro	IO	; <name> <port> <pin>
.set	prt_@0		= PORT@1
.set	pin_@0		= PIN@1		; Input use PIN
.set	ddr_@0		= DDR@1
.set	bit_@0		= P@1@2
.endmacro

.macro	IOInp	; <name>
	cbi	ddr_@0,bit_@0
.endmacro

.macro	IOOut	; <name>
	sbi	ddr_@0,bit_@0
.endmacro

.macro	IOSet	; <name>
	sbi	prt_@0,bit_@0
.endmacro

.macro	IOClear	; <name>
	cbi	prt_@0,bit_@0
.endmacro

.macro	storeX	; <memory>
	ldi	ZL,low(@0)
	rcall	stX
.endmacro

.macro	storeY	; <memory>
	ldi	ZL,low(@0)
	rcall	stY
.endmacro

.macro	loadX	; <memory>
	ldi	ZL,low(@0)
	rcall	ldX
.endmacro

.macro	loadY	; <memory>
	ldi	ZL,low(@0)
	rcall	ldY
.endmacro


;***** Global 
;
;            +--+-+--+
;     !RESET |  |_|  | VCC
;    (RX)PD0 |       | PB7(SCK)
;    (TX)PD1 |       | PB6(MISO)
;      XTAL2 |       | PB5(MOSI)
;      XTAL1 |       | PB4
;  (INT0)PD2 |       | PB3(OC1)
;  (INT1)PD3 |       | PB2
;    (T0)PD4 |       | PB1(AIN1)
;    (T1)PD5 |       | PB0(AIN0)
;        GND |       | PD6(ICP)
;            +-------+
;
;
; PD0	RS232 	RX
; PD1	RS232 	TX
; PD2	Relay input
; PD3	Relay ref C
; PD4	
; PD5	Frequence input
; PD6	
; PD7	Bestaat niet
;
; PB0	Switch Calibrate
; PB1	Switch Mode
; PB2	LCD		RS
; PB3	LCD		EN
; PB4	LCD		D4
; PB5	LCD		D5
; PB6	LCD		D6
; PB7	LCD		D7

IO	freq,D,5		; T1 Freq input 16 bit counter
IO	relay_refc,D,3		; Relay reference C
IO	relay_inp,D,2		; Relay L or C
IO	mode,B,1		; Switch L or C
IO	calibr,B,0		; Switch calibrate

.EQU	lcd_port	= PORTB
.EQU	lcd_ddr		= DDRB 
.EQU	lcd_rs		= PB2
.EQU	lcd_en		= PB3

;***** Registers
.undef	XH	; r27
.undef	XL	; r26
.undef	YH	; r29
.undef	YL	; r28

.def	freqL	= r1
.def	freqH	= r2

.def	RY0	= r4
.def	RY1	= r5
.def	RY2	= r6
.def	RY3	= r7

.def	RR0	= r8
.def	RR1	= r9
.def	RR2	= r10
.def	RR3	= r11
.def	RR4	= r12

.def	ssreg	= r28
.def	porttim	= r29
.def	portval	= r15

.def	RX0	= r16
.def	RX1	= r17
.def	RX2	= r18
.def	RX3	= r19
.def	RX4	= r20
.def	RX5	= r21
.def	RX6	= r22
.def	RX7	= r23

.def	ticks	= r24		; decrement every timer1 interrupt (8,192ms)

.def	menu	= r25
.def	chr	= r26		; output char
.def	tmp	= r27		; Scratchregister

.undef	ZH
.def	zero	= r31		; ZH

.equ	mCap	= 0		; Menu values
.equ	mInd	= 1
.equ	mFrqC	= 2
.equ	mFrqL	= 3
.equ	mCnst	= 4		; Display constants L & C
.equ	mEnd	= 5

;--------------------------------------------------------------------
;-- SRAM variablen
;--------------------------------------------------------------------
.dseg
.equ	buflen	= 12

Cref:	.byte	4			; C Reference
N1:	.byte	4			; Count freq F1
N2:	.byte	4			; Count freq F2
NQ1:	.byte	4			; Count freq F1^2
NQ2:	.byte	4			; Count freq F2^2
ND:	.byte	4			; Count freq F1^2 - F2^2
C:	.byte	4			; OSC Capaciteit
L:	.byte	4			; OSC Inductie
buf:	.byte	buflen			; Buffer number string
endbuf:	
KM1:	.byte	4			; -2
K10:	.byte	4			; 10
K100:	.byte	4			; 100
;KTMP:	.byte	4			; 
KL:	.byte	4			; Constant L calculation
KFRQ:	.byte	4			; Constant frequence counter

;--------------------------------------------------------------------
;-- Interrupt table AT90S2313
;--------------------------------------------------------------------
.cseg
.org 0
	rjmp	RESET			; Reset Handle
	reti				; IRQ0 Handler
	reti				; IRQ1 Handler
	reti				; Timer1 Capture Handler
	reti				; Timer1 Compare Handler
	rjmp	TIMER1			; Timer1 Overflow Handler
	rjmp	TIMER0			; Timer0 Overflow Handler
	reti				; UART RX Complete Handler
	reti				; UDR Empty Handler
	reti				; UART TX Complete Handler
	reti				; Analog Comparator Handler

;--------------------------------------------------------------------
;-- Text constants
;--------------------------------------------------------------------

tHead:	.db	"LC Meter, PE0FKO",0,0
tCap:	.db	"   Capaciteit   ",0,0
tIndc:	.db	"   Inductie     ",0,0
tFreqC:	.db	"  Frequentie C  ",0,0
tFreqL:	.db	"  Frequentie L  ",0,0
tConst:	.db	"                ",0,0
tCali:	.db	" *Calibrating*  ",0,0
tNoOSC:	.db	" *No osc*       ",0,0
tpF:	.db	" pF",0
tnH:	.db	" nH",0
tHz:	.db	" Hz",0

ptrHdr:	.db	tCap,tIndc+0x80,tFreqC,tFreqL+0x80,tConst,0
ptrFnc:	.dw	fCap,fIndc,fFrqC,fFrqL,fCnst

;ptrFnc:	.dw	ret0,ret0,ret0,ret0,ret0,ret0
;ret0:	ret

;--------------------------------------------------------------------
;-- Timer0 overflow interupt
;-- Every 6,5536ms, Xtal=10Mhz, div=256, cnt=256
;--------------------------------------------------------------------
TIMER0:	in	ssreg,SREG

	dec	ticks			; Ticks for extra timer!

	dec	porttim
;	andi	porttim,portmax		; Only portmax value
	brne	ret1

	brts	x0
	  in	freqL,tcnt1l		; Save Timer1 16 bit freq value
	  in	freqH,tcnt1h
	  ;set				; T=1, ready sample
;	  sbr	ssreg,1<<SREG_T		; T=1, ready sample
	x0:

	out	TCNT1H,porttim		; Clear the freq counter
	out	TCNT1L,porttim

	mov	porttim,portval		; Next timer count

	out	SREG,ssreg
	set				; T=1, ready sample
	reti


;--------------------------------------------------------------------
;-- Timer1 overflow interupt
;--------------------------------------------------------------------
TIMER1:	in	ssreg,SREG

	tst	portval			; Only the first time
	brne	ret1			;  portval is zero then.

	neg	porttim			; portval = -porttim
;	andi	porttim,portmax
	mov	portval,porttim

	ldi	porttim,1		; Next timer count

ret1:	out	SREG,ssreg
	reti


;--------------------------------------------------------------------
;-- Reset / Main
;--------------------------------------------------------------------
RESET:
	cli				;Disable global interrupts

	; Load stack pointer
	ldi	tmp,low(RAMEND)
	out	SPL,tmp

	; Port B is output (LCD)
	ldi	tmp,0b11111100		; PB7..PB2 Output LCD
	out	lcd_ddr,tmp
	cbi	lcd_port,lcd_rs		; LCD RS=0
	cbi	lcd_port,lcd_en		; LCD EN=0

	IOInp	freq			; Freq counter input
	IOClear	freq			; High ohm input

	IOOut	relay_refc		; Relay output
	IOSet	relay_refc		; Relay off (via TOR)

	IOOut	relay_inp		; Relay output
	IOSet	relay_inp		; Relay off (via TOR)

	IOInp	mode			; Switch mode
	IOSet	mode			; Source sync

	IOInp	calibr			; Switch calibrate
	IOSet	calibr			; Source sync


	; GIMSK: General Interrupt FLAG
	ldi	tmp,(0<<INT0)|(0<<INT1)
	out	GIMSK,tmp

	; TIMSK: Timer/Counter Interrupt Mask
	; TOIE1: Timer/Counter1 Overflow Interrupt Enable
	; OCIE1A: Timer/Counter1 Output Compare Match Interrupt Enable
	; TICIE1: Timer/Counter1 Input Capture Interrupt Enable
	; TOIE0: Timer/Counter0 Overflow Interrupt Enable
	ldi	tmp,(1<<TOIE1)|(0<<OCIE1A)|(0<<TICIE1)|(1<<TOIE0)
	out	TIMSK,tmp

	; MCUCR
	; SE: Sleep Enable
	; SM: Sleep Mode
	; ISC11, ISC10: Interrupt Sense Control 1 Bit 1 and Bit 0
	; ISC01, ISC00: Interrupt Sense Control 0 Bit 1 and Bit 0
	ldi	tmp,(1<<SE)|(0<<SM)|(0<<ISC10)|(0<<ISC11)|(0<<ISC00)|(0<<ISC01)
	out	MCUCR,tmp

	; Timer0, Clock/256
	; Maximale tijd die we meten voor pulsen.
	; Bij 10MHz en prescaler van 64 is dat minimaal 610 Hz (1,6384ms)
.if prescale == 64
	ldi	tmp,(0<<CS02)|(1<<CS01)|(1<<CS00)	; 64
.endif
.if prescale == 256
	ldi	tmp,(1<<CS02)|(0<<CS01)|(0<<CS00)	; 256
.endif
	out	TCCR0,tmp

	; Timer1
	; COM1A1, COM1A0: Compare Output Mode1, Bits 1 and 0
	; PWM11, PWM10: Pulse Width Modulator Select Bits
	ldi	tmp,(0<<COM1A1)|(0<<COM1A0)|(0<<PWM11)|(0<<PWM10)
	out	TCCR1A,tmp
	;
	; ICNC1: Input Capture1 Noise Canceler
	; ICES1: Input Capture1 Edge Select
	; CTC1: Clear Timer/Counter1 on Compare Match
	; CS12, CS11, CS10: Clock Select1, Bits 2, 1 and 0 
	; 	111 = External Pin T1, rising edge
	ldi	tmp,(0<<ICNC1)|(0<<ICES1)|(0<<CTC1)|(1<<CS12)|(1<<CS11)|(1<<CS10)
	out	TCCR1B,tmp

	sei				; Enable the interrupts

	; Long delay of 15ms needded for the LCD
	ldi	ticks,3			; 20ms
	rcall	dlong

;	Init LCD interface
	rcall	lcd_init

;	RS232 Interface
;	rcall	rs232_init

	clr	zero			; ZH Always zero for store functions

	; Store the constants in SRAM
	ldi	RX0,-1			; KM1 = -1
	ldi	RX1,-1
	ldi	RX2,-1
	ldi	RX3,-1
	storeX	KM1

	; f = N * KFRQ / Portval / 100
	; KFRQ = Xtal / (prescaler * 256) * 100
.equ	_KFRQ = xtal*100/(256*prescale)
	ldi	RX0,byte1(_KFRQ)
	ldi	RX1,byte2(_KFRQ)
	ldi	RX2,byte3(_KFRQ)
	ldi	RX3,byte4(_KFRQ)
	storeX	KFRQ


	;-- K[48] = 10^9 * 2^7 * prescale / (pi * Xtal) * sqrt(1000)
.equ	_KL = 8245938		; prescale=64 @10MHz
;.equ	_KL = 			; prescale=256 @10MHz

;	;-- K[48] = 10^9 * 2^7 * prescale / (pi * Xtal) * 10^2
;.equ	_KL = 26075945		; prescale=64 @10MHz
;.equ	_KL = 104303783		; prescale=256 @10MHz

;	;-- K[48] = 10^9 * 2^7 * prescale / (pi * Xtal) * 10^2 * sqrt(10)
;.equ	_KL = 82459381		; prescale=64 @10MHz
;.equ	_KL = 329837524		; prescale=256 @10MHz
	ldi	RX0,byte1(_KL)		; RX = K
	ldi	RX1,byte2(_KL)
	ldi	RX2,byte3(_KL)
	ldi	RX3,byte4(_KL)
	storeX	KL

	ldi	RX0,byte1(_Cref)	; Cref = _Cref
	ldi	RX1,byte2(_Cref)
	clr	RX2
	clr	RX3
	storeX	Cref

	ldi	RX0,byte1(10)		; K10 = 10
	ldi	RX1,byte2(10)
	clr	RX2
	clr	RX3
	storeX	K10

	ldi	RX0,byte1(100)		; K100 = 100
	ldi	RX1,byte2(100)
	clr	RX2
	clr	RX3
	storeX	K100

;--	DEBUG
.if 0
.equ	xN1	=	64810
.equ	xN2	=	47058
.equ	xN3	=	51061

	ldi	RX0,low(xN1)
	ldi	RX1,high(xN1)
	clr	RX2
	clr	RX3
	storeX	N1
	rcall	rxquad			; RX = RX^2
	storeX	NQ1

	ldi	RX0,low(xN2)
	ldi	RX1,high(xN2)
	clr	RX2
	clr	RX3
	storeX	N2

	ldi	tmp,60
	mov	portval,tmp

	rcall	debug1

	ldi	RX0,byte1(xN3)
	ldi	RX1,byte2(xN3)
	clr	RX2
	clr	RX3

	rcall	debug2
	loadX	C			; RX = C
	rcall	calc2
	rcall	cvt

	ldi	RX0,byte1(xN3)
	ldi	RX1,byte2(xN3)
	clr	RX2
	clr	RX3

	rcall	debug2
	loadX	L			; RX = L
	rcall	calc2
	rcall	cvt

.endif
;--	DEBUG

	; Display welkome string
	rcall	lcd_line0		; Line 0, Char 0
	ldi	ZL,low(2*thead)		; "LC Meter, PE0FKO"
	rcall	print_str

	rcall	calibrate		; Do calibrate

;--------------------------------------------------------------------
;-- Start of the menu selection
;--------------------------------------------------------------------
	ldi	menu,mCap		; Start C menu

	sbis	pin_mode,bit_mode	; Mode key pressed?
	ori	menu,0x80		;  Special menu's enabled

MenuStart:
	mov	ZL,menu			; Get menu number
	andi	ZL,0x0f			; Remove the flag(s)
	subi	ZL,-low(2*ptrHdr)	; Add pointer to header table
	lpm				; Get ptr text and bit7
	sbrc	r0,7
	IOClear	relay_inp		; Switch input relay On

	mov	ZL,r0
	andi	ZL,0x7f			; Remove bit7
	lsl	ZL			; To real cseg pointer

	rcall	lcd_clear		; LCD Clear
	rcall	lcd_line0		; Line 0, Char 0
	rcall	print_str

wait2:	sbis	pin_mode,bit_mode	; Wait release mode switch
	rjmp	wait2

MenuMain:
	mov	ZL,menu			; Menu * 2 and remove special flags
	andi	ZL,0x0f
	lsl	ZL
	subi	ZL,-low(2*ptrFnc)	; Pointer to menu function table
	rcall	MenuFunc		; Call the menu in Z

	ldi	ticks,80
wait:	sleep

	sbis	pin_calibr,bit_calibr	; Calibrate key
	rjmp	DoCalibrate		;  then calibrate

	sbis	pin_mode,bit_mode	; Mode key
	rjmp	MenuNext		;  then next menu

	tst	ticks
	brpl	wait

	rjmp	MenuMain

DoCalibrate:
	rcall	calibrate		; Do calibrate
wait0:	sbis	pin_calibr,bit_calibr	; Wait release calibrate switch
	rjmp	wait0
	rjmp	MenuMain

;-- Start tail code
MenuNext:
	mov	ZL,menu			; Get menu number
	andi	ZL,0x0f			; Remove the flag(s)
	subi	ZL,-low(2*ptrHdr)	; Add pointer to header table
	lpm				; Get ptr text and bit7
	sbrc	r0,7
	IOSet	relay_inp		; Switch input relay Off

;-- Increment the menu
;	sbrc	menu,7
;	jmp	highMenu
	andi	menu,0x0f		; TIJDELIJK

	inc	menu
	cpi	menu,mEnd
	brlt	mi
	clr	menu
mi:	rjmp	MenuStart

;highMenu:



MenuFunc:				; Jump via the return stack
	lpm				; to the func pointed by Z
	push	r0
	subi	ZL,-1			; ZL++
	lpm
	push	r0
ret6:	ret

;--------------------------------------------------------------------
;-- Calculate the unknow Cx
;--------------------------------------------------------------------
fCap:	rcall	calc1
	brcs	ret6			; OSC not running
	loadX	C			; RX = C
	rcall	calc2
	ldi	ZL,low(2*tpF)
	rjmp	print1

;--------------------------------------------------------------------
;-- Calculate the unknow Lx
;--------------------------------------------------------------------
fIndc:	rcall	calc1
	brcs	ret6			; OSC not running
	loadX	L			; RX = L
	rcall	calc2
	ldi	ZL,low(2*tnH)
	rjmp	print1

;--------------------------------------------------------------------
;-- Calculate Frequence C
;--------------------------------------------------------------------
fFrqC:	rcall	sample			; Get a freq sample
	brcs	ret6			; OSC not running

	loadY	KFRQ			; 15.2588*100*10	10e6/2^16 (10MHz)
	rcall	mul32			; RX *= RY

	rcall	ldYportval		; RY = portval
	rcall	div48			; RX = RX / RY

	loadY	K100
	rcall	div48			; RX = RX / RY

	ldi	ZL,low(2*tHz)
	rjmp	print1

;--------------------------------------------------------------------
;-- Calculate Frequence L
;--------------------------------------------------------------------
fFrqL:	rcall	sample			; Get a freq sample
	brcs	ret6			; OSC not running

	loadY	KFRQ			; 152,587890625 * 100 * 10	10e6/2^16 (10MHz)
	rcall	mul32			; RX *= RY

	rcall	ldYportval		; RY = portval
	rcall	div48			; RX = RX / RY

	loadY	K100
	rcall	div48			; RX = RX / RY

	ldi	ZL,low(2*tHz)
	rjmp	print1

;--------------------------------------------------------------------
;-- Display constants
;--------------------------------------------------------------------
fCnst:	loadX	L
	ldi	ZL,low(2*tnH)
	rcall	print1

	loadX	C
	ldi	ZL,low(2*tpF)
	rjmp	print0

;--------------------------------------------------------------------
;-- Calibrate the timer
;--------------------------------------------------------------------
calibrate:
	rcall	lcd_line1		; Line 1, Char 0
	ldi	ZL,low(2*tcali)		; Print header
	rcall	print_str

	ldi	ticks,16		; 100ms for stable OSC
	rcall	dlong

	cli				; Disable interrupt

	; Init the timer registers to default zero
	clr	tmp
	out	TCNT1H,tmp		; Clear the freq counter
	out	TCNT1L,tmp
	out	TCNT0,tmp		; Timer0 init

	clr	porttim			; Start counting the porttim
	clr	portval

	ldi	tmp,(1<<TOV0)|(1<<TOV1)	; Reset hanging timer interrupt
	out	TIFR,tmp

	clt				; T=0
	sei				; Enable interrupt
	rcall	sample			; Wait for the counter init

	rcall	sample			; Delay only

	rcall	sample			; Get N1
	brcs	calibrate		; OSC not running

	storeX	N1			; N1 = N1
	rcall	rxquad			; RX = N1^2
	storeX	NQ1			; NQ1 = N1^2

	IOClear	relay_refc		; Switch Cref On
	rcall	sample			; Delay only, stable osc

	rcall	calc1			; Get N2 and calc

	IOSet	relay_refc		; Switch Cref Off

	brcs	calibrate		; OSC not running?

	;--------------------------------------------------------------------
	;-- Calculate the unknow C
	;-- C[32] = Cref * N2^2 / (N1^2 - N2^2)
	;--------------------------------------------------------------------
debug1:
	loadX	Cref			; RX = Cref
	loadY	NQ2			; RY = N2^2
	rcall	mul32			; RX = RX * RY
	loadY	ND			; RY = N1^2 - N2^2
	rcall	div48			; RX = RX / RY
	storeX	C			; C = RX

	;--------------------------------------------------------------------
	;-- Calculate the unknow L
	;-- L[32] = ( K * portval / N1 )^2 / C
	;-- K[48] = 10^9 * 2^8 * prescaler / (2 * pi * Xtal) * 100 * sqrt(10)
	;-- *100 (10^2) is nodig omdat we de C & L een factor 10 te groot is! ( x,0nH )
	;--------------------------------------------------------------------

	loadX	KL			; RX = K
	rcall	ldYportval		; RY = portval
	rcall	mul32			; RX *= RY
	loadY	N1			; RY = N1
	rcall	div48			; RX = RX / RY
	rcall	rxquad			; RX = RX^2
	loadY	C			; RY = C
	rcall	div48			; RX = RX / RY
	storeX	L			; L = RX

.if 0
	loadX	N1
	ldi	chr,'N'
	rcall	rs232_putch
	ldi	chr,'1'
	rcall	rs232_putch
	ldi	chr,'='
	rcall	rs232_putch
	rcall	rs232_print

	loadX	N2
	ldi	chr,'N'
	rcall	rs232_putch
	ldi	chr,'2'
	rcall	rs232_putch
	ldi	chr,'='
	rcall	rs232_putch
	rcall	rs232_print

	mov	RX0,portval
	clr	RX1
	clr	RX2
	clr	RX3
	ldi	chr,'P'
	rcall	rs232_putch
	ldi	chr,'V'
	rcall	rs232_putch
	ldi	chr,'='
	rcall	rs232_putch
	rcall	rs232_print

	loadX	C
	ldi	chr,'C'
	rcall	rs232_putch
	ldi	chr,'='
	rcall	rs232_putch
	rcall	rs232_print

	loadX	L
	ldi	chr,'L'
	rcall	rs232_putch
	ldi	chr,'='
	rcall	rs232_putch
	rcall	rs232_print
	ret

rs232_print:
	rcall	cvt
	ldi	ZL,low(buf)
nxt1:	ld	chr,Z+
	tst	chr
	breq	endstr1
	rcall	rs232_putch
	rjmp	nxt1
endstr1:
	rcall	rs232_crlf
	ret

.else
	ret
.endif

;--------------------------------------------------------------------
;-- Sample
;--------------------------------------------------------------------

sam0:	sleep				; Wait for a interrupt (timer)
sample:	brtc	sam0			; Is there a sample?
	mov	RX0,freqL		; RX = freq count
	mov	RX1,freqH
	clt				; T=0, take next freq sample

	mov	r0,RX0			; Test zero value
	or	r0,RX1
	breq	err1			; if not zero there is a sample!
	clr	RX2
	clr	RX3
	storeX	N2			; N2 = sample

	clc				; No error
	ret

err1:	; Freq count is zero then error message
	rcall	lcd_line1		; Line 1, Char 0
	ldi	ZL,low(2*tNoOSC)
	rcall	print_str

	clt				; Next sample now
	sec				; Error
	ret


;--------------------------------------------------------------------
;-- Calculate the unknow Cx
;-- Cx[32] = C * (NQ1^2 - NQ2^2) / NQ2^2
;--
;-- Calculate the unknow Lx
;-- Lx[32] = L * (NQ1^2 - NQ2^2) / NQ2^2
;--------------------------------------------------------------------

calc1:	rcall	sample			; Get N2
	brcs	ret5			; OSC not running
debug2:
	rcall	rxquad			; RX = RX^2
nxt5:	storeX	NQ2			; NQ2 = N2^2

	loadY	NQ1			; RY = N1^2
	rcall	sub32			; RY = RY - RX
	brcs	negy			; C=1, Negative

	storeY	ND			; ND = N1^2 - N2^2
	clc				; No error

ret5:	ret

negy:
;	rcall	rs232_text
;	.db	"F1 Correction!",10,13,0,0
	loadY	N1
	loadX	N2
	rcall	sub32			; RY = N1 - N2

	loadX	KM1			; RX = -1
	rcall	sub32			; RY = RY + 1
	brmi	err3			; N1=1, meer dan 2 afwijking

	loadY	N1			; RY = N1
	rcall	sub32			; RY = RY + 1
	storeY	N1			; N1 = RY

	mov	RX0,RY0			; RX = N1^2
	mov	RX1,RY1
	mov	RX2,RY2
	mov	RX3,RY3
	rcall	mul32
	storeX	NQ1			; NQ1 = N1^2
	rjmp	nxt5

err3:	sec				; Error
	clr	RX0			; RX = 0
	clr	RX1
	clr	RX2
	clr	RX3
	ret


calc2:	loadY	ND			; RY = NQ1^2 - NQ2^2
	rcall	mul32			; RX = RX * RY
	loadY	NQ2			; RY = NQ2^2
	rcall	div48			; RX = RX / RY
	ret


;--------------------------------------------------------------------
;-- RX[64] = RX[32]^2
;--------------------------------------------------------------------
rxquad:	mov	RY0,RX0			; RX = RX^2
	mov	RY1,RX1
	mov	RY2,RX2
	mov	RY3,RX3

;--------------------------------------------------------------------
;-- RX[64] = RX[32] * RY[32]
;--------------------------------------------------------------------
mul32:	clr	RX7
	clr	RX6
	clr	RX5
	sub	RX4,RX4		; RX4=0, C=0
	ldi	tmp,32+1

mnxtb:	brcc	mnoadd
	add	RX4,RY0
	adc	RX5,RY1
	adc	RX6,RY2
	adc	RX7,RY3
mnoadd:
	ror	RX7
	ror	RX6
	ror	RX5
	ror	RX4

	ror	RX3
	ror	RX2
	ror	RX1
	ror	RX0

	dec	tmp
	brne	mnxtb
	ret


;--------------------------------------------------------------------
;-- RX[48] = RX[48] / RY[32]
;--------------------------------------------------------------------
div48:	ldi	tmp,48+1		; init loop counter

	clr	RR0			; clear remainder Low byte
	clr	RR1
	clr	RR2
	clr	RR3
	sub	RR4,RR4			; clear remainder High byte and carry

div1:	rol	RX0
	rol	RX1
	rol	RX2
	rol	RX3
	rol	RX4
	rol	RX5

	dec	tmp
	brne	div3
	ret

div3:	rol	RR0
	rol	RR1
	rol	RR2
	rol	RR3
	rol	RR4

	sub	RR0,RY0			;remainder = remainder - divisor
	sbc	RR1,RY1
	sbc	RR2,RY2
	sbc	RR3,RY3
	sbc	RR4,zero

	brcc	div2			;if result negative
	add	RR0,RY0			;   restore remainder
	adc	RR1,RY1
	adc	RR2,RY2
	adc	RR3,RY3
	adc	RR4,zero		; ZH is zero!
	clc				;   clear carry to be shifted into result
	rjmp	div1			;else
div2:	sec				;   set carry to be shifted into result
	rjmp	div1


;--------------------------------------------------------------------
;-- RY[32] = RY[32] - RX[32]
;--------------------------------------------------------------------
sub32:	sub	RY0,RX0
	sbc	RY1,RX1
	sbc	RY2,RX2
	sbc	RY3,RX3
	ret

;--------------------------------------------------------------------
;-- Store and load X & Y from SRAM
;--------------------------------------------------------------------
stX:	st	Z+,RX0
	st	Z+,RX1
	st	Z+,RX2
	st	Z+,RX3
	ret

stY:	st	Z+,RY0
	st	Z+,RY1
	st	Z+,RY2
	st	Z+,RY3
	ret

ldX:	ld	RX0,Z+
	ld	RX1,Z+
	ld	RX2,Z+
	ld	RX3,Z+
	ret

ldY:	ld	RY0,Z+
	ld	RY1,Z+
	ld	RY2,Z+
	ld	RY3,Z+
	ret

ldYportval:
	mov	RY0,portval		; RY = portval
	clr	RY1
	clr	RY2
	clr	RY3
	ret

;--------------------------------------------------------------------
;-- Convert 32bits RX to decimal string
;-- 12 char buffer (9 digits, 2 punt, null)
;;;-- 14 char buffer (10 digits, 3 punt, null)
;--------------------------------------------------------------------
cvt:	ldi	ZL,low(endbuf)
	ldi	chr,0
	st	-Z,chr
;	rcall	digit
;	ldi	chr,','
;	st	-Z,chr
	rcall	digit
	rcall	digit
	rcall	digit
	ldi	chr,'.'
	st	-Z,chr
	rcall	digit
	rcall	digit
	rcall	digit
	ldi	chr,'.'
	st	-Z,chr
	rcall	digit
	rcall	digit
	rcall	digit

nxt2:	ld	chr,Z
	cpi	chr,'0'
	breq	cvt1
	cpi	chr,'.'
	brne	ends
cvt1:	ldi	chr,' '
	st	Z+,chr
	rjmp	nxt2

ends:	cpi	chr,0
	brne	ret2
	ldi	chr,'0'
	st	-Z,chr
ret2:	ret


digit:	sub	chr,chr		; clear remainder and carry
	ldi	tmp,32+1	; init loop counter
digit1:	rol	RX0
	rol	RX1
	rol	RX2
	rol	RX3
	dec	tmp		; decrement counter
	breq	digit3		; if done then return
	rol	chr		; shift dividend into remainder
	subi	chr,10		; remainder = remainder - divisor
	brcc	digit2		;if result negative
	subi	chr,-10		;   restore remainder
	clc			;   clear carry to be shifted into result
	rjmp	digit1		;else
digit2:	sec			;   set carry to be shifted into result
	rjmp	digit1
digit3:	subi	chr,-'0'
	st	-Z,chr
	ret

;--------------------------------------------------------------------
;-- Print line 0 & 1
;--------------------------------------------------------------------

print1:	push	ZL			; Save for later print_str
	rcall	lcd_line1		; Line 1
p0:	rcall	cvt			; RX to string
	ldi	ZL,low(buf)
nxt:	ld	chr,Z+
	tst	chr
	breq	endstr
	rcall	putch
	rjmp	nxt
endstr:
	pop	ZL
print_str:
	lpm
	adiw	ZL,1
	tst	r0
	breq	ret3
	mov	chr,r0
	rcall	putch
	rjmp	print_str
ret3:	ret

print0:	push	ZL			; Save for later print_str
	rcall	lcd_line0		; Line 0
	rjmp	p0

;--------------------------------------------------------------------
;-- LCD Functions
;--------------------------------------------------------------------

lcd_init:
	cbi	lcd_port,lcd_rs

	ldi	chr,$30			;set DB4&DB5="H"
	rcall	wr_lcd_1		;1) reset sequence 1
	ldi	chr,200			;5mS
	rcall	dX25uS			;(4.1mS minimum)

	ldi	chr,$30			;set DB4&DB5="H"
	rcall	wr_lcd8			;2) reset sequence 2 (100uS minimum in LCD spec)

	ldi	chr,$30			;set DB4&DB5="H"
	rcall	wr_lcd8			;3) reset sequence 3

	ldi	chr,$20			;$20
	rcall	wr_lcd8			;4) set data mode = 4bit transfer

	ldi	chr,0b00101000		;$28
	rcall	wr_ins			;5) set charactor mode = 7 dots

	ldi	chr,0b00000110
	rcall	wr_ins			;6) set entry mode

	ldi	chr,0b00001100
	rcall	wr_ins			;7) Display ON, cursor OFF, blink OFF

lcd_clear:
	ldi	chr,$01			;clear display
	rcall	wr_ins
	ldi	chr,1750/25		;1,75mS (Spec 1,64mS)
	rcall	dX25uS
ret4:	ret

lcd_line0:
	ldi	chr,$80+0x00		; Line 0
	rjmp	wr_ins

lcd_line1:
	ldi	chr,$80+0x40		; Line 1
	rjmp	wr_ins

wr_lcd8:				; Write comment in 8 bits mode
	rcall	wr_lcd_1
	ldi	chr,200/25		; 200uS
	rjmp	dX25uS			; Delay 32uS

putch:	sbi	lcd_port,lcd_rs
	rcall	wr_lcd
	ldi	tmp,50/25		; 50uS
	rjmp	dX25uS			; Delay 32uS

wr_lcd:	push	chr
	rcall	wr_lcd_1
	pop	chr
	swap	chr
wr_lcd_1:
	andi	chr,0xF0
	in	tmp,lcd_port
	andi	tmp,0x0F
	or	chr,tmp
	out	lcd_port,chr
	nop
	sbi	lcd_port,lcd_en
	nop
	cbi	lcd_port,lcd_en
duS:					; Delay small 7 cylces, at 8Mhz clock
					;+3 cycles being "rcall"ed
	ret				;+4 cycles to return. 10 cycles total

wr_ins:	cbi	lcd_port,lcd_rs
	rcall	wr_lcd
	ldi	chr,4			; 100uS

dX25uS:	;Xm Sec delay, X=chr
	ldi	tmp,25			; 10MHz
;	ldi	tmp,11			; 4.433619Mhz
d25uS1:	rcall	duS			;+7
	dec	tmp			;+1
	brne	d25uS1			;+2
	dec	chr
	brne	dX25uS
	ret

; N = uS * xtal / (256 * 256) / 10e6 
; 6,5536ms @ 10Mhz
; 8,192ms @ 8Mhz
; 14.78160ms @ 4.433619Mhz
dlong:	; Delay long in steps of ticks * 6,5536ms
	sleep
	tst	ticks
	brne	dlong
	ret

;.include "rs232.inc"

	