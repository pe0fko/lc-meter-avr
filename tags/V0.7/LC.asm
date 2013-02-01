;************************************************************************
;**
;** Project......: L & C Meter, http://home.ict.nl/~fredkrom/
;**
;** Platform.....: AT90S2313 or ATtiny2313
;**
;** Licence......: This software is freely available for non-commercial 
;**                use - i.e. for research and experimentation only!
;**
;** Programmer...: F.W. Krom, PE0FKO
;** 
;** Description..: Meet de L & C waarde, zelf calibrerend door bekende C.
;**
;** History......: 11/04/2005 V0.1 First complete working version.
;**                12/04/2005 V0.2 Instellen reference C
;**                15/04/2005 V0.3 Format 5 digits
;**                22/12/2006 V0.4 Bugfix prescale & LCD
;**                02/05/2007 V0.5 Ref-C in EEProm or ROM, Compile switch
;**                17/02/2008 V0.6 If eeprom 0xFF the write _Cref
;**                12/01/2012 V0.7 _Cref in eeprom on deferent possition.
;**
;**************************************************************************/
.nolist
; There is no code deference between the AT90S2313 and ATtiny2313!
;.include "2313def.inc"		; AT90S2313
.include "tn2313def.inc"	; ATtiny2313
.list

;.equ	xtal		= 8000000	; 8MHz
.equ	xtal		= 10000000	; 10MHz

; Bekende referentie C
.equ	_Cref		= 940	; 940pF
.equ	_UseEEProm	= 1	; Store/Load CRef from EEProm
.equ	_IdxEEProm	= 10	; Index in eeprom start.

.equ	prescale	= 64	; timer0 prescaler count 64
;.equ	prescale	= 256	; timer0 prescaler count 256

; N = uS * xtal / (prescale * 256) / 10e6 
.equ	ticks_40ms	= 40*xtal/prescale/256000
.equ	ticks_400ms	= 400*xtal/prescale/256000

; Maximale tijd die we meten voor pulsen.
; Bij 10MHz en prescaler van 64 is dat minimaal 610 Hz (1,6384ms)
; Bij 10MHz en prescaler van 256 is dat minimaal 152 Hz (6.554ms)
;.equ	portmax	= 0x7f		; 128 * 64 / 10MHz = 1.22 kHz

;            AT90S2313
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
; PD0	X	Not in use
; PD1	X	Not in use
; PD2	O	Relay C/L switch
; PD3	O	Relay ref C
; PD4	X	Not in use
; PD5	I	Frequence input
; PD6	X	Not in use
; PB0	I	Switch Calibrate
; PB1	I	Switch Mode
; PB2	O	LCD - RS
; PB3	O	LCD - EN
; PB4	O	LCD - D4
; PB5	O	LCD - D5
; PB6	O	LCD - D6
; PB7	O	LCD - D7

.macro	IO	; <name> <port> <pin>
.set	prt_@0		= PORT@1
.set	pin_@0		= PIN@1		; Input use PIN
.set	ddr_@0		= DDR@1
.set	bit_@0		= P@1@2
.endmacro

IO	freq,D,5		; T1 Freq input 16 bit counter
IO	relay_refc,D,3		; Relay reference C
IO	relay_inp,D,2		; Relay L or C
IO	mode,B,1		; Switch L or C
IO	calibr,B,0		; Switch calibrate
IO	lcd,B,4			; LCD Data B4..B7
IO	rs,B,2			; LCD RS B2
IO	en,B,3			; LCD EN B3

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
	ldi	ZL,byte1(@0)
	rcall	stX
.endmacro

.macro	storeY	; <memory>
	ldi	ZL,byte1(@0)
	rcall	stY
.endmacro

.macro	loadX	; <memory>
	ldi	ZL,byte1(@0)
	rcall	ldX
.endmacro

.macro	loadY	; <memory>
	ldi	ZL,byte1(@0)
	rcall	ldY
.endmacro

.macro	loadXi	; <value>
	ldi	RX0,byte1(@0)
	ldi	RX1,byte2(@0)
	ldi	RX2,byte3(@0)
	ldi	RX3,byte4(@0)
.endmacro


;***** Registers
.undef	XL	; r26
.undef	XH	; r27
.undef	YL	; r28
.undef	YH	; r29

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

.def	RX0	= r24;r16
.def	RX1	= r25;r17
.def	RX2	= r18
.def	RX3	= r19
.def	RX4	= r20
.def	RX5	= r21
.def	RX6	= r22
.def	RX7	= r14;r23

.def	ticks	= r16;r24	; decrement every timer1 interrupt (8,192ms)

.def	menu	= r17;r25
.def	chr	= r26		; output char
.def	tmp	= r27		; Scratchregister

.def	status	= r23

.undef	ZH
.def	zero	= r31		; ZH

.equ	sMenuAll	= 1
.equ	sRecal		= 2

;--------------------------------------------------------------------
;-- SRAM variablen
;--------------------------------------------------------------------
.dseg
.equ	buflen	= 11			; Number buffer length
.equ	varlen	= 4

Cref:	.byte	varlen			; C Reference
C:	.byte	varlen			; OSC Capaciteit
L:	.byte	varlen			; OSC Inductie
N1:	.byte	varlen			; Count freq F1
N2:	.byte	varlen			; Count freq F2
NQ1:	.byte	varlen			; Count freq F1^2
NQ2:	.byte	varlen			; Count freq F2^2
ND:	.byte	varlen			; Count freq F1^2 - F2^2
BUFFER:	.byte	buflen			; Buffer number string
KM1:	.byte	varlen			; -1
K100:	.byte	varlen			; 100
KL:	.byte	varlen			; Constant L calculation
KFRQ:	.byte	varlen			; Constant frequence counter

.if _UseEEProm
;--------------------------------------------------------------------
;-- Reference C value at EEPROM
;--------------------------------------------------------------------
.eseg
	.org	_IdxEEProm
	.dw	_Cref
.endif
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
tCap:	.db	"   Capacitance  ",0,0
tIndc:	.db	"   Inductance   ",0,0
tFreqC:	.db	"   Frequency C  ",0,0
tFreqL:	.db	"   Frequency L  ",0,0
tCali:	.db	"--Calibrating-- ",0,0
tNoOSC:	.db	"--No oscilator--",0,0
tCref:	.db	"   Reference C  ",0,0

tCf:	.db	"m",0xE4,0xE4,0xE4,"nnn","ppp"
tLf:	.db	" ","mmm",0xE4,0xE4,0xE4,"nnn"
tFf:	.db	"G","MMM","KKK   "

tCx:	.db	"Cx = ",0
tLx:	.db	"Lx = ",0
tC:	.db	" C = ",0
tL:	.db	" L = ",0
tF:	.db	" F = ",0

tCe:	.db	"F  ",0
tLe:	.db	"H  ",0
tFe:	.db	"Hz ",0

; Menu values
ptrHdr:	.db	tCap,tIndc+0x80,tFreqC,tFreqL+0x80,tHead,tHead+0x80
ptrFnc:	.dw	fCap,fIndc,fFrq,fFrq,fCnst,fCnst

.equ	mEndAll	= 6		; End for all the menu's
.equ	mEnd	= 2		; End for normal menu's

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
	brne	err0			;  portval is zero then.

	neg	porttim			; portval = -porttim
;	andi	porttim,portmax
	mov	portval,porttim

	ldi	porttim,1		; Next timer count

	rjmp	ret1

err0:	sbr	status,1<<sRecal	; Recalibartion needed

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
	out	ddrb,tmp
	cbi	prt_rs,bit_rs		; LCD RS=0
	cbi	prt_en,bit_en		; LCD EN=0

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
	; TOIE0: Timer/Counter0 Overflow Interrupt Enable
	ldi	tmp,(1<<TOIE1)|(0<<OCIE1A)|(1<<TOIE0)
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

;	Init LCD interface
	ldi	ticks,ticks_40ms	; Min 20ms LCD
	rcall	dlong			;  Delay
	rcall	lcd_init

	; Store the constants in SRAM
	loadXi	-1			; KM1 = -1
	storeX	KM1

	; KFRQ = Xtal / (prescaler * 256) * 100
	loadXi	xtal*100/(256*prescale)
	storeX	KFRQ

	; K = 10^9 * 2^7 * prescale / (pi * Xtal) * sqrt(1000)
	loadXi	1288427829875*prescale/xtal
;	loadXi	8245938		; prescale=64 @10MHz
	storeX	KL

	loadXi	100		; K100 = 100
	storeX	K100

	; Get the reference C from EEPROM or ROM
.if _UseEEProm
	rcall	eepLd
;-- V0.6 ---------------
;	In case the eeprom is not initialized
	cpi		RX0,0xff ; if eeprom[0..1] == 0xffff
	brne	eep_ok		; then
	cpi		RX1,0xff
	brne	eep_ok

	loadXi	_CRef		;   Store Cref in eeprom
	rcall	eepSt
eep_ok:
;-- ---- ---------------
.else
	loadXi	_CRef
.endif
	storeX	Cref

;--------------------------------------------------------------------
;-- Start of the menu selection
;--------------------------------------------------------------------

	clr	zero			; ZH Always zero for store functions
	clr	menu			; Start first menu
	clr	status

	sbis	pin_mode,bit_mode	; Mode key pressed?
	sbr	status,1<<sMenuAll	;  Special menu's enabled

	sbis	pin_calibr,bit_calibr	; Calibrate key pressed?
	rcall	changeRefC		;  Change reference C

	rcall	calibrate		; Do calibrate

MenuStart:
	ldi	ZL,2*ptrHdr		; Get pointer to header table
	add	ZL,menu			; Add menu number
	lpm				; Get ptr text and bit7
	sbrc	r0,7
	IOClear	relay_inp		; Switch input relay On

	mov	ZL,r0
	andi	ZL,0x7f			; Remove bit7
	lsl	ZL			; To real cseg pointer

	rcall	lcd_clear		; LCD Clear
	rcall	lcd_line0		; Line 0, Char 0
	rcall	print_str

	 sbis	pin_mode,bit_mode	; Wait release mode switch
	rjmp	PC-1

MenuMain:
	mov	ZL,menu			; Menu * 2 and remove special flags
	lsl	ZL
	subi	ZL,-(2*ptrFnc)		; Pointer to menu function table
	rcall	MenuFunc		; Call the menu in Z

	ldi	ticks,ticks_400ms
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
	 sbis	pin_calibr,bit_calibr	; Wait release calibrate switch
	rjmp	PC-1
	rjmp	MenuStart

;-- Start tail code
MenuNext:
	ldi	ZL,2*ptrHdr		; Get pointer to header table
	add	ZL,menu			; Add menu number
	lpm				; Get ptr text and bit7
	sbrc	r0,7
	IOSet	relay_inp		; Switch input relay Off

;-- Increment the menu
	ldi	tmp,mEnd
	sbrc	status,sMenuAll
	ldi	tmp,mEndAll

	inc	menu
	cp	menu,tmp
	brne	PC+2
	 clr	menu
	rjmp	MenuStart

MenuFunc:				; Jump to *Z via the return stack
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

	rcall	lcd_line1		; Line 1
	ldi	ZL,2*tCx
	rcall	print_str
	ldi	ZL,2*tCf
	ldi	chr,2*tCe
	rjmp	print_nr

;--------------------------------------------------------------------
;-- Calculate the unknow Lx
;--------------------------------------------------------------------
fIndc:	rcall	calc1
	brcs	ret6			; OSC not running
	loadX	L			; RX = L
	rcall	calc2

	rcall	lcd_line1		; Line 1
	ldi	ZL,2*tLx
	rcall	print_str
	ldi	ZL,2*tLf
	ldi	chr,2*tLe
	rjmp	print_nr

;--------------------------------------------------------------------
;-- Calculate Frequence
;-- F = N * KFRQ / Portval / 100
;-- KFRQ = Xtal / (prescaler * 256) * 100
;--------------------------------------------------------------------
fFrq:	rcall	sample			; Get a freq sample
	brcs	ret6			; OSC not running
	loadY	KFRQ			; RY = KFRQ
	rcall	mul32			; RX *= RY
	rcall	ldYportval		; RY = portval
	rcall	div48			; RX /= RY
	loadY	K100			; RY = 100
	rcall	div48			; RX /= RY

	rcall	lcd_line1		; Line 1
	ldi	ZL,2*tF
	rcall	print_str
	ldi	ZL,2*tFf
	ldi	chr,2*tFe
	rjmp	print_nr

;--------------------------------------------------------------------
;-- Display constants
;--------------------------------------------------------------------
fCnst:	loadX	L
	rcall	lcd_line0		; Line 0
	ldi	ZL,2*tL
	rcall	print_str
	ldi	ZL,2*tLf
	ldi	chr,2*tLe
	rcall	print_nr

	loadX	C
	rcall	lcd_line1		; Line 1
	ldi	ZL,2*tC
	rcall	print_str
	ldi	ZL,2*tCf
	ldi	chr,2*tCe
	rjmp	print_nr


;--------------------------------------------------------------------
;-- Change reference C in eeprom
;-- Init: boot with mode + cali key pressed
;-- mode key is -1 pF, cali key is +1pF
;-- Reboot for exit!
;--------------------------------------------------------------------
changeRefC:
	sbrs	status,sMenuAll		; Beide knoppen ingedrukt!
	ret

	rcall	lcd_line0		; Print de header
	ldi	ZL,2*tCref
	rcall	print_str

crcprt:	loadX	Cref			; Print Cref 
	rcall	lcd_line1
	ldi	ZL,2*tC
	rcall	print_str
	ldi	ZL,2*tCf
	ldi	chr,2*tCe
	rcall	print_nr

	; Wacht key release
	 sbis	pin_calibr,bit_calibr	; Calibrate key
	rjmp	PC-1
	 sbis	pin_mode,bit_mode	; Mode key
	rjmp	PC-3

	ldi	tmp,0			; Debounce key's
	 dec	tmp
	brne	PC-1

	loadX	Cref
crc1:	sbis	pin_calibr,bit_calibr	; Calibrate key
	rjmp	crcinc
	sbis	pin_mode,bit_mode	; Mode key
	rjmp	crcdec
	rjmp	crc1

.if _UseEEProm
crcinc:	adiw	RX0,1			; RX0:RX1 += 1
	rjmp	crc2

crcdec:	sbiw	RX0,1			; RX0:RX1 -= 1
crc2:	storeX	Cref			; Cref = RX
	rcall	eepSt
.else
crcinc:
crcdec:
.endif
	rjmp	crcprt

;--------------------------------------------------------------------
;-- Calibrate the timer
;--------------------------------------------------------------------
calibrate:
	rcall	lcd_line0		; Line 0, Char 0
	ldi	ZL,2*tHead		; "LC Meter, PE0FKO"
	rcall	print_str

	rcall	lcd_line1		; Line 1, Char 0
	ldi	ZL,2*tCali		; Print header
	rcall	print_str

	ldi	ticks,ticks_400ms	; 400ms for stable OSC
	rcall	dlong
	cli				; Disable interrupt

	; Init the timer registers to default zero
	out	TCNT1H,zero		; Clear the freq counter
	out	TCNT1L,zero
	out	TCNT0,zero		; Timer0 init

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

	cbr	status,1<<sRecal	; Recalibartion done

	;------------------------------------------------------------
	;-- Calculate the unknow C
	;-- C[32] = Cref * N2^2 / (N1^2 - N2^2)
	;------------------------------------------------------------
	loadX	Cref			; RX = Cref
	loadY	NQ2			; RY = N2^2
	rcall	mul32			; RX = RX * RY
	loadY	ND			; RY = N1^2 - N2^2
	rcall	div48			; RX = RX / RY
	storeX	C			; C = RX

	;------------------------------------------------------------
	;-- Calculate the unknow L
	;-- L[32] = ( K * portval / N1 )^2 / C
	;-- K[32] = 10^9 * 2^7 * prescaler / (pi * Xtal) * sqrt(1000)
	;------------------------------------------------------------
	loadX	KL			; RX = K
	rcall	ldYportval		; RY = portval
	rcall	mul32			; RX *= RY
	loadY	N1			; RY = N1
	rcall	div48			; RX = RX / RY
	rcall	rxquad			; RX = RX^2
	loadY	C			; RY = C
	rcall	div48			; RX = RX / RY
	storeX	L			; L = RX

	ret

;--------------------------------------------------------------------
;-- Sample
;--------------------------------------------------------------------
	sleep				; Wait for a interrupt (timer)
sample:	brtc	PC-1			; Is there a sample?
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
	ldi	ZL,2*tNoOSC
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

negy:	loadY	N1
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
mnoadd:	ror	RX7
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

.if _UseEEProm
;--------------------------------------------------------------------
;-- Eeprom Store / Load reference C
;--------------------------------------------------------------------
eepSt:
	ldi	tmp,_IdxEEProm
	out	EEAR,tmp
	out	EEDR,RX0
	rcall	eep0
	inc	tmp
	out	EEAR,tmp
	out	EEDR,RX1
eep0:
	sbi	EECR,EEMWE
	sbi	EECR,EEWE
	 sbic	EECR,EEWE
	rjmp	PC-1
	ret

eepLd:	; RX = EEPROM[16]
	ldi	tmp,_IdxEEProm
	out	EEAR,tmp
	sbi	EECR,EERE
	in	RX0,EEDR
	inc	tmp
	out	EEAR,tmp
	sbi	EECR,EERE
	in	RX1,EEDR
	clr	RX2
	clr	RX3
	ret
.endif

;--------------------------------------------------------------------
;-- Convert 32bits RX to decimal string
;-- 10 char buffer + null
;--------------------------------------------------------------------
cvt:	ldi	ZL,BUFFER+buflen
	ldi	chr,0
	st	-Z,chr		; NULL  0 EOS

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
	cpi	ZL,BUFFER
	brne	digit

	ret

;--------------------------------------------------------------------
;-- Print RX and text on line 0 or 1
;-- ZL = rom text
;-- chr type ('F','H')
;--------------------------------------------------------------------
print_nr:
	push	chr
	push	ZL

	rcall	cvt

	ldi	ZL,BUFFER
nxt2:	ld	chr,Z+			; Skip leading '0'
	cpi	chr,'0'
	breq	nxt2

	cpi	chr,0			; EOS?
	brne	PC+2
	 dec	ZL			; Reset pointer to last char+1

	push	ZL			; Save string start+1

	subi	ZL,-(5-1)		; New end of string
	cpi	ZL,BUFFER+buflen	; Check for array size
	brmi	PC+2
	 ldi	ZL,BUFFER+buflen-1	; Litmit to array size

	clr	chr			; Set end of string
	st	Z,chr
	subi	ZL,5			; ZL is begin of string

l8:	ld	chr,Z+			; Output space for leading '0'
	cpi	chr,'0'
	brne	l11
	ldi	chr,' '
	rcall	putch
	rjmp	l8

l11:	cpi	ZL,BUFFER+buflen-3	; < 1000 extra space for missing comma
	brmi	l12
	 push	chr
	 cpi	chr,0
	 breq	PC+3
	  ldi	chr,' '
	 rjmp	PC+2
	  ldi	chr,'0'
	 rcall	putch
	 pop	chr

l12:	cpi	chr,0			; Output digits left comma
	breq	ends
	rcall	putch
	ld	chr,Z+
	cpi	ZL,BUFFER+buflen-(3*3)
	breq	l14
	cpi	ZL,BUFFER+buflen-(2*3)
	breq	l14
	cpi	ZL,BUFFER+buflen-(1*3)
	brne	l12

l14:	dec	ZL			; Output comma
	ldi	chr,','

L15:	rcall	putch			; Output digits rigth comma
	ld	chr,Z+
	cpi	chr,0
	brne	L15

ends:	ldi	chr,' '
	rcall	putch

	pop	ZL			; 
	subi	ZL,BUFFER+1
	pop	tmp
	add	ZL,tmp

	lpm
	mov	chr,r0
	rcall	putch

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

; N = uS * xtal / (prescale * 256) / 10e6 
dlong:	; Delay long in steps of ticks
	sleep
	tst	ticks
	brne	dlong
	ret

;--------------------------------------------------------------------
;-- LCD Functions
;--------------------------------------------------------------------

lcd_init:
	cbi	prt_rs,bit_rs

	ldi	chr,$30			;
	rcall	wr_lcd8			;1) Function Set (8-Bit Interface)

	ldi	chr,200			;5mS Delay
	rcall	dX25uS

	ldi	chr,$30			;
	rcall	wr_lcd8			;2) Function Set (8-Bit Interface)

	ldi	chr,$30			;
	rcall	wr_lcd8			;3) Function Set (8-Bit Interface)

	ldi	chr,$20			;
	rcall	wr_lcd8			;4) Function Set (4-Bit Interface)

	; 4 Bit mode active!

	ldi	chr,0b00101000		;$28 [0 0 1 DL.N F x x]
	rcall	wr_ins			;5) Interface data length 4 bits (DL), 2 line (N), Font 5*8 (F)

	ldi	chr,0b00001000		;$08 [0 0 0 0 1 D C B]
	rcall	wr_ins			;6) Display, Cursor, Blinking OFF

	ldi	chr,0b00000001		;$01 [0 0 0 0.0 0 0 1]
	rcall	wr_ins			;6) Display clear

	ldi	chr,2000/25		;2mS delay
	rcall	dX25uS

	ldi	chr,0b00000110		;$06 [0 0 0 0 0 1 I/D SH]
	rcall	wr_ins			;6) Entry mode

	ldi	chr,0b00001100		;$0C [0 0 0 0.1 D C B]
	rcall	wr_ins			;7) Display ON, cursor OFF, blink OFF

	ret

lcd_clear:
	ldi	chr,$01			;clear display
	rcall	wr_ins
	ldi	chr,2000/25		;2mS (Spec 1,64mS)
	rcall	dX25uS
	ret

lcd_line0:
	ldi	chr,$80+0x00		; Line 0
	rjmp	wr_ins

lcd_line1:
	ldi	chr,$80+0x40		; Line 1
	rjmp	wr_ins

; space()
; register: chr, tmp
space:	ldi	chr,' '

; putch( chr=Char )
; register: chr, tmp
putch:	sbi	prt_rs,bit_rs
	rcall	wr_lcd
	ldi	chr,50/25		; 50uS
	rjmp	dX25uS			; Delay 32uS

wr_lcd8:rcall	wr_lcd_1		; Write code in 8 bits mode
	ldi	chr,200/25		; 200uS
	rjmp	dX25uS			; Delay 32uS

; wr_lcd( chr=Code )
; local: tmp
wr_lcd:	push	chr
	rcall	wr_lcd_1
	pop	chr
	swap	chr
wr_lcd_1:
	push	tmp
	sbi	prt_en,bit_en		; Enable strobe up
	andi	chr,0xF0
	in	tmp,prt_lcd
	andi	tmp,0x0F
	or	chr,tmp
	out	prt_lcd,chr
	cbi	prt_en,bit_en		; Enable strobe down
	pop	tmp
	ret

wr_ins:	cbi	prt_rs,bit_rs
	rcall	wr_lcd
	ldi	chr,100/25		; 100uS

;Xm Sec delay, X=chr
dX25uS:	push	tmp
	ldi	tmp,25*xtal/10000000	; 25us (10 cycles)
	rcall	duS			;+7
	dec	tmp			;+1
	brne	pc-2			;+2
	dec	chr
	brne	pc-5
	pop	tmp
duS:					; Delay small 7 cylces, at 8Mhz clock
					;+3 cycles being "rcall"ed
					;+4 cycles to return. 10 cycles total
	ret

; EOF
