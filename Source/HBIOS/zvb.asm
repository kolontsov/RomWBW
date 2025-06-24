;======================================================================
;	VIDEO DRIVER FOR ZEAL 8-BIT VIDEO BOARD
;       https://zeal8bit.com/getting-started-zvb/
;
;       WRITTEN BY: VADIM KOLONTSOV -- 23/6/2025
;       LARGELY BASED ON CODE WRITTEN BY: ZEAL 8-BIT COMPUTER
;======================================================================

TERMENABLE      .SET    TRUE            ; INCLUDE TERMINAL PSEUDODEVICE DRIVER
KBDENABLE       .SET    TRUE            ; INCLUDE KBD KEYBOARD SUPPORT

; ---------------------------------------------------------------------
; ZVB CONSTANTS
; ---------------------------------------------------------------------
VID_MODE_TEXT_640       .EQU 0          ; TEXT MODE 640x480
VID_640480_X_MAX        .EQU 80
VID_640480_Y_MAX        .EQU 40

ZVB_DEFAULT_MODE        .EQU VID_MODE_TEXT_640
ZVB_ROWS                .EQU VID_640480_Y_MAX
ZVB_COLS                .EQU VID_640480_X_MAX

DEFAULT_CHAR_COLOR      .EQU $07        ; WHITE ON BLACK
DEFAULT_CHAR_COLOR_INV  .EQU $70        ; BLACK ON WHITE
DEFAULT_CURSOR_BLINK    .EQU 30         ; EVERY 30 FRAMES

VID_MEM_PHYS_ADDR_START .EQU $100000
VID_MEM_PALETTE_OFFSET  .EQU $0E00
VID_IO_MAPPER           .EQU $80
IO_MAPPER_BANK          .EQU VID_IO_MAPPER+$0E
VID_IO_CTRL_STAT        .EQU $90
IO_CTRL_VID_MODE        .EQU VID_IO_CTRL_STAT+$0C
IO_CTRL_STATUS_REG      .EQU VID_IO_CTRL_STAT+$0D

BANK_IO_TEXT_NUM        .EQU $00                    ; TEXT CONTROL MODULES
VID_IO_BANKED_ADDR      .EQU $A0                    ; BANKED I/O MODULES WILL BE AT $A0
IO_TEXT_PRINT_CHAR      .EQU VID_IO_BANKED_ADDR+$00 ; PRINT CHARACTEE
IO_TEXT_CURS_Y          .EQU VID_IO_BANKED_ADDR+$01 ; CURSOR Y
IO_TEXT_CURS_X          .EQU VID_IO_BANKED_ADDR+$02 ; CURSOR X
IO_TEXT_SCROLL_Y        .EQU VID_IO_BANKED_ADDR+$03 ; SCROLL Y
IO_TEXT_SCROLL_X        .EQU VID_IO_BANKED_ADDR+$04 ; SCROLL X
IO_TEXT_COLOR           .EQU VID_IO_BANKED_ADDR+$05 ; CURRENT CHAR COLOR
IO_TEXT_CURS_TIME       .EQU VID_IO_BANKED_ADDR+$06 ; CURSOR: BLINK TIME (FRAMES)
IO_TEXT_CURS_CHAR       .EQU VID_IO_BANKED_ADDR+$07 ; CURSOR: CHARACTER
IO_TEXT_CURS_COLOR      .EQU VID_IO_BANKED_ADDR+$08 ; CURSOR: COLOR
IO_TEXT_CTRL_REG        .EQU VID_IO_BANKED_ADDR+$09 ; CONTROL REGISTER

; CONTROL REGISTER BITS
IO_TEXT_SAVE_CURSOR_BIT    .EQU 7   ; SAVE THE CURRENT CURSOR POSITION (SINGLE SAVE ONLY)
IO_TEXT_RESTORE_CURSOR_BIT .EQU 6   ; RESTORE THE PREVIOUSLY SAVED POSITION
IO_TEXT_AUTO_SCROLL_X_BIT  .EQU 5
IO_TEXT_AUTO_SCROLL_Y_BIT  .EQU 4
IO_TEXT_WAIT_ON_WRAP_BIT   .EQU 3   ; WHEN THE CURSOR IS ABOUT TO WRAP TO THE NEXT LINE 
    ; (MAXIMUM AMOUNT OF CHARACTERS SENT TO THE SCREEN), THIS FLAG CAN WAIT FOR THE NEXT
    ; CHARACTER TO COME BEFORE RESETTING THE CURSOR X POSITION TO 0 AND POTENTIALLY SCROLL 
    ; THE WHOLE SCREEN. USEFUL TO IMPLEMENT AN EAT-NEWLINE FIX.
IO_TEXT_SCROLL_Y_OCCURRED  .EQU 0   ; ON READ, TELLS IF THE PREVIOUS PRINT_CHAR (OR NEWLINE) 
    ; TRIGGERED A SCROLL IN Y ON WRITE, MAKES THE CURSOR GO TO THE NEXT LINE
IO_TEXT_CURSOR_NEXTLINE    .EQU 0

; MAP TEXT CONTROLLER TO VID_IO_BANKED_ADDR PORT SPACE
#DEFINE ZVB_TEXT_CTRL \
#defcont \      XOR     A
#defcont \      OUT     (IO_MAPPER_BANK),A

; MAP VIDEO RAM TO SPECIFIED MMU PAGE
#DEFINE ZVB_ENTER(_PAGE) \
#defcont \      LD      A,_PAGE<<6 & $FF
#defcont \      IN      A,(_PAGE)
#defcont \      LD      (MMU_PAGE_PREV),A
#defcont \      LD      A,VID_MEM_PHYS_ADDR_START>>14
#defcont \      OUT     (_PAGE),A

; RESTORE ORIGINAL PAGE
#DEFINE ZVB_LEAVE(_PAGE)
#defcont \      LD      A,(MMU_PAGE_PREV)
#defcont \      OUT     (_PAGE),A

ZVB_INIT:
        CALL    ZVB_VDAINI      ; INITIALIZE

        LD      IY,ZVB_IDAT     ; POINTER TO INSTANCE DATA
        CALL    KBD_INIT        ; INIT PS2 DECODER
        CALL    Z8B_PIO_INIT    ; INIT PS2 INTERRUPTS WITH PIO

        ; ADD OURSELVES TO VDA DISPATCH TABLE
        LD      BC,ZVB_FNTBL    ; BC := FUNCTION TABLE ADDRESS
        LD      DE,ZVB_IDAT     ; DE := FPGA VGA INSTANCE DATA PTR
        CALL    VDA_ADDENT      ; ADD ENTRY, A := UNIT ASSIGNED

        ; INITIALIZE EMULATION
        LD      C,A             ; C := ASSIGNED VIDEO DEVICE NUM
        LD      DE,ZVB_FNTBL    ; DE := FUNCTION TABLE ADDRESS
        LD      HL,ZVB_IDAT     ; HL := FPGA VGA INSTANCE DATA PTR
        CALL    TERM_ATTACH     ; DO IT

        XOR     A               ; SIGNAL SUCCESS
        RET

ZVB_FNTBL:
        .DW     ZVB_VDAINI      ; INITIALIZE VDU
        .DW     ZVB_VDAQRY      ; QUERY VDU STATUS
        .DW     ZVB_VDARES      ; SOFT RESET VDU
        .DW     ZVB_VDADEV      ; DEVICE INFO
        .DW     ZVB_VDASCS      ; SET CURSOR STYLE
        .DW     ZVB_VDASCP      ; SET CURSOR POSITION
        .DW     ZVB_VDASAT      ; SET CHARACTE ATTRIBUTE
        .DW     ZVB_VDASCO      ; SET CHARACTER COLOR
        .DW     ZVB_VDAWRC      ; WRITE CHARACTER
        .DW     ZVB_VDAFIL      ; FILL
        .DW     ZVB_VDACPY      ; COPY
        .DW     ZVB_VDASCR      ; SCROLL
        .DW     KBD_STAT        ; GET KEYBOARD STATUS
        .DW     KBD_FLUSH       ; FLUSH KEYBOARD BUFFER
        .DW     KBD_READ        ; READ KEYBOARD
        .DW     ZVB_VDARDC      ; READ CHARACTER
#IF (($ - ZVB_FNTBL) != (VDA_FNCNT * 2))
        .ECHO   "*** INVALID ZVB FUNCTION TABLE ***\n"
        !!!!!
#ENDIF

; ==== INITIALIZE VDU ====
ZVB_VDAINI:
        ZVB_ENTER(Z8B_MMU_PAGE1)
        ZVB_TEXT_CTRL()

        ; PALETTE
        LD      DE,$4000+VID_MEM_PALETTE_OFFSET
        LD      HL,ZVB_PALETTE
        LD      BC,ZVB_PALETTE_END-ZVB_PALETTE
        LDIR

        ; VIDEO MODE
        LD      A,ZVB_DEFAULT_MODE
        OUT     (IO_CTRL_VID_MODE),A    ; SET TEXT MODE 80x40

        ; CURSOR
        XOR     A
        OUT     (IO_TEXT_CURS_CHAR),A   ; CURSOR: CHAR
        LD      A,DEFAULT_CHAR_COLOR_INV
        OUT     (IO_TEXT_CURS_COLOR),A  ; CURSOR: COLOR
        LD      A,DEFAULT_CURSOR_BLINK
        OUT     (IO_TEXT_CURS_TIME),A   ; CURSOR: BLINK TIME

        ; CHAR COLOR
        LD      A,DEFAULT_CHAR_COLOR
        OUT     (IO_TEXT_COLOR),A       ; CHARACTER COLOR

        ; ENABLE THE SCREEN
        LD      A,$80
        OUT     (IO_CTRL_STATUS_REG),A
        ; DISABLE AUTOSCROLL
        LD      A,1<<IO_TEXT_WAIT_ON_WRAP_BIT
        OUT     (IO_TEXT_CTRL_REG),A

        ; CLEAR SCREEN
        LD      HL, $4000               ; VRAM TEXT
        LD      BC, ZVB_ROWS*ZVB_COLS
        LD      E, 0
        CALL    ZVB_VRAM_SET

        LD      E, DEFAULT_CHAR_COLOR
        LD      HL, $4000+$1000         ; VRAM ATTRIBUTES
        LD      BC, ZVB_ROWS*ZVB_COLS
        CALL    ZVB_VRAM_SET

        ; CLEAR POS
        XOR     A
        OUT     (IO_TEXT_CURS_Y),A
        OUT     (IO_TEXT_CURS_X),A
        OUT     (IO_TEXT_SCROLL_Y),A
        OUT     (IO_TEXT_SCROLL_X),A

        ZVB_LEAVE(Z8B_MMU_PAGE1)

        XOR     A                       ; SIGNAL SUCCESS
        RET

; ==== QUERY VDU STATUS ====
ZVB_VDAQRY:
        LD      C,0                     ; CURRENT MODE ALWAYS 0 (NO MODES)
        LD      D,ZVB_ROWS              ; ROWS
        LD      E,ZVB_COLS              ; COLS
        LD      HL,0                    ; FONT DATA EXTRACTION NOT SUPPORTED NOEW
                                        ; (BUT CAN BE IMPLEMENTED)
        XOR     A                       ; SIGNAL SUCCESS
        RET

; ==== SOFT RESET VDU ====
ZVB_VDARES:
        ; PERFORMS A NON-DESTRUCTIVE RESET OF THE SPECIFIED VIDEO UNIT (C).
        ; SHOULD RE-INITIALIZE THE VIDEO HARDWARE WITHOUT DESTROYING THE SCREEN
        ; CONTENTS OR CURSOR POSITION. THE CURRENT VIDEO MODE WILL NOT BE CHANGED.
        ;
        ; NOTHING TO DO
        XOR     A                       ; SIGNAL SUCCESS
        RET

; ==== DEVICE INFO ====
ZVB_VDADEV:
        LD      C,0                     ; DEVICE ATTRS (NOT DEFINED)
        LD      D,VDADEV_ZVB            ; DEVICE TYPE
        LD      E,0                     ; DEVICE NUMBER (ALWAYS 0)
        LD      H,0                     ; DEVICE MODE (ALWAYS 0)
        LD      L,0                     ; BASE I/O ADDR (NOT USED FOR NOW)
        XOR     A                       ; SIGNAL SUCCESS
        RET

; ==== SET CURSOR STYLE (D) ====
ZVB_VDASCS:
        ; ADJUST THE FORMAT OF THE CURSOR SUCH THAT THE CURSOR STARTS AT THE PIXEL SPECIFIED
        ; IN THE TOP NIBBLE OF START/END (D) AND ENDS AT THE PIXEL SPECIFIED IN THE BOTTOM NIBBLE
        ; OF START/END (D). SO, IF D=0X08, A BLOCK CURSOR WOULD BE USED THAT STARTS AT THE TOP
        ; PIXEL OF THE CHARACTER CELL AND ENDS AT THE NINTH PIXEL OF THE CHARACTER CELL.
        ; ---
        ; CAN BE IMPLEMENTED USING CUSTOM CHAR FOR CURSOR (CURRENTLY USING CHAR CODE 0)
        SYSCHKERR(ERR_NOTIMPL)          ; NOT IMPLEMENTED YET
        RET

; ==== SET CURSOR POSITION (E, D) =====
ZVB_VDASCP:
        ZVB_TEXT_CTRL()
        LD      A,E
        OUT     (IO_TEXT_CURS_X),A
        LD      A,D
        OUT     (IO_TEXT_CURS_Y),A
        XOR     A               ; SIGNAL SUCCESS
        RET

; ==== SET CHARACTER ATTRIBUTE (E) - UNDERLINE, REVERSE, BLINK =====
ZVB_VDASAT:
        SYSCHKERR(ERR_NOTIMPL)          ; NOT IMPLEMENTED: ZVB DOESN'T SUPPORT READING FROM RAM
        RET

; ==== SET CHARACTER COLOR (E) FOR SCOPE (D) =====
ZVB_VDASCO:
        ZVB_TEXT_CTRL()
        LD      A,E
        OUT     (IO_TEXT_COLOR),A       ; SET COLOR
        LD      A,D
        OR      A                       ; SHOULD WE FILL WHOLE SCREEN?
        JR      Z,_ZVB_VDASCO_BYE       ; NO, IF D==0

        ZVB_ENTER(Z8B_MMU_PAGE1)        ; MAP VRAM
        LD      HL, $4000+$1000         ; VRAM ATTRIBUTES
        LD      BC, ZVB_ROWS*ZVB_COLS
        CALL    ZVB_VRAM_SET
        ZVB_LEAVE(Z8B_MMU_PAGE1)        ; MAP VRAM BACK

_ZVB_VDASCO_BYE:
        XOR     A                       ; SIGNAL SUCCESS
        RET

; ==== WRITE CHARACTER (E) =====
ZVB_VDAWRC:
        ZVB_TEXT_CTRL()
        LD      A,E
        OUT     (IO_TEXT_PRINT_CHAR),A
        XOR     A                       ; SIGNAL SUCCESS
        RET

; ==== FILL (HL) CHARS WITH (E) =====
ZVB_VDAFIL:
        LD      A,H                     ; CHECK IF COUNT IS ZERO
        OR      L
        RET     Z                       ; IF SO, NOTHING TO DO

        ZVB_TEXT_CTRL()                 ; SELECT TEXT I/O BANK

        ; GET CURRENT CURSOR POSITION
        IN      A,(IO_TEXT_CURS_Y)
        LD      B,A                     ; B = CURSOR Y
        IN      A,(IO_TEXT_CURS_X)
        LD      C,A                     ; C = CURSOR X

        PUSH    DE

        ; CALCULATE REMAINING CHARACTERS ON SCREEN
        ; DE = (ZVB_ROWS - Y) * ZVB_COLS - X
        LD      A,ZVB_ROWS
        SUB     B                       ; A = ZVB_ROWS - Y
        LD      H,A
        LD      E,ZVB_COLS
        CALL    MULT8                   ; H*E -> HL
        LD      D,H
        LD      E,L                     ; DE = (ZVB_ROWS - Y) * ZVB_COLS

        LD      A,C                     ; A = CURSOR X
        SUB     E                       ; E = E - A
        LD      A,D                     ; A = D
        SBC     A,0                     ; A = A - 0 - CARRY
        LD      D,A                     ; DE = DE - X

        ; USE THE SMALLER OF THE REQUESTED COUNT (HL) AND REMAINING SPACE (DE)
        LD      A,D
        SUB     H
        JR      NC,ZVB_VDAFIL_USE_HL    ; IF DE >= HL, USE HL
        EX      DE,HL                   ; USE DE
ZVB_VDAFIL_USE_HL:
        LD      B,H
        LD      C,L                     ; HL NOW CONTAINS THE FINAL COUNT

        POP     DE                      ; RESTORE FILL CHARACTER IN E
ZVB_VDAFIL_LOOP:
        LD      A,E
        OUT     (IO_TEXT_PRINT_CHAR),A
        DEC     BC
        LD      A,B
        OR      C
        JR      NZ,ZVB_VDAFIL_LOOP

        XOR     A                       ; SIGNAL SUCCESS
        RET
        
; ==== COPY (L) CHARS FROM (D,E) TO CUR POSITION =====
ZVB_VDACPY:
        SYSCHKERR(ERR_NOTIMPL)          ; NOT IMPLEMENTED: ZVB DOESN'T SUPPORT READING FROM RAM
        RET

; ==== SCROLL (E) LINES =====
ZVB_VDASCR:
        LD      A,E                     ; GET LINE COUNT
        OR      A                       ; CHECK IF ZERO
        RET     Z                       ; IF SO, NOTHING TO DO

        ZVB_TEXT_CTRL()                 ; SELECT TEXT I/O BANK

        ; SAVE CURSOR POSITION
        ; NB: IT SEEMS NATIVE EMULATOR DOESN'T HANDLE IO_TEXT_RESTORE_CURSOR_BIT
        ; WELL AFTER SCROLLING, SO USE NAIVE APPROACH FOR NOW
        IN      A,(IO_TEXT_CURS_X)
        LD      B,A
        IN      A,(IO_TEXT_CURS_Y)
        LD      C,A
        PUSH    BC

        LD      A,E                     ; GET LINE COUNT AGAIN
        OR      A                       ; TEST SIGN
        JP      M,ZVB_VDASCR_REV        ; JUMP IF NEGATIVE (REVERSE SCROLL)

; FORWARD SCROLL
ZVB_VDASCR_FWD:
        IN      A,(IO_TEXT_SCROLL_Y)    ; GET CURRENT SCROLL Y
        ADD     A,E                     ; ADD LINES TO SCROLL
        CP      ZVB_ROWS
        JR      C,ZVB_VDASCR_FWD1
        SUB     ZVB_ROWS
ZVB_VDASCR_FWD1:
        OUT     (IO_TEXT_SCROLL_Y),A    ; SET NEW SCROLL Y

        LD      A,ZVB_ROWS              ; CALCULATE STARTING Y FOR CLEARING
        SUB     E
        JR      ZVB_VDASCR_CLEAR_LOOP

; REVERSE SCROLL
ZVB_VDASCR_REV:
        NEG                             ; E = -E (MAKE IT POSITIVE)
        LD      A,E                     ; A = NUMBER OF LINES
        LD      L,A                     ; L = NUMBER OF LINES TO CLEAR

        IN      A,(IO_TEXT_SCROLL_Y)    ; GET CURRENT SCROLL Y
        SUB     E                       ; SUBTRACT LINES TO SCROLL
        JP      NC,ZVB_VDASCR_REV1
        ADD     A,ZVB_ROWS
ZVB_VDASCR_REV1:
        OUT     (IO_TEXT_SCROLL_Y),A    ; SET NEW SCROLL Y

        XOR     A                       ;  START CLEARING AT Y=0

; CLEARING LOOP
ZVB_VDASCR_CLEAR_LOOP:
        OUT     (IO_TEXT_CURS_Y),A
        XOR     A
        OUT     (IO_TEXT_CURS_X),A      ; GO TO X=0

        LD      A,' '                   ; CHARACTER TO FILL WITH
ZVB_VDASCR_LOOP_LINES:
        LD      B, ZVB_COLS             ; PREPARE TO CLEAR ZVB_COLS CHARS
ZVB_VDASCR_LOOP:
        OUT     (IO_TEXT_PRINT_CHAR),A
        DJNZ    ZVB_VDASCR_LOOP         ; REPEAT UNTIL B>0
        DEC     E
        JP      NZ,ZVB_VDASCR_LOOP_LINES

        ; RESTORE CURSOR POSITION
        POP     BC
        LD      A,B
        OUT     (IO_TEXT_CURS_X),A
        LD      A,C
        OUT     (IO_TEXT_CURS_Y),A

        XOR     A                       ; SIGNAL SUCCESS
        RET

; ==== READ CHARACTER =====
ZVB_VDARDC:
        SYSCHKERR(ERR_NOTIMPL)          ; NOT IMPLEMENTED: ZVB DOESN'T SUPPORT READING FROM RAM
        RET
        
;----------------------------------------------------------------------
; FILL VRAM WITH VALUE
; HL=DEST ADDR, BC=LENGTH, E=VALUE
;----------------------------------------------------------------------
ZVB_VRAM_SET:
        LD      A,E
        LD      (HL),A
        INC     HL
        DEC     BC
        LD      A,B
        OR      C
        JP      NZ,ZVB_VRAM_SET
        RET

;----------------------------------------------------------------------
; CONVERT XY TO VRAM OFFSET IN HL
; D=ROW (Y), E=COL (X)
;----------------------------------------------------------------------
ZVB_VRAM_OFFSET:
        LD      A,E                     ; SAVE COLUMN NUMBER IN A
        LD      H,D                     ; SET H TO ROW NUMBER
        LD      E,ZVB_COLS              ; SET E TO ROW LENGTH
        CALL    MULT8                   ; MULTIPLY TO GET ROW OFFSET, H * E = HL, E=0, B=0
        LD      E,A                     ; GET COLUMN BACK
        ADD     HL,DE                   ; ADD IT IN
        LD      BC,$4000
        ADD     HL,BC
        RET

; PIO Ports
IO_PIO_DATA_A   .EQU $D0
IO_PIO_DATA_B   .EQU $D1
IO_PIO_CTRL_A   .EQU $D2
IO_PIO_CTRL_B   .EQU $D3

; PIO Modes
IO_PIO_OUTPUT   .EQU $0F
IO_PIO_INPUT    .EQU $4F
IO_PIO_BIDIR    .EQU $8F
IO_PIO_BITCTRL  .EQU $CF

; PIO Interrupt control word (BITCTRL mode ONLY)
IO_PIO_CTRLW_INT        .EQU 7

; INTERRUPT CONTROL
IO_PIO_DISABLE_INT      .EQU $03
IO_PIO_ENABLE_INT       .EQU $83

; 2ND PORT IS USED AS ZEAL 8-BIT SYSTEM PORT
IO_PIO_SYSTEM_DATA      .EQU IO_PIO_DATA_B
IO_PIO_SYSTEM_CTRL      .EQU IO_PIO_CTRL_B
IO_PIO_SYSTEM_VAL       .EQU $FF

; PIN DEFINITIONS FOR SYSTEM PORT
IO_I2C_SDA_OUT_PIN      .EQU 0
IO_I2C_SCL_OUT_PIN      .EQU 1
IO_I2C_SDA_IN_PIN       .EQU 2
IO_UART_RX_PIN          .EQU 3
IO_UART_TX_PIN          .EQU 4
IO_HBLANK_PIN           .EQU 5
IO_VBLANK_PIN           .EQU 6
IO_KEYBOARD_PIN         .EQU 7

; PORT DIRECTION (INPUT PINS AS 1, OUTPUT PINS AS 0)
IO_PIO_SYSTEM_DIR       .EQU (1 << IO_KEYBOARD_PIN) | (1 << IO_VBLANK_PIN) | (1 << IO_HBLANK_PIN) | (1 << IO_UART_RX_PIN) | (1 << IO_I2C_SDA_IN_PIN)

; PIO SYSTEM PORT INTERRUPT CONTROL WORD
; BIT 7: 1 = INTERRUPT FUNCTION ENABLE
; BIT 6: 1 = AND FUNCTION (0 = OR)
; BIT 5: 1 = ACTIVE HIGH
; BIT 4: 1 = MASK FOLLOWS
IO_PIO_SYSTEM_INT_CTRL  .EQU $90 | IO_PIO_CTRLW_INT

; Only KEYBOARD pins is monitored, H_BLANK/V_BLANK ARE IGNORED
; NOTE: 0 means monitored!
IO_PIO_SYSTEM_INT_MASK  .EQU ~(1 << IO_KEYBOARD_PIN) & $FF

Z8B_PIO_INIT:
        LD      A,IO_PIO_DISABLE_INT            ; DISABLE INTERRUPTS FOR SYSTEM PORT
        OUT     (IO_PIO_SYSTEM_CTRL),A
        LD      A,IO_PIO_BITCTRL                ; SET SYSTEM PORT AS BIT-CONTROL
        OUT     (IO_PIO_SYSTEM_CTRL),A
        LD      A,IO_PIO_SYSTEM_DIR             ; SET THE PROPER DIRECTION FOR EACH PIN
        OUT     (IO_PIO_SYSTEM_CTRL),A
        LD      A,IO_PIO_SYSTEM_VAL             ; SET DEFAULT VALUE FOR ALL THE (OUTPUT) PINS
        OUT     (IO_PIO_SYSTEM_DATA),A
        LD      A,INT_PIO0B<<1                  ; SET INTERRUPT VECTOR
        OUT     (IO_PIO_SYSTEM_CTRL),A

#IF (INTMODE != 2 | !KBDINTS)
        .ECHO   "*** INVALID CONFIG: ZVB REQUIRES IM2 AND KBDINT ENABLED ***\n"
        !!!
#ENDIF
        LD      HL,Z8B_KBD_ISR                   ; MY INTERRUPT HANDLER
        LD      (IVT(INT_PIO0B)), HL

        LD      A,IO_PIO_ENABLE_INT             ; ENABLE THE INTERRUPTS FOR THE SYSTEM PORT
        OUT     (IO_PIO_SYSTEM_CTRL),A
        LD      A,IO_PIO_SYSTEM_INT_CTRL        ; ENABLE INTERRUPTS FOR THE REQUIRED PINS ONLY
        OUT     (IO_PIO_SYSTEM_CTRL),A
        LD      A,IO_PIO_SYSTEM_INT_MASK        ; MASK MUST FOLLOW
        OUT     (IO_PIO_SYSTEM_CTRL),A
        XOR     A
        RET

KB_IO_ADDRESS   .EQU $E8                        ; PS/2 KEYBOARD ADDRESS

; THIS INTERRUPT HANDLER PUSHES INTO KBD.ASM QUEUE
Z8B_KBD_ISR:
        IN      A,(KB_IO_ADDRESS)               ; READ KB
        OR      A                               ; IS ANY DATA PRESENT?
        RET     Z                               ; RETURN IF NOT
        LD      B,A

        CALL    KBDQLEN                         ; CHECK IF
        SUB     KBDQSIZ                         ; QUEUE IS FULL
        RET     Z                               ; EXIT IF THE QUEUE IS FULL

        LD      HL,KBDQPUTX                     ; CREATE QUEUE INDEX
        CALL    KBD_Q_IDX                       ; POINTER IN HL

        LD      (HL),A                          ; SAVE THE CHARACTER IN THE QUEUE
        RET

; ---------------------------------------------------------------------
; ZVB CONSTANTS
; ---------------------------------------------------------------------
MMU_PAGE_PREV .DB 0

ZVB_PALETTE:
        .DW $0000
        .DW $0015
        .DW $1540
        .DW $0555
        .DW $A800
        .DW $A815
        .DW $AAA0
        .DW $AD55
        .DW $52AA
        .DW $52BF
        .DW $57EA
        .DW $57FF
        .DW $FAAA
        .DW $FABF
        .DW $FFEA
        .DW $FFFF
ZVB_PALETTE_END:

; Instance data 
ZVB_IDAT:
        .DB     KBDMODE_ZVB      ; ZEAL 8-BIT KEYBOARD CONTROLLER
        .DB     0                ; UNUSED
        .DB     0                ; UNUSED
