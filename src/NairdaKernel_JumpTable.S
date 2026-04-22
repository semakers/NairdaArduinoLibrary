#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
// ============================================================================
// NairdaKernel_JumpTable.S
// Jump Table en sección fija .nairda_vectors (dirección 0x5F80)
// Archivo assembly puro (no pasa por LTO).
// ============================================================================

.section .nairda_vectors,"ax",@progbits
.global nairda_jump_table

nairda_jump_table:
    ; ── Outputs (slots 0-9) ──
    .word pm(jt_setupDigitalOut)    ; slot 0
    .word pm(jt_runDigitalOut)      ; slot 1
    .word pm(jt_setupServo)         ; slot 2
    .word pm(jt_runServo)           ; slot 3
    .word pm(jt_setupMotor)         ; slot 4
    .word pm(jt_runMotor)           ; slot 5
    .word pm(jt_setupNeoPixel)      ; slot 6
    .word pm(jt_runNeoPixel)        ; slot 7
    .word pm(jt_setupFrequency)     ; slot 8
    .word pm(jt_runFrequency)       ; slot 9

    ; ── Inputs (slots 10-15) ──
    .word pm(jt_setupDigitalIn)     ; slot 10
    .word pm(jt_readDigitalIn)      ; slot 11
    .word pm(jt_setupAnalogic)      ; slot 12
    .word pm(jt_readAnalogic)       ; slot 13
    .word pm(jt_setupUltrasonic)    ; slot 14
    .word pm(jt_readUltrasonic)     ; slot 15

    ; ── Utilidades (slots 16-18) ──
    .word pm(jt_nairdaDelay)        ; slot 16
    .word pm(jt_nairdaRandom)       ; slot 17
    .word pm(jt_nairdaMap)          ; slot 18

    ; ── Strings (slots 19-21) ──
    .word pm(jt_stringCompare)      ; slot 19
    .word pm(jt_stringToLower)      ; slot 20
    .word pm(jt_stringToUpper)      ; slot 21

    ; ── Tables (slots 22-30) ──
    .word pm(jt_tableCreate)        ; slot 22
    .word pm(jt_tableSet)           ; slot 23
    .word pm(jt_tableGet)           ; slot 24
    .word pm(jt_tableHeight)        ; slot 25
    .word pm(jt_tableWidth)         ; slot 26
    .word pm(jt_tableAddRow)        ; slot 27
    .word pm(jt_tableAddColumn)     ; slot 28
    .word pm(jt_tableRemoveRow)     ; slot 29
    .word pm(jt_tableRemoveColumn)  ; slot 30

#endif
