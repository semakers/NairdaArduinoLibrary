// ============================================================================
// bootjacker_loader.ino
// Mini-kernel con jump table + bootjacker loader via Serial.
//
// Protocol (driven entirely by the Python/Flutter script):
//   'C' n        → Configure: expect n pages
//   'L' [64B]    → Load chunk into page buffer (2 chunks = 1 page)
//   'E' lo hi    → Erase page at addr (lo,hi). WDT resets.
//   'W' lo hi    → Write page_buf to addr. WDT resets.
//   'R'          → Run user program at 0x6002
//   '?'          → Print status (flag, JT)
//
// Flow per page:
//   Script sends chunks via 'L' → sends 'E' addr → waits 300ms (WDT)
//   → sends 'W' addr → waits 300ms (WDT) → repeat for next page
// ============================================================================

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <string.h>

// ═══════════════════════════════════════════════════════════════════
// Memory map
// ═══════════════════════════════════════════════════════════════════
#define JT_ADDR                0x5F80
#define USER_SPACE_ADDR        0x6000
#define USER_PROGRAM_WORD_ADDR 0x3001
#define USER_FLAG_VALID        0x01

// ═══════════════════════════════════════════════════════════════════
// Bootjacker gadgets (Optiboot 4.4)
// ═══════════════════════════════════════════════════════════════════
#define GADGET_ERASE_W       0x3F99
#define GADGET_FILL_WRITE_W  0x3F80
#define OPTIBOOT_BUF         0x0100

// ═══════════════════════════════════════════════════════════════════
// Page buffer in .noinit (survives WDT reset)
// ═══════════════════════════════════════════════════════════════════
static volatile uint8_t page_buf[SPM_PAGESIZE] __attribute__((section(".noinit")));

// ═══════════════════════════════════════════════════════════════════
// Syscalls (exposed via jump table at 0x5F80)
// ═══════════════════════════════════════════════════════════════════
void pin13_setup(void) { pinMode(13, OUTPUT); }
void led_on(void)      { digitalWrite(13, HIGH); }
void led_off(void)     { digitalWrite(13, LOW); }
void delay_ms(unsigned long ms) { delay(ms); }

// ═══════════════════════════════════════════════════════════════════
// Bootjacker core — both noreturn, trigger WDT reset
// ═══════════════════════════════════════════════════════════════════
static void bj_erase(uint16_t addr) __attribute__((noreturn));
static void bj_erase(uint16_t addr) {
    cli();
    wdt_enable(WDTO_15MS);
    asm volatile(
        "mov r12, %A[a] \n\t" "mov r13, %B[a] \n\t"
        "ldi r30, 0x03  \n\t" "mov r10, r30   \n\t"
        "ldi r30, 0x11  \n\t" "mov r9,  r30   \n\t"
        "ldi r30, lo8(%[g]) \n\t" "ldi r31, hi8(%[g]) \n\t"
        "ijmp \n\t"
        : : [a] "r" (addr), [g] "n" (GADGET_ERASE_W)
    );
    __builtin_unreachable();
}

static void bj_fill_write(uint16_t addr) __attribute__((noreturn));
static void bj_fill_write(uint16_t addr) {
    cli();
    for (uint8_t i = 0; i < SPM_PAGESIZE; i++)
        ((volatile uint8_t *)OPTIBOOT_BUF)[i] = page_buf[i];
    wdt_enable(WDTO_15MS);
    asm volatile(
        "mov r12, %A[a] \n\t" "mov r13, %B[a] \n\t"
        "ldi r30, 0x01  \n\t" "mov r8,  r30   \n\t"
        "ldi r30, 0x05  \n\t" "mov r10, r30   \n\t"
        "ldi r30, 0x11  \n\t" "mov r9,  r30   \n\t"
        "ldi r30, lo8(%[g]) \n\t" "ldi r31, hi8(%[g]) \n\t"
        "ijmp \n\t"
        : : [a] "r" (addr), [g] "n" (GADGET_FILL_WRITE_W)
    );
    __builtin_unreachable();
}

// ═══════════════════════════════════════════════════════════════════
// Write jump table to 0x5F80 (first boot only)
// ═══════════════════════════════════════════════════════════════════
static void ensure_jump_table() {
    uint16_t first = pgm_read_word(JT_ADDR);
    // Check if already has valid-looking pointer (not erased Flash)
    if (first != 0xFFFF && first != 0x0000) return;

    memset((void *)page_buf, 0xFF, SPM_PAGESIZE);
    uint16_t *entries = (uint16_t *)((void*)page_buf);
    entries[0] = (uint16_t)(void *)pin13_setup;
    entries[1] = (uint16_t)(void *)led_on;
    entries[2] = (uint16_t)(void *)led_off;
    entries[3] = (uint16_t)(void *)delay_ms;

    // Erase JT page — after WDT, we'll end up in setup() again
    // with JT page erased. But page_buf still has JT data (.noinit).
    // The script isn't running yet, so we just need to get back to
    // a state where we can write. We'll use a simple flag.
    bj_erase(JT_ADDR);
    // Never returns — WDT resets. Next boot: JT page is 0xFF.
    // We detect this and write it.
}

// ═══════════════════════════════════════════════════════════════════
// Serial helpers
// ═══════════════════════════════════════════════════════════════════
static uint8_t serial_read_byte() {
    while (!Serial.available());
    return Serial.read();
}

// ═══════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    // On boot after JT erase: write JT data from page_buf (.noinit)
    uint16_t jt_first = pgm_read_word(JT_ADDR);
    if (jt_first == 0xFFFF) {
        // JT page was just erased — write it now
        // page_buf still has JT data from ensure_jump_table
        bj_fill_write(JT_ADDR);  // WDT resets
        // Next boot: JT is written, we fall through to READY
    }

    // First-ever boot: write JT
    ensure_jump_table();

    Serial.println(F("BJ:READY"));
}

void loop() {
    if (!Serial.available()) return;
    uint8_t cmd = Serial.read();

    switch (cmd) {
        case 'L': {
            // Load chunk into page buffer
            // Format: 'L' [offset_lo] [len] [data...]
            uint8_t offset = serial_read_byte();
            uint8_t len = serial_read_byte();
            if (len > 64) break;
            for (uint8_t i = 0; i < len; i++) {
                uint8_t b = serial_read_byte();
                if (offset + i < SPM_PAGESIZE)
                    page_buf[offset + i] = b;
            }
            Serial.println(F("L:OK"));
            break;
        }

        case 'F': {
            // Fill page buffer (reset to 0xFF)
            memset((void *)page_buf, 0xFF, SPM_PAGESIZE);
            Serial.println(F("F:OK"));
            break;
        }

        case 'E': {
            // Erase page at address (2 bytes: lo, hi)
            uint8_t lo = serial_read_byte();
            uint8_t hi = serial_read_byte();
            uint16_t addr = lo | ((uint16_t)hi << 8);
            Serial.print(F("E:0x")); Serial.println(addr, HEX);
            Serial.flush();
            bj_erase(addr);  // WDT resets
            break;
        }

        case 'W': {
            // Write page_buf to address (2 bytes: lo, hi)
            uint8_t lo = serial_read_byte();
            uint8_t hi = serial_read_byte();
            uint16_t addr = lo | ((uint16_t)hi << 8);
            Serial.print(F("W:0x")); Serial.println(addr, HEX);
            Serial.flush();
            bj_fill_write(addr);  // WDT resets
            break;
        }

        case 'R': {
            uint8_t flag = pgm_read_byte(USER_SPACE_ADDR);
            if (flag == USER_FLAG_VALID) {
                Serial.println(F("R:GO"));
                Serial.flush();
                ((void (*)(void))USER_PROGRAM_WORD_ADDR)();
            } else {
                Serial.println(F("R:NOPROG"));
            }
            break;
        }

        case '?': {
            Serial.print(F("JT=0x")); Serial.println(pgm_read_word(JT_ADDR), HEX);
            Serial.print(F("FL=0x")); Serial.println(pgm_read_byte(USER_SPACE_ADDR), HEX);
            // Dump first 8 bytes at 0x6000
            for (uint8_t i = 0; i < 8; i++) {
                uint8_t b = pgm_read_byte(USER_SPACE_ADDR + i);
                if (b < 0x10) Serial.print('0');
                Serial.print(b, HEX); Serial.print(' ');
            }
            Serial.println();
            break;
        }
    }
}
