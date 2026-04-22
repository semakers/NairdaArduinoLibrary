// ============================================================================
// bootjacker_loader.ino
// Mini-kernel con jump table + bootjacker loader via Serial (bare-metal UART).
//
// Protocol (script-driven, delay-based):
//   'F'          → Fill page buffer with 0xFF
//   'L' off len [data]  → Load chunk at offset
//   'E' lo hi    → Erase page at addr. WDT resets.
//   'W' lo hi    → Write page_buf to addr. WDT resets.
//   'R'          → Run user program at 0x6002
//   '?'          → Print status
// ============================================================================

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <string.h>

// ═══════════════════════════════════════════════════════════════════
// Bare-metal UART (no Arduino Serial — saves ~170 bytes RAM)
// ═══════════════════════════════════════════════════════════════════
static void uart_init(void) {
    uint16_t ubrr = (F_CPU / 4 / 115200 - 1) / 2;  // U2X mode
    UBRR0H = ubrr >> 8;
    UBRR0L = ubrr;
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8N1
}

static void uart_tx(uint8_t c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_print(const char *s) {
    while (*s) uart_tx(*s++);
}

static void uart_print_P(const char *s) {
    char c;
    while ((c = pgm_read_byte(s++))) uart_tx(c);
}

static void uart_hex(uint8_t b) {
    const char hex[] = "0123456789ABCDEF";
    uart_tx(hex[b >> 4]);
    uart_tx(hex[b & 0x0F]);
}

static void uart_println(const char *s) { uart_print(s); uart_tx('\r'); uart_tx('\n'); }
static void uart_println_P(const char *s) { uart_print_P(s); uart_tx('\r'); uart_tx('\n'); }

static void uart_flush(void) {
    while (!(UCSR0A & (1 << TXC0)));
    UCSR0A |= (1 << TXC0);
}

static uint8_t uart_rx(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

static uint8_t uart_available(void) {
    return UCSR0A & (1 << RXC0);
}

// ═══════════════════════════════════════════════════════════════════
// Memory map
// ═══════════════════════════════════════════════════════════════════
#define JT_ADDR                0x5F80
#define USER_SPACE_ADDR        0x6000
#define USER_PROGRAM_WORD_ADDR 0x3001   // 0x6002 / 2
#define USER_FLAG_VALID        0x01

// ═══════════════════════════════════════════════════════════════════
// Bootjacker gadgets (Optiboot 4.4)
// ═══════════════════════════════════════════════════════════════════
#define GADGET_ERASE_W       0x3F99
#define GADGET_FILL_WRITE_W  0x3F80
#define OPTIBOOT_BUF         0x0100

// ═══════════════════════════════════════════════════════════════════
// Page buffer in .noinit
// ═══════════════════════════════════════════════════════════════════
static volatile uint8_t page_buf[SPM_PAGESIZE] __attribute__((section(".noinit")));

// ═══════════════════════════════════════════════════════════════════
// Syscalls
// ═══════════════════════════════════════════════════════════════════
static void pin13_setup(void) {
    DDRB |= (1 << 5);
}
static void led_on(void) {
    PORTB |= (1 << 5);
}
static void led_off(void) {
    PORTB &= ~(1 << 5);
}
static void delay_ms(unsigned long ms) {
    // Simple busy-wait delay (~16 cycles per iteration at 16MHz)
    while (ms--) {
        for (volatile uint16_t i = 0; i < 1995; i++);
    }
}

// ═══════════════════════════════════════════════════════════════════
// Bootjacker core
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
// Jump table — filled via 'J' command, written via E/W by script
// ═══════════════════════════════════════════════════════════════════
static void fill_jt_page_buf(void) {
    for (uint8_t i = 0; i < SPM_PAGESIZE; i++) page_buf[i] = 0xFF;
    uint16_t ptrs[4] = {
        (uint16_t)(void *)pin13_setup,
        (uint16_t)(void *)led_on,
        (uint16_t)(void *)led_off,
        (uint16_t)(void *)delay_ms,
    };
    for (uint8_t i = 0; i < 4; i++) {
        page_buf[i * 2]     = ptrs[i] & 0xFF;
        page_buf[i * 2 + 1] = ptrs[i] >> 8;
    }
}

// ═══════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════
int main(void) {
    uart_init();
    sei();

    uart_println_P(PSTR("BJ:READY"));

    while (1) {
        if (!uart_available()) continue;
        uint8_t cmd = uart_rx();

        switch (cmd) {
            case 'F':
                memset((void *)page_buf, 0xFF, SPM_PAGESIZE);
                uart_println_P(PSTR("F:OK"));
                break;

            case 'L': {
                uint8_t off = uart_rx();
                uint8_t len = uart_rx();
                for (uint8_t i = 0; i < len && (off + i) < SPM_PAGESIZE; i++)
                    page_buf[off + i] = uart_rx();
                uart_println_P(PSTR("L:OK"));
                break;
            }

            case 'E': {
                uint8_t lo = uart_rx();
                uint8_t hi = uart_rx();
                uint16_t addr = lo | ((uint16_t)hi << 8);
                uart_print_P(PSTR("E:0x")); uart_hex(hi); uart_hex(lo);
                uart_tx('\r'); uart_tx('\n'); uart_flush();
                bj_erase(addr);
                break;
            }

            case 'W': {
                uint8_t lo = uart_rx();
                uint8_t hi = uart_rx();
                uint16_t addr = lo | ((uint16_t)hi << 8);
                uart_print_P(PSTR("W:0x")); uart_hex(hi); uart_hex(lo);
                uart_tx('\r'); uart_tx('\n'); uart_flush();
                bj_fill_write(addr);
                break;
            }

            case 'J':
                // Fill page_buf with jump table data
                fill_jt_page_buf();
                uart_println_P(PSTR("J:OK"));
                break;

            case 'R': {
                uint8_t flag = pgm_read_byte(USER_SPACE_ADDR);
                if (flag == USER_FLAG_VALID) {
                    uart_println_P(PSTR("R:GO"));
                    uart_flush();
                    ((void (*)(void))USER_PROGRAM_WORD_ADDR)();
                } else {
                    uart_println_P(PSTR("R:NOPROG"));
                }
                break;
            }

            case '?': {
                uart_print_P(PSTR("JT=0x"));
                uint16_t j = pgm_read_word(JT_ADDR);
                uart_hex(j >> 8); uart_hex(j & 0xFF);
                uart_tx('\r'); uart_tx('\n');
                uart_print_P(PSTR("FL=0x"));
                uart_hex(pgm_read_byte(USER_SPACE_ADDR));
                uart_tx('\r'); uart_tx('\n');
                for (uint8_t i = 0; i < 8; i++) {
                    uart_hex(pgm_read_byte(USER_SPACE_ADDR + i));
                    uart_tx(' ');
                }
                uart_tx('\r'); uart_tx('\n');
                break;
            }
        }
    }
}
