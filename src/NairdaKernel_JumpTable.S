// ============================================================================
// NairdaKernel_JumpTable.S
// DEPRECATED: La Jump Table ahora se escribe dinámicamente via BootJacker.
//
// Flutter envía el comando 'J' al kernel, que llena bj_page_buf con los
// punteros de las funciones del kernel. Luego Flutter escribe la página
// a 0x5F80 via los comandos 'E' (erase) y 'W' (write).
//
// Este archivo se mantiene vacío para no romper el build system.
// ============================================================================
