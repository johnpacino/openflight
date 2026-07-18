/*----------------------------------------------------------------------------*/
/* OpenFlight Stage-1c L3 burst-dump — MSS application linker command file.   */
/*                                                                            */
/* The platform file (ti/platform/xwr68xx/r4f_linker.cmd, added on the make   */
/* link line) defines the MEMORY regions (VECTORS/PROG_RAM/DATA_RAM/L3_RAM/   */
/* HS_RAM) and places the standard sections (.text/.const/.bss/.data/.stack). */
/* Here we only add the two app-specific sections:                            */
/*   systemHeap  - the SYS/BIOS system heap created in mss.cfg                 */
/*   .l3ring     - the raw-ADC rolling buffer (g_ring in l3_dump.c). It is the */
/*                 dominant consumer of L3; if it overflows L3_RAM the link    */
/*                 fails with the exact byte overage -> lower RING_FRAMES or   */
/*                 raise MMWAVE_L3RAM_NUM_BANK on the make line.               */
/*----------------------------------------------------------------------------*/
--retain="*(.intvecs)"

SECTIONS
{
    systemHeap : {} > DATA_RAM
    .l3ring    : {} > L3_RAM
}
/*----------------------------------------------------------------------------*/
