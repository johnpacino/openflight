/*----------------------------------------------------------------------------*/
/* OpenFlight Stage-1c L3 burst-dump — MSS application linker command file.   */
/*                                                                            */
/* The platform file (ti/platform/xwr68xx/r4f_linker.cmd, added on the make   */
/* link line) defines the MEMORY regions (VECTORS/PROG_RAM/DATA_RAM/L3_RAM/   */
/* HS_RAM) and places the standard sections (.text/.const/.bss/.data/.stack). */
/* Here we only add the app-specific sections:                                */
/*   systemHeap  - the SYS/BIOS system heap created in mss.cfg                 */
/*   .l3ring     - the rolling buffer (raw ADC for legacy builds, compact      */
/*                 FFT range snapshots for LIVE_SNAPSHOT_RING builds).         */
/*   .l3scratch  - optional raw-frame scratch space used by live snapshot       */
/*                 builds before HWA compression into .l3ring. If these        */
/*                 sections overflow L3_RAM, the link fails loudly.            */
/*----------------------------------------------------------------------------*/
--retain="*(.intvecs)"

SECTIONS
{
    systemHeap : {} > DATA_RAM
    .l3ring    : {} > L3_RAM
    .l3scratch : {} > L3_RAM
}
/*----------------------------------------------------------------------------*/
