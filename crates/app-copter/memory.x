/* STM32H753 (Pixhawk 6X reference board).
 *
 *   Flash: 2 MiB at 0x0800_0000
 *   AXI SRAM (contiguous data/stack): 512 KiB at 0x2400_0000
 *
 * H753 has a complex memory map (ITCM / DTCM / SRAM banks / backup SRAM),
 * but for a minimal firmware link AXI SRAM is fine — it's the largest
 * contiguous region and data/stack live there. Fancier layouts (ITCM
 * for IRQ hot paths, DTCM for EKF matrices) are a follow-up tuning pass.
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 2M
    RAM   : ORIGIN = 0x24000000, LENGTH = 512K
}
