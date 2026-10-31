# Hardware vs. software CRC on the STM32L432KC

Benchmark code for the paper
 
> E. Fischer and G. Bausch, "When Does the Hardware CRC Unit Pay Off? Cycle-Accurate Comparison with
> Lookup-Table and Sliced Software CRC on the STM32L432KC," *2026*.
> [Link / DOI – to be added]

**Motivation:** A CRC can be computed in software or with the dedicated CRC peripheral of the
microcontroller, which can be fed by the CPU or by DMA. Which method is best, in particular with respect
to speed and memory? The benchmark answers this for an STM32L432KC (Arm Cortex-M4F, 80 MHz) and
three algorithms: CRC-32, CRC-16/MODBUS and CRC-8.

**What is measured:** CPU cycles per call (DWT cycle counter) for block sizes from 4 to 4096 bytes, for
**three software variants** (bitwise, lookup table, slicing-by-4) and **three hardware variants** (CPU access, CPU access with unrolled loop, DMA). Every variant is self-tested against a bitwise reference.

**Main results** (peripheral already configured, warm cache, data in SRAM, interrupts disabled):
 
| | CRC-32 | CRC-16/MODBUS | CRC-8 |
|---|---|---|---|
| Hardware faster than best software from | 4 bytes | 4 bytes | 4 bytes |
| Speedup > 2x from | 16 bytes | 16 bytes (13 bytes: 2.05x) | 32 bytes |
| Speedup at 4096 bytes vs. slicing-by-4 | 4.6x | 4.8x | 3.0x |
| Speedup at 4096 bytes vs. lookup table | 7.9x | 8.7x | 4.5x |

- The often quoted factor of about 60 refers to a bitwise software loop. Against slicing-by-4 the
  advantage is about 3 to 5x.
- The non-reflected CRC-8 gains least: the unit has no input bit reversal for it, so the CPU has to swap
  words and the DMA can only transfer byte by byte.
- DMA is only slightly faster than CPU access (CRC-32/CRC-16, from about 64 bytes) and slower for CRC-8.
  The CPU load during DMA is not measured.
- Software tables need RAM (lookup table 256 B to 1 KB, slicing-by-4 1.25 KB to 4 KB); the hardware needs none.

**Limits:** One device, one run, best-case conditions. The cost of reconfiguring the peripheral, the CPU
load with DMA, the HAL function `HAL_CRC_Calculate()`, tables in flash and other STM32 families are not
covered. See the paper for details.

**Cite.**
 
```bibtex
@inproceedings{fischer2026crc,
  author    = {Fischer, Eric and Bausch, Gerold},
  title     = {When Does the Hardware {CRC} Unit Pay Off? Cycle-Accurate Comparison with
               Lookup-Table and Sliced Software {CRC} on the {STM32L432KC}},
  year      = {2026}
}
```