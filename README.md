# onew

1-Wire master.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. Software writes an operation to `cmd` (bus reset, write a bit, read a bit, write a byte, read a byte), waits for `status.busy` to fall or for the interrupt, and reads the result. Timing follows table 1 of Maxim application note 126, *1-Wire Communication Through Software*, at standard speed.

The slot table is `OnewSlot.bs`, written in Bluespec Haskell: one equation per kind of slot giving how long the master pulls the line low, when it samples and how long the slot lasts, all in microseconds. `tick` says how many clock cycles make a microsecond, so nothing is multiplied at build time. `Onew.bsv` walks one slot after another and picks the next slot of a byte operation. Bytes read are run through the CRC-8/MAXIM-DOW model of `Gf2` in `hwcore`, and `crc` reads back the CRC of the bytes since the last bus reset.

The testbench drives a 1-Wire device model. It checks that a reset gets a presence pulse and pulls low for exactly 480 microseconds, that a written 0x33 reaches the device with 6 and 60 microsecond low times, that Read ROM returns the device ROM number and its CRC, that `done` and the interrupt follow `ien`, that a bus with no device reads no presence, that three cycles per microsecond triple every time, and that lowering `tick` in the middle of an operation, after the counter has passed the new value, does not stall the slot.

| `crc` | off | on |
| :--: | --: | --: |
| Area, um2 | 1336 | 1490 |

## Registers

| Offset | Register | Fields |
| :--: | :-- | :-- |
| 0x00 | `ctrl` | `ien` |
| 0x04 | `tick` | clock cycles per microsecond, minus one (99 at reset, for 100 MHz) |
| 0x08 | `txd` | byte or bit to write |
| 0x0C | `cmd` | `op`: 0 reset, 1 write bit, 2 read bit, 3 write byte, 4 read byte |
| 0x10 | `status` | `busy`, `presence`, `done` (write 1 to clear) |
| 0x14 | `rxd` | byte or bit read by the last operation |
| 0x18 | `crc` | CRC-8/MAXIM-DOW of the bytes read since the last reset |

The line is open drain: `ow_pull` high means pull the pad low, and `ow_i` is the pad level. The board needs a pull-up of about 4.7 kilohm. Overdrive speed, strong pull-up and a hardware search are not implemented; software builds the search from bit reads and writes.

## License

Mulan PSL v2.
