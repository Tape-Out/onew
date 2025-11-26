## One-Wire Bus IP

> Timing sequence description references:
>
> - https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
>
> - https://www.analog.com/en/resources/technical-articles/1wire-communication-through-software.html

wire_io requires an external 5kΩ pull-up resistor.

> TODO: Improve the FSM for special commands and CRC8
>
> yosys: Checking module wire_mmio...
>
> Warning: multiple conflicting drivers for wire_mmio.\slot_bit_cmd [1]:
>
>     port Q[0] of cell $auto$ff.cc:266:slice$85578 ($_DFFE_PP_)
>
>     port Q[0] of cell $auto$ff.cc:266:slice$85596 ($_DFFE_PP_)
>
> Warning: multiple conflicting drivers for wire_mmio.\slot_bit_valid:
>
>     port Q[0] of cell $auto$ff.cc:266:slice$102932 ($_DFFE_PP_)
>
>     port Q[0] of cell $auto$ff.cc:266:slice$102946 ($_SDFFE_PN0P_)
>
> Found and reported 2 problems.

## Usage

> Write the number of bits to send, write to the send buffer, and set ctrl:send_is_submit to send bits to the 1-Wire bus.
>
> Read operation is similar to write.
>
> Special Commands:
>
> **Search | Alarm** - Configure ctrl:alarm_only_search, and write to the search MMIO address to search until completed or ROM buffer is full.
>
> > Write search tree for next search and retrieve the last ROMs.
>
> **Get1ROM Command**:
>
> Directly obtain ROM when there is only one device present.
>
> **Change Speed**:
>
> Write to the speed_reg to change speed, and send 0x3C to the bus if ctrl:change_speed is enabled.
