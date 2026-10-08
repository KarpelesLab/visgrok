# Sipeed SLogic USB protocol

This is the USB protocol specification for the Sipeed SLogic logic analyzers that visgrok targets.
The main focus is the **SLogic16 U3** (`359f:3031`), which is attached to the development machine.
It was assembled from every public Sipeed source and checked against the live device.

Conventions:

* All multi-byte values are **little-endian** unless stated otherwise.
* `bmRequestType` values are given as raw bytes: `0x40` = vendor | device | OUT, and `0xC0` = vendor | device | IN.
* **[HW]** marks a fact measured on the attached unit with the rawusb probe (section 12). Everything else cites source code.
* "Sample" means one parallel snapshot of all enabled channels.

---

## 0. Sources and citation keys

| Key | Repository / file | Revision |
|---|---|---|
| **DEV** | `github.com/sipeed/libsigrok`, branch `slogic-dev`, `src/hardware/sipeed-slogic-analyzer/{api.c,protocol.c,protocol.h}` | `e7c1be07` (2026-09-29) |
| **CORE** | `github.com/sipeed/SLogic`, `slogic/slogic.{c,h}`, `slogic16u3.c`, `slogic32u3.c`, `slogic8u2.c` ("libslogic"). DEV's `slogic/` subdirectory and ALL-LOGIC's copy are byte-identical. | `1505058` |
| **SPEC** | `github.com/sipeed/SLogic`, `build/docs/slogic-protocol.md` (Sipeed's own "canonical" spec) | `1505058` |
| **BENCH** | `github.com/sipeed/SLogic`, `build/bench/slogic_control_vectors.py`, `slogic_pack_ref.py`, `build/docs/slogic-capture-baseline.md`, `slogic-driver-plan.md` | `1505058` |
| **PRE** | DEV at `0c36240d` (2026-07-24): the last pre-core driver, one big `api.c`. This is what released PulseView/sigrok-cli builds contain. | `0c36240d` |
| **AL** | `github.com/sipeed/ALL-LOGIC`, `libsigrok4DSL/hardware/sipeed-slogic/slogic16u3.c` (DSView port) | `e2ce17b` |
| **WEB** | `github.com/sipeed/SLogicWeb`, `docs/PROTOCOL-SLOGIC16U3.md` (WebUSB port, measured on a 16U3) | `5ec9b19` |
| **TOOLS** | `github.com/sipeed/slogic16u3-tools` (factory test tool: `slogicpt/mode_switch.py`, `resources/products/*/product.toml`) | `618e8aa` |
| **OLD** | `github.com/sipeed/sigrok_slogic` branches `hardware-sipeed-slogic-support` (`7e025da3`, 2024), `hardware-sipeed-slogic-analyzer-support` (`4397c41c`, 2025-02), `hardware-sipeed-slogic-basic-u16-demo` (`c9340eba`, 2025-04) | as listed |

Upstream `sigrokproject/libsigrok` master (`0bc24877`, 2025-11-20) has **no** SLogic driver. Support lives only in Sipeed's fork, on the `slogic-dev` branch. Sipeed's `libsigrok` `master` branch has no SLogic driver either.

PID `0x3031` is handled by DEV through `slogic_model_16u3` (CORE `slogic16u3.c:34-45`), which uses `SLOGIC_PROTO_U3`. The driver name is `sipeed-slogic-analyzer`.

---

## 1. Device family

| Model | VID:PID | Bulk IN | USB | Phys. ch | Channel modes → max rate | Control protocol | Source |
|---|---|---|---|---|---|---|---|
| SLogic Combo 8 (sold as "Combo 8" and "Combo 8 Lite Kit"; BL616-based, USB2) | `359f:0300` | `0x81` | High-Speed | 8 | 2ch→160 MHz, 4ch→80 MHz, 8ch→40 MHz | "Combo8" (one `0xB1` start command) | CORE `slogic8u2.c:19-43`, `slogic.h:53` |
| **SLogic16 U3** | **`359f:3031`** | **`0x82`** | SuperSpeed 5 Gb/s | 16 | 4ch→800, 8ch→400, 16ch→200 MHz | "U3" register/AUX | CORE `slogic16u3.c:20-45`, `slogic.h:54` |
| SLogic32 U3 | `359f:3032` | `0x82` | USB 3.2 Gen2 (per wiki) | 32 | 4ch→1400, 8ch→800, 16ch→400, 32ch→200 MHz | "U3" register/AUX | CORE `slogic32u3.c:18-44`, `slogic.h:55` |
| DFU / bootloader, 16U3 (also older 32U3 firmware) | `359f:30f1` | – | – | – | – | DFU over USB-SPI; **never touch** | `slogic.h:56`, TOOLS `products/slogic16u3/product.toml` |
| DFU / bootloader, 32U3 | `359f:30f2` | – | – | – | – | as above | TOOLS `products/slogic32u3/product.toml` |
| 2023 prototype ("sipeed-slogic" driver) | `359f:3001` | `0x82` | – | 16 | 1/2ch→1200, 4ch→600, 8ch→300, 16ch→150 MHz | none: streams after claim, no vendor requests | OLD `hardware-sipeed-slogic-support` `api.c:101`, `protocol.h:30-60`, `api.c:495-499` |

"SLogic Lite 8": no Sipeed source code or wiki page names a product with this exact name or a distinct PID. The closest product is the "SLogic Combo 8 Lite Kit", which is the Combo 8 (`0x0300`). Treat a "Lite 8" as a Combo 8 unless a device with another PID turns up.

Bandwidth ceilings (CORE `max_bandwidth_hz`, in channels × Hz):

* Combo 8: 320 M (40 MB/s). The wiki quotes 160 Mb/s on Windows.
* 16U3: 3200 M (400 MB/s). [HW] 400–402 MB/s was sustained on this Mac.
* 32U3: 6400 M (800 MB/s).

The wiki says the 16U3 accepts 0–10 V inputs with a 0–6 V adjustable threshold. Triggering is software-only. The 16U3 has no PWM or signal generator; the only "extra" is the "Extend ADC → Oscilloscope" add-on, and no protocol for it is published anywhere.

Windows quirk: PRE `api.c:117-123` capped 16U3 at 400/200/100 MHz on `_WIN32` because the Windows USB stack tops out around 390 MB/s. CORE dropped that guard (SPEC §1, lines 59-64).

USB2 fallback: AL caps U3 devices that enumerate at High-Speed to `320 MHz / nch` (AL `slogic16u3.c:319-341`). DEV has no such cap.

---

## 2. Descriptors of the attached SLogic16 U3 [HW]

Enumerated on macOS with rawusb. Bus 2, port 2, link speed **5 Gb/s (SuperSpeed)**.

### Device descriptor

```
12 01 20 03 ff ff ff 09 9f 35 31 30 02 00 01 02 03 01
```

| Field | Value |
|---|---|
| bcdUSB | `0x0320` (USB 3.2) |
| bDeviceClass / SubClass / Protocol | `0xff / 0xff / 0xff` (vendor specific; no OS class driver binds) |
| bMaxPacketSize0 | `9` (512-byte EP0, SuperSpeed encoding) |
| idVendor / idProduct | `0x359f` / `0x3031` |
| bcdDevice | `0x0002` |
| iManufacturer / iProduct / iSerialNumber | 1 / 2 / 3 |
| bNumConfigurations | 1 |

### Configuration descriptor (44 bytes)

```
09 02 2c 00 01 01 00 80 70
09 04 00 00 02 ff ff ff 00
07 05 02 02 00 04 00
06 30 0f 00 00 00
07 05 82 02 00 04 00
06 30 0f 00 00 00
```

* Configuration 1:
  * `bmAttributes = 0x80`: bus powered, no remote wakeup.
  * `bMaxPower = 0x70`: 112 × 8 mA = 896 mA at SuperSpeed.
  * `iConfiguration = 0`.
* Interface 0, alt 0: class `ff/ff/ff`, 2 endpoints, `iInterface = 0`.
* EP `0x02` (bulk OUT):
  * wMaxPacketSize 1024.
  * SS companion: `bMaxBurst = 15` (16 packets per burst), `bmAttributes = 0` (no streams).
  * **No driver uses this endpoint.** Its purpose is unknown; do not write to it.
* EP `0x82` (bulk IN):
  * wMaxPacketSize 1024.
  * SS companion: `bMaxBurst = 15`, `bmAttributes = 0`.
  * **This is the sample stream.**

### Strings (LANGID `0x0409`)

* `[0]` = `04 03 09 04`
* `[1]` = `"Sipeed"`
* `[2]` = `"SLogic16 U3"`
* `[3]` = `"202512191855"` (serial number; looks like a date and time stamp)

### Other descriptors and requests

* **BOS** (5 bytes): `05 0f 05 00 00`. `bNumDeviceCaps = 0`, so there is no SuperSpeed USB Device Capability descriptor. That is spec-noncompliant, but harmless.
* **GET_STATUS(device)** returns `01 00` (self-powered bit set, even though the config descriptor says bus powered).

### Quirks

1. **Requests for nonexistent descriptors are not STALLed.** String indexes 4–8 and the Device Qualifier request all **time out** (500 ms each) instead of returning STALL. Never probe for optional descriptors. Read only strings 1–3.
2. **The device can enumerate unconfigured.** On macOS it came up with `bConfigurationValue == 0`, and `claim_interface(0)` failed with NotFound until the host sent `SET_CONFIGURATION(1)`. The driver must check `active_configuration()` and call `set_configuration(1)` when it returns 0. WEB also calls `selectConfiguration(1)` (WEB lines 23-29).

---

## 3. Control transport: U3 models (16U3, 32U3)

All configuration goes through **32-bit register accesses** using vendor requests on EP0, recipient device:

| Op | bmRequestType | bRequest | wValue | wIndex | wLength | Data |
|---|---|---|---|---|---|---|
| REG_READ | `0xC0` | `0x00` | register byte address | `0x0000` | **4** | IN: register value, u32 LE |
| REG_WRITE | `0x40` | `0x01` | register byte address | `0x0000` | **4** | OUT: value, u32 LE |

The raw setup packets are:

* write `40 01 aL aH 00 00 04 00` followed by 4 data bytes;
* read `C0 00 aL aH 00 00 04 00`.

Sources: CORE `slogic.c:238-291`; DEV adapter `api.c:536-556`; PRE `api.c:641-723`; SPEC lines 80-92; BENCH `slogic_control_vectors.py:17-22`.

Timeout: 500 ms per transfer in every implementation (CORE `slogic.c:251`).

**Exactly one 32-bit word per control transfer.**

* Multi-word payloads are sent as consecutive transfers, with `wValue` stepping by 4 (CORE `slogic.c:254-284`).
* Lengths are always rounded **up** to a multiple of 4.
* [HW] An 8-byte REG_READ is accepted, but it returns the *same* word twice. Reading `0x0000` returned `01 00 01 00 01 00 01 00`, and `0x000c` returned the header twice. The address does not auto-increment, so multi-word transfers silently return wrong data.

Control transfers are accepted while the bulk stream is running [HW]:

* register reads and the STOP write during RUN worked on this unit/firmware, with no errors;
* AL and DEV nevertheless defer the STOP write until all bulk URBs have drained, because "some U3 firmware reports BUSY" (AL `slogic16u3.c:1626-1640`; DEV `protocol.c:213-219`).

A safe driver issues configuration only while stopped. It may still write STOP during streaming; if that write fails, retry it after the transfers drain.

### 3.1 Register map

| Addr | Name | Access | Meaning |
|---|---|---|---|
| `0x0000` | (ID?) | R | [HW] always reads `0x00010001`. Never accessed by any driver. Probably a version or ID word; unverified. |
| `0x0004` | `R32_CTRL` | R/W | bit0 = RUN, bit1 = RST. Values: `0x00000000` = STOP, `0x00000001` = RUN, `0x00000002` = RST asserted (CORE `slogic.c:240-245`). Reads back the last written value [HW]. |
| `0x0008` | `R32_FLAG` | R | `#define`d but never used (PRE `api.c:906`; SPEC line 97). [HW] reads 0 in every state, including during a forced FIFO overflow, so it is **not** an overflow flag. |
| `0x000c` | `R32_AUX` | R/W | AUX mailbox command and header (section 3.2). |
| `0x0010`… | AUX payload | R/W | AUX payload words: `0x10`, `0x14`, … (CORE `slogic.c:242`). |
| `0x0014`–`0x001c` | – | R | [HW] read 0 when idle. |

### 3.2 CTRL semantics [HW + source]

**RST pulse** = write `0x02`, then write `0x00` (CORE `slogic_reset`, `slogic.c:461-473`; PRE `api.c:965-977`).

* [HW] While CTRL = `0x02` (reset held), the **AUX mailbox does not respond**: the ready bit never sets. The device was found in that state when first opened, presumably left by the previous host session. **Always finish with CTRL = 0 before using AUX.**
* [HW] **RST restores every AUX setting to its power-on default**:
  * channel mask `0xFFFF`;
  * rate: base index 0, base 800 MHz, `divm1 = 10` (72.7 MHz);
  * vref code `0x136` (310);
  * pattern 0.
* [HW] Without RST, AUX settings **persist across host sessions**. Opening the device without a reset showed the previous capture's mask `0xFF`, `divm1 = 15`, vref `0xF6` and pattern 2.
* This contradicts SPEC lines 101-104 ("a capture start ... never writes RST (which would discard pattern-generator and vref setup)") and BENCH `slogic_control_vectors.py:127-128`. The shipping code does the opposite: DEV `slogic_dev_start` and AL acquisition start both write RST and then reconfigure everything, every time (DEV `api.c:725-734`; AL `slogic16u3.c:2386-2402`). The commit message for `27407f7a` gives the reason: the device only reliably restarts the stream from a clean reset.

Other CTRL writes:

* **RUN**: write `0x01`. Streaming starts. [HW] First data arrives after about one transfer's worth of sample time; there is no measurable start latency beyond that.
* **STOP**: write `0x00`. [HW] Streaming stops immediately. Bulk reads issued after STOP time out with 0 bytes; nothing is left in the pipe.

### 3.3 AUX mailbox transaction

AUX selectors are listed in CORE `slogic.c:246-249`:

| Selector | Function | Payload length reported [HW] |
|---|---|---|
| `1` | channel enable mask | 2 bytes |
| `2` | samplerate (base table + divider) | 8 bytes |
| `3` | threshold DAC ("vref") | 2 bytes |
| `5` | test-pattern mode | 1 byte |

Selector 4 and selectors above 5 are unused by any driver; do not send them.

One transaction (CORE `aux_begin` / `aux_write_confirm`, `slogic.c:297-347`; PRE `api.c:909-963`):

1. **Write the selector**: REG_WRITE `0x000c` ← `u32 selector`.
2. **Poll the header**: REG_READ `0x000c` until the **ready bit** (`header >> 16) & 1` is set, which is bit 0 of byte 2.
   * Retries: CORE 8 (`slogic.c:252,308-318`); PRE 6 (`api.c:921-930`).
   * [HW] Ready on the first read every time.
3. **Decode the header** (u32, layout confirmed on [HW]):

   ```
   bits  0..8   : selector echo  (1, 2, 3, 5)
   bits  9..15  : payload length in BYTES
   bit   16     : ready
   bits 17..31  : 0
   ```

   * [HW] headers seen: `0x00010401` (sel 1, len 2), `0x00011002` (sel 2, len 8), `0x00010403` (sel 3, len 2), `0x00010205` (sel 5, len 1).
   * The length is in **bytes**, despite SPEC line 115 / BENCH line 52 calling it "words". All code treats it as bytes and rounds up to 4 (CORE `slogic.c:320-326`; PRE passes it as a byte length, `api.c:933-936`).
   * Do **not** round it down: WEB lines 83-89 records that rounding down turns lengths of 1 and 2 into 0, which makes every setting a no-op.
   * The selector echo is not checked by any C driver. WEB checks it (line 92-94). Recommended: check it.
4. **Read the payload**: `ceil(len/4)` words from `0x0010`, `0x0014`, ….
5. **Modify the payload and write it back** to `0x0010…` (same word count).
6. **Confirm**: read the payload again and compare. CORE always confirms (`slogic.c:337-347`). AL used to skip this, and PRE confirmed vref against a hard-coded 1024 (bug B10).

### 3.4 Selector 1: channel mask

Payload word 0 = `(1 << nch) - 1`, u32 LE. Only the low 16 bits exist on the 16U3. For 32 channels the value is `0xFFFFFFFF` (CORE `aux_channel`, `slogic.c:359-378`; PRE `api.c:1011`).

| Mode | Payload bytes |
|---|---|
| 16ch | `ff ff 00 00` |
| 8ch | `ff 00 00 00` |
| 4ch | `0f 00 00 00` |

The 16U3 supports only nch ∈ {4, 8, 16} (CORE `slogic16u3.c:28-32`). The mask also selects the **packing mode** of the stream (section 5). Only contiguous low masks are ever used; arbitrary masks are untested, so don't send them.

### 3.5 Selector 2: samplerate

Payload, 8 bytes (CORE `aux_rate`, `slogic.c:380-422`; PRE `api.c:1034-1106`; WEB lines 107-118):

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | `base_idx`: index into the device's base-clock table (R/W) |
| 2 | u16 | `base_mhz`: base clock in MHz for `base_idx` (R; the device updates it when `base_idx` is written) |
| 4 | u32 | `divm1`: divider minus one. Sample rate = `base_mhz × 1e6 / (divm1 + 1)` |

Algorithm (CORE):

1. Read the payload.
2. If `base_mhz*1e6 % want != 0`:
   * increment `base_idx`;
   * write **word 0 only** (`idx | base_mhz << 16`) to `0x0010`;
   * re-read the payload, which now has the new `base_mhz`;
   * retry, up to 8 times.
3. If `base_mhz == 0`, fail.
4. On a hit, set `divm1 = base/want - 1`, write the whole 8-byte payload, and read it back.

[HW] base table on the 16U3 (walked by writing `base_idx` = 0…5):

| base_idx | base_mhz |
|---|---|
| 0 | 800 |
| 1 | 800 |
| 2 | 0 |
| 3 | 0 |
| 4 | 800 (index wraps modulo 4) |
| 5 | 800 |

So on the 16U3 **every rate is 800 MHz / N**. On the 32U3 the two bases are 1400 and 800 MHz (WEB lines 178-181; commit `68573db8`). The 4-channel limit is 1400, not 1600, because 1600 MHz cannot be expressed with those bases.

[HW] **The divider is effectively 8 bits.**

* Requesting 1 MHz writes `divm1 = 799` (`0x31F`). The register reads back `0x31F`, but the device streams at **25 MHz** = 800/(0x1F+1).
* Requesting 2 MHz (`0x18F`) streamed at 5.55 MHz = 800/(0x8F+1).

So `divm1` is truncated to `& 0xFF`:

* the valid range is `divm1` 0…255;
* the minimum rate is **3.125 MHz**;
* [HW] `divm1 = 199` (4 MHz) and `159` (5 MHz) were verified exact.

No driver clamps this, but the advertised tables start at 5 MHz, so nothing in Sipeed's code triggers it.

[HW] `divm1` semantics are verified by throughput, where in Normal mode the stream is paced exactly by the sample clock:

| Mode | divm1 | Expected | Measured |
|---|---|---|---|
| 16ch @ 200 MHz | 3 | 400 MB/s | 400.5 MB/s |
| 8ch @ 400 MHz | 1 | 400 MB/s | 401.4 MB/s |
| 4ch @ 800 MHz | 0 | 400 MB/s | 402.6 MB/s |
| 16ch @ 100 MHz | 7 | 200 MB/s | 201.8 MB/s |

The 2025 demo branch wrote `divm1 = base/rate`, without the `-1` (OLD `basic-u16-demo` `api.c:611-612`). That is wrong for current firmware.

Advertised rates (CORE `slogic16u3.c:20-25`): 5, 8, 10, 16, 20, 25, 32, 40, 50, 80, 100, 160, 200, 400, 800 MHz. That is the integer-MHz subset of 800/N with N ≤ 160. Any 800/N with N ∈ [1, 256] is accepted by the hardware. 4 MHz (N = 200) was verified on [HW].

Per-mode ceiling: `nch × rate ≤ 3.2 Gbit/s`, giving 4ch ≤ 800, 8ch ≤ 400, 16ch ≤ 200 MHz (CORE `slogic16u3.c:27-32`).

### 3.6 Selector 3: threshold DAC ("vref")

Payload word 0 = DAC code (2 significant bytes).

**Nominal formula**: `code = round(V / 3.33 / 2 * 1024)`, with V clamped to 0…6 V (CORE `vth_to_dac`, `slogic.c:349-357`). This is about 153.75 codes per volt (a 6.66 V full scale over 1024 codes).

* PRE truncated the code and averaged libsigrok's two-value threshold first (PRE `api.c:1136-1139`; DEV `api.c:574-585`).
* AL clamps V to 0…5 V instead (AL `slogic16u3.c:95,2201`).

**Measured transfer function** (WEB lines 120-132; one unit, serial `202512261505`, three-point fit within 13 mV):

```
V_threshold = 0.005166 * code + 0.4318
code        = clamp(round((V - 0.4318) / 0.005166), 0, 1023)
```

This has a +0.43 V intercept that the nominal formula lacks. For example, nominal 1.7 V gives code 261, which actually means ≈ 1.78 V. Linearity degrades above about 4 V.

The power-on default after RST is code 310 [HW]: nominal 2.02 V, measured fit ≈ 2.03 V. sigrok's default threshold is 1.7 V (DEV `api.c:184-185, 263`).

### 3.7 Selector 5: test-pattern mode

Payload word 0 = mode (CORE `aux_test`, `slogic.c:443-459`; `slogic.h:59-64`):

| Mode | Name | Behaviour [HW] |
|---|---|---|
| 0 | Normal | Real inputs, paced by the sample clock. |
| 1 | "USB connection test" | Unpaced counter. In 16ch mode it is a 16-bit LE value incrementing by 1 per sample. [HW] ~413 MB/s regardless of the configured rate. It is **flow-controlled**: zero discontinuities even with slow, synchronous reads. |
| 2 | "Emulation" | Paced by the sample clock. Sample *i* = `(i & ~7) \| (7 − (i & 7))`, truncated to the channel width. The stream is therefore `7,6,5,4,3,2,1,0,15,14,…,8,23,…`. 16ch: `07 00 06 00 05 00 …`; 8ch: `07 06 05 04 …`; 4ch: `67 45 23 01 ef cd ab 89 …` (WEB lines 267-272, confirmed [HW]). |

Notes:

* The pattern register persists until RST. PRE never reset it, so a device left in Emulation kept producing fake data (WEB lines 274-276).
* PRE, when Normal was selected, also issued RST (PRE `api.c:510-514`).
* CORE programs the pattern on every configure (`slogic.c:494`).
* **Always program the pattern explicitly.**

---

## 4. Combo 8 control protocol (PID `0x0300`)

There is no register map. The only command is one vendor OUT request (CORE `slogic_run`, `slogic.c:504-511`; PRE `api.c:859-898`; OLD `analyzer-support` `protocol.h:105-120`, `protocol.c:271-282`):

| Op | bmRequestType | bRequest | wValue | wIndex | Data |
|---|---|---|---|---|---|
| CMD_START | `0x40` | `0xB1` | 0 | 0 | `[u16 LE samplerate in MHz][u8 channel count]` + pad |
| CMD_STOP | `0x40` | `0xB3` | 0 | 0 | none. **Unreliable in firmware; not used** (PRE `api.c:895-897`). |

Payload length:

* OLD sends **3** bytes (`wLength = 3`).
* CORE sends **4** bytes, `[mhz_lo, mhz_hi, nch, 0]`.
* PRE meant to send 3, but its `slogic_usb_control_write` rounds the length up to 4 and reads one byte past the 3-byte struct (bug B11).

The firmware evidently tolerates a trailing pad byte. That is unverified here because no Combo 8 is attached.

Other Combo 8 details:

* **Stop** = drain the bulk IN endpoint: read repeatedly with a 100 ms timeout until a read returns 0 bytes (DEV `clear_ep`, `api.c:689-704`). AL caps this at 32 × 64 KiB reads with 50 ms timeouts (`slogic16u3.c:562-581`).
* There is no configure step; rate and channel count travel in CMD_START (CORE `slogic.c:482-483`).
* There is no threshold control (wiki: "Adjustable threshold: No") and no pattern modes.
* Rates (CORE `slogic8u2.c:19-23`): 1, 2, 4, 5, 8, 10, 16, 20, 32, 40, 80, 160 MHz, where 160 = 2⁵·5 MHz. Ceilings: 2ch 160, 4ch 80, 8ch 40 MHz.
* An older firmware's table also listed 36, 64, 120, 128 and 144 MHz, and chose the channel count from the rate: ≥120 MHz → 2ch, ≥40 → 4ch, else 8ch (OLD `analyzer-support` `protocol.h:35-103`).
* OLD used 210 × 512-byte transfers (107 520 B), a single transfer, and 64 max (`protocol.h:31-33,130-155`).

---

## 5. Sample stream format

Samples arrive on bulk IN (`0x82` for U3, `0x81` for Combo 8) as a **raw, unframed, sample-major stream**: no headers, no markers, no timestamps (SPEC lines 154-175). Channel *k* is bit *k* of the sample, counting from the LSB.

| nch | Bytes per sample | Packing |
|---|---|---|
| 2 (Combo 8 only) | 1/4 | 4 samples per byte. Sample *j* of the byte = `(b >> (2*j)) & 3`, so sample 0 is the **lowest** two bits. |
| 4 | 1/2 | 2 samples per byte. Sample 0 = **low nibble** `b & 0xF`, sample 1 = `b >> 4`. |
| 8 | 1 | 1 byte per sample. |
| 16 | 2 | u16 **little-endian**: `b0 \| b1 << 8`. D0–D7 are in b0. |
| 32 (32U3) | 4 | u32 little-endian. |

Sources: BENCH `slogic_pack_ref.py:22-60` (vectors at lines 64-76); AL `slogic_unpack_append`, `slogic16u3.c:689-750`; DEV `slogic_submit_raw_data`, `api.c:587-620`.

[HW] confirmations with the Emulation pattern:

* 16ch: `07 00 06 00` = samples `0x0007, 0x0006`;
* 4ch: `67 45 23 01` = samples 7, 6, 5, 4, 3, 2, 1, 0, so the low nibble comes first.

WEB lines 216-219 also showed with an external generator that GP*k* → bit *k*, with no byte swap, at 16ch.

Channel numbering in reduced modes:

* On the 16U3, an *n*-channel mode samples D0…D(n−1) (AL's mode labels: "Use Channels 0~7 (Max 400MHz)", "0~3", `slogic16u3.c:117-122`).
* On the 32U3, BENCH `slogic-driver-plan.md:187-190` claims reduced modes carry the *high* channel bytes. In addition, TOOLS `TODO.md` reports that 32U3 grouped modes read all-zero on real pins (a firmware bug). Neither affects the 16U3 that I know of. The factory profile for the 16U3 does test 16ch only (TOOLS `slogic16u3/product.toml`).

### 5.1 The 4-byte head artifact (mandatory)

The first 4 bytes of every acquisition's stream are not sample data and must be dropped. Drop them **once per acquisition, not once per transfer**, across however many transfers it takes (CORE `slogic_stream_init` sets `drop_left = 4`, `slogic.c:143`; `slogic_apply_first_drop`, `slogic.c:154-164`; DEV `protocol.c:67-77`; AL `slogic16u3.c:1187-1194`; SPEC lines 177-182).

[HW] findings:

* The artifact is **4 bytes in every mode**, not 2 samples: it is 2 samples at 16ch, 4 samples at 8ch, and 8 samples at 4ch.
* After an RST it is normally `00 00 00 00`.
* Once it contained `fd 17 fc 17`, which are stale Emulation values from an earlier run. It is a leftover 32-bit word in the device's output path.
* Commit history: `cb9fa2a3` "drop 2 samples for hardware BUG", then `4a15246e` "Update sample drop size" to the fixed 4 bytes.

Combo 8: the old Combo 8 driver never dropped anything (OLD `analyzer-support` `protocol.c:52-86`). DEV/CORE now drop 4 bytes for all models. Whether the Combo 8 has the artifact is **unverified** (see B5).

[HW] Emulation start phase: at ≥ 200 MHz, after the 4 dropped bytes, the Emulation stream sometimes begins with a truncated first group. Three runs started `1,0,7,6…`, `3,2,1,0,7,6…` and `5,4,3,2,1,0,7…`. At ≤ 20 MHz it always began at `7`. Treat the first few words after the drop as possibly belonging to a startup transient; it is irrelevant for real signals.

### 5.2 Unpacking reference (Rust-ish)

```rust
match nch {
    4  => for b in raw { out.push(b & 0xF); out.push(b >> 4); },
    2  => for b in raw { for j in 0..4 { out.push((b >> (2*j)) & 3); } },
    8  => out.extend(raw),
    16 => for c in raw.chunks_exact(2) { out.push(u16::from_le_bytes([c[0], c[1]])); },
    32 => for c in raw.chunks_exact(4) { out.push(u32::from_le_bytes(c.try_into()?)); },
}
```

Carry a partial sample (16/32ch) across transfer boundaries; AL does this with `stream_res`, `slogic16u3.c:719-749`. Transfers can complete short on timeout, and the 4-byte drop shifts alignment only by whole words, so 16/32ch stays aligned unless a transfer ends on an odd byte.

---

## 6. Acquisition sequence (U3)

This is the canonical sequence used by DEV (`protocol.c:491-584` + `api.c:717-735`) and AL (`slogic16u3.c:2330-2420`). Steps marked **(visgrok)** are recommended hardening from [HW] findings.

```
open:
  1. open device; if active_configuration()==0 → SET_CONFIGURATION(1)        (visgrok, [HW] macOS)
  2. claim interface 0 (detach kernel driver if any — none binds; class ff)   (DEV api.c:234; AL :1856-1858)
  3. RST pulse: CTRL←2, CTRL←0                                                  (DEV api.c:261 → CORE slogic.c:461-473)

start (every capture):
  4. drain EP 0x82: bulk reads (≥64 KiB, 50–100 ms timeout) until 0 bytes / timeout   (AL :2385)
  5. CTRL←0 (STOP)                                                               (DEV protocol.c:505)
  6. RST pulse: CTRL←2, CTRL←0                                                   (DEV api.c:730; AL :2397)
  7. CTRL←0 (pre-arm STOP)                                                       (CORE slogic.c:485)
  8. AUX sel 1: channel mask       ┐ each: write sel → poll ready → read payload →
  9. AUX sel 2: samplerate search  │       modify → write → read-back confirm
 10. AUX sel 3: threshold DAC      │       (CORE slogic.c:488-495, fixed order)
 11. AUX sel 5: pattern mode       ┘
 12. allocate + submit the bulk IN ring (N transfers on 0x82)                   (DEV protocol.c:535)
 13. CTRL←1 (RUN)                                                                (CORE slogic.c:512)
 14. stream: on each completion, drop first 4 bytes of the acquisition, unpack,
     resubmit; count bytes; stop when limit reached
stop:
 15. CTRL←0 (STOP) (AL/DEV defer until all URBs returned; [HW] immediate works)
 16. cancel remaining transfers, wait for all to complete/cancel
 17. optionally drain EP (nothing arrives after STOP [HW])
close:
 18. (optional) CTRL←0; release interface 0; close.
```

A full start (steps 5–13) is about 30 control transfers.

### 6.1 Bulk transfer ring

* At most **16** transfers in flight (CORE `SLOGIC_MAX_TRANSFERS`, `slogic.h:181`; DEV `protocol.h:37`).
* Sizes are multiples of **32 KiB**, which is a multiple of the 1024-byte SuperSpeed packet.
* Expected byte rate = `samplerate × nch / 8`.

Two sizing strategies exist (SPEC §6.5 lists this as an *open* decision):

* **DEV**: probe a 250 ms buffer, align to 32 KiB, then quarter it (≈ 62 ms each, no upper cap). At 400 MB/s that is about 25 MB per transfer (`protocol.c:313-392`).
* **AL**: about 4 ms of data, clamped to [32 KiB, 3 MiB] (`slogic16u3.c:1390-1404`). At least 4 transfers, at most 16 (`:1445-1452`).

Per-transfer timeouts:

* DEV: `1.3 × duration_ms × (k+2)` (`protocol.c:426-428`).
* AL: `1.3 × duration_ms × 4`, minimum 10 ms (`slogic16u3.c:1352-1356`).
* A timed-out transfer may still carry data; both drivers treat `TIMED_OUT` like `COMPLETED` (DEV `protocol.c:56-57`).

**The device has no flow control in Normal and Emulation modes** [HW]:

* When the host stops reading, the device's small output FIFO overflows and **data is silently dropped**. There is no flag (`0x0008` stays 0) and no marker.
* Test 1: RUN with no reads for 1 s, then read. Only the first **28 672 bytes** were continuous, then the stream jumped.
* Test 2: during sustained 16ch@200 MHz (400 MB/s) with a 16 × 1 MiB ring, a gap of about 700 samples (≈ 1.4 KB) appeared at stream offset ≈ 28 676 of many transfers. That is 327 discontinuities over 200 MB.
* With 4 MiB × 8 there were 19 discontinuities. At 100 and 200 MB/s (16 × 1 MiB) there were **zero** in 100–200 MB.
* The same overflow conditions also produced **4-byte words delivered out of place**: the word belonging at stream offset X+12288 appeared at a 4 KiB-aligned offset X, and its own slot was lost. This looks like a FIFO read-pointer race at overflow.

Conclusions:

* Keep many transfers queued at all times. Never do processing between "transfer completed" and "resubmit".
* At the 400 MB/s ceiling on this Mac, expect occasional loss.
* A driver cannot detect loss in Normal mode. Offer an Emulation-pattern self-test that checks continuity (section 11).

The USB-test pattern (mode 1), by contrast, is flow-controlled.

### 6.2 Stall and "RUN swallowed" handling

The policy below is CORE `slogic_stream_watch` (`slogic.c:166-229`) and SPEC §3/§6.2:

* **Before any byte has arrived:** a completion is "slow" if the gap since the previous completion exceeds `1.3 × transfer_size / expected_rate`, or if `actual_rate < 0.7 × expected_rate`. After `ring_count` consecutive slow completions, re-issue the start sequence **once**: STOP, reset counters, resubmit the ring, then RST + configure + RUN. DEV does this in `restart_acquisition_after_stall`, `protocol.c:455-489`, from the session thread and never from inside a bulk callback. If it still does not start, abort.
* **After data flows:** "slow" is only a one-shot warning. Only **1 s of total silence** is fatal (`SLOGIC_STREAM_IDLE_US`).
* Any `STALL`, `OVERFLOW` or `NO_DEVICE` status → abort.

### 6.3 Samplerate latch failure (newly found) [HW]

In about **10–20 % of captures, the device streamed at the previous capture's sample rate.** The AUX read-back showed the new `divm1`, but the pacing of the bulk data matched the old rate. Example: a 16ch capture configured for 100 MHz delivered 102 MB/s (50 MHz pacing) right after a 50 MHz capture.

Workarounds that did **not** fix it:

* delays of 200 ms after RST or after configuration;
* holding RST for 20 or 200 ms;
* configuring twice;
* skipping RST;
* skipping the vref and pattern blocks;
* a dummy RUN → STOP → drain before the real RUN. This reduced the rate to about 1 in 20 but did not remove it.

The failure happens both with and without a channel-count change. No Sipeed source mentions it.

Recommended host mitigation:

1. After RUN, time the first about 50–100 ms of Normal-mode data. With no backpressure, it arrives at exactly `rate × nch / 8`; [HW] measured within 0.3–4 %.
2. If the measured rate deviates from the expected rate by more than about 25 % (in practice it is the old rate), discard the data and redo STOP + RST + configure + RUN.

This needs a host that keeps up. Alternatively, run a short dummy capture whenever the rate changes and verify it before the real capture.

---

## 7. Valid samplerates (16U3 summary)

* Rate = `800 MHz / (divm1 + 1)`, with `divm1` ∈ [0, 255]. That gives 800, 400, 266.67, 200, 160, 133.33, … down to 3.125 MHz.
* Constraint: `nch × rate ≤ 3200 MHz`, so 4ch ≤ 800, 8ch ≤ 400, 16ch ≤ 200.
* Sipeed's advertised list (all ≤ the mode limit): 5, 8, 10, 16, 20, 25, 32, 40, 50, 80, 100, 160, 200, 400, 800 MHz.
* Non-integer-MHz rates (for example 800/3) cannot be requested through CORE's `base % want` test unless the rate is given in Hz exactly, since 266 666 666.67 Hz is not an integer. A custom driver can write `divm1` directly. Sample-clock accuracy at those settings is unverified.

---

## 8. Packet and handshake diagram (16ch @ 200 MHz, Normal, 1.6 V)

```
OUT 40 01 0400 0000 [00 00 00 00]   CTRL=STOP
OUT 40 01 0400 0000 [02 00 00 00]   CTRL=RST
OUT 40 01 0400 0000 [00 00 00 00]   CTRL=0 (release)
OUT 40 01 0400 0000 [00 00 00 00]   CTRL=STOP (pre-arm)
OUT 40 01 0c00 0000 [01 00 00 00]   AUX sel=1
IN  C0 00 0c00 0000 → 01 04 01 00    hdr: sel 1, len 2, ready
IN  C0 00 1000 0000 → ff ff 00 00    payload (default mask)
OUT 40 01 1000 0000 [ff ff 00 00]    mask = 0xFFFF
IN  C0 00 1000 0000 → ff ff 00 00    confirm
OUT 40 01 0c00 0000 [02 00 00 00]    AUX sel=2
IN  C0 00 0c00 0000 → 02 10 01 00    hdr: sel 2, len 8, ready
IN  C0 00 1000 0000 → 00 00 20 03    idx 0, base 800 MHz
IN  C0 00 1400 0000 → 0a 00 00 00    divm1 10 (default)
OUT 40 01 1000 0000 [00 00 20 03]
OUT 40 01 1400 0000 [03 00 00 00]    divm1 = 800/200 − 1 = 3
IN  C0 00 1000 0000 → 00 00 20 03    confirm
IN  C0 00 1400 0000 → 03 00 00 00
OUT 40 01 0c00 0000 [03 00 00 00]    AUX sel=3
IN  C0 00 0c00 0000 → 03 04 01 00
IN  C0 00 1000 0000 → 36 01 00 00    default code 310
OUT 40 01 1000 0000 [f6 00 00 00]    code 246 (nominal 1.6 V)
IN  C0 00 1000 0000 → f6 00 00 00
OUT 40 01 0c00 0000 [05 00 00 00]    AUX sel=5
IN  C0 00 0c00 0000 → 05 02 01 00
IN  C0 00 1000 0000 → 00 00 00 00
OUT 40 01 1000 0000 [00 00 00 00]    pattern Normal
IN  C0 00 1000 0000 → 00 00 00 00
(submit bulk IN ring on 0x82)
OUT 40 01 0400 0000 [01 00 00 00]    CTRL=RUN
… bulk IN data …
OUT 40 01 0400 0000 [00 00 00 00]    CTRL=STOP
```

(Setup fields shown as `bmRequestType bRequest wValue(LE) wIndex(LE)`; wLength is always 4.)

---

## 9. Other vendor requests seen in Sipeed code: do NOT send

* **RECONFIG** (TOOLS `slogicpt/mode_switch.py:14-36`): `bmRequestType 0x40`, `bRequest 0x30`, `wValue 0x0001` (0 = no-op probe), `wIndex 0x5253` (`'RS'` magic), `wLength 0`.
  * It triggers an FPGA reconfiguration and re-enumeration between application and DFU modes.
  * The 32U3 supports it in both directions. The 16U3 profile says it is unsupported (`usb_reconfig = false`); 16U3 mode switching uses JTAG or the MODE button.
  * The tool cites an internal "USB LA 协议规范 (USB LA protocol specification), 0x30 device-management extension" document, which is not public.
* **DFU / USB-SPI flash** (PID `0x30f1` / `0x30f2`, TOOLS `slogicpt/dfu/*`): this writes SPI flash. It is out of scope; never implement it in visgrok.

---

## 10. Known bugs and quirks in the sigrok drivers

"DEV" is the current `slogic-dev` driver, and "PRE" is the `0c36240d` driver that most binaries ship. For each bug, the correct behaviour is given in **bold**.

**B1. Continuous mode is advertised but cannot work (DEV).**
`SR_CONF_CONTINUOUS` is in `devopts` (`api.c:45`). With `limit_samples == 0`, `samples_need_nbytes = 0` (`protocol.c:510-512`). The ring loop condition `got + used*size < need` (`protocol.c:403-406`) is then false, so no transfers are submitted and start returns `SR_ERR_IO` (`protocol.c:445, 535-537`). The resubmit condition at `protocol.c:123-127` has the same problem.
**Correct: treat need == 0 as unbounded** (AL does: `slogic16u3.c:1445-1453`, `is_loop` + `slogic16u3.c:2371-2373`).

**B2. Heap overflow and over-read in the <8-channel unpack (DEV and PRE).**
`slogic_submit_raw_data` and `slogic_soft_trigger_raw_data` loop `for (i = 0; i < len; i += nCh)` and read `data[i + j/nsp]` for j < 8, which touches up to `i + 3` at 4ch. They write `ptr[i*nsp + j]` into `malloc(len*nsp)` (`api.c:595-607, 632-645`). When `len % nCh != 0`, which happens after the trigger or limit trim (`protocol.c:79-82`) or after a short or timed-out transfer, they read past the input and write past the output.
**Correct: unpack per input byte** (AL `slogic16u3.c:703-715`).

**B3. Head-drop bookkeeping (DEV).**
DEV still uses PRE's `first_here` heuristic (`protocol.c:59, 67-77`), which compares timestamps. It does not use CORE's `drop_left` counter: `slogic_stream_init` sets it, but `slogic_apply_first_drop` is never called. A first transfer with 1–3 bytes is not dropped, and then 4 more bytes are dropped from the next transfer.
`time_start` is also assigned after RUN (`protocol.c:579-581`) while callbacks run on the event thread, which is a race.
**Correct: a byte counter `drop_left = 4`, decremented across transfers** (CORE `slogic.c:154-164`).

**B4. Stall watchdog: integer truncation of the expected rate (DEV).**
`expected_rate_MBps = (sr*ch/8/1000)/1000` is integer MB/s (`protocol.c:322-328`). It feeds the watchdog as `expected_rate_MBps * 1e6` (`protocol.c:543-544`). Examples: 4ch@5 MHz = 2.5 MB/s becomes 2 (a 20 % error), and anything below 1 MB/s becomes 0, which disables the watchdog.
PRE's rule (the "slow transfer is fatal on every transfer" rule, which never reset once data flowed) truncated long captures: 4ch over libsigrok completed 1 of 120 runs in Sipeed's own baseline (BENCH `slogic-capture-baseline.md:56-64`).
**Correct: compute in bytes/s with u64 or f64, and use the CORE never-started vs. flowing policy.**

**B5. The 4-byte drop is applied to the Combo 8 too (DEV/CORE).**
The pre-2025 Combo 8 driver never dropped bytes (OLD `analyzer-support` `protocol.c:52-86`). The drop moved to all models when the code was unified, apparently without hardware evidence for the Combo 8.
**Correct: unverified. Measure on a Combo 8 before assuming the artifact.**

**B6. Unsupported rate or channel count silently becomes the maximum (DEV/PRE).**
`config_set(SAMPLERATE)` with a value that is not in the table or is above the limit sets `cur_samplerate = limit_samplerate` (the *maximum*) and only logs a warning (`api.c:367-378`). Asking for 1 MHz therefore captures at 200 MHz. Invalid `NUM_LOGIC_CHANNELS` likewise becomes the maximum channel count (`api.c:384-387`).
The factory tool parses stderr for "wrap to" to catch this (TOOLS `slogicpt/sigrok.py:11-12,218-242`).
**Correct: reject the value, or snap to the nearest valid rate at or below the request** (AL `slogic_pick_rate`, `slogic16u3.c:377-406`).

**B7. No divider range check (CORE/DEV/AL).**
`aux_rate` writes `divm1 = base/want − 1` unchecked (`slogic.c:414-419`). [HW] showed the hardware uses only 8 bits, so rates below 3.125 MHz silently run at `800/((divm1 & 0xFF)+1)`. For example, 1 MHz runs at 25 MHz. The tables happen to stop at 5 MHz.
**Correct: reject `divm1 > 255`.**

**B8. Samplerate latch failure is not detected (all drivers, firmware bug).**
See §6.3. About 10–20 % of starts run at the previous rate. No driver detects this.

**B9. Multi-device selection is impossible (DEV/PRE).**
`SR_CONF_CONN` returns "Not supported now!" (`api.c:119-127`), and TOOLS `TODO.md` lists it as an open bug. Scan opens every device just to read strings (`api.c:141-159`).
**Correct: select by bus/port or serial.**

**B10. Bogus vref confirm and missing error checks (PRE).**
The vref read-back is compared against a hard-coded `1024` (`api.c:1154`), so it always logs a failure.
Neither poll timeouts nor control-transfer errors are propagated:

* `slogic_usb_control_write`'s return values are ignored inside `remote_run`;
* channel and rate mismatch only `sr_dbg` (`api.c:1026-1031, 1101-1105`).

The rate loop `while (idx <= 1)` (`api.c:1054`) would, with a zero base, compute `div = 0` and write `divm1 = 0xFFFFFFFF`.
CORE fixed the vref compare and the base==0 case (`slogic.c:399-400`). **CORE still ignores the selector echo** in the header.

**B11. Combo 8 start-command stack over-read (PRE).**
`cmd_start_acquisition` is 3 bytes (packed), but `slogic_usb_control_write` rounds `len` up to 4 and sends 4 bytes from a 3-byte stack object (`api.c:659-672, 860-887`).
**Correct: send an explicit 4-byte `[mhz_lo, mhz_hi, nch, 0]`, as CORE does** (`slogic.c:505-509`). Sending exactly 3 bytes, as OLD did, also works.

**B12. Out-of-bounds read in `config_channel_set` (PRE).**
The loop runs to `samplerate_table_size` but indexes `samplechannel_table` (`api.c:554-559`). It was fixed in DEV by looping over `nchans` (`api.c:445-450`).

**B13. Use-after-close at `dev_close` (fixed in DEV `e7c1be07`).**
The libusb event thread was still inside `libusb_handle_events_timeout_completed()` while the handle was released and closed. It caused intermittent crashes, worse on macOS (DEV `api.c:285-296`).

**B14. Unbounded queue and allocation churn (DEV).**
Every completion `malloc`s a fresh `per_transfer_nbytes` buffer, which can be 25 MB, and pushes the old one onto an unbounded `GAsyncQueue` (`protocol.c:108-117`). The session callback consumes **one** buffer per invocation (`protocol.c:292-307`). Memory grows without bound if the consumer lags.
**Correct: a fixed buffer pool and bounded queueing**, which is BENCH's Phase-1 plan.

**B15. Spec versus code inconsistencies (SPEC/BENCH documentation bugs).**

* The AUX length is documented as "words" but is bytes.
* SPEC says "never writes RST" on capture start, but both drivers do.
* SPEC says "Selecting Normal on the U3 also issues an RST"; that was PRE-only behaviour.

**B16. Soft trigger only, with a fixed 10 % pre-trigger** (`protocol.c:558-571`).
Until the trigger fires, `samples_got_nbytes` is reset to 0 on every transfer (`protocol.c:120-121`). The hardware has no trigger.

---

## 11. Recommendations for visgrok's driver (derived)

1. Match by VID `0x359f` and PID ∈ {`0x3031`, `0x3032`, `0x0300`}. Report DFU PIDs `0x30f1`/`0x30f2` as "in bootloader" and never open them.
2. Open: SET_CONFIGURATION(1) if unconfigured; claim interface 0; RST pulse; read strings 1–3 only.
3. Before each capture: drain, STOP, RST, STOP, then AUX 1/2/3/5 with selector-echo, ready and read-back checks. Validate `divm1 ≤ 255`, `base ≠ 0`, and `nch × rate ≤ 3.2 G`. Then submit the ring, then RUN.
4. Ring: 16 transfers of 1–4 MiB (32 KiB-aligned). Resubmit straight from the completion callback and process on a separate thread with a bounded buffer pool.
5. Drop exactly 4 bytes per acquisition using a counter. Unpack per §5.
6. Verify the effective rate early (§6.3) and re-arm on mismatch.
7. Stop: STOP (retry after the ring drains if it fails), cancel, wait, drain.
8. Self-test mode: Emulation pattern (mode 2). Check `s[i] == (i&~7)|(7-(i&7))` after the 4-byte drop, allowing a short startup transient at high rates. This detects FIFO overflow and drops on the host.
9. Threshold: offer volts → code with the measured fit (`(V−0.4318)/0.005166`), and keep the nominal Sipeed formula as a compatibility option.

---

## 12. Hardware verification log (this Mac, 2026-10-08)

Probe source: `scratchpad/probe/src/main.rs` (rawusb path dependency). Device: SLogic16 U3, serial `202512191855`, bcdDevice `0x0002`, SuperSpeed.

| Test | Result |
|---|---|
| Descriptors / strings / BOS | As in §2. Strings > 3 and the device qualifier time out. |
| Active configuration on open | 0 (unconfigured). SET_CONFIGURATION(1) was required before claim. |
| REG_READ `0x00..0x1c` at open | `0x00010001`, CTRL `0x00000002` (reset held), the rest 0. |
| AUX with CTRL=2 | Ready bit never set (8 polls). |
| AUX after RST pulse | Headers `0x00010401` / `0x00011002` / `0x00010403` / `0x00010205`. Defaults: mask `ffff`, rate `00 00 20 03 0a 00 00 00`, vref `36 01`, pattern `00`. |
| Multi-word (8-byte) read | Returns the same word twice; no auto-increment. |
| Base table | idx0 = 800, idx1 = 800, idx2/3 = 0, index wraps mod 4. |
| Emulation 16/8/4ch @ 20 MHz, 4 MB | 4 zero bytes, then the pattern of §3.7, 0 discontinuities. Data rate 42/23/11 MB/s (paced; slight overshoot from buffered start). |
| Normal throughput | 16ch: 20→42.0, 50→101.8, 80→161.6, 100→201.8, 200→400.5 MB/s. 8ch@400→401.4. 4ch@100→52.1, 4ch@800→402.6 (when not hit by §6.3). |
| Divider width | 1 MHz (`0x31F`) → 25 MHz; 2 MHz (`0x18F`) → 5.55 MHz; 4 MHz (`199`) → 4 MHz. Only 8 bits are used. |
| Rate latch failure | Alternating 16ch 50/100 MHz, 16–20 runs per variant: 3–5 wrong runs without workaround; 1/20 with a dummy RUN/STOP. |
| USB-test pattern 16ch | Counter `+1` per sample, ~413 MB/s, 0 discontinuities even with synchronous single reads (flow-controlled). |
| Overflow | RUN then 1 s without reads: first 28 672 bytes continuous, then jumps. `0x0008` stays 0. |
| 400 MB/s sustained, 16 × 1 MiB | 327 discontinuities in 200 MB (≈ 700-sample gaps near offset 28 676 of transfers, plus 4-byte words displaced by +12288 B). 100 and 200 MB/s were clean. |
| STOP | No bytes after STOP; control writes during streaming were accepted (no BUSY seen). |
| Config persistence | Without RST, the previous capture's mask, divider, vref and pattern are retained. RST restores the defaults. |

---

## 13. Open questions / not verified

1. Meaning of register `0x0000` (`0x00010001`): firmware version or ID?
2. Purpose of bulk OUT endpoint `0x02` (unused by all known software).
3. Root cause of the samplerate latch failure (§6.3) and a reliable firmware-side fix. Does it also happen with sigrok-cli? It probably does, since the sequence is the same.
4. Whether the 400 MB/s drop and word-displacement behaviour (§6.1) is host-specific (macOS / rawusb transfer turnaround) or happens on Linux and libusb too. Sipeed's Linux baseline shows full captures at 800 MB/s on the 32U3 with sigrok-cli, but nobody checked continuity there.
5. Real input sampling correctness in reduced-channel modes on the 16U3. TOOLS `TODO.md` asks whether the 32U3's "grouped mode inputs read zero" bug also affects the 16U3 at 8ch@400 MHz. Only floating inputs were captured here (all zeros).
6. Threshold calibration: only one unit has been measured (WEB). Unit-to-unit spread is unknown. The DAC is assumed to be 10-bit.
7. Combo 8: the 4-byte head artifact, the 3- vs 4-byte CMD_START tolerance, and the stop behaviour are unverified (no device attached).
8. Whether non-integer-MHz rates (800/3, 800/6, …) sample accurately.
9. The 32U3's base table indices, and whether its divider is also 8-bit.
10. Behaviour at USB 2.0 High-Speed for the 16U3 (AL caps at 320 MHz/nch; not measured).
