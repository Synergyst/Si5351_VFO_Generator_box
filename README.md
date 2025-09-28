# Si5351 VFO + RP2040 Zero USB‑C Mod (for common "Mobile SDR QRP HF Transceiver" variants)

Overview
- Many Si5351 VFO generator modules sold on Amazon/eBay/AliExpress share the same internals: an SSD1306 OLED front panel and a Si5351 clock generator, driven by an Arduino Nano‑328-class MCU.
- This project swaps control of the OLED and adds a WaveShare RP2040 Zero for USB‑C, keeping the rest of the radio/VFO hardware intact and allowing the modification to be fully reversible (only if you follow the recommended instructions).
- Rationale: the Nano’s limited SRAM (~2 KB) makes the planned UI/feature set tight; moving the UI/I2C control to RP2040 Zero avoids RAM constraints and adds ability to add further modifications otherwise not possible due to SRAM-constraints.

Important cautions
- Remove 3.7V Li-Po cell from Nano PCB BEFORE modifications! I personally flipped the cell so it is adhered to the bottom of the Nano PCB so I can slide the PCB in/out easier in the future ever.
- Logic levels: the RP2040 Zero and Nano is 3.3V only! Do not feed 5V signals into RP2040 Zero or Nano pins!
- ESD and shorts: heatshrink any "hanging" or clipped conductors and double‑check clearances before powering on. Modifications will be tight and adjacent to dangerous voltage differentials for each board. Please thoroughly check the voltages and pins and insulate the WaveShare RP2040 Zero thoroughly before you try to plug the 3.7V Li-Po cell back into the Nano-PCB, or plug in any USB-C cable!

Compatibility note
- If your unit has a SSD1306 OLED and a Si5351 clock generator, it is almost certainly the same platform regardless of listing source or branding, and these instructions should apply with minor mechanical variations.

What you'll do
- Make a small case mod to mount the WaveShare RP2040 Zero and expose its USB‑C port (I cut a slot near the ICSP header of the Nano MCU board after soldering the required 6-pins. This will keep the RP2040 Zero sitting behind the rotary encoder and buttons. I soldered the pins so I needed to keep the BOOT and RESET buttons facing down to the bottom of the enclosure, this was a mistake since the RP2040 Zero has a programmable WS2812 RGB LED that would've been nice for diagnosing errors/states in the future).
- Isolate the OLED's I2C (SDA/SCL) from the stock front‑panel/Nano and rewire those lines to the RP2040 Zero (this will require unlatching the SDA and SCL wires from the cable going to the front-panel! Please heatshrink the hanging SDA and SCL wires that are from the Nano-side so they do not short!).
- Bridge a pair of RP2040 pins to the Nano's D11/D12 (also available on the stock PCB's ICSP header) per the wiring notes below.

Bill of materials
- WaveShare RP2040 Zero (USB‑C)
- Thin insulated wire (I've used magnet wire with the last VFO mod I did, it works fine too if you want a less-reversible/more robust and permanent modification), narrow heatshrink tubing
- Optional: Small flat thin file
- Dremel for the USB-C port cutout/case mod
- A trusty black marker to hide the modification ;)

Wiring and hardware notes
- SAFETY:
  - Remove 3.7V Li-Po cell from Nano PCB BEFORE modifications!
  - I personally flipped the cell so it is adhered to the bottom of the Nano PCB so I can slide the PCB in/out easier in the future ever.
  - MAKE SURE THE USB-C AND LI-PO CABLES ARE ALWAYS DISCONNECTED UNTIL YOU FINISH READING AND FOLLOWING ALL STEPS OF THIS GUIDE.
  - PROCEED AT YOUR OWN RISK, I HOLD NO LIABILITY.

- Case mod:
  - Create a minimal cutout to mount the RP2040 Zero so its USB‑C is accessible. Ensure strain relief and that no metal shields/edges can short the boards/adjacent pins.

- Isolate the OLED I2C from the stock front panel:
  - Unlatch the SDA and SCL pins from the front‑panel header so they no longer connect to the Nano side.
  - Do not clip the header if you can avoid it! If you must clip, insulate the now‑loose wires and header pins with heatshrink like you would either way. The goal is: OLED I2C is no longer electrically tied to the Nano/front‑panel bus.

- Rewire SSD1306 OLED I2C to the RP2040 Zero:
  - RP2040 Zero GP4 → OLED SDA
  - RP2040 Zero GP5 → OLED SCL

- Bridge RP2040 Zero to Nano ICSP lines (Nano D11 and D12 are also broken out on the Nano‑side ICSP header):
  - RP2040 Zero GP0 → Nano D11 (ICSP pin 4 = D11 (MOSI))
  - RP2040 Zero GP1 → Nano D12 (ICSP pin 1 = D12 (MISO))
  - You can solder to ICSP pins 4 and 1 if that's more convenient. Keep leads short and insulated. I personally just made and used some short Dupont-terminated jumpers.

Product reference (this is what I used, please find the best price yourself via your preferred supplier):
- Amazon listing: https://www.amazon.com/dp/B0CB1RQKTQ
- Product image: ![Si5351 VFO generator](https://m.media-amazon.com/images/I/61U2V2WqoSL._SL1500_.jpg)

Search keywords (typical marketplace wording used on eBay, AliExpress, etc):
- Mobile SDR Qrp HF Transceiver Qrp Transceiver, VfoVFO Functional RF Generator with USB Cable 10K‑220MHz Si5351
