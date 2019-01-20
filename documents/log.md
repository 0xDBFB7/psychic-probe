I'm not sure whether the offset resistors will affect the first stage. We'll have to see.

Plotting with mirrored top and small drill marks. 

Offset the board from the edge to stay within the page border.

- F./B. Mask + SVG + Do not tent vias + Not mirrored

SVG F and B output was scaled for some reason.

Pdf output document viewer -> page_setup -> orientation landscape.

Gerbers exported with aux axis

Export drills with aux axis and npth+pth in one file

<hr>

Imported into MeshCAM, set to mm,

1/16" tool, 1.58mm Margin 0.5mm 0.15mm gap sizex4

-1.5mm Z cut

7mm travel Z

300 mm/min feed

24krpm speed

% to prepend and append Gcode

Edited in 7s spin-up dwell after generation

<hr>

Imported gerber

No tool changes

24krpm

<hr>

Okay, now let's get rigorous.

Screwed down the board to the leveled MDF spoilboard.

Using 8560K171 acrylic from McMaster, 1/16", since the polycarbonate I was using previously did not seem  to transmit UV. Scoring and breaking this was quite effective.

The instructions recommend a 5-10 minute exposure. Let's go with 8 at 5 inches away.

Those milling and drilling parameters were perfect. Couldn't have gone better. Protective cover wasn't damaged in the slightest. Used the ShopVac for dust removal, which worked admirably. Edges were smooth and burr-free. I marked the endmill so it wouldn't be used for other purposes.

Exposed the first board for 7 minutes on each side at slightly over 6 inches under the closest light to the sink.

Immersed in developer and began brushing. It took several minutes for the image to develop - far longer than the 1 to 2 minutes suggested by M.G. Some of the resist appeared to remain. I erred on the side of caution for this board, and began etching with some resist still attached. The developer used in this test had been stored in a mason jar for several days - this may have contributed to its ineffectiveness.

The etchant was comprised of 30ml 9% peroxide (30 volume hair bleach) and 30ml muriatic acid. Foaming was immediately observed, turning the etchant into something resembling a forbidden suffle. 

There seems to be far too much resist remaining. Even several minutes in the developer was ineffective. The board is simply underexposed. I believe I will try putting it back under the UV for another few minutes. 

I exposed the second board for 10 minutes. 

The developer discolored significantly.

<hr>

Cut a test coupon out and exposed it gradually at 5 minutes per cm. I took care to put the board as on-axis with the light as I could. After 25 minutes (5cm) I developed it until the slightest patina of resist remained and inserted it into the etchant.

The 25 and 20 minute areas were well-exposed. A few minutes more may have been helpful; but on the whole, 25 minutes was perfect. The copper discolored after a few minutes in the etchant. 

<hr>

Exposed and etched the other boards. Too much photoresist remained, and the etch failed.

<hr>

Trying trace isolation milling again. Using some 1/16" tungsten rod as alignment pins.

4mm alignment pin drill depth. 0.2mm offset, -0.05 mm isolation milling, 600 mm/min, 24krpm.

Failure. Board excessively warped and Z axis not rigid enough. 

<hr>

Let's try toner transfer! 

I drilled and cut a controller out of 0.8mm PCB. There were some burrs on the underside from drilling. I sanded the board lightly with 600 grit - far too aggressive. Tried polishing it with Mothers mag and aluminium, which failed miserably. 

Used some brillo pad to bring it to a decent shine.  

Printed the layout on a very glossy photo paper.

https://hackaday.com/2016/09/12/take-your-pcbs-from-good-to-great-toner-transfer/ is an excellent resource.

Heated two pieces of scrap aluminium in the oven, then pressed them with the hydraulic press. Zero adhesion. It is quite difficult to heat the part while heating it.

https://www.instructables.com/id/Heatless-cold-Toner-Transfer-for-PCB-Making/

AHA! Someone ^ suggested applying 100% acetone to the top of the paper. I aligned the paper, soaked the top of the paper with nail polish remover, and pressed the sandwich hydraulically. This worked brilliantly, an almost perfect transfer! My aluminium press dies were pre-heated to ~120c - I'm not sure if that affected the results.  

Okay, that works disturbingly well, even with no heat at all. An eyedropper was used to apply a layer of acetone. Some areas did not release properly: I assume that this is due to an uneven coating of acetone. 

Again! Made a little boat out of paper, soaked in acetone for 60 seconds, and applied a few tons of pressure for 10 minutes.

Again! Applied acetone with a dropper, made sure there was an even coat, then sandwiched the board between two pieces of hardwood flooring, with an aluminum block at the top to spread the load a bit. I applied a considerable amount of pressure. 

Perfect success! Absolutely zero toner remained on the paper. Consistent pressure is the key. Removing the paper after just a few minutes didn't seem to have any great effect on the quality of the transfer.

The deposited layer is quite delicate. I may have to postprocess with an oven binding step. 

I held the paper in place with thin Kapton tape, for reference.

I find it helpful to crease the paper slightly after applying it to make it easier to see where the FR4 is located.

I flipped the board over and repeated the process for the other layer, adding a piece of the paper to the bottom surface to protect the newly set toner. 

The bottom did not adhere well. I may have misaligned the bottom. 

Tried again, really whaled away at the press this time. Complete success. Etched.

Success. The bottom had a much larger exposed area than the top, which caused the top to be over-etched. I could mask the top after a bit, but I just added a copper pour to the design and re-did it.

<hr>

I sent the board out to be produced professionally. The vias failed to conduct on the other board in this batch, and AP circuits reopened after the holidays.

<hr>

Board populated.

Using some boilerplate code from the capacitive sensor.

Retaining the ADC DMA code - interestingly, neither project has any DMA entries listed in STM32Cube. I guess I've forgotten how all that worked.

The optocoupler refused to function when powered by a CH340's 3v3 rail. Interesting! Taking power from the 5v bus works fine:

![DS1Z_QuickPrint17](../media/DS1Z_QuickPrint17.png)

Channel 1 and 2 are input and output respectively. The trailing edge of the signal is fascinating. I guess the UART pin goes high-z after it's done transmitting? I think you can also see the "knee" of the LED current curve. Very cool.

The VFD on my multimeter seems to still emit IR even when the standby button is pressed. 

The board consumes around 30 milliamps.

One of my CH340G boards was toast, hindering troubleshooting efforts. 

<hr>

The x10 side has a about 0.15v of noise before it's shifted. That should be okay?

<hr>

Enabling the "highlight" option on both pcbnew and eeschema allows cross-probing of schematic and board.

Note about ADC impedance

I made the gain of the offset amplifiers twice as high as it should be.

Mention having a spare pin is useful for timing

```
stty -F /dev/ttyUSB0 2000000
```

Holy crap! This thing has *zero* difficulty hitting 2 megabaud. I'm not seeing any corruption of any sort.

<hr>

Now let's discuss timestamping. If I know the sample rate precisely enough I can just send a row count.

Oh hey! I can just let the master computer do all the timestamping! This offloads even more processing onto the faster system. 

2 megabaud / (16 channels * 2 bytes per channel + checksum + 1 byte terminator)  = 58.823 khz - very reasonable. I could glean even more performance if I didn't abandon the extra 4 bytes per channel (16-12), but I really couldn't be bothered.

The top bit of each channel can be used to indicate the scale factor.

This is becoming a regular occurrence

Design goals:

- cheap, so that I'm not hugely concerned if it explodes
- only requires a day or two to design and program - I'm going to university, so I don't have copious free time at the moment.



It would convenient for calibration data to reside on the probe itself.
On the other hand, any changes to the calibration would require a JTAG programmer.

If I put the scale indicator on the second-to-last bit of the high byte, I can make the terminator FFFFFFFF without fear of collision, since the high byte of each channel can never be all 1s.

I contemplated putting most of the logic in the ADC interrupt and double buffering.

The ADC interrupt breaks at ADC_SAMPLETIME_1CYCLE_5 for some reason. 

| Function                          | Time               |
| --------------------------------- | ------------------ |
| select_mux_channel(0) -Os         | 65 cycles, 1.35 us |
| select_mux_channel(0) or (16) -O3 | 9 cycles, 0.187 us |
| Complete buffer fill, 16 channels |                    |

We have ~30 cycles to work with before 



Almost always takes 1358, but sometimes takes 1437.