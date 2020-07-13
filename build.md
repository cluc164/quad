# Quadcopter Build

I saw a video on how quadcopters worked and it inspired me to really learn the science behind quadcopters, how they're constructed, and to try to build my own! The first step was to essentially read as much as possible: watching YouTube videos, reading research papers, relearning physics...

I wanted a challenge, but I also didn't want to be completely in over my head. Luckily, people have built quadcopters before and I was able to find a guy on YouTube who had built one and provided both source code and a schematic! But I didn't want to be boring and just yoink someone else's code for a project like this without understanding it... So I yoinked his schematic, bought all the parts, and began learning how to program flight control software. 

## Transmitter

For some reason, I figured that starting with the thing that allowed me to actually control the quadcopter was a good idea! I bought an FS-T6 transmitter/receiver combo, and from reading the manual I knew that controls were linked with certain channels; e.g. the right stick's x-axis (roll) corresponded with channel 1 on the receiver. I connected channels 1-4 to the Arduino's I/O pins 8-11 via jumper cables, and set to work.

At first, I didn't understand how the state of the transmitter was communicated at all, so I just tried to set the pins that I had connected to the receiver to digital input, and from there simply digitalRead() the values; easy, right? Well, not really. 

Turns out we've thought up some really, really clever ways to communicate information. Transmitters and receivers for RC control make use of a concept called Pulse-Width Modulation (PWM) that allows the transmitter to communicate data about its state using digital pulses of a certain duration. Each of these signals is transmitted via a single channel as discussed above. Reading the length of these pulses gives us the information about the position of the control: when the stick is pushed all the way to the left the duration of its pulse is 1ms; pushed to the right it's 2ms; in the middle it's 1.5ms. (these will be measured in microseconds, so 1000us = stick left, 1500us = stick middle, 2000us = stick right)

<img src="pictures\PulseDuration.png" style="zoom:50%; float:left;" />

So, calling digitalRead() on the pin was wrong, and I somehow needed to measure the pulse length of four signals... I googled it and found [this](https://ryanboland.com/blog/reading-rc-receiver-values), which told me I needed to find a way to listen to state changes on each I/O pin corresponding to a channel from the receiver. Arduino provides an `attachInterrupt()` method, which listens to the state of a pin and triggers a callback based on that; however, it's only implemented for pins 2 & 3. I needed four channels, which meant 4 pins. More Google and I found resource (1), which showed me how to interact directly with the Atmega328p microcontroller on the Arduino board.

<img src="pictures\pcicr.jpg" style="zoom:50%; float:left;" /> 

<img src="pictures\pcmsk0.jpg" style="zoom:50%; float:left;" />

By setting the PCICR within the Atmega (pin change interrupt control register) to 0b00000001, it allows external interrupts on a range of registers (PCINT0-PCINT7). Of interest to us are PCINT0-PCINT3, which are the registers corresponding to digital pins 8-11. To define which pins trigger an interrupt, we set PCMSK0 to 0x0F, which enables interrupts for state changes on registers PCINT0-PCINT3. Now, whenever the state of one of these pins is changed from high to low or vice versa, the microcontroller will trigger a software response called an Interrupt Service Routine (ISR) where I can run logic and do something useful, like read the state of a register! What I have at this point is a program that listens for state changes on I/O pins 8-11 and when that occurs, a function called `ISR(PCINT0_vect) {}` is called.  

We want to measure the time from when the signal goes high to when it goes back low. First, I started with a single channel: when the ISR is triggered, read the least significant bit of PINB (more info in resource (2); address that allows direct read access to PORTB, which is the register corresponding to pin 8). If this bit is 1, then the state is high and we know the state just went from low to high (rising edge) so we can record the current time. Now, we just wait until the state falls from high to low, and record the difference between the time at that point and the time recorded at the rising edge. This gives us the pulse length for a single channel.

Excited that I'd correctly obtained data from the transmitter, I copy and pasted the code and edited it to look at the data from all 4 channels and rather than getting nice, clean numbers between 1000 and 2000, I got this data where each column corresponds to a channel's input. 

 <img src="pictures\unstable.png" style="zoom:50%;" />

Looking into this, you can notice that some columns do actually get nice data once and awhile, but it's inconsistent. I realized that in some cases, it's entirely possible that one pin's interrupt could occur during the time in which another pin has already been high for a certain amount of time. Blindly recording the time works for a single channel when we can guarantee the ISR is triggered on a rising or falling edge; this will not work when there are 4 pins that can each individually trigger the ISR at different times. To fix this, simply add in a state machine that verifies whether or not we have already detected the rising edge for a specific pulse. This ensures that only the rising/falling edge is detected for each channel, and the pulse lengths are recorded properly. After adding in these checks for each channel, the data columns are much nicer looking, albeit minutely noisy. 

<img src="pictures\stable.png" style="zoom:50%; float: left" />

At this point, I have stable data that corresponds to the position of each control stick on the transmitter. All I will do is absolutely ensure that the values are within a certain range via mapping.

# Resources

(1) [External Interrupts on Arduino](https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328)

(2) [Atmega328p Documentation](http://ee-classes.usc.edu/ee459/library/documents/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf)

(2) [Port Information](https://web.ics.purdue.edu/~jricha14/Port_Stuff/)

(3) [FS-T6 Manual](https://fccid.io/N4ZFLYSKYT6/User-Manual/User-manual-1740934.pdf)

(4) [Information about PWM](https://oscarliang.com/pwm-ppm-difference-conversion)