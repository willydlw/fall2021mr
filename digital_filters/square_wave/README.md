# Noisy Square Wave Example

This [source code](noisy_square_wave.py) generates the following noisy square wave signal.

![Noisy square wave](./images/unfiltered.png "noisy square wave")
<br>
<br>

The noise and signal spectrum are shown below. The signal spectrum contains much of its energy at lower frequencies, with an amplitude much greater than the noise spectrum. Square wave signal have harmonics that are odd multiples of the fundamental frequency. The square wave frequency is 4 Hz, with harmonics at 12 Hz, 20 Hz, 28 Hz, etc. The harmonic content decays as 1/f. 

The white noise spectrum is fairly evenly spread across all harmonics. The much lower noise amplitude means we can filter out the noise, using a low pass filter.
<br>

![Energy Spectrum](./images/spectrum.png "energy spectrum")
<br>
<br>

Plotting the spectrum on a log scale, we look for the point where the noise level is above the signal amplitude. In the plots below, it looks like the noise energy rises above and separates from the signal energy at about 1000 Hz.

![Energy Spectrum, Log Scale](./images/full_spectrum_log.png "energy spectrum, log scale")
<br>
<br>


![Lower Spectrum, Log Scale](./images/spectrum_log.png "0-1000 Hz,, log scale")
<br>
<br>

Setting the cutoff frequency at 1000 Hz, and calculating &alpha; with the rough approximation: &alpha; = &Delta;t / &Tau;, the EMA filter produces the filtered signal shown below.<br>

>Note: The &alpha approximation is based on &alpha; = 1 - e<sup>-&Delta;t/&Tau;</sup>.
>
>The approximation is accurate for time constants where the time period is much longer than the time step, &Tau; >> &Delta;t<br>
>
>For fc = 1000 Hz, &Tau = 1 ms. The data was generated with &Delta;t = 15.26 microseconds<br>
<br>

![Filtered, fc=1000](./images/filtered.png "filtered, fc = 1000")
<br>
<br>

Lowering the cutoff frequency to 100 Hz, produces a distorted signal, due to the phase lag. Note that &alpha; is a much smaller value, slower to respond to changes in the signal. The signal does appear to have less noise, but at a cost.

![Filtered, fc=100](./images/filterfc100.png "filtered, fc = 100")
<br>
<br>

<p>Experiment with different cutoff frequencies. How is alpha affected? How is the filtered signal changed?<br></p>
 

### Fixed Point Implementation of the Low-Pass EMA Filter

<p>The formula y<sub>n</sub> =&alpha;x + (1-&alpha;)y<sub>n-1</sub> is written for floating point math. Many embedded processors do not have floating-point math, so it must be emulated with fixed point math. Both [1] and [2] contain a good discussion of a fixed point math implementation.<br></p>
<br>

**References**

Credit for this example goes to [Ten Little Algorithms, Part 2: The Single-Pole Low-Pass Filter][1]

[1]https://www.embeddedrelated.com/showarticle/779.php


[2]https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Exponential%20Moving%20Average/C++Implementation.html 
