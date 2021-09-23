# Digital Low-Pass IIR Filter based on Analog Passive RC Low-Pass Filter

## Analog Passive RC Low-Pass Filter

![RC Filter](./images/rc-low-pass-filter.jpg "Low-pass RC Filter")

[Image 1: Low-pass RC Filter][1]

[1]: https://ecstudiosystems.com/discover/textbooks/basic-electronics/filters/images/rc-low-pass-filter.jpg
</br>

## Behavioral Circuit Analysis

The opposition to the flow of alternating current due to capacitance is called "capacitive reactance." The capacitive reactance of the capacitor in the circuit is X<sub>C</sub> = 1 / (2 &pi; fC), units: Ohms

Mathematical circuit analysis, using a voltage divider relationship

V<sub>out</sub> = (-jX<sub>C</sub> / (R=jX<sub>C</sub>)) V<sub>in</sub>  

The ratio of V<sub>out</sub> / V<sub>in</sub> is called the transfer function. It provides the gain/attenuation relationship between the input and output voltage.

G(j&omega;) =  V<sub>out</sub> / V<sub>in</sub> = (1/(j&omega;C)) / (R + 1/(j&omega;C))

G(j&omega;) = 1 / (1 + j&omega;/&omega;<sub>c</sub>)

where &omega;<sub>c</sub> = 1/(RC) and is called the characteristic frequency. </br>

The cutoff frequency in hertz form is given by f<sub>-3dB</sub> = 1/(2&pi RC)
</br></br>

### Polar Form of Transfer function

G(j&omega;) = 1/sqrt(1+(&omega;/&omega;<sub>c</sub>)<sup>2</sup>) < -arctan(&omega;&omega;<sub>c</sub>)

Capacitor reactance varies inversely with frequency, while the resistor remains constant as frequency changes. At low frequencies, X<sub>C</sub> will be large compared to the resistive value of the resistor.
</br></br>

### Frequency Response

Bode plots are used to display the frequency characteristics of a filter. The magnitude of the transfer function (amplitude characteristic) versus frequency is plotted on one curve and the phase characteristic as a separate curve but with the same frequency axis. The amplitude characteristic, which may vary over a wide range, can be conveniently plotted in terms of decibels. 
</br></br>

#### Amplitude 

The filtered signal's amplitude is the absolute value (magnitude) of the transfer function,

G(&omega;) = 1 / sqrt( 1 + (&omega;/&omega;<sub>c</sub>)<sup>2</sup>)

In decibels,

G<sub>dB</sub> = 20 log<sub>10</sub> G(&omega;)

G<sub>dB</sub> = 20 log<sub>10</sub> (1 / sqrt( 1 + (&omega;/&omega;<sub>c</sub>)<sup>2</sup>))

G<sub>dB</sub> = -10 log<sub>10</sub> [ 1 + (&omega;/&omega;<sub>c</sub>)<sup>2</sup>)] </br></br>

From this relationship, we see that for low frequencies, &omega;/&omega;<sub>c</sub> << 1, G<sub>dB</sub> = -10 log<sub>10</sub> 1 = 0 

The low-frequency behavior is essentially independent of frequency and can be represented by a horizontal straight line at 0 dB as in the figure below. The actual amplitude characteristic given by the transfer function is asymptotic to this straight line for small &omega;.
</br></br>

![Amplitude Characteristic](./images/amplitude-characteristic-line.jpg) [3](https://ecstudiosystems.com/discover/textbooks/basic-electronics/filters/images/amplitude-characteristic-line.jpg)
</br></br>

For high frequencies, &omega;/&omega;<sub>c</sub> << 1, G<sub>dB</sub> = -10 log<sub>10</sub> (&omega;/&omega;<sub>c</sub>)<sup>2</sup>) = -20 log<sub>10</sub>(&omega;/&omega;<sub>c</sub>) </br></br>

This is of the form GdB = -20x, where x = log10 (&omega;/&omega;<sub>c</sub>). The straight line so defined is the high-frequency asymptote of the actual characteristic. The slope of the asymptote is dG<sub>dB</sub>/dx = -20; that is, when x increases one unit, G<sub>dB</sub> decreases by 20 dB. Therefore, the slope of the high-frequency asymptote is -20 dB per decade.

The characteristic frequency (aka breakpoint or cutoff) occurs where the two straight-line asymptotes intersect at at &omega;/&omega;<sub>c</sub> = 1. Together, the two asymptotes form a broken-line approximation to the actual characteristic. Depending upon the accuracy desired, neither line may be a sufficiently good approximation to the actual characteristic in the neighborhood of ω = ωC. It can be shown that the maximum error occurs at ω/ωC = 1 and is approximately 3 dB.
</br></br>

#### Phase Angle

The phase angle of the transfer function of the filter is given by

&theta; = -arctan(&omega;/&omega;<sub>c</sub>)

The phase angle starts at zero for ω = 0 and approaches -π/2 radians at large ω. 

![Phase characteristic](./images/phase-characteristic-line.jpg) [4](https://ecstudiosystems.com/discover/textbooks/basic-electronics/filters/images/phase-characteristic-line.jpg)
</br></br>

## Time-Domain Analysis

Using Kirchhoff's Laws and the definition of capacitance, the current through the resistor and the current through the capacitor are the same when there is no load on the circuit.

1) (V<sub>out</sub>(t)- V<sub>in</sub>(t))/R =  C dV<sub>out</sub>(t)/(dt)

Rearrange to differential equation form
2) V<sub>out</sub>(t) + RC dV<sub>out</sub>(t)/(dt) = V<sub>in</sub>(t)
</br>

We need to move from differential equation (continuous time) to difference equation (discrete time). Here, we use the backward Euler method for simplicity. For the best frequency domain match from analog to digital, use Tustin's method. Typically, we do not need to match an exact frequency for filtering our sensors, so the backward Euler method is preferred due to simpler computational requirements.

We approximate continuous-time derivatives by

dV/dt = (V[n] - V[n-1])/ T

where V[n] is a sample at index n and T is the sampling time in seconds. 

This relationship comes from the Taylor series. 

Discrete form of the differential equation:

3) V<sub>out</sub>[n] + RC ((V[n] - V[n-1])/ T) = V<sub>in</sub>[n]
</br>

Rearranging the difference equation:

4) V<sub>out</sub>[n] = (T / (T+RC)) V<sub>in</sub>[n] + (RC/(T+RC)) V<sub>out</sub>[n-1]
</br></br>

## General Discretize Time Domain Equations

Assume that input and output are sampled at evenly spaced points in time, separated by &Delta;t time. Let the samples of v<sub>in</sub> be represented by the sequence (x<sub>1</sub>, x<sub>2</sub>, x<sub>3</sub>, ..., x<sub>n</sub>). Let v<sub>out</sub> be represented by the sequence (y<sub>1</sub>, y<sub>2</sub>, y<sub>3</sub>, ..., y<sub>n</sub>).
</br>

Subsituting x, y gives the recurrence relation

5) y<sub>i</sub> = x<sub>i</sub> (&Delta;t)/(RC + &Delta;t) + y<sub>i-1</sub> (RC)/(RC + &Delta;t)
</br></br>

Let &alpha; = (&Delta;t)/(RC + &Delta;t).

The discrete time implementation of the RC low-pass filter is

6) y<sub>i</sub> = &alpha; x<sub>i</sub> + (1-&alpha;) y<sub>i-1</sub></br></br>

Recognize that this is in the form of an infinite impulse response filter. It is an exponentially weighted moving average. The &alpha; value is an exponential smoothing factor, whose function is to remove high-frequency noise.

By definition, the smoothing factor, &alpha; is subject to 0 <= &alpha; <= 1

Larger values of &alpha; reduce the level of smoothing. Values of &alpha; close to 1 give greater weight to recent changes in data, while values of &alpha; closer to zero have a greater smoothing effect and are less responsive to recent changes. There is no formally correct procedure for choosing &alpha; </br>
</br>

The time constant RC is 

7) RC = &Delta;t ((1-&alpha;)/&alpha;)log<sub>10</sub> (&omega;/&omega;<sub>c</sub>)<sup>2</sup>) = -20 
</br>

The cutoff frequency f<sub>c</sub> is

8) f<sub>c</sub> = 1 / (2 &pi; RC)</br>

The cutoff frequency f<sub>c</sub> is

9) f<sub>c</sub> = 1 / (2 &pi; RC)</br>

Rearranging equation 9,

10) RC = 1 / (2 &pi; f<sub>c</sub>)</br>

Making &alpha; and f<sub>c</sub> related by

11) &alpha; = (2 &pi; &Delta;t f<sub>c</sub>) / (2 &pi; &Delta;t f<sub>c</sub> + 1)

12) f<sub>c</sub> = &alpha; / ( (1-&alpha;) 2&pi;&Delta;t)</br></br>

### Sampling Time

The sampling time T must be sufficiently small to ensure an accurate match between analog and digital domains. T must be chosen to ensure an adequate Nyquist limit.

Example: Sampling at 100 Hz, T = 1/100 sec, the maximum frequency that can be filtered is 50 Hz. Signals from 0 Hz - 50 Hz may be filtered.


### IIR Filter Stability

IIR filters can become unstable for certain values of filter coefficients. For this first order filter, keeping &alpha; in the range [0,1] will ensure output signal stability.
