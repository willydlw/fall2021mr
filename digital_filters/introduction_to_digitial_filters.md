# Introduction to Digital Filters

In signal processing, a filter is used to remove undesirable parts of the signal. Its purpose may be the removal of random noise or to extract parts of the signal within a frequency range.

There are two main types of filters: analog and digital. Analog filters use electronic circuit components (resistors, capacitors, op amps) to produce the filtered signal. The analog signal is a voltage or current.

Digital filters use a digital processor to perform numerical calculations on sampled values of the signal. An analog input signal is sampled and digitized using an ADC (analog to digital converter). Successive sampled values produce a binary number representation of the input signal, which is sent to the digital processor. The digital processor performs numerical calculations to produce the digitally filtered signal. If an analog output is required, a DAC (digital to analog converter) converts the signal back to analog form.

<br>

![dsp system](./images/dspsystem.gif "DSP System")

[Image 1: DSP System][1]

[1]: https://www.networxsecurity.org/fileadmin/user_upload/images/2015-08/dsp.system.gif
<br>
<br>

## Digital Filter Advantages

1. A digital filter is programmable. It can easily be changed without affecting the hardware circuitry. Analog filters can only be changed by redesigning the filter circuit.
2. Digital filters are easily, designed, tested, and implemented on a computer.
3. Digital filters are more stable than analog circuits, with respect to time and temperature.
4. Complex combinations of filters in parallel or cascade are easier to implement digitally, compared to the analog circuitry requirements.
<br>
<br>

## Digital Filter Operation

Let's define the "raw" signal to be filtered as a voltage waveform described by the function *V = x(t)* where *t* is time.<br>

The voltage signal is sampled at time intervals **&Delta;t**. 
<br>
The sampled time value is 
<p align="center">
    <b>x<sub>i</sub> = x(i &Delta;t)</b><br>
</p>

<br>
The digital sequence transferred from the ADC to the processor is represented by the sequence 
<p align="center">
    <b>x<sub>0</sub>, x<sub>1</sub>, x<sub>2</sub>, x<sub>3</sub>,...</b><br>
</p>
corresponding to the values of the signal at time 
<p align="center">
    <b>t = 0, &Delta; t, 2&Delta;t, 3&Delta;t, ... </b><br>
</p>
t = 0 is the time sampling begins.

<br>

At time t = n&Delta;t, where n is a positive integer, the sampled input values are x<sub>0</sub>, x<sub>1</sub>, x<sub>2</sub>, x<sub>3</sub>, ..., x<sub>n</sub>.

Over the time interval 0 to n, the digital outputs y<sub>0</sub>, y<sub>1</sub>, y<sub>2</sub>, y<sub>3</sub>, ..., y<sub>n</sub>

The digital processor output y<sub>n</sub> is calculated from the values x<sub>0</sub>, x<sub>1</sub>, x<sub>2</sub>, x<sub>3</sub>, ..., x<sub>n</sub>. The calculated y values depend on the digital filter implementation.
<br>
<br>

## Simple Digital Filter Examples

1. **Simple gain filter**:
<p align="center">
    <b>y<sub>n</sub> = Kx<sub>n</sub></b><br><br>
    where K is a constant gain factor. K > 1 makes the filter an amplifier, while 0 < K < 1 makes the filter an attenuator.<br>
</p>
<br>

2. **Pure delay filter**: The output value at time t = n&Delta;t is the input at time t = (n-1)&Delta;t<br>

<p align="center">
    <b>y<sub>n</sub> = x<sub>n-1</sub></b><br>
</p>
<br>

3. **Two term difference filter**: The output value at time t = n&Delta;t is equal to the difference between the current input and the previous input. This is similar to an analog differentiator circuit. <br>

<p align="center">
    <b>y<sub>n</sub> = x<sub>n</sub> - x<sub>n-1</sub></b><br>
    
</p>
<br>

4. **Two term average filter**: The output of the current input and the previous input. This is a simple type of low pass filter as it tends to smooth out high-frequency variations in a signal.<br>

<p align="center">
    <b>y<sub>n</sub> = (x<sub>n</sub> + x<sub>n-1</sub>)/2</b><br>
</p>
<br>
<br>

## Digital Filter Order

The order of a digital filter is the number of previous inputs (stored in the processor's memory) used to calculate the current output. Example 1, the simple gain filter is a zero order filter. The current output y<sub>n</sub> depends only on the current input x<sub>n</sub>. The pure delay filter, two term difference filter, and two-term average filter are first order filters because y<sub>n</sub> depends on the current input x<sub>n</sub> and the previous input x<sub>n-1</sub>.

Filters may be of any order from zero upwards.
<br>
<br>

## Digital Filter Coefficients

Digital filters can be written in these general forms:

<p align="center">
    <b>Zero order: y<sub>n</sub> = a<sub>0</sub> x<sub>n</sub> </b><br>
    <b>First order: y<sub>n</sub> = a<sub>0</sub> x<sub>n</sub> + a<sub>1</sub> x<sub>n-1</sub> </b><br>
    <b>Second order: y<sub>n</sub> = a<sub>0</sub> x<sub>n</sub> + a<sub>1</sub> x<sub>n-1</sub> + a<sub>2</sub> x<sub>n-2</sub> </b><br>
</p>

The constants a<sub>0</sub>, a<sub>1</sub>, a<sub>2</sub>, ... are called the filter coefficients.
<br>
<br>

## Recursive and non-recursive filters

The four examples above are non-recursive filters. The current output y<sub>n</sub> is calculated only from the current and previous input values (x<sub>n</sub>, x<sub>n-1</sub>, x<sub>n-2</sub>, ...)

A recursive filter's current output value y<sub>n</sub> is calculated from both input values and previously calculated output values. These previous output values are stored in the processor's memory.

The term recursive refers to the "going back" to previously calcuated output values, y<sub>n-1</sub>, y<sub>n-2</sub>, ...

In general, recursive filters require lower order filters to achieve the desired frequency response than non-recursive filters. Lower order filters translate to fewer calculations required by the processor.

>Note: Non-recursive filters are known as FIR (Finite Impulse Response) filters, while recursive filters are called IIR (Infinite Impulse Response) filters.
>
> The terms FIR and IIR refer to their differing impulse respones. The digital filter's impulse response is the filter's output sequence when a *unit impulse* is applied at its input. A unit impulse is a simple input sequence consisting of a single value 1 at time *t = 0*, followed by all zeros at subsequent sampling instants.
>
>A FIR filter's impulse response is of finite duration. An IIR filter's impulse response theoretically continues forever because the previous output terms feed energy back into the filter input. An IIR's actual impulse response reduces virtually to zero in a finite time.
<br>
<br>

## General Recursive Filter Form



## Simple Recursive Filter Example

The simple recursive digital filter represented as

<p align="center">
    <b>y<sub>n</sub> = a<sub>0</sub>x<sub>n</sub> + y<sub>n-1</sub> </b><br>
</p>

The current ouput y<sub>n</sub> is the sum of the current input x<sub>n</sub> and the previous output y<sub>n-1</sub>.

At time *t=0*, y<sub>-1</sub> is undefined and usually assumed to be zero.

Over time, the filter is the sum of the current input and all the previous inputs. This filter is the digital equivalent of an analog integrator circuit.

<p align="center">
    <b>y<sub>0</sub> = x<sub>0</sub> + y<sub>-1</sub> =  x<sub>0</sub> </b><br>
    <b>y<sub>1</sub> = x<sub>1</sub> + y<sub>0</sub> =  x<sub>1</sub> + x<sub>0</sub> </b><br>
    <b>y<sub>2</sub> = x<sub>2</sub> + y<sub>1</sub> =  x<sub>2</sub> + x<sub>1</sub> + x<sub>0</sub> </b><br>
</p>

The recursive filter output is based on the current input value and the previous filter output value. The non-recursive output is based only on current and previous input values. In this general form, these formulas appear to be the same algebraically. As we further explore filter design, we will see that the input and output values have differing coefficients that weight the input and output values at various time steps.



<p align="center">
    <b>Recursive</b>: y<sub>5</sub> = a<sub>0</sub>x<sub>5</sub> + b<sub>1</sub>y<sub>4</sub><br>
    <b>Non-recurseive</b>: y<sub>5</sub> = a<sub>0</sub>x<sub>5</sub> + a<sub>1</sub>x<sub>4</sub> +  a<sub>2</sub>x<sub>3</sub> + a<sub>3</sub>x<sub>2</sub> + a<sub>4</sub>x<sub>1</sub> + a<sub>5</sub>x<sub>0</sub><br>
</p>
<br>
<br>

### Recursive (IIR) Filter Order

The order of a recursive filter is the largest number of previous input or output values required to compute the current output.

The simple recursive example above is classified as first order because it uses one previous output value y<sub>n-1</sub>.
<br>
<br>

### Coefficients of recursive (IIR) digital filters

The general symmetrical form of a first-order recursive filter is 
<p align="center">
    <b>b<sub>0</sub> y<sub>n</sub> + b<sub>1</sub> y<sub>n-1</sub>= a<sub>0</sub>x<sub>n</sub> + a<sub>1</sub> x<sub>n-1</sub> </b><br>
</p>

Solving for y<sub>n</sub>, the general form is

<p align="center">
    <b>y<sub>n</sub> = (a<sub>0</sub>x<sub>n</sub> + a<sub>1</sub> x<sub>n-1</sub> - b<sub>1</sub> y<sub>n-1</sub>) / (b<sub>0</sub>) </b><br>
</p>
<br>
<br>

## Transfer Function 

The digital filter transfer function is obtained from the symmetrical form of the general equation, with all the output terms on one side of the equation and all the input terms on the other side.

The delay operator, z<sup>-1</sup>, is applied to the equation. When applied to a sequence of digital values, this operator gives us the previous value in the sequence. Effectively, it introduces a delay of one sampling interval.

Applying a delay to x<sub>n</sub> gives the previous input x<sub>n-1</sub>.

<p align="center">
    <b>z<sup>-1</sup>x<sub>n</sub> = x<sub>n-1</sub></b><br>
</p>

Using the delay operator to describe a recursive digital filter,

<p align="center">
    <b>b<sub>0</sub> y<sub>n</sub> + b<sub>1</sub> y<sub>n-1</sub>= a<sub>0</sub>x<sub>n</sub> + a<sub>1</sub> x<sub>n-1</sub> </b><br>
</p>

becomes

<p align="center">
    <b>b<sub>0</sub> y<sub>n</sub> + b<sub>1</sub>z<sup>-1</sup>y<sub>n</sub> + b<sub>2</sub> z<sup>-2</sup>y<sub>n</sub> = a<sub>0</sub>x<sub>n</sub> + a<sub>1</sub> z<sup>-1</sup>x<sub>n</sub> + a<sub>2</sub> z<sup>-2</sup>x<sub>n</sub>
    </b><br>
</p>

Factoring and rearranging provides the relationship between the ouput and input,

<p align="center">
    <b>
    y<sub>n</sub> / x<sub>n</sub> = (a<sub>0</sub> + a<sub>1</sub> z<sup>-1</sup> + a<sub>2</sub> z<sup>-2</sup>) / (b<sub>0</sub> + b<sub>1</sub>z<sup>-1</sup> + b<sub>2</sub> z<sup>-2</sup>)
    </b><br>
</p>
<br>
This is the general form of the transfer function for a second-order recursive (IIR) filter.

For higher order filters, further terms of z<sup>-1</sup> are added. For a first order filter, the terms in z<sup>-2</sup> are omitted.
<br>

A non-recursive (FIR) filter has a simpler transfer function. The second-order FIR filter's general form is

<p align="center">
    <b>
    y<sub>n</sub> / x<sub>n</sub> = a<sub>0</sub> + a<sub>1</sub> z<sup>-1</sup> + a<sub>2</sub> z<sup>-2</sup> / (b<sub>0</sub>)
    </b><br>
</p>
