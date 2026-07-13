# **3D Radar Golf Launch Monitor: Multi-Antenna Phase Interferometer Architecture**

A custom, low-cost bistatic radar framework for tracking high-speed golf ball trajectories, calculating velocity, vertical launch angle (VLA), and horizontal deviation angle (HLA) simultaneously without oscillator drift.

## **👶 ELI5: Explain Like I'm 5**

Imagine you are standing in a completely pitch-black gym, and you want to track a flying tennis ball.

1. **The Spotlight (OPS243):** We turn on one giant, invisible microwave spotlight. It shines a constant beam of energy out into the room. It doesn't listen or do math; it just floods the room with "light" (radio waves).  
2. **The Ball:** When you hit a golf ball through that spotlight, the ball glows brightly in microwaves and bounces the light back toward us. Because the ball is moving fast, the color of the bounced light shifts slightly (this is the Doppler effect, like how an ambulance siren changes pitch as it drives past).  
3. **The Three Ears (K-LC2s):** We set up three passive listening ears arranged in the shape of an "L". They don't make any light of their own; they just listen.  
4. **The Magic Trick (RF Phase Locking):** Because the ears are sitting right next to the giant spotlight, they can "hear" the spotlight directly through the air *and* "hear" the reflection bouncing off the ball at the exact same time. By mixing them together right at the ear, any tiny wobbles or flickering from the spotlight are erased instantly.  
5. **The Brain (Raspberry Pi & Octo HAT):** A specialized multi-lane sound card records what all three ears hear at the exact same microsecond. A computer script looks at the tiny time delays (phase shifts) between when the echo hits the **Corner Ear** vs. the **Top Ear** (telling us if the ball went up or down) and the **Corner Ear** vs. the **Right Ear** (telling us if the ball went left or right).

## **🏗️ System Architecture & Data Flow**

```
                      +-----------------------------+  
                      |   OmniPreSense OPS243       |  
                      |  (Continuous 24 GHz Tx)     |  
                      +--------------+--------------+  
                                     |  
                                     v (Microwave Spotlight)  
                               (( 24 GHz ))  
                                     |  
                               [ GOLF BALL ]  
                              (Flight & Spin)  
                                     |  
             +-----------------------+-----------------------+  
             | (Echo Reflection)                             | (Direct Air Leakage)  
             v                                               v  
+---------------------------------------------------------------------------+  
| PASSIVE RECEIVER ARRAY (L-Shape Topology)                                 |  
|                                                                           |  
|   [ K-LC2 SENSOR 1: TOP ]                                                 |  
|     (Output: I1 / Q1 Baseband)                                            |  
|                |                                                          |  
|                | 6.25 mm (Vertical Baseline, λ/2)                        |  
|                v                                                          |  
|   [ K-LC2 SENSOR 2: CORNER ] <--- 6.25 mm ---> [ K-LC2 SENSOR 3: RIGHT ]    |  
|     (Output: I2 / Q2 Baseband)     (λ/2)      (Output: I3 / Q3 Baseband)   |  
+------------------------------------+-----------------------+--------------+  
                                     |                       |  
                                     v (6x Raw Microvolts)   v  
                      +--------------------------------------+  
                      |  2x SparkFun LMV358 Dual Op-Amps     |  
                      |  (Analog Pre-Amp, Boosts to ~1V)      |  
                      +------------------+-------------------+  
                                         |  
                                         v (6x Clean Line-Level Volts)  
                      +--------------------------------------+  
                      |      Audio Injector Octo HAT         |  
                      | (Synchronous 24-bit / 96 kHz ADC)    |  
                      +------------------+-------------------+  
                                         |  
                                         v (Direct I2S / DMA Protocol)  
                      +--------------------------------------+  
                      |         Raspberry Pi 4               |  
                      |   (Python / STFT / MUSIC Engine)     |  
                      +--------------------------------------+
```

## **🛠️ Hardware Component Bill of Materials (BOM)**

| Component | Quantity | Purpose | Key Verification Metric   |
| :---- | :---- | :---- | :---- |
| **OmniPreSense OPS243** | 1 | Master Transmitter (Tx Spotlight) | Must operate in continuous wave (CW) mode; onboard processing data outputs are ignored. |
| **RFbeam K-LC2 Breakout** | 3 | Passive Phase-Locked Receivers (Rx) | Square 25x25 mm form factor. Transmit paths are completely disabled/unpopulated. |
| **SparkFun LMV358 Breakout** | 2 | Dual-Channel Analog Baseband Op-Amp | Boosts raw microvolt radar baseband lines to line-level voltage (~1V) with manually tuneable gain trim-pots. |
| **Audio Injector Octo HAT** | 1 | Synchronous Multi-Channel Digitizer | Direct Raspberry Pi 4 compliance. Drives a single master hardware clock codec across 8 continuous channels. |
| **Raspberry Pi 4 (4GB/8GB)** | 1 | Host Computing Core | Manages synchronous DMA audio capture via ALSA kernel drivers and processes matrix geometry in Python. |
| **Custom 3D Enclosure / Mount** | 1 | Geometric Alignment Chassis | Must securely hold the three K-LC2 modules in a staggered 3D configuration to clear physical boundaries while maintaining strict center-to-center spacing. |

## **📐 Geometric Sensor Array Design**

### **The 6.25mm Spatial Constraint**

To implement true 2D phase interferometry without spatial ambiguity arcs (aliasing), the geometric distance (d) between the center of the antenna elements must equal exactly half a wavelength (λ / 2). At the standard ISM band frequency used by these sensors (24.125 GHz), the calculation is:  
*λ = c / f = (3 × 10^8 m/s) / (24.125 × 10^9 Hz) ≈ 12.436 mm*  
*d = λ / 2 = 12.436 mm / 2 ≈ 6.218 mm → Target Spacing: 6.25 mm*

### **The 3D Staggering Solution**

Because each physical K-LC2 package is a 25 × 25 mm square, placing them flush next to each other on a flat surface results in a minimum center-to-center baseline of 25 mm, which breaks the phase geometry.  
To clear this mechanical boundary, the sensors must be **staggered in depth (3D Z-axis)** using a stepped internal mounting bracket:

```
                  [ SENSOR 1: TOP ]  
                +=======================+  
                |   ( Antenna Patch )   |  
                +=======================+  
                            |  
                            | 6.25 mm Vertical Axis Delta  
                            v  
                    +=======================+  
                    |   ( Antenna Patch )   |  
                    [ SENSOR 2: CORNER ]
```

* **Corner Sensor (2):** Mounted flush to the floor of the device housing.  
* **Top Sensor (1):** Shifted 10 mm forward in depth and slid down vertically so its physical PCB overlaps the corner module, forcing its gold antenna center array to sit exactly **6.25 mm above** the corner array center.  
* **Right Sensor (3):** Shifted 10 mm backward in depth and slid left horizontally so its physical PCB overlaps the corner module, forcing its gold antenna center array to sit exactly **6.25 mm to the right** of the corner array center.

*Note: Since the target golf ball flies 6 to 10 feet out in the field, a depth disparity of 10 mm creates an analytically negligible path delta, while perfectly preserving the 2D cross-axis angular resolution.*

## **⚛️ Why the RF Phase Locking Natively Cancels Drift**

A standard critique of multi-receiver arrays is that independent local oscillators (LO) drift over time, destroying the fine phase differences required by direction-finding algorithms like MUSIC:  
*Δ φ_Error = φ_Geometry + (φ_Oscillator A - φ_Oscillator B)*  
This system bypasses this limitation entirely by acting as a **Bistatic / Multistatic Radar Network**:

1. Only **one module** (the OPS243) actively generates a 24 GHz microwave wave (f_OPS).  
2. Because the three passive K-LC2 modules are located immediately adjacent to the high-power OPS243 module inside the enclosure, the pure un-bounced transmitting signal naturally bleeds across the air gaps and floods the receiving patches directly as **direct-air leakage**.  
3. When the echo from the moving golf ball (f_OPS + f_Doppler) reaches the K-LC2 internal mixers, it is down-converted directly against this direct-air leakage signal.

*Baseband Mixer Output = (f_OPS + f_Doppler) - f_OPS = f_Doppler*  
Because the reference local oscillator wave being used across all three independent mixers is the **exact same physical wave** radiating from the OPS243, any phase noise or thermal frequency drift inside the master transmitter is applied to all three receiver channels simultaneously. When computing the spatial phase delta between paths in software, the oscillator noise term cancels out completely, leaving pure geometric data.

## **💻 Python Signal Processing Pipeline**

Once the Audio Injector Octo HAT converts the 6 analog lines synchronously at **24-bit / 96 kHz**, the data enters a direct ALSA buffer loop on the Raspberry Pi 4.

### **1. Vector Extraction via Short-Time Fourier Transform (STFT)**

The script captures the 6 channels simultaneously, executes an STFT to locate the peak Doppler frequency matching the ball velocity (e.g., 10.77 kHz), and pulls the complex vector values (a + bj) for that specific bin across all channels:

```python
import numpy as np

# Phase-Synchronous complex vectors extracted from the target velocity FFT bin  
S_top    = I1 + 1j * Q1  
S_corner = I2 + 1j * Q2  
S_right  = I3 + 1j * Q3
```

### **2. Computing the Spatial Trajectories**

By evaluating the vector cross-products against the normalized physical antenna geometry baselines, the script extracts the independent trajectories instantly:

#### **Vertical Launch Angle (VLA)**

*Δ φ_Vertical = ∠ ( S_top · S_corner* )*  
*VLA = arcsin( Δ φ_Vertical / (2π · d_norm) ) · (180 / π)*

#### **Horizontal Deviation Angle (HLA)**

*Δ φ_Horizontal = ∠ ( S_right · S_corner* )*  
*HLA = arcsin( Δ φ_Horizontal / (2π · d_norm) ) · (180 / π)*  
*(Where d_norm = 0.5, representing our precise half-wavelength spatial spacing variable).*
