# Indoor Golf Simulator Radar Architecture & Signal Processing Guide
*Technical implementation notes for optimizing the RFbeam K-LD7 24 GHz FSK radar for short-range indoor launch monitors.*

---

## 1. Physical Geometry Configuration
The physical relationship between the radar sensor, the tee box, and the floor creates the baseline signal-to-noise ratio. Hardware optimization significantly reduces the computational overhead required by software.

### The Problem with standard Mounting
When the radar sits high (e.g., 4+ inches) or uses a shallow tilt, a low-launch driver shot (8°–12°) travels almost completely parallel to the floor plane. The direct path to the ball and the triangular reflection path off the floor have a microscopic distance delta (~7.7 mm), completely merging them into the same raw Range-Doppler FFT bins.

### The Optimized Setup
* **Radar Center Height:** 2 inches off the ground.
* **Vertical Axis Orientation:** Rotate the physical sensor 90 degrees so the **80° beamwidth is vertical** and the **34° beamwidth is horizontal** (enabling high-wedge tracking).
* **Upward Sensor Tilt:** 18° to 20° upward tilt.
* **Tee-to-Radar Distance:** 5 feet (approx. 1.5 meters).
* **Tee-to-Net Distance:** 12 feet (highly preferred over 10 feet to buy ~9ms of extra flight time).

[12ft Hitting Net]|                                 --- (Driver Launch: 10°)|                             ---|                         --- [Golf Ball]|                     ---        ||                 ---            | (Rapidly widening physical delta)|             ---                ||         ---                    v [Floor Reflection Plane]|     ---                      xxxxxxxxxxxxxxxxxxxxxxxxxxxxx| ---[Radar] (2" High, 20° Upward Tilt)


### Why This Geometry Works
1. **Beam Edge Suppression:** Tilting the radar 20° upward forces the floor reflection into the extreme outer edge of the 80° vertical beam, dropping reflection amplitude (dB).
2. **Immediate Path Separation:** Because the radar sits practically on the floor, the ball flies up and *away* from the radar's origin point, causing the reflection path to lengthen much faster than the direct path.
3. **Clutter Subtraction:** The floor surface directly in front of the radar becomes a static ground plane easily nullified by a baseline DC clutter subtraction filter.

---

## 2. Launch Physics & Software Modeling
To run an indoor flight simulator software engine, you do not track the ball down the range. You capture the **initial launch conditions** in the first few milliseconds and pipe them into an aerodynamic physics engine.

### Critical Metrics Needed by Software Engines
* **Ball Speed:** Absolute launch velocity.
* **Launch Angle (Vertical):** Pitch angle relative to the ground plane.
* **Launch Direction (Horizontal):** Azimuth angle relative to the center target line.
* **Spin Rate:** Total rotation speed (RPM) which dictates aerodynamic lift.
* **Spin Axis:** The side-to-tilt angle which induces the Magnus effect (hooks/slices).

### The Doppler Short-Flight Problem
Doppler systems measure frequency shift (velocity), not static spatial 3D arrays. Indoors, with only a 5-to-7-foot flight window before net impact, a radar cannot naturally see enough full rotations to calculate Spin Rate or Spin Axis. 
* **Solution 1:** Use specialized Radar Capture Technology (RCT) golf balls with metallic internal patterns or apply physical metallic foil dots. This creates a rhythmic "flash" in the raw FFT data, giving the software an explicit frequency to calculate RPM.
* **Solution 2 (Inference):** Use the radar to track the incoming club head metrics (Club Speed, Path Angle, Face Angle) and estimate ball spin using standard impact spin-matrix friction models.

---

## 3. High-Resolution Raw FFT Algorithms
Because you are tracking the ball over just 1–2 frames, traditional multi-frame tracking loops (like standard Kalman Filters) fail. You must handle isolation at the raw complex data level.

### Phase-Angle of Arrival (AoA) Limitations
The K-LD7 features two receiver channels (`Rx1` and `Rx2`) spaced at exactly $d = 6.223\text{ mm}$ (which corresponds to exactly half a wavelength, $\lambda/2$, for 24 GHz). 
Because the receiver arrays are aligned on a single line, **the radar can only calculate phase-angles along its physical layout axis.**
* If the sensor is oriented horizontally, it is blind to vertical phase angles.
* If the sensor is rotated vertically (80° vertical beamwidth), the phase difference calculates the vertical elevation launch angle, but sacrifices the horizontal azimuth phase channel.

### Extracting and Splitting Combined Vectors
When the ball and floor reflections merge into the same Range-Doppler bin, the resultant data is a complex sum of two vectors: 

$$\vec{X}_{\text{measured}} = \vec{V}_{\text{ball}} + \vec{V}_{\text{reflection}}$$

To isolate them using raw Complex ($I + jQ$) FFT blocks:
1. Locate the absolute peak velocity index $(R, D)$ in your FFT matrix.
2. Extract the complex values for both arrays: $X_1 = I_1 + jQ_1$ and $X_2 = I_2 + jQ_2$.
3. Compute the phase angles: $\phi_1 = \text{atan2}(Q_1, I_1)$ and $\phi_2 = \text{atan2}(Q_2, I_2)$.
4. Track the **Phase Velocity Derivative** ($\frac{d\Delta\phi}{dt}$) between Frame 1 and Frame 2. Because the real ball moves upward away from the ground plane, its phase vector will actively shift toward your positive expected launch window, while the reflection vector moves in an inverted direction.

### Advanced Subspace Separation
If the magnitude peaks are completely merged, bypass basic Fast Fourier Transforms (FFT) and apply **MUSIC (Multiple Signal Classification)** or **ESPRIT** algorithms over your receiver covariance matrix. These high-resolution subspace techniques mathematically divide signals into distinct sub-spaces, separating overlapping targets that sit inside the same range resolution cell.

---

## 4. Frame Rate Optimization (Unlocking 50 Hz)
A 150 mph driver ball travels at **220 feet per second**. Over a 5-foot flight path to a 10-foot net, the flight time is a mere **22 milliseconds**. At a 35 Hz sample rate (one frame every 28.57ms), you will likely catch only **1 frame** of clean flight data. 

To guarantee **2+ frames**, you must bypass the 115,200 bps serial wire bottleneck to push the K-LD7 to its hardware limit of **50 Hz (20ms intervals)**.

### Step-by-Step Optimization Workflow
1. **Disable Internal Processors:** Modify the serial hexadecimal initialization command to set `DataOutput` flags to **disable** Tracking Data (`TDAT`), Target Lists (`DLOG`), and Digital Zone Allocations (`AZON`). This frees the onboard chip from wasting clock cycles on tracking algorithms.
2. **Enable Exclusive Raw Blocks:** Set the output flag to stream **strictly raw thresholded FFT bins** or **magnitude-only blocks** (`RFFT`). 
3. **Minimize Payload Size:** Do not stream the entire 256-point complex spectrum if your serial bandwidth chokes. Truncate the transmission array to include only bins that cross a minor baseline noise threshold, keeping data packets below the maximum allowable byte budget of 11,520 bytes/sec.
4. **Optimize Constraints:** Reduce your maximum distance setting down to 5–10 meters and lower maximum velocity settings. Shorter parameters shorten the required frequency step length of the FSK modulation frontend, shrinking the physical time required for the ADC conversion.
5. **Implement Async Read Buffers:** Ensure your host-side processing loop (Raspberry Pi, PC, or microcontroller) uses raw asynchronous byte-array interrupts. Avoid linear blocking reading methods like `Serial.readString()` which introduce polling latencies that cause dropped frames.