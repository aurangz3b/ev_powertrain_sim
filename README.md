# Modeling and Simulation of a Cascaded EV Powertrain: Bidirectional Boost Converter and Three-Phase Voltage Source Inverter

## Executive Summary

This project presents the design, modeling, and simulation of a cascaded electric vehicle (EV) traction powertrain implemented in MATLAB/Simulink using Simscape Specialized Power Systems. The system interfaces a low-voltage battery bank to an intermediate $500\text{ V}$ DC bus using a DC-DC boost converter stage, which supplies a three-phase, two-level Voltage Source Inverter (VSI) driving a balanced three-phase AC load.

The primary objectives achieved include:

1. Regulating the intermediate DC link at $500\text{ V}$ across operating conditions.


2. Eliminating inductive flyback spikes (which reached $-2200\text{ V}$ to $+2400\text{ V}$) through dead-time logic and switching synchronization.


3. Generating symmetrical, low-distortion three-phase sinusoidal AC output currents ($I \approx 9.2\text{ A}_{\text{peak}}$, $50\text{ Hz}$) and bipolar line-to-line voltages ($V_{ab} \approx \pm 350\text{ V}$).



---

## 1. System Architecture and Circuit Specifications

The simulated powertrain consists of an energy storage stage, an intermediate step-up conversion stage, a high-voltage smoothing capacitor, and a traction inverter.

<img width="2735" height="1133" alt="forecasting-05-00002-g029" src="https://github.com/user-attachments/assets/92e8393f-5a5e-4cda-86f0-08f8036f268b" />

```
+--------------+      +-------------------+      +-------------------+      +-------------------+      +--------------------+
| Battery Bank | ---> |  DC-DC Converter  | ---> |  DC-Link Buffer   | ---> |  3-Phase Inverter | ---> |  3-Phase AC Load   |
|   (201.6 V)  |      | (Boost Operation) |      | (888 uF / 500 V)  |      |  (6-Switch IGBT)  |      | (Isolated Neutral) |
+--------------+      +-------------------+      +-------------------+      +-------------------+      +--------------------+

```

### System Parameter Specifications

* **Battery Input Voltage ($V_{\text{bat}}$):** $201.6\text{ V}$ nominal


* **Target DC-Bus Voltage ($V_{\text{dc}}$):** $500\text{ V}$

* **Boost Inductor ($L$):** $225.6\ \mu\text{H}$

* **DC-Link Smoothing Capacitor ($C_{\text{dc}}$):** $888\ \mu\text{F}$

* **Safety Bleeder Resistor ($R_{\text{bleeder}}$):** $53.8\text{ k}\Omega$

* **Nominal Active Test Load ($R_{\text{load}}$):** $50\ \Omega$ ($5\text{ kW}$) to $100\ \Omega$ ($2.5\text{ kW}$)
* **Switching Frequency ($f_{sw}$):** $10\text{ kHz}$ ($T_{sw} = 100\ \mu\text{s}$)


* **Inverter Modulation:** Sinusoidal PWM (SPWM), $m_a = 0.7\text{--}0.8$, $f_1 = 50\text{ Hz}$

* **Simulink Solver Configuration:** Discrete `powergui`, Fixed-step sample time $T_s = 5\times 10^{-7}\text{ s}$ ($0.5\ \mu\text{s}$)



---

## 2. Engineering Challenges and Root-Cause Analysis

During development, early iterations exhibited severe bus collapse and inductive overvoltages. The underlying mechanisms and corrective actions were systematically isolated:

### A. Inductive Flyback Voltage Spikes ($-2200\text{ V}$ to $+2400\text{ V}$)

* **Symptom:** The inverter output phase node and the DC-bus capacitor experienced transient voltage spikes plunging down to $-2200\text{ V}$ and spiking up to $+2400\text{ V}$ every $20\text{ ms}$ ($50\text{ Hz}$).
* **Root Cause:** Complementary half-bridge switches were commanded via direct logical `NOT` operators without a dead-time blanking interval. Finite turn-off delays in discrete semiconductor models caused momentary bridge-arm cross-conduction (shoot-through), directly shorting the $500\text{ V}$ DC-link capacitor. The rapid forced turn-off of the short-circuit path caused inductive energy to collapse into an open circuit ($v = -L \frac{di}{dt}$), causing the observed voltage spikes.


* **Resolution:** Implemented a dedicated $1\ \mu\text{s}$ dead-time blanking interval (equivalent to 2 discrete sample steps at $T_s = 0.5\ \mu\text{s}$) between upper and lower gating signals.

### B. High-Frequency Carrier Mismatch

* **Symptom:** Boost converter control instability, excessive numerical chatter, and duty-cycle truncation.
* **Root Cause:** The `Repeating Sequence` carrier period in the boost converter was initially set to $T = 1\times 10^{-5}\text{ s}$ ($100\text{ kHz}$), a tenfold mismatch against the $10\text{ kHz}$ controller tuning and beyond the sample-step resolution of the discrete solver.


* **Resolution:** Corrected the repeating sequence time vector to $T = 1\times 10^{-4}\text{ s}$ ($100\ \mu\text{s}$ / $10\text{ kHz}$) using a symmetrical triangle `[0  0.5e-4  1e-4]` with bounds `[-1  1  -1]` for SPWM, and a sawtooth ramp `[0  1e-4]` with bounds `[0  1]` for the boost stage.



### C. Ground-Loop Bias and Differential Measurement

* **Symptom:** The measured line voltage appeared unipolar and DC-offset rather than balanced across zero.
* **Root Cause:** The negative terminal (`-`) of the `Voltage Measurement` block was initially grounded to the battery negative/chassis rail rather than connected differentially. Furthermore, tying the load star point to ground created a zero-sequence path that dumped current into the negative rail.


* **Resolution:** Wired the voltage sensor differentially directly between Phase A and Phase B midpoints ($V_{ab}$) and maintained a strictly floating load neutral point.



---

## 3. Implementation Details

### Modulation and Gate Drive Logic

1. **Pulse Generation:** The modulation scheme employs three-phase sinusoidal references:

$$u_a(t) = m_a \sin(2\pi \cdot 50 \cdot t)$$


$$u_b(t) = m_a \sin(2\pi \cdot 50 \cdot t - 120^\circ)$$


$$u_c(t) = m_a \sin(2\pi \cdot 50 \cdot t + 120^\circ)$$



with $m_a = 0.70$, compared against a $10\text{ kHz}$ carrier.


2. **Gate Signal Mapping:** The gate sequence was corrected to eliminate channel inversion:
* **Phase Leg A:** $S_1$ (Upper) $\leftarrow$ Comparator 1; $S_2$ (Lower) $\leftarrow$ Inverted Comparator 1 + $1\ \mu\text{s}$ Delay


* **Phase Leg B:** $S_3$ (Upper) $\leftarrow$ Comparator 2; $S_4$ (Lower) $\leftarrow$ Inverted Comparator 2 + $1\ \mu\text{s}$ Delay


* **Phase Leg C:** $S_5$ (Upper) $\leftarrow$ Comparator 3; $S_6$ (Lower) $\leftarrow$ Inverted Comparator 3 + $1\ \mu\text{s}$ Delay





### Filtering and Signal Processing

* The differential line-to-line output was processed through a continuous first-order low-pass filter ($f_c \approx 800\text{ Hz}$, time constant $\tau = 2\times 10^{-4}\text{ s}$) to filter the $10\text{ kHz}$ switching harmonics and reconstruct the fundamental $50\text{ Hz}$ line voltage.



---

## 4. Verification and Simulation Results

The simulation was evaluated across a $2.0\text{ s}$ runtime at a discrete step size of $T_s = 5\times 10^{-7}\text{ s}$.

### Steady-State Key Performance Indicators (KPIs)

* **DC-Link Voltage ($V_{\text{dc}}$):** Reached $+500\text{ V}$ steady state with $<1.5\%$ peak-to-peak switching ripple and zero uncommanded sags.


* **Phase Output Current ($I_a$):** Symmetrical sinusoidal AC waveform centered at $0.0\text{ A}$ DC offset, peaking at $\pm 9.2\text{ A}$ ($50\text{ Hz}$ fundamental).


* **Line-to-Line Voltage ($V_{ab}$):** Symmetrical bipolar waveform oscillating at $\pm 350\text{ V}_{\text{peak}}$, aligning with theoretical linear modulation:

$$V_{ab,\text{peak}} = m_a \cdot V_{\text{dc}} = 0.70 \times 500\text{ V} = 350\text{ V}$$



* **Transient Overvoltage:** Flyback voltage spikes were reduced from $2400\text{ V}$ down to $0\text{ V}$ overshoot beyond the DC-link clamp rail.



---

## 5. Repository Structure

```
├── models/
│   ├── buckboost_selfmade.slx               # Initial Buck-Boost model
│   ├── buckboost_withControl.slx            # Model with Control implementation
│   ├── buckboostwithinverter.slx            # Main model with Inverter
│   └── parameters.m                         # Pre-load initialization script
├── docs/
│   ├── waveforms/
│   │   ├── dc_link_voltage.png              # 500V DC-link bus regulation
│   │   ├── phase_current_50hz.png           # Symmetrical 9.2A AC current
│   │   └── line_voltage_vab.png             # Filtered +/-350V AC voltage
│   └── architecture_diagram.png             # Block diagram
└── README.md                                # Project documentation

```

---

## 6. How to Run the Simulation

1. Open MATLAB (R2020b or later recommended with Simscape Electrical).
2. Set the current directory to the project root.
3. Open and run `parameters.m` to load system inductance, capacitance, and switching parameters into the workspace.
4. Open `models/buckboostwithinverter.slx` (or your other model variants).
5. Run the model (`Ctrl+T` or `Cmd+T`).
6. Open `Scope2` to view the three synchronized axes:
* **Axis 1:** Output Phase Current ($I_a$)
* **Axis 2:** Regulated DC-Bus Voltage ($V_{\text{dc}}$)
* **Axis 3:** Filtered Line-to-Line Voltage ($V_{ab}$)
