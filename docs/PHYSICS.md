# Aerospace Physics Specification: LokYS Flight Simulator

**Revision:** 1.0  
**Status:** Approved Mathematical Specification  

---

## 1. Reference Coordinate Frames

LokYS implements four distinct, rigorously defined right-handed orthogonal coordinate frames:

```
               [ World Frame (ECEF / LTP) ]
                            │
                            ▼
               [ Navigation Frame (NED) ]
                X: North, Y: East, Z: Down
                            │  Attitude Quaternion q
                            ▼
               [ Aircraft Body Frame (FRD) ]
                X: Forward (Nose)
                Y: Right (Starboard Wing)
                Z: Down (Ventral)
                            │  Aerodynamic Angles (alpha, beta)
                            ▼
               [ Aerodynamic Wind Frame ]
                -X: Drag vector
                +Y: Sideforce vector
                -Z: Lift vector
```

### 1.1 Reference Frame Definitions
1. **World Frame (ECEF / Local Tangent Plane):** Global cartesian coordinates stored in double-precision 64-bit floats (`Vec3d`).
2. **Navigation Frame (NED):** North ($X_N$), East ($Y_E$), Down ($Z_D$). Down is positive towards the center of Earth. Geometric altitude $h = -Z_D$.
3. **Aircraft Body Frame (FRD):** Fixed to the airframe center of gravity.
   - Linear velocity: $\mathbf{v}_b = [u, v, w]^T$ (forward, right, down).
   - Angular velocity: $\boldsymbol{\omega}_b = [p, q, r]^T$ (roll rate, pitch rate, yaw rate).
4. **Wind Frame:** Aligned with the relative wind vector:
   - True Airspeed: $V_{TAS} = \sqrt{u^2 + v^2 + w^2}$
   - Angle of Attack: $\alpha = \arctan2(w, u)$
   - Sideslip Angle: $\beta = \arcsin\left(\frac{v}{V_{TAS}}\right)$

---

## 2. Transformation Equations

### 2.1 Attitude Transformation (Body $\longleftrightarrow$ NED)
Attitude is maintained strictly via unit quaternion $\mathbf{q} = [w, x, y, z]^T$.
A vector $\mathbf{v}_b$ in the Body frame is transformed to the NED navigation frame by:
$$\mathbf{v}_{NED} = \mathbf{q} \cdot \mathbf{v}_b \cdot \mathbf{q}^*$$
And reverse transformation:
$$\mathbf{v}_b = \mathbf{q}^* \cdot \mathbf{v}_{NED} \cdot \mathbf{q}$$

### 2.2 Aerodynamic Forces (Wind $\longrightarrow$ Body)
Forces calculated in the wind frame (Lift $L$, Drag $D$, Sideforce $Y$) are rotated into the Body frame:
$$\begin{bmatrix} F_x \\ F_y \\ F_z \end{bmatrix}_{body} = 
\begin{bmatrix}
-D \cos\alpha \cos\beta - Y \cos\alpha \sin\beta + L \sin\alpha \\
-D \sin\beta + Y \cos\beta \\
-D \sin\alpha \cos\beta - Y \sin\alpha \sin\beta - L \cos\alpha
\end{bmatrix}$$

---

## 3. Strict SI Units & Conversion Standards

Internal simulation physics computes exclusively in standard SI units:
* **Mass:** Kilograms ($\text{kg}$)
* **Length:** Meters ($\text{m}$)
* **Time:** Seconds ($\text{s}$)
* **Force:** Newtons ($\text{N}$)
* **Torque:** Newton-meters ($\text{N}\cdot\text{m}$)
* **Angle:** Radians ($\text{rad}$)
* **Pressure:** Pascals ($\text{Pa}$)

Aviation units are strictly bounded to cockpit instrument presentation and pilot control input parsers:
* $1 \text{ knot} = 0.514444 \text{ m/s}$
* $1 \text{ foot} = 0.3048 \text{ m}$
* $1 \text{ inHg} = 3386.389 \text{ Pa}$
* $1 \text{ lb} = 0.453592 \text{ kg}$
