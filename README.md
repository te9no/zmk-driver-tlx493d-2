# ZMK TLX493D-A2BW Driver

This module provides a ZMK driver for the Infineon TLX493D-A2BW 3D magnetic sensor with Z-axis state-dependent behavior binding support.

## Features

- **TLX493D-A2BW Support**: Full support for the improved A2BW variant with enhanced reliability
- **Z-axis State Detection**: Configurable threshold-based detection of pressed/normal states
- **Behavior Bindings**: Configure different ZMK behaviors for normal and pressed states
- **Hysteresis**: Prevents bouncing with configurable hysteresis values
- **Input Processing**: Integrates with ZMK's input processor system
- **Auto-calibration**: Automatic sensor calibration and error recovery

## Configuration

### Device Tree Properties

Add the sensor to your device tree overlay:

```dts
&i2c0 {
    magnet: tlx493d@35 {
        compatible = "infineon,tlx493d-a2bw";
        reg = <0x35>;
        status = "okay";
        
        // Basic configuration
        z-press-threshold = <50>;          // Z-axis threshold for press detection
        z-hysteresis = <10>;              // Hysteresis to prevent bounce
        
        // Optional: Behavior bindings for Z-axis state changes
        normal-binding = <&mkp MCLK>;              // Middle click when normal
        pressed-binding = <&mkp MCLK &kp LSHIFT>;  // Shift+middle click when pressed
    };
};
```

### Available Properties

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `z-press-threshold` | int | 50 | Z-axis threshold for press detection |
| `z-hysteresis` | int | 10 | Z-axis hysteresis to prevent bounce |
| `normal-binding` | phandle-array | - | Behavior binding for normal state (optional) |
| `pressed-binding` | phandle-array | - | Behavior binding for pressed state (optional) |

### Kconfig Options

Enable the driver in your configuration:

```
CONFIG_INPUT_TLX493D=y
```

## Behavior Bindings

The driver supports optional behavior bindings that are triggered when the Z-axis state changes:

- **Normal State**: When Z-axis displacement is below the threshold
- **Pressed State**: When Z-axis displacement exceeds the threshold

### Examples

**Simple key press:**
```dts
normal-binding = <&kp A>;
pressed-binding = <&kp B>;
```

**Mouse operations:**
```dts
normal-binding = <&mkp MCLK>;              // Middle click
pressed-binding = <&mkp MCLK &kp LSHIFT>;  // Shift + middle click
```

**Layer switching:**
```dts
normal-binding = <&mo 1>;                  // Momentary layer 1
pressed-binding = <&to 2>;                 // Switch to layer 2
```

## Usage with Input Processors

The driver integrates with ZMK's input listener system:

```dts
magnet_listener: magnet_listener {
    compatible = "zmk,input-listener";
    device = <&magnet>;
    move {
        layers = <0>;
        input-processors = <&zip_xy_scaler 1 2>;
    };
};
```

## Technical Details

### Implementation (Phase 1-2 Complete)
- **Official Library Knowledge**: Utilizes Infineon A2BW-specific initialization and data format
- **Improved Architecture**: Clean separation between sensor driver and behavior logic
- **Unified State Management**: Single source of truth for Z-axis state across modules
- **A2BW-Optimized**: 12-bit data format, proper reset sequence, diagnostic validation

### Operation
1. **A2BW Initialization**: Uses official reset sequence (0xFF×2 → 0x00×2 + 30μs delay)
2. **Sensor Reading**: Reads 12-bit magnetic field values with proper sign extension
3. **Calibration**: Auto-calibrates origin point for relative movement
4. **State Detection**: Compares Z-axis displacement against threshold with hysteresis
5. **State Update**: Updates global state for behavior system consumption
6. **Input Reporting**: Reports relative X/Y movement as input events

### State Machine
- Normal state → Pressed state: When `abs(delta_z) > z-press-threshold`
- Pressed state → Normal state: When `abs(delta_z) < (z-press-threshold - z-hysteresis)`
- Global state synchronized via `tlx493d_set_z_axis_pressed()`

### Error Handling
- A2BW-specific reset and validation
- Automatic sensor re-initialization on errors
- Diagnostic register monitoring (PD0/PD3 bits)
- Frame counter validation

## Troubleshooting

### Build Issues
1. Ensure `CONFIG_INPUT_TLX493D=y` is set
2. Verify I2C is enabled: `CONFIG_I2C=y`
3. For mouse behaviors, enable: `CONFIG_ZMK_POINTING=y`

### Runtime Issues
1. Check I2C wiring and address (default 0x35)
2. Verify sensor placement and magnetic field strength
3. Adjust threshold values if sensitivity is incorrect
4. Monitor logs for initialization errors

## Development

See `CLAUDE.md` for detailed development instructions and architecture information.

## License

This module is licensed under the MIT License.