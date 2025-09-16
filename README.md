# DEXI LED Package

## Installation

For Debian Bookworm it's necessary to install this package and break system packages so that it can be run outside a Python virtual environment:

```bash
sudo pip install --break-system-packages pi5neo
```

Thank you to https://github.com/vanshksingh/Pi5Neo for the repo to make controlling the LED ring dead simple.

## Flight Mode LED Colors

The LED ring automatically changes color based on the current PX4 flight mode:

| Flight Mode | LED Color | Description |
|-------------|-----------|-------------|
| STABILIZED  | White     | Manual attitude control |
| ALTITUDE    | Yellow    | Altitude hold mode |
| POSITION    | Green     | Position hold mode |
| OFFBOARD    | Purple    | Offboard control mode |
| TAKEOFF     | Blue      | Automatic takeoff |
| LAND        | Pink      | Automatic landing |
| HOLD        | Cyan      | Loiter/Hold position |
| ACRO        | Orange    | Acrobatic/Rate mode (advanced) |

### Unknown Flight Modes
- **Cyan Blinking**: When the flight mode is not recognized or not in the standard configuration, the LED ring will blink cyan to alert the user
- **Black**: Recognized flight modes that don't have a specific color mapping will show black

## Services

The LED service provides the following ROS2 services:

- `/dexi/led_service/set_led_ring_color` - Set solid color for entire ring
- `/dexi/led_service/set_led_pixel_color` - Set individual pixel color
- `/dexi/led_service/set_led_effect` - Start LED effects (including `blink_<color>`)

## Hardware Support

### Pi5 Implementation (`led_service_pi5.py`)
- **Library**: Pi5Neo for SPI-based LED control
- **Features**: 
  - Advanced effects (galaxy spiral, meteor, rainbow, comet, ripple, loading)
  - RGB color tuples (e.g., `(255, 0, 0)` for red)
  - SPI interface via `/dev/spidev1.0`
  - Configurable LED count, brightness, and SPI speed
  - Simulation mode support
- **LED Control**: `strip.fill_strip()` and `strip.update_strip()`

### CM4 Implementation (`led_service_cm4.py`)  
- **Library**: Adafruit NeoPixel and LED Animation
- **Features**:
  - Predefined color constants from Adafruit library
  - GPIO-based control (board.D12)
  - Solid color animations via `Solid()` class
  - Blink effects support
- **LED Control**: `pixels.fill()` and `pixels.show()`

Both implementations provide the same ROS2 service interface and flight mode color mapping functionality.

## Local Development and Testing

### LED Visualization Bridge (`led_visualization_bridge.py`)

For local development and testing without deploying to hardware, use the LED Visualization Bridge. This service provides **identical functionality** to the hardware services but displays LEDs as visual markers in RViz instead of controlling physical hardware.

#### Key Features
- **100% API Compatible**: Same ROS2 services as CM4/Pi5 implementations
- **Visual Feedback**: LEDs displayed as colored spheres in RViz at configurable radius/height
- **All Effects Supported**: galaxy, rainbow, meteor, comet, ripple, loading, red_flash, blink_*
- **Same Color Mapping**: Identical color names and RGB values as hardware services
- **Real-time Updates**: 10Hz marker publishing for smooth animations

#### Getting Started

1. **Start the visualization bridge:**
   ```bash
   ros2 launch dexi_led led_visualization_bridge.launch.py
   ```

2. **Open RViz2** and add a MarkerArray display:
   - Topic: `/led_visualization`
   - Namespace: `dexi_led_ring`

3. **Test with service calls** (identical to hardware):
   ```bash
   # Set ring color
   ros2 service call /led_visualization_bridge/set_led_ring_color dexi_interfaces/srv/LEDRingColor '{color: "cyan"}'

   # Start galaxy effect
   ros2 service call /led_visualization_bridge/set_led_effect dexi_interfaces/srv/LEDEffect '{effect_name: "galaxy"}'

   # Set individual pixel
   ros2 service call /led_visualization_bridge/set_led_pixel_color dexi_interfaces/srv/LEDPixelColor '{index: 0, r: 255, g: 0, b: 0}'
   ```

#### Configuration Parameters
- `led_count`: Number of LEDs (default: 78)
- `brightness`: LED brightness 0.0-1.0 (default: 0.2)
- `visualization_radius`: Ring radius in meters (default: 1.0)
- `visualization_height`: Height above ground in meters (default: 2.0)
- `led_marker_size`: Marker size in meters (default: 0.04)

#### Testing Your Apps

Since the visualization bridge exposes the **same exact services** as your hardware, you can:

1. **Test Node-RED flows** - Point service calls to `/led_visualization_bridge/...` instead of `/dexi/led_service/...`
2. **Test Vue applications** - Same ROS2 service calls, just different node namespace
3. **Verify effects and colors** - See exactly how your LED patterns will look on hardware
4. **Debug timing and sequences** - Perfect for testing complex LED choreography

This allows you to develop and test your LED applications locally with full confidence they'll work identically on your Raspberry Pi hardware.
