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
