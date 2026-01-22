## mq2_pcf8563

Rust crate for reading **MQ-2 gas sensor** (ADC) + **HZ-8563/PCF8563 RTC** (I²C) on **Arduino Uno** / AVR.

Features:
- Clean-air baseline calibration
- Gas index calculation with smoothing
- Timestamped logging via serial
- High-gas alert threshold

## Hardware Connections

- MQ-2: VCC → 5V, GND → GND, AOUT → A0
- HZ-8563/PCF8563: VCC → 5V, GND → GND, SDA → A4, SCL → A5
- 4.7kΩ pull-up resistors on SDA and SCL to 5V (required for reliable I²C)

## Usage Example

See the `examples/basic_logger.rs` for a complete runnable binary that:
- Calibrates the sensor
- Logs timestamp + raw + index
- Prints alert when index exceeds threshold

```bash
cargo run --example basic_logger --release

## License
Licensed under either of

- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)


## Contribution

Contributions are welcome! You can help by:

1. Reporting Issues
- Found a bug or unexpected behavior? Open an issue on GitHub.

2. Suggesting Features
- Ideas for improvements, new sensors, or functionality are always appreciated.

3. Submitting Pull Requests
- Fork the repository and make changes in a separate branch.

4. Ensure your code compiles and follows the project style.
- Include clear commit messages and, if applicable, update documentation.

5. Testing
- Test any new features or fixes on an Arduino with the MQ-2 sensor and PCF8563 RTC (or compatible hardware).


