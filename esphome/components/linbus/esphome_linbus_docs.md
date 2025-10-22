# LIN Bus Component

The LIN (Local Interconnect Network) bus component allows you to communicate with LIN bus devices using ESPHome. LIN bus is a low-cost serial communication protocol commonly used in automotive applications for communication between sensors and actuators.

```yaml
# Example configuration entry
linbus:
  id: lin_master
  tx_pin: GPIO10
  rx_pin: GPIO6
  cs_pin: GPIO7
  baud_rate: 19200
  uart_port: UART_NUM_1
  mode: master  # or slave or listener
  schedule:  # Only for master mode
    - lin_id: 34  # Decimal LIN ID (0-63)
      interval: 100ms
    - lin_id: 24  # Decimal LIN ID
      interval: 50ms
  responses:  # Pre-configured response data
    - lin_id: 33  # Decimal LIN ID
      data: [0x00, 0x00, 0x00, 0x00]  # Initial/default data
      checksum: enhanced
    - lin_id: 34  # Decimal LIN ID
      data: [0x01, 0x02]
      checksum: classic
  on_frame:
    - lin_id: 33  # Decimal LIN ID
      then:
        - lambda: |-
            ESP_LOGD("linbus", "Received frame for ID 33");
            ESP_LOGD("linbus", "Data length: %d", data_length);
            for (size_t i = 0; i < data.size(); i++) {
              ESP_LOGD("linbus", "Data[%d]: 0x%02X", i, data[i]);
            }
```

## Configuration Variables

- **id** (*Optional*, [ID](https://esphome.io/guides/configuration-types.html#config-id)): Manually specify the ID for this LIN bus.
- **uart_id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the UART component to use for communication.
- **cs_pin** (**Required**, [Pin](https://esphome.io/guides/configuration-types.html#config-pin)): The chip select pin for the LIN transceiver.
- **baud_rate** (*Optional*, int): The baud rate for LIN bus communication. Common values are `9600`, `19200` (default). Must be one of the standard LIN baud rates.
- **mode** (*Optional*, string): The operation mode of the LIN bus. Can be `master` or `slave`. Defaults to `master`.
- **schedule** (*Optional*, list): Schedule table for master mode. Defines which frame IDs the master will request and at what intervals. Only applicable in `master` mode. See [Schedule Configuration](#schedule-configuration).
- **responses** (*Optional*, list): Pre-configured response data for frame IDs. Defines which frame IDs this node will respond to and with what data. The data is buffered and sent immediately when the master requests this frame ID. See [Response Configuration](#response-configuration).
- **on_frame** (*Optional*, [Automation](https://esphome.io/guides/automations.html#automation)): An automation to perform when a complete frame with a specific LIN ID is received. See [on_frame Trigger](#on-frame-trigger).

## Schedule Configuration

In LIN bus networks, the master node controls all bus communication by periodically sending frame headers according to a predefined schedule table. This schedule determines which frame IDs are requested and how often.

**CRITICAL: Network-wide coordination required**. The LIN bus has no collision detection or self-healing mechanisms. You must carefully design your schedule and response configuration to ensure:
1. **Each frame ID has exactly ONE publisher** - Only one node (master or slave) should be configured to respond to each frame ID
2. **All nodes must use the same schedule** - Every device on the bus must know which frame IDs will be requested and when
3. **If two nodes respond to the same frame ID, the bus will fail** - This causes data corruption and the entire bus will stop functioning correctly

The schedule configuration is only used in `master` mode. Each schedule entry defines a frame ID and the interval at which the master should send the header for that frame.

```yaml
schedule:
  - lin_id: 34  # Decimal LIN ID (34 = 0x22 in hex)
    interval: 100ms
  - lin_id: 24  # Decimal LIN ID (24 = 0x18 in hex)
    interval: 50ms
  - lin_id: 48  # Decimal LIN ID (48 = 0x30 in hex)
    interval: 200ms
```

### Schedule Entry Variables

- **lin_id** (**Required**, int): The LIN frame ID in decimal format (0-63) to request. Note: LIN frame IDs are 6-bit values.
- **interval** (**Required**, [Time](https://esphome.io/guides/configuration-types.html#config-time)): The interval at which the master should send a header for this frame ID.

**How it works**: When the master sends a frame header, the node that is configured to respond to that frame ID (via the `responses` section) will immediately transmit the data bytes. This can be the master itself or any slave on the bus.

**Important**: Only configure ONE node to respond to each frame ID. If multiple nodes respond to the same ID, their transmissions will collide and corrupt the data, causing the entire LIN bus to malfunction.

## Response Configuration

The `responses` section defines which frame IDs this node will respond to and what data will be sent. This is how both masters and slaves provide data on the LIN bus.

**Critical timing requirement**: In the LIN protocol, when a frame header is transmitted, the responding node must send data bytes immediately (within microseconds) to maintain bus timing. To achieve this, response data must be prepared in advance and stored in a buffer.

**Master-to-slave communication**: The LIN protocol uses the same mechanism for both directions:
- Master requests sensor data from slave: Master sends header for slave's frame ID → Slave responds with sensor data
- Master sends commands to slave: Master sends header for a frame ID that the master responds to → Master sends command data → Slave receives it

The key point: **Whoever is configured to respond to a frame ID will send the data bytes when that frame header appears on the bus.**

**CRITICAL: Each frame ID must have exactly ONE publisher**. Never configure multiple nodes to respond to the same frame ID. LIN has no collision detection - if two nodes transmit simultaneously, the data will be corrupted and the bus will fail.

```yaml
responses:
  - lin_id: 33  # This node will respond when frame ID 33 is requested
    data: [0x00, 0x00, 0x00, 0x00]  # Initial/default response data
    checksum: enhanced
  - lin_id: 48  # This node will respond when frame ID 48 is requested
    data: [0x01, 0x02]  # Command data (e.g., pump control)
    checksum: classic
```

### Response Entry Variables

- **lin_id** (**Required**, int): The LIN frame ID in decimal format (0-63) that this node will respond to.
- **data** (**Required**, list): Pre-configured data bytes to send (up to 8 bytes) when this frame ID is requested.
- **checksum** (*Optional*, string): Checksum type to use. Can be `classic` (LIN 1.x) or `enhanced` (LIN 2.x). Defaults to `enhanced`.

### Updating Response Data at Runtime

Response data can be updated dynamically using the `linbus.update_response` action. The new data will be buffered and used for the next transmission when the frame ID is requested.

```yaml
# Example: Update sensor data to be sent when master requests it
sensor:
  - platform: homeassistant
    id: temperature
    entity_id: sensor.temperature
    on_value:
      then:
        - linbus.update_response:
            id: lin_bus_1
            lin_id: 33
            data: !lambda |-
              uint16_t temp = (uint16_t)(x * 10);
              return {(uint8_t)(temp >> 8), (uint8_t)(temp & 0xFF), 0x00, 0x00};
```

## on_frame Trigger

This automation will be triggered when a complete frame (header + data + checksum) with the specified LIN ID is received.

```yaml
on_frame:
  - lin_id: 33  # Decimal LIN ID
    then:
      - lambda: |-
          ESP_LOGD("linbus", "Frame received");
          // data: std::vector<uint8_t> - received data bytes (up to 8)
          // data_length: uint8_t - actual length of received data
          // checksum_type: string - "classic" or "enhanced"
```

**Important Performance Note**: The `on_frame` automation is triggered every time a complete frame with the specified ID is received. In typical LIN networks, frames are transmitted very frequently (e.g., every 10-50ms according to the schedule). This high trigger rate can overwhelm ESPHome's automation system, especially if the automation contains complex logic or calls to other components.

**Best Practice**: For high-frequency frames, avoid heavy processing in `on_frame` automations. Instead:
1. Use `on_frame` only to store the received data in a global variable (lightweight operation)
2. Use template sensors with controlled update intervals to extract and process the data

See [Template Sensor Example](#template-sensor-example) below.

### Variables

- **data** (`std::vector<uint8_t>`): The received data bytes (up to 8 bytes).
- **data_length** (`uint8_t`): The actual length of the received data (1-8).
- **checksum_type** (`std::string`): The type of checksum used - `"classic"` or `"enhanced"`.
- **id** (`uint8_t`): The LIN frame ID (decimal).

## Actions

### `linbus.update_response` Action

Update the pre-buffered response data for a specific LIN frame ID. This is the standard way to provide data on the LIN bus according to the LIN specification.

```yaml
# Update response data from a sensor
sensor:
  - platform: homeassistant
    id: temperature
    entity_id: sensor.temperature
    on_value:
      then:
        - linbus.update_response:
            id: lin_bus_1
            lin_id: 33
            data: !lambda |-
              uint16_t temp = (uint16_t)(x * 10);
              return {(uint8_t)(temp >> 8), (uint8_t)(temp & 0xFF), 0x00, 0x00};

# Update with static data
on_...:
  then:
    - linbus.update_response:
        id: lin_bus_1
        lin_id: 34
        data: [0xFF, 0x00, 0x01]
        checksum: classic
```

Configuration variables:

- **id** (*Optional*, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the LIN bus. Defaults to the only one in YAML.
- **lin_id** (**Required**, int): The LIN frame ID in decimal format (0-63).
- **data** (**Required**, list): List of data bytes (1-8 bytes). Can be a static list or a lambda expression returning `std::vector<uint8_t>`.

**Note**: Event-triggered frames (spontaneous frames) and diagnostic frames (frame IDs 60-63) are not yet supported in this implementation. These features may be added in future versions.

**Note**: Use `linbus.resume_send` to restart transmission of this frame ID (future feature).

## Sensors

The LIN bus component can expose diagnostic sensors for monitoring bus health.

### Bus Load Sensor

Monitors the percentage of bus utilization.

```yaml
sensor:
  - platform: linbus
    linbus_id: lin_bus_1
    bus_load:
      name: "LIN Bus Load"
      unit_of_measurement: "%"
```

### Error Rate Sensor

Monitors the rate of communication errors on the bus (checksum errors, framing errors, etc.).

```yaml
sensor:
  - platform: linbus
    linbus_id: lin_bus_1
    error_rate:
      name: "LIN Bus Error Rate"
      unit_of_measurement: "errors/s"
```

## Template Sensor Example

For high-frequency LIN frames, it's recommended to use template sensors instead of complex `on_frame` automations. Template sensors extract data from the LIN bus at a controlled update rate, preventing ESPHome from being overwhelmed by processing every single frame.

```yaml
# Store the latest frame data in globals
globals:
  - id: lin_frame_33_data
    type: std::vector<uint8_t>
    restore_value: no
    initial_value: '{0, 0, 0, 0, 0, 0, 0, 0}'

linbus:
  id: lin_slave
  tx_pin: GPIO10
  rx_pin: GPIO6
  cs_pin: GPIO7
  uart_port: UART_NUM_1
  baud_rate: 19200
  mode: slave
  responses:
    - lin_id: 33
      data: [0x00, 0x00, 0x00, 0x00]
      checksum: enhanced
  on_frame:
    - lin_id: 33
      then:
        # Lightweight operation: just store the data
        - lambda: |-
            id(lin_frame_33_data) = data;

# Extract temperature from bytes 0-1 every second
sensor:
  - platform: template
    name: "LIN Temperature"
    unit_of_measurement: "°C"
    device_class: temperature
    accuracy_decimals: 1
    update_interval: 1s  # Update only once per second, not every frame
    lambda: |-
      auto data = id(lin_frame_33_data);
      if (data.size() >= 2) {
        uint16_t raw = (data[0] << 8) | data[1];
        return raw / 10.0;  // Convert to temperature
      }
      return {};  // Return empty optional if no data

  # Extract status bits from byte 2
  - platform: template
    name: "LIN Status Flags"
    update_interval: 1s
    lambda: |-
      auto data = id(lin_frame_33_data);
      if (data.size() >= 3) {
        return (float)data[2];
      }
      return {};

  # Extract voltage from bytes 4-5
  - platform: template
    name: "LIN Bus Voltage"
    unit_of_measurement: "V"
    device_class: voltage
    accuracy_decimals: 2
    update_interval: 1s
    lambda: |-
      auto data = id(lin_frame_33_data);
      if (data.size() >= 6) {
        uint16_t raw = (data[4] << 8) | data[5];
        return raw / 100.0;  // Convert to voltage
      }
      return {};
```

**Advantages of this approach**:
- The `on_frame` automation only performs a lightweight memory copy operation
- Heavy processing (conversions, calculations) happens at a controlled rate (e.g., 1 second)
- Multiple sensors can extract different data fields from the same frame
- ESPHome's automation system is not overwhelmed by high-frequency triggers
- Template sensors can handle missing or invalid data gracefully

## Full Example

```yaml
# LIN Bus configuration
linbus:
  id: lin_master
  tx_pin: GPIO10
  rx_pin: GPIO6
  cs_pin: GPIO7
  uart_port: UART_NUM_1
  baud_rate: 19200
  mode: master
  schedule:
      # Request sensor data from slave every 200ms
      - lin_id: 33
        interval: 200ms
      # Request status from slave every 100ms
      - lin_id: 34
        interval: 100ms
      # Send pump control commands every 100ms (master responds)
      - lin_id: 48
        interval: 100ms
  responses:
    # Master responds to ID 48 to control pump (slave device)
    - lin_id: 48
      data: [0x00, 0x00, 0x00, 0x00]  # Initial: pump off
      checksum: enhanced
  on_frame:
    - lin_id: 33
      then:
        - lambda: |-
            ESP_LOGD("linbus", "Temperature sensor data received");
            if (data.size() >= 2) {
              float temp = (data[0] << 8 | data[1]) / 10.0;
              ESP_LOGD("linbus", "Temperature: %.1f°C", temp);
            }
    - lin_id: 34
      then:
        - lambda: |-
            ESP_LOGD("linbus", "Status frame received");

# Store latest frame data for template sensors
globals:
  - id: lin_temp_frame
    type: std::vector<uint8_t>
    restore_value: no
    initial_value: '{0, 0, 0, 0}'

# Alternative linbus for listener mode
linbus:
  id: lin_listener
  tx_pin: GPIO4
  rx_pin: GPIO5
  uart_port: UART_NUM_0
  baud_rate: 19200
  mode: listener
  on_frame:
    - lin_id: 50
      then:
        - lambda: id(lin_temp_frame) = data;

# Extract sensor data at controlled rate
sensor:
  - platform: template
    name: "LIN Device Temperature"
    unit_of_measurement: "°C"
    device_class: temperature
    update_interval: 1s
    lambda: |-
      auto data = id(lin_temp_frame);
      if (data.size() >= 2) {
        uint16_t raw = (data[0] << 8) | data[1];
        return raw / 10.0;
      }
      return {};

  # Bus diagnostic sensors
  - platform: linbus
    linbus_id: lin_master
    bus_load:
      name: "LIN Bus Load"
      update_interval: 1s
    error_rate:
      name: "LIN Bus Error Rate"
      update_interval: 1s

# Example: Master controlling a pump (slave device)
# Update pump speed based on a number input
number:
  - platform: template
    name: "Pump Speed"
    min_value: 0
    max_value: 100
    step: 1
    optimistic: true
    on_value:
      then:
        - linbus.update_response:
            id: lin_master
            lin_id: 48  # Pump control frame ID
            data: !lambda |-
              // Master prepares command data to send to pump
              uint8_t speed = (uint8_t)x;
              return {speed, 0x01, 0x00, 0x00};  // speed, enable, reserved, reserved

# Example: Future diagnostic command capability (not yet supported)
# binary_sensor:
#   - platform: gpio
#     pin: GPIO0
#     name: "Request Diagnostics"
#     on_press:
#       - linbus.send_diagnostic:  # Future feature
#           id: lin_bus_1
#           nad: 0x01  # Node Address for Diagnostics
#           data: [0x22, 0xF1, 0x8C]  # Read Data By Identifier
```

## Notes

### LIN Protocol Overview

- **Master-slave architecture**: LIN bus uses a single-master, multiple-slave architecture. Only the master can initiate communication by sending frame headers. Slaves listen and respond when their frame ID is requested.
- **Master controls all timing**: The master sends frame headers according to a predefined schedule table. This is fundamentally different from CAN bus where any node can transmit at any time.
- **No collision detection or arbitration**: Unlike CAN bus, LIN has no mechanism to detect or resolve collisions. If two nodes transmit at the same time, the data will be corrupted and the bus will fail. This makes proper configuration absolutely critical.
- **Static configuration required**: All nodes must be configured with a coordinated schedule and response assignments before deployment. There is no automatic address assignment or dynamic reconfiguration.
- **Frame structure**: A LIN frame consists of:
  1. Break field (master only)
  2. Sync field (master only)
  3. Protected Identifier (PID) field (master only)
  4. Data bytes (0-8 bytes, from publisher node)
  5. Checksum (from publisher node)

### Communication Patterns

- **Slave-to-master**: Master sends header for frame ID assigned to slave → Slave responds with sensor data
- **Master-to-slave**: Master sends header for frame ID that master publishes → Master sends command data → All nodes receive it
- **Slave-to-slave**: Master sends header for frame ID assigned to slave A → Slave A publishes data → Slave B receives it (master also receives it)

The key principle: **The schedule determines which frame IDs are requested. The responses configuration determines which node publishes data for each frame ID.**

**Configuration Responsibility**: As the network designer, you must:
- Create a master schedule that includes all frame IDs needed by all nodes
- Assign exactly one publisher (responder) to each frame ID
- Ensure all nodes use the same timing expectations
- Document your frame ID assignments to prevent conflicts

There is no automatic discovery or conflict resolution - the network will simply fail if misconfigured.

### Configuration Guidelines

- **Frame IDs**: Specified in decimal format (0-63) throughout the configuration. Frame IDs are 6-bit values. Some IDs are reserved:
  - 60-61: Diagnostic frames (not yet supported)
  - 62-63: Reserved for future LIN protocol enhancements
- **Frame ID assignment**: Each frame ID must be assigned to exactly ONE publisher. Create a frame ID assignment table before configuring your network:
  ```
  ID  Publisher  Purpose          Update Rate
  --  ---------  ---------------  -----------
  10  Slave A    Temperature      200ms
  11  Master     Pump Command     100ms
  12  Slave B    Status           100ms
  13  Slave A    Pressure         500ms
  ```
- **Checksum types**:
  - **Classic**: Checksum calculated over data bytes only (LIN 1.x compatibility)
  - **Enhanced**: Checksum calculated over protected ID and data bytes (LIN 2.x, recommended)
- **Baud rates**: Common standard values are 9600, 19200, and 20000 baud. The baud rate must match across all nodes in the network.
- **Maximum data length**: 8 bytes per frame (LIN specification limit)

### Performance Considerations

- **Response timing is critical**: When a frame header is transmitted, the response must begin within a few bit times (typically < 1ms). Always use the `responses` configuration with pre-buffered data to meet this requirement.
- **High-frequency frames**: Frames with intervals < 100ms should use the template sensor pattern to avoid overwhelming ESPHome's automation system.
- **Schedule design**: Balance between data freshness and bus load. Typical intervals: 10-200ms for control signals, 100-1000ms for status information.

### Current Limitations

- **Event-triggered frames**: Spontaneous frames (frames triggered by events rather than schedule) are not yet supported.
- **Diagnostic frames**: Frames with IDs 60-63 and diagnostic services (NAD, SID) are not yet implemented.
- **Sleep/wakeup**: LIN bus sleep mode and wake-up signals are not yet supported.
- **Configuration services**: LIN configuration and identification services are not yet implemented.

These features may be added in future versions of the component.

## See Also

- [UART Bus](https://esphome.io/components/uart.html)
- [CAN Bus](https://esphome.io/components/canbus.html)
- [Sensor Component](https://esphome.io/components/sensor/)
- [Template Sensor](https://esphome.io/components/sensor/template.html)
