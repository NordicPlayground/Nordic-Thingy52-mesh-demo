# Thingy Mesh Demo v0.1
### Overview
This is a short guide for how to deploy the Thingy mesh demo into Thingy devices.
The current Thingy mesh demo consists with one Thingy device as bridge and up to 10 Thingy devices as nodes.(The maximum number of nodes can be configured)

User can use APP on mobile phone like nRFMesh or nRFConnect to connect with bridge and send commands to control the nodes in the mesh network.

In the Thingy mesh demo, a experimental simple Thingy model is introduced. Which provides the LED control and also the humidity/temperature sensor information feedback function.


![demo overview](https://github.com/NordicPlayground/Nordic-Thingy52-mesh-demo/blob/master/pics/demo_pic.jpg)


### Requirements
- Nordic nRF5x-DK or Segger J-Link debugger
- 2x5 1.27mm SWD wire
- Nordic Thingy:52 (more than two would be great)
- Nordic Thingy:52 SDK v2.1.0
    - [https://github.com/NordicSemiconductor/Nordic-Thingy52-FW](https://github.com/NordicSemiconductor/Nordic-Thingy52-FW "Github link")
- Nordic nRF5 SDK for Mesh v1.0.1
    - [https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF5-SDK-for-Mesh](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF5-SDK-for-Mesh "nRF-Mesh-SDK")
- Segger Embedded Studio 

### Running the demo
To run the demo, you can use the precompiled firmware, so use Segger Embedded Studio to compile the firmware by self.

To compile the demo firmware and run the demo, please follow the steps:
1. Download and extract the Nordic Thingy:52 SDK and extract it, run the setup_sdk.bat.
2. Download Nordic nRF5 SDK for Mesh v1.0.1 and extract it under Thingy:52 SDK, rename the directory from "nrf5_SDK_for_Mesh_v1.0.1_src" to "mesh_sdk".
3. Download and place the Thingy_node, Thingy_bridge under Thingy:52 SDK, and place simple_thingy under mesh_sdk/models/ 
4. Open Segger Embedded Studio, and install the "nRF CPU Support Package". You can check it by click the main tool bar "Tools" -> "Package Manager", and search "nRF CPU Support Package".
5. Open the workspace under "Thingy_node\pca20020_s132\ses", compile and download the firmware in to multiple Thingy devices which will play the role as "Nodes".
6. Open the workspace under "Thingy_bridge\pca20020_s132\ses", compile and download the firmware in to the Thingy device which will play the role as "Bridge".
7. When the Thingy bridge is turned on, it will blink in red breath light, and start to do provisioning to the nearby unprovisoned Thingy nodes automatically. You can disable the auto-provisioning function by undefine the macro "AUTO_PROV" in the code, or check the detail of the protocol to send a command to turn it off.
8. When the unprovisioned Thingy node is turned on, it will blink in green breathe light, after it be provisioned, it will turn to constant light blue.
9. For using nRFMesh APP to run the demo, please add node 0x0000 ~ 0x0009 in the list, and you can control the specific node or all the nodes in the mesh network.
10. The button on the bridge can toggle the LED on every nodes 
11. If you don't have nRFMesh APP, you can use nRFConnect to send the commmand packets to run the demo, please check the "protocol section" for more information.
12. We provide macros for nRFconnect on Android, which is located in nRFConnect_macros in the respository. To import the macro, please check the guide video in [here](https://www.youtube.com/watch?v=rWOws3uhN6o "importing macro").
13. To erase the provisioning information stored in the bridge or nodes, please turn off the Thingy and press the button on the top of the Thingy when turn on the power.

### Known issues
This is a draft version of the demo example, there are some issue waiting for fixed:
 - once if the configuration failed, there's no way to recover, both of the bridge and nodes need to erase the provisioning information and restart again
 - now the health status send from the bridge to the mobile device may lead nRFMesh APP crash, so send back health status is disable in current version
 - The protocol design is not very systematic, will need to redesign


### Protocol Description
#### Communication protocol
##### How the bridge exchange information between mesh network and mobile device
The bridge using Nordic UART Service to conduct the command/response between mesh nodes and mobile device
- Nordic UART Service UUID 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
    - TX Characteristic (UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E)
        - After enabled the notification of TX Characteristic, the response from the nodes will show in here
    - RX Characteristic (UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E)
        - Write the command in here, the bridge will send the command to the nodes through mesh
##### Basic communication frame
The basic command frame is: [node index][service][parameter]
- The address consists with two bytes [node index MSB][node index LSB]
- The service consists with one byte [Service], and now there are multiple types of service
    - switch the auto provisioning - 0x01
    - config the sensor feedback rate - 0x02
    - config the LED status - 0x03
    - add a specific node to a custom group - 0x04
    - delete a specific node from a custom group - 0x05
    - scan the nearby unprovisioned device - 0x06
    - do provisioning to a specific unprovisioned device - 0x07
The parameter packets will be different depends on the service.

The basic response frame is: [node index][response type][response data]
- The address consists of two bytes [node index MSB][node index LSB]
- The response type consists with one bytes, and the current support response types are
    - scan result of the nearby unprovisioned device - 0x00
    - the provisioning result - 0x01
    - the configuration result -0x02
    - the health state - 0x03, current not support in this demo
    - the response of LED set command - 0x04
    - the response of sensor set command - 0x05
    - the sensor reading - 0x06
    - the LED status - 0x07

#### Command description and example
##### 1. Switch the auto provisioning
In the current firmware, the bridge will do provisioning to nearby devices automatically.
To turn off this feature, please send:

| ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------  | --------- |
| 0x00   | 0x00   | 0x01    | 0x01      |

To turn on this feature, please send 

| Node ID MSB | Node ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x00   | 0x01   | 0x00   |

##### 2. Manual provisioning
If the user disable the auto provisioning feature, then the user need to run the scan manually and choose the nearby device for provisioning.
To start a scan, the packets will be 

| Node ID MSB | Node ID LSB | Service | Parameter |
| ----------- | ----------- | ------- | --------- |
| 0x00        | 0x00        | 0x06    | --------- |

The bridge will start scan the nearby unprovisioned devices for five seconds, and send the scan result through TX notification.
The example result is like:

| Dev. No MSB | Dev. No LSB | Rsp  | UUID                             |
| ----------- | ----------- | ---- | -------------------------------- |
| 0x00        | 0x00        | 0x00 | AFAC5AED23FE2951D8BA9FA945493E54 |
| 0x00        | 0x01        | 0x00 | B3C2374B5C5DA96AC3020B9E713C2790 |
| 0x00        | 0x02        | 0x00 | 0E5D94F21219CB20BF6792482EE8945E |


There are three unprovisioned devices nearby, and if we want to run provisioning for the device 2  in the list, the packets will be:

| Dev. No MSB | Dev. No LSB | Service | Parameter |
| ----------- | ----------- | ------- | --------- |
| 0x00        | 0x02        | 0x07    | --------- |

Once if the provisioning and configuration successed, the response packets will be:

| Dev. No. MSB | Dev. No. LSB | Service | Parameter           |
| ------------ | ------------ | ------- | ------------------- |
| 0x00         | 0x01         | 0x07    | 0x00 0x00 0x00 0x00 |

The Dev. No. will be the index of the nodes.

If the provisioning and configuration failed, the response packets will be:

| Dev. No MSB | Dev. No LSB | Service | Parameter           |
| ----------- | ----------- | ------- | ------------------- |
| 0x00        | 0x02        | 0x07    | 0x08 0x00 0x00 0x00 |


##### 3. Config the sensor feedback rate
For the default setting, the Thingy node won't send back humidify and temperature information automatically.
The user needs to send the command to Thingy node to set the report timer.
The commands set can be found in the code:
``` c
typedef enum
{
    SENSOR_REPORT_NONE = 0,    //turn off the sensor report
    SENSOR_REPORT_EVERY_1S,    //config the sensor send report every 1 sec
    SENSOR_REPORT_EVERY_2S,    //config the sensor send report every 2 sec
    SENSOR_REPORT_EVERY_5S,    //config the sensor send report every 5 sec
    SENSOR_REPORT_EVERY_10S,   //config the sensor send report every 10 sec
    SENSOR_REPORT_MANUAL       //config the sensor send report manually
} sensor_report_config_t;
```
If the user only send service without command packets, the Thingy node will send sesnor report every 1 second.
If the user want Thingy node 2 send sensor report every 2 seconds, the packets will be:

|ID MSB  | ID LSB | Service | Temperature MSB     | Temperature LSB     | Humidity     |
| ------ | ------ | ------- | ---------           | -------             | ---------    |

The temperature is consisted with two parts, which using Celisus:
-  MSB: the integer part, which is a int8_t integer
- LSB: the decimal part, which is a uint8_t integer

The humidify shows in percentage, which is a uint8_t integer

If the TX notification on the bridge got "0x00 0x02 0x06 0x19 0x11 0x1a", which means

|ID MSB  | ID LSB | Service | Temperature MSB     | Temperature LSB     | Humidity     |
| ------ | ------ | ------- | ---------           | -------             | ---------    |
| 0x00   | 0x02   | 0x06    | 0x19                | 0x11                | 0x1a         |

- this sensor feedback came from the node address 0x0002    
- 0x19 is the integer part of temperature, it is 25 in this example 
- 0x11 is the decimal part of temperature, which is 0.17 in this example
-- So the temperature is 25.17 Celsius degree
- 0x1a is humidity percentage, which is 26% in the example


##### 4. Read back the sensor information
To read back the sensor information from a node, first you have to configure the sensor feedback rate for the node.
After that, please turn on the TX notification on the bridge, the sensor feedback information will show on the TX notification characteristic.
The data format shows like follow:

|ID MSB  | ID LSB | Service | Parameter |
| ------ | ------ | ------- | --------- |
| 0x00   | 0x02   | 0x02    | 0x02      |

##### 5. Set the Thingy LED status
For LED control, the command is very similar to the Thingy UI LED setting, the first byte of the commands is mode, and follow with the detail setting:
- Mode (uint8_t)
    - 0x00 = off
    - 0x01 – constant
    - 0x02 – breathe mode
    - 0x03 – one shot
- Detail setting byte
    - Constant mode (uint8_t)
        - Red intensity: 0~255
        - Green intensity: 0~255
        - Blue intensity: 0~255
    - Breathe mode
        - color (uint8_t)
            - 0x01 - Red
            - 0x02 – Green
            - 0x03 – Yellow
            - 0x04 – Blue
            - 0x05 – Purple
            - 0x06 – Cyan
            - 0x07 – White
        - intensity (uint8_t)
            - 1 ~ 100 (%)
        - delay (uint16_t)
            - 1~10000 (ms)
    - One shot mode
        - color (uint8_t)
            - 0x01 - Red
            - 0x02 – Green
            - 0x03 – Yellow
            - 0x04 – Blue
            - 0x05 – Purple
            - 0x06 – Cyan
            - 0x07 – White
        - intensity (uint8_t)
            - 1 ~ 100 (%)

If the user wants Thingy node 2 with constant red light, the packets will be 

| ID MSB | ID LSB | Service | Parameter           |
| ------ | ------ | ------- | ------------------- |
| 0x00   | 0x02   | 0x03    | 0x01 0xff 0x00 0x00 |

If the user want to turn off the LED on Thingy node 7, the packets will be 

| ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------- | --------- |
| 0x00   | 0x07   | 0x03    | 0x00      |

If the user want to set red breath light with intensity 50, delay 200ms to Thingy node 1, the packets will be 

|ID MSB  | ID LSB | Service | Parameter                |
| ------ | ------ | ------- | ------------------------ |
| 0x00   | 0x01   | 0x03    | 0x02 0x01 0x32 0xC8 0x00 |

##### 6. Add/delete a specific Thingy node to/from a custom group
The setting in the current firmmware configures every nodes subscribe the LED status from the group address 0xCAFE, which is binded with client 0x000A. User can add specific nodes into another group which address is 0xCAFF and the binded client is 0x000B.

To add node 4 into this custom group, the packets will be

| ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------- | --------- |
| 0x00   | 0x04   | 0x04    | --------  |

Set LED status into constand red for this group, the packets will be

| ID MSB | ID LSB | Service | Parameter           |
| ------ | ------ | ------  | ------------------- |
| 0x00   | 0x0B   | 0x03    | 0x01 0xff 0x00 0x00 |

To remove node 4 from this custom group, the packets will be

| ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------- | --------- |
| 0x00   | 0x04   | 0x05    | --------- |





