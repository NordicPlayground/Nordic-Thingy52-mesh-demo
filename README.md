# Thingy Mesh Demo v0.1
### Overview
This is a short guide for how to deploy the Thingy mesh demo into Thingy devices.
The current Thingy mesh demo consists with one Thingy device as bridge and up to 10 Thingy devices as nodes.(The maximum number of nodes can be configured)

User can use APP on mobile phone like nRFMesh or nRFConnect to connect with bridge and send commands to control the nodes in the mesh network.

In the Thingy mesh demo, a experimental simple Thingy model is introduced. Which provides the LED control and also the humidity/temperature sensor information feedback function.


### Requirements
- Nordic nRF5x-DK or Segger J-Link debugger
- 2x5 1.27mm SWD wire
- Nordic Thingy:52 (more than two would be great)
- Nordic Thingy:52 SDK v2.1.0
    - [https://github.com/NordicSemiconductor/Nordic-Thingy52-FW](https://github.com/NordicSemiconductor/Nordic-Thingy52-FW "Github link")
- Nordic nRF5 SDK for Mesh v0.10.1
    - [https://www.nordicsemi.com/eng/nordic/Products/nRF5-SDK-for-Mesh/nRF5-SDK-for-Mesh/62377](https://www.nordicsemi.com/eng/nordic/Products/nRF5-SDK-for-Mesh/nRF5-SDK-for-Mesh/62377)
- Segger Embedded Studio 

### Running the demo
To run the demo, you can use the precompiled firmware, so use Segger Embedded Studio to compile the firmware by self.

To compile the demo firmware and run the demo, please follow the steps:
1. Download and extract the Nordic Thingy:52 SDK and extract it.
2. Download Nordic nRF5 SDK for Mesh v0.10. and etract it under Thingy:52 SDK, rename the directory from "nrf5_SDK_for_Mesh_v0.10.1-Alpha_src" to "mesh_sdk".
3. Download and place the Thingy_node, Thingy_bridge under Thingy:52 SDK, and place simple_thingy under mesh_sdk/models/ 
4. Open Segger Embedded Studio, and install the "nRF CPU Support Package". You can check it by click the main tool bar "Tools" -> "Package Manager", and search "nRF CPU Support Package".
5. Open the workspace under "Thingy_node\pca20020_s132\ses", compile and download the firmware in to multiple Thingy devices which will play the role as "Nodes".
6. Open the workspace under "Thingy_bridge\pca20020_s132\ses", compile and download the firmware in to the Thingy device which will play the role as "Bridge".
7. When the Thingy bridge is turned on, it will blink in red breath light, and start to do provisioning to the nearby unprovisoned Thingy nodes automatically. You can disable the auto-provisioning function by undefine the macro "AUTO_PROV" in the code, or check the detail of the protocol to send a command to turn it off.
7. When the unprovisioned Thingy node is turned on, it will blink in green breathe light, after it be provisioned, it will turn to constant light blue.
8. For using nRFMesh APP to run the demo, please add node 0x0000 ~ 0x0009 in the list, and you can control the specific node or all the nodes in the mesh network.
9. If you don't have nRFMesh APP, you can use nRFConnect to send the commmand packets to run the demo, please check the "protocol section" for more information.
10. To erase the provisioning information stored in the bridge or nodes, please turn off the Thingy and press the button on the top of the Thingy when turn on the power.

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
| ------ | ------ | ------ | ------ |
| 0x00   | 0x00   | 0x01 | 0x01 |
To turn on this feature, please send 
| Node ID MSB | Node ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x00   | 0x01 | 0x00 |

##### 2. Manual provisioning
If the user disable the auto provisioning feature, then the user need to run the scan manually and choose the nearby device for provisioning.
To start a scan, the packets will be 
| Node ID MSB | Node ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x00   | 0x06 | ------ |
The bridge will start scan the nearby unprovisioned devices for five seconds, and send the scan result through TX notification.
The example result is like:
| Dev. No MSB | Dev. No LSB | Rsp | UUID |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x00  | 0x00 | AFAC5AED23FE2951D8BA9FA945493E54 |
| 0x00   | 0x01  | 0x00 | B3C2374B5C5DA96AC3020B9E713C2790 |
| 0x00   | 0x02  | 0x00 | 0E5D94F21219CB20BF6792482EE8945E |


There are three unprovisioned devices nearby, and if we want to run provisioning for the device 2  in the list, the packets will be:
| Dev. No MSB | Dev. No LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x02   | 0x07 | ------ |

Once if the provisioning and configuration successed, the response packets will be:
| Dev. No. MSB | Dev. No. LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x01   | 0x07 | 0x00 0x00 0x00 0x00 |
The Dev. No. will be the index of the nodes.


If the provisioning and configuration failed, the response packets will be:
| Dev. No MSB | Dev. No LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x02   | 0x07 | 0x08 0x00 0x00 0x00 


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
|ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x02   | 0x02 | 0x02 |



##### 4. Set the Thingy LED status
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

If the user wants Thingy node 0 with constant red light, the packets will be 

|ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x02   | 0x03 | 0x01 0xff 0x00 0x00 |

If the user want to turn off the LED on Thingy node 7, the packets will be 
|ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x07   | 0x03 | 0x00 |

If the user want to set red breath light with intensity 50, delay 200ms to Thingy node 1, the packets will be 
|ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x01   | 0x03 | 0x02 0x01 0x32 0xC8 0x00 |

##### 5. Add/delete a specific Thingy node to/from a custom group
The setting in the current firmmware configs every nodes subscribe the LED status from the group address 0xCAFE, which is binded with client 0x000A. User can add specific nodes into another group which address is 0xCAFF and the binded client is 0x000B.

To add node 4 into this custom group, the packets will be
|ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x04   | 0x04 | ------  |

Set LED status into constand red for this group, , the packets will be
|ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x0B  | 0x03 | 0x01 0xff 0x00 0x00 |

To remove node 4 from this custom group, the packets will be
|ID MSB | ID LSB | Service | Parameter |
| ------ | ------ | ------ | ------ |
| 0x00   | 0x04   | 0x05 | ------  |





# Dillinger

[![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid)

Dillinger is a cloud-enabled, mobile-ready, offline-storage, AngularJS powered HTML5 Markdown editor.

  - Type some Markdown on the left
  - See HTML in the right
  - Magic

# New Features!

  - Import a HTML file and watch it magically convert to Markdown
  - Drag and drop images (requires your Dropbox account be linked)


You can also:
  - Import and save files from GitHub, Dropbox, Google Drive and One Drive
  - Drag and drop markdown and HTML files into Dillinger
  - Export documents as Markdown, HTML and PDF

Markdown is a lightweight markup language based on the formatting conventions that people naturally use in email.  As [John Gruber] writes on the [Markdown site][df1]

> The overriding design goal for Markdown's
> formatting syntax is to make it as readable
> as possible. The idea is that a
> Markdown-formatted document should be
> publishable as-is, as plain text, without
> looking like it's been marked up with tags
> or formatting instructions.

This text you see here is *actually* written in Markdown! To get a feel for Markdown's syntax, type some text into the left window and watch the results in the right.

### Tech

Dillinger uses a number of open source projects to work properly:

* [AngularJS] - HTML enhanced for web apps!
* [Ace Editor] - awesome web-based text editor
* [markdown-it] - Markdown parser done right. Fast and easy to extend.
* [Twitter Bootstrap] - great UI boilerplate for modern web apps
* [node.js] - evented I/O for the backend
* [Express] - fast node.js network app framework [@tjholowaychuk]
* [Gulp] - the streaming build system
* [Breakdance](http://breakdance.io) - HTML to Markdown converter
* [jQuery] - duh

And of course Dillinger itself is open source with a [public repository][dill]
 on GitHub.

### Installation

Dillinger requires [Node.js](https://nodejs.org/) v4+ to run.

Install the dependencies and devDependencies and start the server.

```sh
$ cd dillinger
$ npm install -d
$ node app
```

For production environments...

```sh
$ npm install --production
$ NODE_ENV=production node app
```

### Plugins

Dillinger is currently extended with the following plugins. Instructions on how to use them in your own application are linked below.

| Plugin | README |
| ------ | ------ |
| Dropbox | [plugins/dropbox/README.md] [PlDb] |
| Github | [plugins/github/README.md] [PlGh] |
| Google Drive | [plugins/googledrive/README.md] [PlGd] |
| OneDrive | [plugins/onedrive/README.md] [PlOd] |
| Medium | [plugins/medium/README.md] [PlMe] |
| Google Analytics | [plugins/googleanalytics/README.md] [PlGa] |


### Development

Want to contribute? Great!

Dillinger uses Gulp + Webpack for fast developing.
Make a change in your file and instantanously see your updates!

Open your favorite Terminal and run these commands.

First Tab:
```sh
$ node app
```

Second Tab:
```sh
$ gulp watch
```

(optional) Third:
```sh
$ karma test
```
#### Building for source
For production release:
```sh
$ gulp build --prod
```
Generating pre-built zip archives for distribution:
```sh
$ gulp build dist --prod
```
### Docker
Dillinger is very easy to install and deploy in a Docker container.

By default, the Docker will expose port 8080, so change this within the Dockerfile if necessary. When ready, simply use the Dockerfile to build the image.

```sh
cd dillinger
docker build -t joemccann/dillinger:${package.json.version}
```
This will create the dillinger image and pull in the necessary dependencies. Be sure to swap out `${package.json.version}` with the actual version of Dillinger.

Once done, run the Docker image and map the port to whatever you wish on your host. In this example, we simply map port 8000 of the host to port 8080 of the Docker (or whatever port was exposed in the Dockerfile):

```sh
docker run -d -p 8000:8080 --restart="always" <youruser>/dillinger:${package.json.version}
```

Verify the deployment by navigating to your server address in your preferred browser.

```sh
127.0.0.1:8000
```

#### Kubernetes + Google Cloud

See [KUBERNETES.md](https://github.com/joemccann/dillinger/blob/master/KUBERNETES.md)


### Todos

 - Write MORE Tests
 - Add Night Mode

License
----

MIT


**Free Software, Hell Yeah!**

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>
