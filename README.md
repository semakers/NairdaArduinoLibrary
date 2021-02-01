![](https://www.nairda.com.mx/TopicForumResources/blink.gif)  
  
  
![](https://www.nairda.com.mx/TopicForumResources/youtube.png)  [YouTube CHANNEL](https://www.youtube.com/channel/UCfOz0bfkMNRf96p4787bPTQ)  
  
![](https://www.nairda.com.mx/TopicForumResources/miniIcon.png)  Nairda Robot Programming  
  
Nairda Robot Programming is a platform for programming hardware projects from the algorithm to run on the microcontroller to the design of the graphic interface to control the device via bluetooth 2.0 or ble 4.2 from your Android or Ios samartphone or tablet.  
  
  
Wireless  
Nairda connects with your prototype either with Nairda board or with arduino UNO, NANO, LEONARDO or MEGA and a Bluetooth Low Energy 4.0 (HC-08 or HM10) or 2.0 (HC-05 or Hc-06) and ESP WROOM-32.  
  
Without computer  
Once our arduino or nairda board has the Nairda firmware loaded, it is no longer necessary to use the computer again to program our projects.  
  
  
Control  
Design your own graphical interface for the control of your prototype just by dragging joysticks or buttons and linking them to the programming blocks.  
  
  
Without code  
The graphic programming blocks that nairda uses are inspired by those of Google's Blockly (Scratch) with some modifications, to work better with hardware components.  
  
How to assemble  
  
Make the following connections between your arduino and your bluetooth:  
  

-   Arduino TX -> bluetooth Rx.
-   Arduino RX -> bluetooth Tx.
-   Arduino Vcc -> bluetooth Vcc.
-   Arduino Gnd -> bluetooth Gnd.

  
![](https://www.nairda.com.mx/TopicForumResources/supported.png)  
  
Remember that to control DC motors you must connect an h bridge like the L293D, L298D or TB612.  
  
  
  
Install Arduino library  
  
To install it we must follow the following steps:  
  

-   To install the Nairda Firmware on your Arduino you must download the library by clicking  [here](https://github.com/semakers/NairdaArduinoLibrary/archive/master.zip), unzip the file "NairdaArduino Library-Master.zip" it contains the "NairdaArduinoLibrary-Master" folder

  
![](https://www.nairda.com.mx/assets/img/install-arduino/unzip_folder.png)  
  

-   Rename the "NairdaArduinoLibrary-master" folder to "NairdaArduinoLibrary".

  
![](https://www.nairda.com.mx/assets/img/install-arduino/rename_folder.png)  
  

-   Copy the NairdArduinoLibrary folder into your Documents-> Arduino-> libraries folder.

  
![](https://www.nairda.com.mx/assets/img/install-arduino/libraries_folder.png)  
  

-   Now open your Aruino IDE and click File-> Examples-> Nairda-> NairdaFirmware.

  

Code:  [Select]

`#include <nairda.h>  
  
  
void setup() {  
#if defined(ARDUINO_ARCH_ESP32)  
nairdaBegin("NairdaESP32");//This is the name of the BLE device  
#else  
nairdaBegin(9600);  
#endif  
}  
  
void loop() {  
nairdaLoop();  
}`  
  

-   In tools select your arduino and the port, connect your arduino to the computer and click upload, when the firmware has finished uploading your arduino is ready to work with Nairda Robot Programming.

  
![](https://www.nairda.com.mx/TopicForumResources/control.png)  Control  
  
When opening a project the first thing we will see is the control. In this area we can add and configure buttons and joysticks to execute code blocks. To add a joystick or button we must click on the control icon.By long pressing on a button or joystick you can drag and drop onto the canvas.  
  
![](https://www.nairda.com.mx/TopicForumResources/controlProject.gif)  
  
To assign programming blocks to buttons or joysticks, we must click on them and select a block, remember that a joystick is a set of four buttons and each button has an event when pressed and another when released.  
  
![](https://www.nairda.com.mx/TopicForumResources/configJoistick.gif)![](https://www.nairda.com.mx/TopicForumResources/configButton.gif)  
  
  
![](https://www.nairda.com.mx/TopicForumResources/puzzle.png)  Programming blocks  
  
By pressing the puzzle button we enter the block programming area. In this area we will program all the logic of our robot with graphic blocks.  
  
![](https://www.nairda.com.mx/TopicForumResources/enterBlocks.gif)  
  
By clicking on the puzzle button, the block panel will be displayed, in which we will find all the different blocks that we can use and generate. To add a block to the canvas we must make a sustained clock over it and drag it onto the canvas.  
  
![](https://www.nairda.com.mx/TopicForumResources/blockTypes.gif)![](https://www.nairda.com.mx/TopicForumResources/dragBlock.gif)  
  
  
You can find more information on our website by clicking  [HERE](http://www.nairda.com.mx/).