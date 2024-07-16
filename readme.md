# Table of Contents
- [Table of Contents](#table-of-contents)
  - [Visual Studio Code and PlatformIO](#visual-studio-code-and-platformio)
  - [Flashing your device](#flashing-your-device)

## Visual Studio Code and PlatformIO
If you want to use or edit this project, please make sure you have [VSCode](https://code.visualstudio.com/Download) installed with the [PlatformIO](https://platformio.org/install/ide?install=vscode) extension.

Please also install the Arduino FreeRTOS.

Any missing libraries can be installed in the platformIO PIO Home tab found on the left side of your screen:

![Where to find the PlatformIO options](/Images/image.png)

## Flashing your device
If you have successfully installed all the needed dependencies you will be able to flash your arduino. The software is made for the Arduino m0. For use on a different device you will have to modify the code.

To flash the screen and esp, please follow these steps:

1. Open the platformIO extension on the left side of your screen.
2. Click on "Open Project": ![OpenProject](/Images/image-3.png)
3. Please select your repository.
4. Check if your IDE shows you any errors and resolve these.
5. Your platformIO should now show these options: ![alt text](/Images/image-4.png)
6. Before flashing, make sure your VSCode will flash to the desired device. you can check this down at the Taskbar:
![alt text](/Images/image-5.png)
1. If you select other options than build, your project will still build before executing the desired task.
2. To flash your screen, select "Upload" or "Upload and Monitor"
