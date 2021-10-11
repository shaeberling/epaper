# Good Display using ESP-IDF on ESP32

This has been tested on an ESP32 connected to a Good Display GDEM213U23 epaper display.

## Setup
 - Ensure that you had the ESP-IDF toolchain installed.
 - Connect the six data pins, plus 3.3V and GND to your ESP32
 - If needed, change the pin mappnigs at the top of `main.c`.
 - Edit `idf.sh` to point to the port of your ESP32.
 - Run `./idf.sh flash`
 - You should see the following on the display:

![Setup](demo_setup.jpg?raw=true)

## Change image
To change the image, you need to provide data for black and red separately:

 - Create a PNG that is exactly 250x122 pixels for your BLACK data.
   - Rotate it 90 degree clockwise and save.
 - Create another PNG with the same dimensions for your RED data
   - Invert the image so that everything white is what you want to be red later.
   - Rotate it 90 degrees clockwise and save.
 - Go to https://javl.github.io/image2cpp/ and for each image:
   - Load it
   - Select flip "vertically".
   - Select "bytes only" for the output format.
   - Hit "generate code"
   - Copy the bytes to the `demo.h` file for the appropriate image (either bw or red).
