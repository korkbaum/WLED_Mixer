Dear WLED-Community,

I would like to share a pretty quirky project with you; a 1960s Grundig Stereo Mixer 422 was turned into a WLED remote control unit with touch screen.

Some technical details:
- ESP32
- 2x ADS1115 for 8 analog channels / 8 potentiometer values over I2C 
- 2x Duppa.net I2C Rotary Encoder Mini
- Nextion Touch Display NX3224T028

This modified mixer essentially converts any input data, analog or touch, into WLED JSON API calls, sent over WIFI/UDP.
While I seriously doubt anyone will do the same conversion at least some of the code might be useful for similar projects where hardware control is desired to control WLED devices. 

This is what it looks like now:
![IMG_0111](https://user-images.githubusercontent.com/16290782/188831693-f6615eca-c747-47a5-b1a7-977c0d5ff49b.JPG)

Inside view: 
![IMG_0102 conv](https://user-images.githubusercontent.com/16290782/188832203-d473bc22-6cb8-4f89-98ea-4d14892e7fd9.jpeg)

This is what it looked like before (the 422 model) (from https://www.hifi-archiv.info/Grundig/1966-1/grundig29.jpg):
![grundig29](https://user-images.githubusercontent.com/16290782/188831966-dc0bdb3e-7b76-41f7-9854-e24520ee03bc.jpg)

The lights you see in the demo video (will be added shortly) are also built by myself, more details can be found at https://mooodlights.com (in German).

Let me have any comments or questions.

- korkbaum
