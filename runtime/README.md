### Description
This is a page to demonstrate how the physics simulation layer(that I wrote during my internship with Bill Budge) works in Google Chrome using [Portable Native Client](https://developer.chrome.com/native-client) technology. The input is a json file which describes the 2D geometry of the pinball board, the various collision shapes on the board and the joints that connect the flippers to the board.

### How to Play
Click anywhere on the game window to get focus.
- Press '1' to enable/disable drawing edges.
- Press '2' to enable/disable drawing the 3D shapes.
- Press '3' to enable/disable drawing debugging texts, showing the position of the shapes on the board.
- Press '4' to add/remove a collision shape from the board.
- Press 'A'/'D' to rotate left/right flipper.
- Press 'Q'/'E' to assemble/disassemble left/right flipper.

### Support or Contact
- If you are not using Google Chrome;
- Or if you are using a mobile version of Chrome. The hope is that [WebAssembly](https://github.com/WebAssembly) will eventually work for all the browsers and platforms.
- If you saw particles stuck inside the flipper. Press 'Q'/'E' to release that flipper, let it fall and then press 'Q'/'E' again to assemble it back. In this way the flipper will shake the particles out. This happens due to box2d (the default physics engine that powered this demo) ghost vertices issue. In the future version I will let the user choose which physics engine to use.

Check out the [source code](https://github.com/billbudge/Diagrams) of the physics simulation layer.
