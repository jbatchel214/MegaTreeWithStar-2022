# Mega Tree 2022

In 2022 I built a Mega Tree with a 3D printed, 26 point Moravian star. The tree consisted of 27 strands of ws2811 led pixels. The 3D printed star totaled 113 pixels. The entire tree is controlled by an ESP32.

## Folder Structure

**MegaTreeAndStar**  This is the PlatformIO project of the tree code

**MoravianStar-FreeCAD**  This is the FreeCAD files for the 3D Printed Moravian Star

**MoravianStar-OBJs**  This is all the OBJs used with by the slicer

## Tree Assembly
The 27 strands for the tree are run off 9 pins on the ESP32, meaning 3 strands are run in series off of the pin. The strands are wired such that the strands are wired from bottom to top, then top to bottom, then bottom to top. On that first strand per pin, power is injected. There is additional power injection in between strands 2 and 3:

    Tree Top

        |--->|    |     |--->|    |
        |    |    |     |    |    |
        |    |    |     |    |    |
        |    |    |     |    |    |
        |    |    |     |    |    |
        |    |--->|     |    |--->|   Repeat for remaining strands
        ^    ^          ^    ^ 
       Pin1  |        Pin2   |
            Pwr             Pwr

    Tree Bottom

There are 65 lights per strand, for a total of 195 lights running off each pin. 12mm pixel mounting strips were used to space out the lights and keep them oriented.  I used every 3rd hole for 3" spacing.

There are several options for the center tree support.  I used the "ASAP Pole" and a "Portable Hole" for the base of the tree.  I have 3 guy lines to support it in the case of winds with rebar anchors as supports.

## Star Assembly:

The 3D printed star uses:
 - 17 of the Square Point files
 - 8 of the Triangle Point files
 - 1 of the pipe support
 - 4 of the cross supports
 - 12 of the center light supports
 - 1 of the Pipe Cover Point
 - 88 of the clips (I found an issue with the triangle points where they interfere on adjacent sides - I replaced some of the plastic clips with metal automotive trip clips)
 - you'll also need a 1" steel pipe.  I used an 18" pipe with a matching floor flange on the bottom.
 - you may not need it, but I included grooves for hose clamps (I used 1 to 3" clamps)

1) 4 pixels were installed into each of the square points (including the pipe cover point)
2) 3 pixels were installed into each of the triangle points
3) a single pixel was installed into each of the center lights supports, each of the cross supports, and the main pipe support
4) The center light supports are installed into the square hole in the center of the square points (they just snap in place) - output wires from the center light are fed into the input of the other 4 lights on the square point
5) The four cross supports are also isntalled into the square hole in the center of the square points
Note:  There should be one square point that does not have anything in it's center square hole... the main pipe support will go here, but I found that it was easier to assemble by putting this in last
6) Start with the square point that is missing the center light and attach 2 of the clips to one of its lower edges
7) Take another square point and slide it into the clips, then push the clips in place until they "snap" in.  Connect the wires from the previous point to the next one
8) A triangle point then installs next to that new square, then another square point, etc. all around
9) The next layer is all square points, however 4 of them are the ones with the cross support lights installed (be aware of the orientation of the square hole in the top-most point and the cross supports and how those all align with the main pipe support)
10) The next layer is square points and triangles again
11) Install the main pipe support (You're pushing this into the ball so that the light on the top of it slides into the square hole in the top-most point, and the cross supports settle into the supports in the main pipe support.  I found hanging a string from each of the cross supports helped, then when I went to "snap them in" I pulled down on the string and they popped in place nicely)
12) Slide the pipe cover point down onto the point (you could do this later if you're not using a floor flange)... also, if you have already installed the floor flange then slide the hose clamps into the right place now
13) Slide the main pipe support over the steel pipe, and optionally install the hose clamp over the groove on the main pipe support and tighten
14) Slide the pipe cover point into place, install the hose clamp and tighten (note there's a groove in one corner of the pipe cover point - this is to allow a space for the wires to come out)

I'm not defining connectors or how to make connections.  I leave this to the user based on their project demands.  I also do not describe how I managed power or wired my control box.  I don't think the low voltage of the LED strips pose a risk, but I'm no expert.  And the 120V (or 240V depending on country) can be dangerous, so do your research. 


Known issues with the 3D printed star:
1) Some of the clips interfere especially on adjacent sides of the triangular points, I found these worked well:  https://www.amazon.com/dp/B09PRF2Y5S?psc=1&ref=ppx_yo2ov_dt_b_product_details although you have to spread them a bit or they bite into the plastic
2) The assembly is difficult - a better approach would be a central frame that holds the lights and wiring and points are then "snapped in" from the outside-in
3) The mapping of the LEDs in the 3D star still needs some work

