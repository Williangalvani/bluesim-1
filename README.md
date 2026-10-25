# BlueSim
BlueROV2 Simulator.

A hosted version (in a pool, with 'fish') is available for short-duration testing at https://blueos.cloud/bluesim/, as part of a simulated vehicle running [BlueOS](https://github.com/bluerobotics/BlueOS), and interfaced with the [Cockpit](https://github.com/bluerobotics/cockpit) control station software. Click "Cockpit" from the BlueOS left sidebar to open the vehicle control interface.

# Download

 - [Windows](https://github.com/bluerobotics/bluesim/releases/download/latest/bluesim-windows.zip)
 - [Mac](https://github.com/bluerobotics/bluesim/releases/download/latest/bluesim-mac.zip)
 - [Linux](https://github.com/bluerobotics/bluesim/releases/download/latest/bluesim-linux.zip)
 - [Web](https://github.com/bluerobotics/bluesim/releases/download/latest/bluesim-web.zip) (Works better on firefox)

# Usage

  1. Open [QGroundControl](http://qgroundcontrol.com/)
  2. Set up a new communication link at TCP 127.0.0.1, port 5760.
  3. Launch BlueSim
  4. Chooset BlueRov2 Heavy
  5. Connect the new link in QGC
  6. Drive around using the gamepad as if controlling a real BlueRov2.


# Default keys:

If there is not SITL instance attached, these keys can be used to control the ROV:

|      Action      |  Key  |
|:----------------:|:-----:|
| Downwards        | Shift |
| Upwards          | Space |
| Forward          |   Up  |
| Backwards        |  Down |
| Rotate right     | Right |
| Rotate left      |  Left |
| Strafe right     |   D   |
| Strafe left      |   A   |
| Lights down      |   1   |
| Lights up        |   2   |
| Close gripper    |   3   |
| Open gripper     |   4   |
| Tilt camera down |   5   |
| Tilt camera up   |   6   |

# SITL integration:

    Before opening Bluesim (it launches its own SITL instance), run:

 `sim_vehicle.py -j6 -L RATBeach -v ArduSub -f vectored_6dof --model JSON --out=udpout:0.0.0.0:14550`

# Web build for Cockpit

[Cockpit](https://github.com/bluerobotics/cockpit) can run ArduSub in the browser with the web build as its physics,
shown in its Simulator widget. The two talk over the `cockpit-sitl-json` BroadcastChannel, so the build has to be
served from the same origin as Cockpit, under `bluesim/`. Cockpit's README covers the ArduSub side.

Export it with Godot 4.7 and its Web export templates installed:

```bash
godot4.7 --headless --path . --export-release Web export/web/index.html
```

The web build boots straight into a level, `?level=NAME` picking `levels/NAME.pck` (`reef` when not given).

Then render the level's map tiles, which Cockpit shows on its map widgets. This runs the export just made in headless
Chrome (set `CHROME` to use another binary) and needs Node.js 18.3 or later:

```bash
node tools/render_map_tiles.mjs --level reef --lat 33.810313 --lon -118.393867
```

`--lat` and `--lon` must match the `home` of Cockpit's SITL connection. The tiles are written to
`export/web/tiles/<level>/{z}/{x}/{y}.jpg`, served beside the build as an XYZ URL, with a ZIP of them in
`export/map-tiles/<level>.zip` for Cockpit's custom tile provider import. A failed run leaves the previous tiles in
place. Finally, copy the export into Cockpit:

```bash
rsync -a --delete export/web/ <cockpit>/public/bluesim/
```

# External Levels

External levels can be loaded by placing a .pck file in the "levels" folder at the same level as the simulator executable (the folder is created at first run if it does not exist).

The pck file must have a `custom_level.tscn` scene, which will be loaded in runtime and added as a child to `baselevel.tscn`, which contains the ROV, water, sky, and other basic functionality.
The root node at "custom_level.tscn" should preferably be a spatial node with all your custom 3D scene within it.

## Sunken Ship Level

![image](https://user-images.githubusercontent.com/4013804/104868028-09e92800-5921-11eb-9b51-67f947707725.png)
A sample level with a sunken ship can be downloaded [here](https://drive.google.com/file/d/1WH4l-l8qXnWUa5BHHtIDgaU-UnMEJ2H_/view?usp=share_link).
