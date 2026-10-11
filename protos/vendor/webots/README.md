# Local Webots R2025a assets

These files were copied from Cyberbotics' Webots **R2025a** release:
https://github.com/cyberbotics/webots/tree/R2025a

They supply the countryside environment in `worlds/drone.wbt` and an optional
Mavic 2 Pro model. All required meshes, paint materials, textures and mountain
sky images are included locally (approximately 100 MB). No runtime asset
downloads are required for this world.

`EXTERNPROTO` references and `webots://` asset paths were changed to relative
local paths. The scene's original Mavic drone was replaced by the project's
Iris; its controller, sensors, physics and flight tune remain project-owned.
The upstream world's 8 ms timestep and damping were replaced with the existing
2 ms Iris setup.

`manifest.json` records the release, upstream Git tree, source URL and SHA-256
hashes of the copied files after path changes. The original license headers are
preserved. Individual files have different licenses: Apache 2.0, CC BY 4.0, or
the [Webots assets license](https://cyberbotics.com/webots_assets_license), which
limits those assets to use with Webots. The root project's GPL license does not
replace these third-party licenses. `LICENSE` contains the upstream Apache 2.0
license text.

## Optional Mavic model

The local model is:

`projects/robots/dji/mavic/protos/Mavic2Pro.proto`

To place a parked Mavic alongside Iris, add this to a world in `worlds/`:

```vrml
EXTERNPROTO "../protos/vendor/webots/projects/robots/dji/mavic/protos/Mavic2Pro.proto"

Mavic2Pro {
  translation 3 2 0.1
  controller "<none>"
}
```

This is the stock Mavic model, including its gimbal camera, GPS, compass and
InertialUnit. It has no accelerometer and uses different motor names, signed
propeller speeds, thrust constants, mass and inertia. Using it with the Iris
Betaflight bridge requires a sensor/motor adapter and a separate flight tune.
The stock Mavic flight controller is not included.
