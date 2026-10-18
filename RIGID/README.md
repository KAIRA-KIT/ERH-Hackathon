# RIGID — variable-stiffness tendon arm (ERH Physical AI Hackathon)

- `firmware/RIGID_v2/RIGID_v2.ino` — XIAO ESP32-C3 firmware: 6x ST3215-HS tendon servos + mecanum base, USB serial and Bluetooth LE ("RIGID"). Calibration saved in flash.
- `controller/RIGID_controller.html` — open in Chrome/Edge, Connect via Bluetooth: IK reach, shape control, lock/release, live 3D view, calibration, WASD driving.
- `simulator/` — 3D simulator of the arm (kinematics, IK, tendon statics, friction lock, servo current). Serve the folder (e.g. `python3 -m http.server`) and open index.html.
- `ik/rigid_ik.py` — Python IK (same model as the simulator).
- `renders/` — CAD renders.

Calibrate: Q (soft-tension, arm straightens itself) -> tap -> Z (save zero). Full command list is in the .ino header.
