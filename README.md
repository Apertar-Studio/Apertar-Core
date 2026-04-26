# ApertarCore

ApertarCore is a libcamera-based capture backend for Raspberry Pi camera systems. It manages camera configuration, preview delivery, recording, still capture, and control-state synchronization for Apertar-UI.

It is designed for embedded camera workflows on Raspberry Pi 5, with a focus on low-latency preview, reliable raw capture, and a clean separation between the UI and the capture pipeline.

## Highlights

- Direct camera control through libcamera
- Dual-stream capture pipeline:
    - preview stream for the UI
   - raw stream for recording and stills
- Low-latency preview transport over a Unix socket with DMA-BUF file descriptor passing
- cDNG sequence recording
- DNG still capture
- JSON control protocol with synchronous replies and asynchronous events
- Sensor-aware mode handling for supported cameras
- Optimized raw-to-DNG write path for Raspberry Pi 5 workflows

## Current Sensor Support

### Sony IMX585

- `3856x2180` up to `30 fps`
- `1928x1090` up to `60 fps`

### Sony IMX477

- `1332x990` up to `100 fps`
- `2028x1080` up to `60 fps`
- `2028x1520` up to `30 fps`

## Architecture

```text
Apertar-UI
  |
  | Unix socket control/events
  | + preview DMA-BUF FDs
  v
ApertarCore
  |
  | direct libcamera API
  v
Camera pipeline
  |
  | raw stream                  | preview stream
  v                             v
cDNG / DNG writer              UI preview renderer
```

### Control Path

- ApertarCore listens on a local Unix socket
- Commands are line-delimited JSON messages
- Replies return command status and updated state
- Asynchronous events report settings, recording state, camera state, and preview-frame metadata

### Preview Path

- Preview frames are published over the same Unix socket as JSON metadata plus an attached DMA-BUF file descriptor using `SCM_RIGHTS`
- The UI imports that DMA-BUF directly into EGL/OpenGL for low-latency display
- Preview delivery is latest-frame-wins so the UI cannot stall capture or recording

### Recording Path

- Raw capture is staged quickly so libcamera requests can be released promptly
- Worker threads write CinemaDNG frames in the background
- The writer uses a raw-first TIFF/DNG layout and a gather-write path to reduce unnecessary memory copies

## Build Requirements

- CMake 3.20 or newer
- A C++20 compiler
- `pkg-config`
- `libcamera` and `libcamera-base` development files

On Raspberry Pi OS this typically means using a matching libcamera development environment for the kernel and camera stack you plan to run.

## Build

```bash
cmake -S ApertarCore -B ApertarCore/build -DCMAKE_BUILD_TYPE=Release
cmake --build ApertarCore/build -j$(nproc)
```

The output binary is:

```text
ApertarCore/build/apertar-core
```

## Run

Basic launch:

```bash
./ApertarCore/build/apertar-core
```

Common Raspberry Pi launch example:

```bash
./ApertarCore/build/apertar-core --socket /tmp/apertar-core.sock --media /media/RAW
```

For IMX585 workflows that use a custom tuning file:

```bash
LIBCAMERA_RPI_TUNING_FILE=/home/pi/imx585.json \
./ApertarCore/build/apertar-core --socket /tmp/apertar-core.sock --media /media/RAW
```

### Command-Line Options

```text
--socket PATH    Control socket path (default: /tmp/apertar-core.sock)
--media PATH     Recording media root (default: /media/RAW)
--simulate       Run without opening the camera
--help           Show usage
```

## Control Protocol

The default control socket is:

```text
/tmp/apertar-core.sock
```

Example commands:

```bash
printf '{"cmd":"get_state"}\n' | socat - UNIX-CONNECT:/tmp/apertar-core.sock
printf '{"cmd":"set_resolution","width":3856,"height":2180}\n' | socat - UNIX-CONNECT:/tmp/apertar-core.sock
printf '{"cmd":"set_fps","value":24.0}\n' | socat - UNIX-CONNECT:/tmp/apertar-core.sock
printf '{"cmd":"record_start"}\n' | socat - UNIX-CONNECT:/tmp/apertar-core.sock
printf '{"cmd":"record_stop"}\n' | socat - UNIX-CONNECT:/tmp/apertar-core.sock
```

## Output Layout

- cDNG recordings are written under the selected media root
- Still captures are written to a `Photos` folder under the media root
- Clip and still naming are date-based with incrementing clip counters

## Notes
- The UI should treat preview delivery as transient and non-blocking
- Sensor-specific limits and capture behavior are handled in the backend, not the UI

## Credit
- Inspiration has been taken from Schoolpost's project cinepi-raw: https://github.com/cinepi/cinepi-raw
