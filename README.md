# ApertarCore

ApertarCore is the camera engine behind [Apertar-UI](https://github.com/Apertar-Studio/Apertar-UI). It talks to the camera through libcamera, manages recording and still capture, and keeps the camera's settings in sync with the touchscreen interface, hardware buttons and WebUI.

You can use it with Apertar-UI for a complete camera setup, or control it from a browser without a touchscreen connected.

## What Can It Do?

- Record CinemaDNG, MLV, H.264 and ProRes, depending on the Raspberry Pi model.
- Take DNG, JPEG, PNG and TIFF stills, including paired RAW + processed images.
- Capture bursts, timelapses and three-shot exposure brackets.
- Use long exposures up to 30 seconds, where supported by the sensor and driver.
- Control ISO, shutter, frame rate, white balance, resolution and crop.
- Use Auto ISO, Auto Shutter and Auto White Balance, with exposure compensation and exposure metering.
- Control the camera over your local network with the built-in WebUI and live preview.
- Use physical buttons, rotary encoders and recording indicator LEDs.
- Record audio and supported motion-sensor data alongside your clips.
- Remember camera settings between restarts and work with Apertar's presets.

Available settings depend on your board, sensor, camera driver and tuning file. A selectable frame rate is not a promise that every recording format or storage device can sustain it.

## Recording Formats

| Format | What You Get |
| --- | --- |
| **CinemaDNG / cDNG** | A folder of RAW DNG frames, ready for a RAW editing workflow. |
| **MLV** | RAW video in an MLV file, with camera and white-balance metadata. |
| **H.264** | Compressed video in an MP4 file for smaller files and convenient playback. |
| **ProRes** | 10-bit 4:2:2 video in a MOV file, with Proxy, LT, 422 and HQ profiles. |

### CinemaDNG Compression

On Pi 5-class boards, the cDNG compression menu includes:

- **Uncompressed:** stores the selected RAW samples without compression.
- **Lossless JPEG:** reduces file size without changing the stored RAW samples.
- **Lossy 3:1:** trades some image information for smaller files.
- **Lossy 5:1:** applies stronger compression for smaller files again.

The lossy options use RAW companding/quantization followed by standard lossless JPEG encoding, with an embedded inverse curve. The ratio names are quality targets, not guaranteed file sizes; scene detail and noise affect compression.

RAW storage precision can be 8, 10 or 12 bits where the sensor and mode support it. Lossless compression preserves those selected stored samples; it does not recover information already lost earlier in the camera pipeline.

### ProRes Colour Profiles

ProRes supports the following colour-space/gamma choices:

- Rec709 / Gamma 2.4
- ARRI Wide Gamut 4 / LogC4
- Sony S-Gamut3.Cine / S-Log3
- DaVinci Wide Gamut / Intermediate
- P3-D60 / Linear

The Rec709 option keeps the selected tuning's rendering; its exact curve depends on that tuning. The other profiles use ISP approximations of the published transforms.

Assign the matching input colour space and gamma in your editor when needed.

CinemaDNG has a separate preview colour-profile setting. It changes monitoring, not the recorded RAW samples or their colour-space interpretation.

## Which Raspberry Pi Can I Use?

| Board Family | Available Clip Recording |
| --- | --- |
| **Pi 5 / CM5-class** | cDNG with uncompressed, lossless and lossy options; MLV; H.264; ProRes. |
| **Pi 4 / Pi 400 / CM4-class** | Uncompressed cDNG, MLV and H.264. |
| **Pi 3 / Pi 3A+ / Zero 2 W and other supported USB 2.0-only boards** | H.264. RAW clip formats and ProRes are disabled. |

**Still photography is separate from clip-format restrictions.** Selecting H.264 does not limit still photos to 1080p. Still resolutions and crops come from the available sensor modes.

On supported Pi 3/4-class boards, H.264 uses the hardware encoder, with output up to 1920x1080 at 30 fps. The sensor may impose a lower limit. Pi 5 uses software H.264 encoding instead.

While older Raspberry Pi boards have more limited processing power and memory, making some features unavailable, they're still excellent candidates for builds focused primarily on still photography.

## Cameras and Sensors

Core includes sensor-specific handling for:

- Sony IMX477
- Sony IMX577
- Sony IMX585
- Sony IMX294

Monochrome sensors are also handled when the driver and libcamera expose a supported monochrome RAW format. Mono capture does not use Bayer colour processing or white balance.

There is no single resolution/FPS list that applies to every setup. Available modes depend on the sensor driver, bit depth, board and recording format. Core reports compatible options to the UI and WebUI.

**Building Core does not install camera drivers, device-tree overlays, libcamera or tuning files.** These must already match your hardware and operating system.

## Still Photography

Still mode includes:

- Single capture and 3-, 5- or 10-shot bursts.
- A self-timer.
- DNG, JPEG, PNG, TIFF and supported RAW + processed pairs.
- Three-shot bracketing with an adjustable EV offset. Each frame keeps its actual exposure metadata; the images are not automatically merged into HDR.
- Long-exposure shutter choices up to 30 seconds, subject to sensor limits.
- Timelapse with selectable or custom intervals, including fractional seconds.

For long exposures, the preview remains responsive using a shorter exposure. During the actual shot, Apertar-UI shows a blackout and exposure countdown. Timelapse exposure choices are limited to fit the selected interval.

The UI reports **WRITING** while capture data is still pending and prevents another capture from starting before the previous work is complete.

## Audio and Motion Data

Clip audio is recorded as a separate Broadcast WAV file with timing metadata. Keep it alongside the picture files when copying a clip.

Audio settings support the sample rates, input channels and sample formats exposed by the connected device, including 16-, 24- and 32-bit PCM and 32-bit float where available. Selecting float does not make a microphone's ADC 32-bit or undo clipping.

Core aligns audio to sensor timestamps and handles capture discontinuities and small clock drift. H.264 and ProRes include a container timecode track; cDNG includes per-frame timecode. MLV stores capture-clock metadata, although an editor may not expose it as a native timecode track.

Supported IMUs can write a timestamped gyro sidecar for stabilization workflows. Supported drivers include MPU6050, BMI160, BMI270 and ICM42688-P. The sensor must be connected and configured correctly.

## WebUI

Open the camera's address from a phone, tablet or computer on the same network:

```text
http://<camera-ip>:8000
```

For example, an Apertar image configured with the hostname `apertar` and working mDNS can be reached at:

```text
http://apertar.local:8000
```

The WebUI provides live preview, camera controls, recording and still capture, settings, presets and media downloads. Core can run without the native touchscreen UI.

**Use a trusted LAN or camera hotspot.** The WebUI is not an authenticated internet-facing service. Same-origin checks are not a replacement for authentication; do not expose port 8000 directly to the internet.

Wi-Fi, hotspot, SSH and `.local` discovery require the corresponding system configuration. They are not installed by building Core alone.

## Tuning and Colour Calibration

Default tuning files are selected for both the sensor and the Raspberry Pi's image-processing pipeline:

| Platform | Default Folder | JSON Target |
| --- | --- | --- |
| Pi 5-class / PiSP | `/home/pi/Tuning-Files/defaults-pisp/` | `pisp` |
| Pi 3/4-class / VC4 | `/home/pi/Tuning-Files/defaults-vc4/` | `bcm2835` |

For example, an IMX585 on Pi 5 uses the compatible `imx585.json` from `defaults-pisp`. Pi 3/4 needs the VC4 version, not a renamed PiSP file.

Custom tuning files can be selected from `/home/pi/Tuning-Files/` or a `Tuning-Files` folder on recording media. Changing files restarts the camera pipeline, without rebooting the Pi. The last custom selection is remembered; clearing it returns to automatic sensor-default selection.

Manual white balance uses the selected tuning's calibration rather than hardcoded sensor gain tables. White balance is unavailable for mono sensors and is disabled for tuning files without the required calibrated AWB data.

User colour-profile overrides live in:

```text
/home/pi/apertar-color-profiles.conf
```

RAW metadata describes the sensor calibration and measured white balance. Preview LUTs, monitoring overlays and display calibration are not baked into RAW recordings.

## Hardware Controls

Hardware mappings are configured in:

```text
/home/pi/apertar-hardware.conf
```

Depending on the connected hardware, you can configure GPIO/PCF8574 buttons, rotary controls, recording LEDs, motion sensors and other supported accessories.

Buttons can step settings, jump to specific values, toggle monitoring tools, trigger standby or launch configured scripts. A button can run up to six actions, for example:

```ini
P0=iso200,fps24,wb5600
P1=focus_peaking,zebra
P2=capture
```

These are mapping examples, not a complete hardware configuration. Use the correct section, pin numbering and wiring for your device. Grouped toggles flip independently; they do not force every option into the same state.

The `run_script` action can run up to five configured scripts. Scripts run as the Core service user, without automatic administrator privileges. Only use scripts you trust.

## Building From Source

### Requirements

- A Linux environment with a matching camera driver and libcamera stack.
- CMake 3.20 or newer and a C++20 compiler.
- `pkg-config`, libyaml and ALSA development headers.
- libcamera and libcamera-base development files for real camera capture.
- FFmpeg/libav development files for the H.264 and ProRes plugins.
- JPEG development files for JPEG-dependent features; TurboJPEG is preferred when available.

On a compatible Raspberry Pi OS installation, the development packages typically include:

```bash
sudo apt install build-essential cmake pkg-config \
  libcamera-dev libyaml-dev libasound2-dev \
  libavcodec-dev libavformat-dev libavutil-dev \
  libjpeg-dev libturbojpeg0-dev
```

If your image uses a custom libcamera build, use its matching headers and libraries. Do not replace a working custom camera stack blindly with distribution packages.

### Compile

From the root of the Core repository:

```bash
cmake -S . -B ../ApertarCore-build -DCMAKE_BUILD_TYPE=Release
cmake --build ../ApertarCore-build -j2
```

Use `-j1` on memory-limited boards if the compiler runs out of RAM.

The main outputs are:

```text
ApertarCore-build/
  apertar-core
  apertar-h264-libav.so
  apertar-prores-libav.so
```

The encoder plugins are built when the required libav development libraries are found. Keep them beside the executable when deploying a build, with their FFmpeg runtime dependencies available. Core loads them when needed.

Check CMake's output: it can configure without libcamera or libav, but that does not provide a working real-camera build or those recording backends.

The optional optimized Pi 5 ProRes codec has a separate build recipe under `src/prores/ffmpeg/`.

## Running Core

On an Apertar image that already has its service installed:

```bash
sudo systemctl start apertar-core.service
```

To run a source build manually, stop any existing Core service first so two processes do not compete for the camera:

```bash
sudo systemctl stop apertar-core.service
../ApertarCore-build/apertar-core \
  --socket /tmp/apertar-core.sock \
  --media /media/RAW
```

Run this from the Core repository folder. For a source-only setup without the service, skip the service command. Press `Ctrl+C` to stop the manual process; restart the service afterward if you normally use it.

Before recording, make sure the intended media is actually mounted at the selected path. An existing folder by itself does not prove that an external drive is mounted there.

### Launch Options

| Option | Purpose |
| --- | --- |
| `--socket PATH` | Local control/preview socket. Default: `/tmp/apertar-core.sock`. |
| `--media PATH` | Recording destination. Default: `/media/RAW`. |
| `--web-bind ADDR` | WebUI listening address. Default: `0.0.0.0`. |
| `--web-port PORT` | WebUI port. Default: `8000`; `0` disables it. |
| `--simulate` | Run without opening a physical camera, for interface development. |
| `--help` | Show command-line help. |

The service user needs access to the camera, recording media and configured hardware. Hardware H.264 also requires access to the encoder and DMA heap devices.

## Where Are Recordings Saved?

The default media root is `/media/RAW`. Clip folders contain the picture files and any associated audio, gyro or recovery sidecars. Still photos are saved in the `Photos` folder.

Names use the capture date and an incrementing counter. Keep a clip's associated files together when moving or backing it up.

Settings are saved separately from recordings. Core's camera settings normally live in `~/.config/apertar-core/camera_settings.conf`; shared UI settings and presets are managed by the Apertar application.

## For Developers

ApertarCore is the source of truth for camera and capture state. Native UI commands, WebUI requests and hardware actions reach the same controller and busy-state checks.

```text
Apertar-UI / WebUI / Hardware Controls
                  |
                  v
             ApertarCore
                  |
               libcamera
                  |
       RAW / processed camera streams
                  |
        Recording, stills and preview
```

The native UI uses a Unix socket with line-delimited JSON commands and asynchronous events. Preview buffers are shared using DMA-BUF file descriptors rather than copying every frame through JSON. Browser preview uses MJPEG instead.

A few socket examples, with `socat` installed:

```bash
printf '{"cmd":"get_state"}\n' | socat - UNIX-CONNECT:/tmp/apertar-core.sock
printf '{"cmd":"set_fps","value":24.0}\n' | socat - UNIX-CONNECT:/tmp/apertar-core.sock
printf '{"cmd":"record_start"}\n' | socat - UNIX-CONNECT:/tmp/apertar-core.sock
printf '{"cmd":"record_stop"}\n' | socat - UNIX-CONNECT:/tmp/apertar-core.sock
```

The WebUI also exposes current state at `GET /api/state`.

Background recording workers and bounded queues help keep camera requests moving. Recording errors and detected dropped frames are reported rather than hidden. Capture and media-operation guards prevent conflicting actions while data is still being written, but cannot prevent every kernel, driver, storage or power failure.

## Recording Safely

- Use storage fast enough for your selected format, resolution and frame rate.
- Wait for recording to stop and **WRITING** to finish before ejecting media.
- Use the camera's normal shutdown before disconnecting power.
- Update Core and Apertar-UI together so their controls and capabilities match.

## Credits

ApertarCore builds on libcamera, Raspberry Pi's camera pipeline and other open-source projects. Inspiration comes from [Schoolpost's cinepi-raw](https://github.com/cinepi/cinepi-raw).
Thanks to the developers, contributors and camera builders who help test and improve Apertar.
