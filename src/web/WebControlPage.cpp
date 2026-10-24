#include "web/WebControlPage.hpp"

namespace apertar {

std::string_view webControlPageHtml()
{
    static const char page[] = R"APERTAR(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Apertar Control</title>
  <link rel="icon" type="image/png" href="/icons/apertar.png">
  <link rel="apple-touch-icon" href="/icons/apertar.png">
  <style>
    :root {
      color-scheme: dark;
      --bg-top: #27272a;
      --bg-mid: #09090b;
      --bg-bottom: #000000;
      --card-bg: rgba(9, 9, 11, 0.72);
      --card-border: #27272a;
      --panel-bg: #09090b;
      --surface: #18181b;
      --surface-2: #111113;
      --surface-3: #171717;
      --text: #ffffff;
      --muted: #a1a1aa;
      --muted-2: #71717a;
      --line: #18181b;
      --ok: #34d399;
      --danger: #ef4444;
      --danger-2: #dc2626;
      --shadow: rgba(0, 0, 0, 0.28);
      --shadow-soft: rgba(255, 255, 255, 0.08);
      --radius-xl: 28px;
      --radius-lg: 22px;
      --radius-md: 18px;
      --radius-sm: 14px;
      --content-width: 1320px;
      font-family: "Inter", "Segoe UI", sans-serif;
    }

    * {
      box-sizing: border-box;
    }

    html, body {
      margin: 0;
      min-height: 100%;
      color: var(--text);
    }

    body {
      min-height: 100vh;
      min-height: 100dvh;
      background:
        radial-gradient(120% 72% at 50% -8%, var(--bg-top) 0%, var(--bg-mid) 46%, var(--bg-bottom) 100%);
      background-repeat: no-repeat;
      background-size: 100vw 100dvh;
      background-position: center top;
      background-color: var(--bg-bottom);
    }

    body.settings-open,
    body.media-browser-open {
      overflow: hidden;
    }

    button,
    input {
      font: inherit;
    }

    button {
      border: 0;
      cursor: pointer;
    }

    .page {
      max-width: var(--content-width);
      min-height: 100vh;
      margin: 0 auto;
      padding: 16px;
      display: flex;
      flex-direction: column;
      gap: 20px;
    }

    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
    }

    .brand {
      display: flex;
      align-items: center;
      gap: 14px;
    }

    .brand-icon {
      width: 56px;
      height: 56px;
      display: flex;
      align-items: center;
      justify-content: center;
      flex-shrink: 0;
    }

    .brand-logo {
      width: 56px;
      height: 56px;
      object-fit: contain;
      display: block;
      user-select: none;
      -webkit-user-drag: none;
    }

    .brand-title {
      font-size: clamp(22px, 2vw, 32px);
      font-weight: 600;
      letter-spacing: -0.03em;
      margin: 0;
    }

    .brand-subtitle {
      margin-top: 6px;
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 14px;
      color: var(--muted);
      flex-wrap: wrap;
    }

    .brand-divider {
      color: #3f3f46;
    }

    .topbar-actions {
      display: flex;
      align-items: center;
      gap: 12px;
    }

    .status-cluster {
      display: flex;
      align-items: center;
      gap: 10px;
      min-width: 0;
    }

    .date-time-pill,
    .status-pill {
      display: flex;
      align-items: center;
      gap: 10px;
      border-radius: 999px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.72);
      padding: 10px 16px;
      justify-content: center;
      color: #d4d4d8;
      font-size: 14px;
      font-weight: 500;
    }

    .date-time-pill {
      color: #f4f4f5;
      white-space: nowrap;
      letter-spacing: -0.01em;
    }

    .status-dot {
      font-size: 14px;
      color: #00c951;
    }

    .status-dot.recording {
      color: #fb2c36;
    }

    .status-dot.timelapse {
      color: #fb2c36;
    }

    .status-dot.writing,
    .status-dot.no-media {
      color: #ffd54a;
    }

    .icon-button {
      width: 44px;
      height: 44px;
      border-radius: 18px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.72);
      color: #d4d4d8;
      font-size: 18px;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: transform 0.12s ease, background 0.12s ease, border-color 0.12s ease;
    }

    .icon-button:hover {
      background: rgba(24, 24, 27, 0.92);
      border-color: #3f3f46;
    }

    .icon-button:active {
      transform: scale(0.96);
    }

    .icon-image {
      width: 20px;
      height: 20px;
      object-fit: contain;
      display: block;
      opacity: 0.96;
      user-select: none;
      -webkit-user-drag: none;
    }

    .icon-button .icon-image {
      width: 22px;
      height: 22px;
    }

    .main-grid {
      display: grid;
      grid-template-columns: minmax(0, 1.1fr) minmax(320px, 0.9fr);
      gap: 20px;
      flex: 1;
      min-height: 0;
    }

    .column {
      display: flex;
      flex-direction: column;
      gap: 20px;
      min-width: 0;
    }

    .card {
      border-radius: var(--radius-xl);
      border: 1px solid var(--card-border);
      background: var(--card-bg);
      box-shadow: 0 24px 50px var(--shadow);
      overflow: hidden;
    }

    .card-body {
      padding: 20px;
    }

    .preview-card {
      background: #000000;
    }

    .preview-stage {
      position: relative;
      min-height: 0;
      aspect-ratio: var(--preview-aspect-ratio, 16 / 9);
      background: linear-gradient(135deg, #18181b, #030303);
      overflow: hidden;
    }

    .preview-stream {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      object-fit: cover;
      display: block;
      background: #000000;
    }

    .preview-processed-canvas {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      display: none;
      background: #000000;
      z-index: 0;
      pointer-events: none;
    }

    .preview-stage.processed-active .preview-processed-canvas {
      display: block;
    }

    .preview-stage.processed-active .preview-stream {
      opacity: 0;
    }

    .preview-grid {
      position: absolute;
      inset: 0;
      opacity: 0.4;
      background-image:
        linear-gradient(rgba(255,255,255,0.05) 1px, transparent 1px),
        linear-gradient(90deg, rgba(255,255,255,0.05) 1px, transparent 1px);
      background-size: 48px 48px;
    }

    .preview-smpte {
      position: absolute;
      inset: 0;
      display: none;
      flex-direction: column;
      background: #000000;
      z-index: 0;
      pointer-events: none;
    }

    .preview-stage.smpte-active .preview-smpte {
      display: flex;
    }

    .preview-stage.smpte-active .preview-stream {
      opacity: 0;
    }

    .preview-stage.smpte-active .preview-processed-canvas {
      display: none;
    }

    .preview-stage.smpte-active .preview-grid {
      opacity: 0;
    }

    .false-color-legend {
      position: absolute;
      left: 12px;
      top: 50%;
      transform: translateY(-50%);
      width: 40px;
      padding: 7px;
      border-radius: 11px;
      border: 1px solid rgba(255,255,255,0.09);
      background: rgba(0, 0, 0, 0.78);
      display: none;
      z-index: 1;
      pointer-events: none;
      box-shadow: 0 8px 18px rgba(0, 0, 0, 0.24);
    }

    .preview-stage.false-color-legend-active .false-color-legend {
      display: block;
    }

    .false-color-legend-row {
      display: flex;
      align-items: flex-start;
      gap: 2px;
    }

    .false-color-legend-labels {
      width: 15px;
      display: flex;
      flex-direction: column;
      gap: 1px;
    }

    .false-color-legend-label {
      height: 14px;
      color: #ffffff;
      font-family: "Inter", sans-serif;
      font-size: 9px;
      font-weight: 700;
      line-height: 14px;
      letter-spacing: -0.01em;
      text-rendering: geometricPrecision;
      -webkit-font-smoothing: antialiased;
      text-shadow: 0 1px 1px rgba(0, 0, 0, 0.5);
    }

    .false-color-legend-scale {
      width: 9px;
      padding: 1px;
      border-radius: 5px;
      border: 1px solid rgba(255,255,255,0.08);
      background: rgba(0, 0, 0, 0.09);
      display: flex;
      flex-direction: column;
      gap: 1px;
    }

    .false-color-legend-swatch {
      width: 100%;
      height: 14px;
      border-radius: 2px;
      opacity: 0.98;
    }

    .smpte-row {
      display: flex;
      width: 100%;
    }

    .smpte-row.top {
      flex: 67 1 0;
    }

    .smpte-row.middle {
      flex: 9 1 0;
    }

    .smpte-row.bottom {
      flex: 24 1 0;
    }

    .smpte-row > span {
      height: 100%;
    }

    .smpte-row.top > span,
    .smpte-row.middle > span {
      flex: 1 1 0;
    }

    .preview-badges {
      position: absolute;
      top: 16px;
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      z-index: 1;
    }

    .preview-badges.left {
      left: 16px;
    }

    .preview-badges.right {
      right: 16px;
    }

    .badge {
      border-radius: 999px;
      background: rgba(0, 0, 0, 0.7);
      padding: 7px 12px;
      font-size: 12px;
      color: #d4d4d8;
      backdrop-filter: blur(12px);
      white-space: nowrap;
    }

    .preview-center {
      position: absolute;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 18px;
      z-index: 1;
      transition: opacity 0.18s ease;
    }

    .preview-center.hidden {
      opacity: 0;
      pointer-events: none;
    }

    .preview-center-card {
      border-radius: 999px;
      border: 1px solid #3f3f46;
      background: rgba(9, 9, 11, 0.72);
      backdrop-filter: blur(12px);
      padding: 32px;
      text-align: center;
      min-width: 220px;
    }

    .preview-center-card.error {
      border-color: rgba(239, 68, 68, 0.45);
    }

    .preview-center-icon {
      font-size: 44px;
      margin-bottom: 10px;
    }

    .preview-center-label {
      font-size: 14px;
      font-weight: 500;
      color: #d4d4d8;
    }

    .preview-footer {
      position: absolute;
      left: 16px;
      right: 16px;
      bottom: 16px;
      display: flex;
      align-items: flex-end;
      justify-content: space-between;
      gap: 16px;
      z-index: 1;
    }

    .time-card {
      border-radius: 18px;
      background: rgba(0, 0, 0, 0.7);
      padding: 14px 16px;
      backdrop-filter: blur(12px);
      min-width: 170px;
      border: 1px solid transparent;
      transition: background 0.18s ease, border-color 0.18s ease;
    }

    .time-card.hidden {
      display: none;
    }

    .time-card.recording {
      background: rgba(255, 68, 68, 0.1);
      border-color: rgba(255, 68, 68, 0.3);
    }

    .timecode-value {
      font-family: "SFMono-Regular", Consolas, "Liberation Mono", monospace;
      font-size: clamp(24px, 3vw, 34px);
      font-weight: 600;
      color: #ffffff;
      transition: color 0.18s ease;
    }

    .time-card.recording .timecode-value {
      color: #ff4d4d;
    }

    .time-value.recording {
      color: var(--danger);
    }

    .record-button {
      width: 96px;
      height: 96px;
      border-radius: 999px;
      border: 4px solid #ef4444;
      background: rgba(239, 68, 68, 0.1);
      box-shadow: 0 24px 48px rgba(220, 38, 38, 0.12);
      display: flex;
      align-items: center;
      justify-content: center;
      transition: transform 0.15s ease, background 0.15s ease, box-shadow 0.15s ease;
    }

    .record-button:active {
      transform: scale(0.95);
    }

    .record-button:disabled {
      filter: grayscale(1);
      opacity: 0.45;
      cursor: not-allowed;
      box-shadow: none;
    }

    .record-button.recording {
      border-color: #f87171;
      background: #dc2626;
      box-shadow: 0 24px 48px rgba(220, 38, 38, 0.3);
    }

    .record-button.photo-mode {
      border-color: rgba(255, 255, 255, 0.22);
      background: rgba(255, 255, 255, 0.1);
      box-shadow: 0 24px 48px rgba(255, 255, 255, 0.12);
    }

    .record-button.photo-mode.photo-pulse {
      border-color: rgba(255, 255, 255, 0.42);
      background: rgba(255, 255, 255, 0.16);
      box-shadow: 0 24px 48px rgba(255, 255, 255, 0.2);
    }

    .record-button.photo-mode.timelapse {
      border-color: #f87171;
      background: #dc2626;
      box-shadow: 0 24px 48px rgba(220, 38, 38, 0.3);
      animation: timelapsePulse 1.25s ease-in-out infinite;
    }

    @keyframes timelapsePulse {
      0%, 100% { box-shadow: 0 24px 48px rgba(220, 38, 38, 0.22); }
      50% { box-shadow: 0 24px 56px rgba(220, 38, 38, 0.42); }
    }

    .record-icon-dot {
      width: 40px;
      height: 40px;
      border-radius: 999px;
      background: #ef4444;
    }

    .record-icon-stop {
      width: 36px;
      height: 36px;
      border-radius: 10px;
      background: #ffffff;
    }

    .record-icon-camera {
      width: 48%;
      height: 48%;
      min-width: 18px;
      min-height: 18px;
      max-width: 34px;
      max-height: 34px;
      background: center / contain no-repeat url('/icons/camera.png');
      opacity: 0.96;
      transition: transform 0.16s ease, opacity 0.16s ease;
    }

    .record-icon-timelapse {
      width: 52%;
      height: 52%;
      min-width: 20px;
      min-height: 20px;
      max-width: 38px;
      max-height: 38px;
      background: center / contain no-repeat url('/icons/timelapse.png');
      opacity: 0.96;
      transition: transform 0.16s ease, opacity 0.16s ease;
    }

    .record-icon-microphone {
      width: 52%;
      height: 52%;
      min-width: 20px;
      min-height: 20px;
      max-width: 38px;
      max-height: 38px;
      background: center / contain no-repeat url('/icons/microphone.png');
      opacity: 0.96;
      transition: transform 0.16s ease, opacity 0.16s ease;
    }

    .record-button.photo-mode.photo-pulse .record-icon-camera {
      transform: scale(1.08);
      opacity: 1;
    }

    .record-button.photo-mode.photo-pulse .record-icon-timelapse {
      transform: scale(1.08);
      opacity: 1;
    }

    .record-button.photo-mode.timelapse .record-icon-camera {
      transform: scale(1.03);
      opacity: 1;
    }

    .record-button.photo-mode.timelapse .record-icon-timelapse {
      transform: scale(1.03);
      opacity: 1;
    }

    .section-head {
      display: flex;
      align-items: center;
      gap: 12px;
      margin-bottom: 24px;
    }

    .section-title {
      margin: 0;
      font-size: 22px;
      font-weight: 600;
      letter-spacing: -0.02em;
    }

    .section-icon {
      width: 40px;
      height: 40px;
      border-radius: 18px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 18px;
      background: #18181b;
      color: #d4d4d8;
      flex-shrink: 0;
    }

    .stack {
      display: flex;
      flex-direction: column;
      gap: 24px;
    }

    .select-block {
      display: flex;
      flex-direction: column;
      gap: 10px;
    }

    .settings-option-group {
      display: flex;
      flex-direction: column;
      gap: 12px;
      margin-top: 18px;
      padding-top: 18px;
      border-top: 1px solid rgba(255, 255, 255, 0.08);
    }

    .settings-option-group:first-of-type {
      margin-top: 0;
      padding-top: 0;
      border-top: none;
    }

    .select-topline {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      font-size: 15px;
      font-weight: 700;
      color: #f3f4f6;
      letter-spacing: -0.01em;
    }

    .select-value {
      color: #f4f4f5;
      font-weight: 700;
    }

    .select-value-pill {
      display: inline-flex;
      align-items: center;
      min-height: 32px;
      padding: 0 12px;
      border-radius: 999px;
      border: 1px solid rgba(255, 255, 255, 0.1);
      background: #202024;
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.03);
      font-size: 13px;
      letter-spacing: 0;
      white-space: nowrap;
    }

    .pill-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
    }

    .pill-grid.three-col {
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }

    .pill-grid.two-col {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }

    .pill-button {
      border: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 18px;
      padding: 11px 10px;
      background: #202024;
      color: #e4e4e7;
      font-size: 14px;
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.03);
      transition: transform 0.12s ease, background 0.12s ease, color 0.12s ease, box-shadow 0.12s ease, border-color 0.12s ease;
    }

    .pill-button:hover {
      background: #2a2a2f;
      border-color: rgba(255, 255, 255, 0.14);
    }

    .pill-button:active {
      transform: scale(0.96);
    }

    .pill-button.active {
      border-color: rgba(255, 255, 255, 0.9);
      background: #ffffff;
      color: #09090b;
      box-shadow: 0 16px 32px rgba(255, 255, 255, 0.08);
    }

    .pill-button.disabled,
    .pill-button:disabled {
      color: rgba(228, 228, 231, 0.34);
      background: #17171b;
      border-color: rgba(255, 255, 255, 0.06);
      box-shadow: none;
      cursor: not-allowed;
    }

    .pill-button.disabled:hover,
    .pill-button:disabled:hover {
      background: #17171b;
      border-color: rgba(255, 255, 255, 0.06);
    }

    .slider {
      width: 100%;
      accent-color: #ffffff;
    }

    .slider-labels {
      display: flex;
      justify-content: space-between;
      font-size: 12px;
      color: var(--muted-2);
    }

    .system-head {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 14px;
      margin-bottom: 18px;
    }

    .system-eyebrow {
      margin: 6px 0 0;
      font-size: 14px;
      color: var(--muted);
    }

    .online-pill {
      border-radius: 999px;
      background: rgba(52, 211, 153, 0.1);
      color: var(--ok);
      padding: 6px 12px;
      font-size: 12px;
      font-weight: 600;
      white-space: nowrap;
    }

    .telemetry-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }

    .telemetry-card {
      border-radius: var(--radius-lg);
      border: 1px solid var(--card-border);
      background: rgba(9, 9, 11, 0.72);
      box-shadow: 0 16px 40px rgba(0, 0, 0, 0.2);
      overflow: hidden;
    }

    .telemetry-inner {
      padding: 16px;
    }

    .telemetry-top {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 12px;
    }

    .telemetry-progress-text {
      font-size: 12px;
      color: var(--muted-2);
    }

    .telemetry-value {
      font-size: 28px;
      font-weight: 600;
      letter-spacing: -0.03em;
      color: #ffffff;
    }

    .telemetry-label {
      margin-top: 4px;
      font-size: 14px;
      color: var(--muted);
    }

    .telemetry-sublabel {
      margin-top: 4px;
      font-size: 12px;
      color: #52525b;
    }

    .progress-track {
      margin-top: 16px;
      height: 6px;
      border-radius: 999px;
      background: #27272a;
      overflow: hidden;
    }

    .progress-fill {
      height: 100%;
      width: 0%;
      border-radius: 999px;
      background: #ffffff;
      transition: width 0.22s ease;
    }

    .capture-list {
      display: flex;
      flex-direction: column;
      gap: 14px;
    }

    .capture-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding-bottom: 14px;
      border-bottom: 1px solid var(--line);
      font-size: 14px;
    }

    .capture-row:last-child {
      border-bottom: 0;
      padding-bottom: 0;
    }

    .capture-key {
      color: var(--muted);
    }

    .capture-value {
      font-weight: 500;
      color: #f4f4f5;
      text-align: right;
    }

    .capture-value-stack {
      display: flex;
      flex-direction: column;
      align-items: flex-end;
      gap: 4px;
      min-width: 0;
    }

    .capture-detail {
      font-size: 12px;
      color: var(--muted);
      text-align: right;
      max-width: 30ch;
      line-height: 1.35;
    }

    .settings-overlay {
      position: fixed;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 16px;
      background: rgba(0, 0, 0, 0.5);
      backdrop-filter: blur(10px);
      z-index: 85;
      opacity: 1;
      transition: opacity 0.18s ease;
      overflow: hidden;
    }

    .settings-overlay.hidden {
      opacity: 0;
      pointer-events: none;
    }

    .settings-shell {
      width: min(var(--content-width), calc(100vw - 32px));
      height: calc(100dvh - 32px);
      max-height: calc(100dvh - 32px);
      display: grid;
      grid-template-columns: 260px minmax(0, 1fr);
      grid-template-rows: minmax(0, 1fr);
      align-items: stretch;
      border-radius: 32px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.96);
      box-shadow: 0 30px 80px rgba(0, 0, 0, 0.45);
      overflow: hidden;
      min-height: 0;
    }

    .settings-sidebar {
      display: flex;
      flex-direction: column;
      gap: 18px;
      padding: 24px;
      background: rgba(17, 17, 19, 0.96);
      border-right: 1px solid #27272a;
      min-width: 0;
      min-height: 0;
    }

    .settings-sidebar-head {
      display: flex;
      align-items: flex-start;
      gap: 12px;
    }

    .settings-sidebar-copy {
      margin: 6px 0 0;
      font-size: 14px;
      color: var(--muted);
    }

    .settings-nav {
      display: flex;
      flex-direction: column;
      gap: 10px;
      min-height: 0;
      overflow: auto;
      overscroll-behavior: contain;
      -webkit-overflow-scrolling: touch;
    }

    .settings-nav-item {
      width: 100%;
      min-height: 56px;
      border-radius: 18px;
      border: 1px solid transparent;
      background: transparent;
      color: #f4f4f5;
      display: flex;
      align-items: center;
      gap: 12px;
      padding: 12px 14px;
      font-size: 18px;
      font-weight: 600;
      text-align: left;
      transition: background 0.12s ease, border-color 0.12s ease, transform 0.12s ease;
    }

    .settings-nav-item:hover {
      background: rgba(255, 255, 255, 0.04);
    }

    .settings-nav-item:active {
      transform: scale(0.985);
    }

    .settings-nav-item.active {
      background: rgba(255, 255, 255, 0.1);
      border-color: rgba(255, 255, 255, 0.1);
    }

    .settings-nav-icon {
      width: 22px;
      height: 22px;
      display: flex;
      align-items: center;
      justify-content: center;
      flex-shrink: 0;
    }

    .settings-nav-icon-image {
      width: 18px;
      height: 18px;
      object-fit: contain;
      display: block;
      opacity: 0.94;
      user-select: none;
      -webkit-user-drag: none;
    }

    .settings-nav-item.active .settings-nav-icon-image {
      filter: brightness(1.18);
      opacity: 1;
    }

    .settings-content {
      min-width: 0;
      min-height: 0;
      display: flex;
      flex-direction: column;
      background: rgba(9, 9, 11, 0.96);
      overflow: hidden;
    }

    .settings-header {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 16px;
      padding: 24px 24px 0;
    }

    .settings-back-button {
      width: auto;
      min-width: 112px;
      padding: 0 20px;
      flex-shrink: 0;
    }

    .settings-panels {
      flex: 1;
      min-height: 0;
      overflow: auto;
      padding: 20px 24px 24px;
      overscroll-behavior: contain;
      -webkit-overflow-scrolling: touch;
      touch-action: pan-y;
    }

    .settings-panel {
      display: none;
      min-height: 0;
    }

    .settings-panel.active {
      display: flex;
      flex-direction: column;
      gap: 16px;
    }

    .media-browser-overlay {
      position: fixed;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 16px;
      background: rgba(0, 0, 0, 0.5);
      backdrop-filter: blur(10px);
      z-index: 84;
      opacity: 1;
      transition: opacity 0.18s ease;
      overflow: hidden;
    }

    .media-browser-overlay.hidden {
      opacity: 0;
      pointer-events: none;
    }

    .media-browser-shell {
      width: min(900px, calc(100vw - 32px));
      height: min(760px, calc(100dvh - 32px));
      max-height: calc(100dvh - 32px);
      display: flex;
      flex-direction: column;
      border-radius: 32px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.96);
      box-shadow: 0 30px 80px rgba(0, 0, 0, 0.45);
      overflow: hidden;
      min-height: 0;
    }

    .media-browser-header {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 16px;
      padding: 24px 24px 10px;
      flex-shrink: 0;
    }

    .media-browser-title-row {
      display: flex;
      align-items: center;
      gap: 12px;
      min-width: 0;
    }

    .media-browser-icon {
      width: 44px;
      height: 44px;
      border-radius: 18px;
      border: 1px solid #27272a;
      background: rgba(24, 24, 27, 0.86);
      display: flex;
      align-items: center;
      justify-content: center;
      flex-shrink: 0;
    }

    .media-browser-body {
      flex: 1;
      min-height: 0;
      overflow: auto;
      padding: 14px 24px 24px;
      overscroll-behavior: contain;
      -webkit-overflow-scrolling: touch;
      touch-action: pan-y;
    }

    .media-browser-toolbar {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 16px;
      flex-wrap: wrap;
    }

    .media-browser-filters {
      display: flex;
      align-items: center;
      gap: 8px;
      flex-wrap: wrap;
    }

    .media-browser-tools {
      margin-left: auto;
      display: flex;
      flex-direction: row;
      align-items: flex-end;
      justify-content: flex-end;
      gap: 8px;
      flex-wrap: wrap;
    }

    .media-browser-tools-row {
      justify-content: flex-end;
    }

    .media-filter-button {
      min-height: 38px;
      border-radius: 999px;
      border: 1px solid #27272a;
      background: rgba(24, 24, 27, 0.72);
      color: #d4d4d8;
      padding: 9px 14px;
      font-size: 12px;
      font-weight: 500;
      letter-spacing: 0.12em;
      text-transform: uppercase;
      transition: background 0.12s ease, transform 0.12s ease, border-color 0.12s ease, color 0.12s ease;
    }

    .media-filter-button.active {
      background: #f4f4f5;
      color: #09090b;
      border-color: #f4f4f5;
    }

    .media-sort-select {
      min-height: 34px;
      width: 118px;
      border-radius: 999px;
      padding: 7px 30px 7px 12px;
      font-size: 10px;
      font-weight: 600;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }

    .media-selection-toolbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      margin: -2px 0 14px;
      padding: 10px;
      flex-wrap: wrap;
      border-radius: 20px;
      border: 1px solid rgba(255, 255, 255, 0.08);
      background: rgba(24, 24, 27, 0.48);
    }

    .media-selection-count {
      color: var(--muted);
      font-size: 12px;
      font-weight: 650;
      letter-spacing: 0.04em;
      text-transform: uppercase;
      white-space: nowrap;
    }

    .media-selection-actions {
      display: flex;
      align-items: center;
      justify-content: flex-end;
      gap: 8px;
      flex-wrap: wrap;
    }

    .media-select-button {
      min-height: 38px;
      min-width: 104px;
      border-radius: 15px;
      padding: 9px 12px;
      font-size: 12px;
      text-align: center;
    }

    .media-select-control {
      width: 34px;
      height: 34px;
      border-radius: 14px;
      border: 1px solid #3f3f46;
      background: rgba(24, 24, 27, 0.78);
      color: #09090b;
      display: flex;
      align-items: center;
      justify-content: center;
      flex-shrink: 0;
      transition: background 0.12s ease, border-color 0.12s ease, transform 0.12s ease;
    }

    .media-select-control::after {
      content: "";
      width: 11px;
      height: 6px;
      border-left: 2px solid currentColor;
      border-bottom: 2px solid currentColor;
      opacity: 0;
      transform: rotate(-45deg) translateY(-1px);
    }

    .media-select-control.selected {
      border-color: #f4f4f5;
      background: #f4f4f5;
    }

    .media-select-control.selected::after {
      opacity: 1;
    }

    .media-select-control:active {
      transform: scale(0.96);
    }

    .media-filter-button:active {
      transform: scale(0.98);
    }

    .media-browser-list {
      display: flex;
      flex-direction: column;
      gap: 12px;
    }

    .media-row {
      border-radius: 20px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.72);
      padding: 14px 16px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 14px;
      transition: background 0.12s ease, border-color 0.12s ease, transform 0.12s ease;
    }

    .media-row.selected {
      border-color: rgba(34, 211, 238, 0.72);
      background: rgba(8, 47, 73, 0.26);
    }

    .media-row-type {
      align-self: flex-start;
      border-radius: 999px;
      border: 1px solid #27272a;
      background: rgba(39, 39, 42, 0.78);
      padding: 6px 10px;
      color: #d4d4d8;
      font-size: 11px;
      font-weight: 800;
      letter-spacing: 0.12em;
      text-transform: uppercase;
      white-space: nowrap;
    }

    .media-row-meta {
      min-width: 0;
      flex: 1;
      display: flex;
      flex-direction: column;
      gap: 6px;
    }

    .media-row-name {
      color: #ffffff;
      font-size: 16px;
      font-weight: 650;
      letter-spacing: -0.02em;
      word-break: break-word;
    }

    .media-row-detail {
      color: var(--muted);
      font-size: 13px;
      word-break: break-word;
    }

    .media-row-stats {
      display: flex;
      align-items: center;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 2px;
    }

    .media-stat {
      border-radius: 999px;
      border: 1px solid #27272a;
      background: rgba(24, 24, 27, 0.64);
      color: #e4e4e7;
      padding: 6px 9px;
      font-size: 12px;
      font-weight: 650;
      white-space: nowrap;
    }

    .media-stat strong {
      color: #ffffff;
      font-weight: 850;
    }

    .media-row-actions {
      display: flex;
      align-items: center;
      justify-content: flex-end;
      gap: 10px;
      flex-wrap: wrap;
      flex-shrink: 0;
    }

    .media-row-action {
      min-width: 112px;
      text-align: center;
      text-decoration: none;
    }

    .settings-card {
      border-radius: 22px;
      border: 1px solid rgba(255, 255, 255, 0.1);
      background: #18181b;
      padding: 20px;
      display: flex;
      flex-direction: column;
      gap: 18px;
    }

    .settings-card-head {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 16px;
    }

    .settings-card-head.compact-head {
      margin-top: 14px;
    }

    .settings-card-title {
      margin: 0;
      font-size: 22px;
      font-weight: 700;
      letter-spacing: -0.02em;
    }

    .settings-card-copy {
      margin: 6px 0 0;
      font-size: 14px;
      color: var(--muted);
      max-width: 56ch;
    }

    .settings-summary-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }

    .settings-summary-tile {
      border-radius: 18px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.72);
      padding: 14px 16px;
    }

    .settings-summary-label {
      font-size: 12px;
      color: var(--muted);
    }

    .settings-summary-value {
      margin-top: 6px;
      font-size: 18px;
      font-weight: 600;
      color: #ffffff;
      letter-spacing: -0.02em;
    }

    .settings-field-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }

    .field-span-2 {
      grid-column: 1 / -1;
    }

    .field-stack {
      display: flex;
      flex-direction: column;
      gap: 10px;
      min-width: 0;
    }

    .input-label {
      font-size: 12px;
      font-weight: 700;
      color: var(--muted);
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }

    .settings-select,
    .settings-input {
      width: 100%;
      min-height: 50px;
      border-radius: 18px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.72);
      color: #f4f4f5;
      padding: 0 16px;
      font-size: 14px;
      outline: none;
    }

    .settings-select {
      appearance: none;
      -webkit-appearance: none;
      background-image:
        linear-gradient(45deg, transparent 50%, #71717a 50%),
        linear-gradient(135deg, #71717a 50%, transparent 50%);
      background-position:
        calc(100% - 22px) calc(50% - 2px),
        calc(100% - 16px) calc(50% - 2px);
      background-size: 6px 6px, 6px 6px;
      background-repeat: no-repeat;
      padding-right: 42px;
    }

    .settings-input::-webkit-calendar-picker-indicator {
      filter: invert(1) opacity(0.7);
      cursor: pointer;
    }

    .settings-select:focus,
    .settings-input:focus {
      border-color: #3f3f46;
      box-shadow: 0 0 0 1px rgba(255, 255, 255, 0.06);
    }

    .settings-input:disabled {
      opacity: 0.56;
      cursor: not-allowed;
    }

    .config-editor {
      min-height: 420px;
      resize: vertical;
      padding: 16px;
      line-height: 1.45;
      font-family: "SFMono-Regular", "Cascadia Mono", "Menlo", "Consolas", monospace;
      font-size: 13px;
      white-space: pre;
      overflow: auto;
    }

    .config-meta-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }

    .config-meta-tile {
      border-radius: 18px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.62);
      padding: 12px 14px;
      min-width: 0;
    }

    .config-meta-label {
      color: var(--muted);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.1em;
      text-transform: uppercase;
    }

    .config-meta-value {
      margin-top: 5px;
      color: #f4f4f5;
      font-size: 13px;
      font-weight: 650;
      word-break: break-word;
    }

    .settings-select:disabled {
      opacity: 0.56;
      cursor: not-allowed;
      background-image:
        linear-gradient(45deg, transparent 50%, rgba(113, 113, 122, 0.38) 50%),
        linear-gradient(135deg, rgba(113, 113, 122, 0.38) 50%, transparent 50%);
    }

    .settings-action-row {
      display: flex;
      align-items: center;
      gap: 12px;
      flex-wrap: wrap;
    }

    .action-grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 12px;
    }

    .action-grid.two-col {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }

    .power-actions-grid > .danger-button {
      grid-column: 1 / -1;
    }

    .primary-button,
    .secondary-button,
    .danger-button {
      min-height: 50px;
      border-radius: 18px;
      padding: 12px 16px;
      font-size: 14px;
      font-weight: 600;
      transition: background 0.12s ease, transform 0.12s ease, border-color 0.12s ease, color 0.12s ease;
    }

    .primary-button {
      border: 1px solid rgba(255, 255, 255, 0.12);
      background: #f4f4f5;
      color: #09090b;
    }

    .primary-button:hover {
      background: #ffffff;
    }

    .secondary-button {
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.72);
      color: #f4f4f5;
    }

    .secondary-button:hover {
      background: #27272a;
      border-color: #3f3f46;
    }

    .secondary-button.standby-button {
      background: #123d59;
      border-color: #3fd0ff;
      color: #e2f6ff;
    }

    .secondary-button.standby-button:hover,
    .secondary-button.standby-button:active {
      background: #1b5578;
    }

    .danger-button {
      border-radius: 18px;
      border: 1px solid rgba(239, 68, 68, 0.4);
      background: rgba(127, 29, 29, 0.32);
      color: #fca5a5;
      padding: 12px 16px;
      font-size: 14px;
      font-weight: 600;
      transition: background 0.12s ease, transform 0.12s ease, border-color 0.12s ease;
    }

    .danger-button:hover {
      background: rgba(127, 29, 29, 0.42);
      border-color: rgba(239, 68, 68, 0.55);
    }

    .danger-button:active {
      transform: scale(0.98);
    }

    .primary-button:active,
    .secondary-button:active {
      transform: scale(0.98);
    }

    .primary-button:disabled,
    .secondary-button:disabled,
    .danger-button:disabled {
      opacity: 0.45;
      cursor: default;
      transform: none;
    }

    .subtle-note {
      font-size: 13px;
      color: var(--muted);
    }

    .imu-live-card {
      border-radius: 20px;
      border: 1px solid rgba(255, 255, 255, 0.1);
      background: rgba(9, 9, 11, 0.58);
      padding: 14px;
      display: grid;
      grid-template-columns: 112px minmax(0, 1fr);
      gap: 14px;
      align-items: center;
    }

    .imu-axis-visual {
      position: relative;
      width: 112px;
      height: 112px;
      border-radius: 999px;
      border: 1px solid rgba(255, 255, 255, 0.18);
      background: rgba(255, 255, 255, 0.035);
      overflow: hidden;
    }

    .imu-axis-canvas {
      width: 100%;
      height: 100%;
      display: block;
    }

    .imu-axis-center,
    .imu-axis-dot {
      position: absolute;
      left: 50%;
      top: 50%;
      border-radius: 999px;
      transform: translate(-50%, -50%);
    }

    .imu-axis-center {
      width: 10px;
      height: 10px;
      background: #ffffff;
      box-shadow: 0 0 16px rgba(255, 255, 255, 0.35);
      z-index: 4;
    }

    .imu-axis-dot {
      width: 6px;
      height: 6px;
      background: #7dd3fc;
      z-index: 5;
    }

    .imu-axis-line {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 3px;
      height: 38px;
      border-radius: 999px;
      transform-origin: 50% 0;
      opacity: 0.95;
    }

    .imu-axis-line span {
      position: absolute;
      left: 50%;
      top: 100%;
      transform: translate(-50%, 6px);
      font-size: 18px;
      font-weight: 800;
      letter-spacing: -0.04em;
    }

    .imu-axis-x {
      background: #ef4444;
      transform: rotate(90deg);
    }

    .imu-axis-y {
      background: #22c55e;
      transform: rotate(180deg);
    }

    .imu-axis-z {
      background: #38bdf8;
      transform: rotate(270deg);
    }

    .imu-axis-x span {
      color: #f87171;
    }

    .imu-axis-y span {
      color: #4ade80;
    }

    .imu-axis-z span {
      color: #7dd3fc;
    }

    .imu-live-copy {
      display: flex;
      flex-direction: column;
      gap: 8px;
      min-width: 0;
    }

    .imu-live-title {
      font-size: 20px;
      font-weight: 800;
      letter-spacing: -0.03em;
      color: #ffffff;
    }

    .imu-live-status {
      display: flex;
      align-items: center;
      gap: 8px;
      color: #d4d4d8;
      font-size: 13px;
      font-weight: 650;
    }

    .imu-live-dot {
      width: 10px;
      height: 10px;
      border-radius: 999px;
      background: #facc15;
      box-shadow: 0 0 14px rgba(250, 204, 21, 0.35);
      flex: 0 0 auto;
    }

    .imu-live-dot.live {
      background: #34d399;
      box-shadow: 0 0 14px rgba(52, 211, 153, 0.35);
    }

    .imu-readout {
      color: var(--muted);
      font-size: 13px;
      line-height: 1.42;
    }

    .imu-readout strong {
      color: #ffffff;
      font-weight: 750;
    }

    .imu-stepper-row {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 116px minmax(0, 1fr);
      gap: 10px;
      align-items: stretch;
    }

    .imu-sync-value {
      min-height: 50px;
      border-radius: 18px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.72);
      display: flex;
      align-items: center;
      justify-content: center;
      color: #ffffff;
      font-size: 18px;
      font-weight: 800;
      letter-spacing: -0.03em;
    }

    .preset-list {
      display: flex;
      flex-direction: column;
      gap: 12px;
    }

    .preset-row {
      border-radius: 18px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.72);
      padding: 14px 16px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 14px;
    }

    .preset-meta {
      min-width: 0;
      display: flex;
      flex-direction: column;
      gap: 6px;
    }

    .preset-name {
      font-size: 16px;
      font-weight: 600;
      color: #ffffff;
      letter-spacing: -0.02em;
      word-break: break-word;
    }

    .preset-detail {
      font-size: 13px;
      color: var(--muted);
      word-break: break-word;
    }

    .preset-download {
      text-decoration: none;
      flex-shrink: 0;
    }

    .preset-actions {
      display: flex;
      align-items: center;
      justify-content: flex-end;
      gap: 10px;
      flex-wrap: wrap;
      flex-shrink: 0;
    }

    .preset-action {
      min-width: 108px;
      text-align: center;
    }

    .preset-empty {
      border-radius: 18px;
      border: 1px dashed #27272a;
      background: rgba(9, 9, 11, 0.42);
      padding: 20px 16px;
      color: var(--muted);
      font-size: 14px;
      text-align: center;
    }

    .favorite-section-pill {
      display: inline-flex;
      align-items: center;
      min-height: 24px;
      padding: 4px 10px;
      border-radius: 999px;
      border: 1px solid rgba(63, 208, 255, 0.24);
      background: rgba(31, 44, 54, 0.72);
      color: #9ddfff;
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.04em;
      text-transform: uppercase;
      width: fit-content;
      max-width: 100%;
    }

    .favorite-summary {
      font-size: 13px;
      color: var(--muted);
      word-break: break-word;
    }

    .settings-card.hidden {
      display: none;
    }

    .hidden {
      display: none !important;
    }

    .toast {
      position: fixed;
      right: 18px;
      bottom: 18px;
      max-width: min(92vw, 420px);
      border-radius: 18px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.92);
      color: #ffffff;
      padding: 14px 16px;
      box-shadow: 0 20px 48px rgba(0, 0, 0, 0.34);
      opacity: 0;
      transform: translateY(12px);
      pointer-events: none;
      transition: opacity 0.18s ease, transform 0.18s ease;
      z-index: 100;
    }

    .toast.visible {
      opacity: 1;
      transform: translateY(0);
    }

    .toast.error {
      border-color: rgba(239, 68, 68, 0.42);
      background: rgba(48, 9, 9, 0.95);
    }

    .loading-overlay {
      position: fixed;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      background: rgba(0, 0, 0, 0.45);
      backdrop-filter: blur(6px);
      z-index: 90;
      transition: opacity 0.18s ease;
    }

    .loading-overlay.hidden {
      opacity: 0;
      pointer-events: none;
    }

    .loading-card {
      border-radius: 22px;
      border: 1px solid #27272a;
      background: rgba(9, 9, 11, 0.92);
      padding: 18px 22px;
      color: #e4e4e7;
      font-size: 14px;
      box-shadow: 0 20px 48px rgba(0, 0, 0, 0.34);
    }

    @media (max-width: 1120px) {
      .main-grid {
        grid-template-columns: 1fr;
      }

      .settings-shell {
        grid-template-columns: 1fr;
        grid-template-rows: auto minmax(0, 1fr);
      }

      .settings-sidebar {
        border-right: 0;
        border-bottom: 1px solid #27272a;
      }

      .settings-nav {
        flex-direction: row;
        overflow: auto;
        padding-bottom: 2px;
      }

      .settings-nav-item {
        flex: 0 0 auto;
        width: auto;
        min-width: 158px;
      }
    }

    @media (max-width: 720px) {
      .page {
        padding: 14px;
      }

      .card-body {
        padding: 18px;
      }

      .settings-overlay,
      .media-browser-overlay {
        padding: 10px;
      }

      .settings-shell,
      .media-browser-shell {
        width: calc(100vw - 20px);
        height: calc(100dvh - 20px);
        max-height: calc(100dvh - 20px);
        border-radius: 28px;
      }

      .settings-sidebar {
        padding: 18px;
      }

      .settings-header,
      .media-browser-header {
        padding: 18px 18px 0;
      }

      .settings-panels,
      .media-browser-body {
        padding: 16px 18px 18px;
      }

      .settings-card {
        padding: 18px;
      }

      .settings-field-grid,
      .action-grid.two-col {
        grid-template-columns: 1fr;
      }

      .preset-row,
      .media-row {
        flex-direction: column;
        align-items: stretch;
      }

      .preset-actions,
      .media-row-actions {
        width: 100%;
        justify-content: stretch;
      }

      .preset-action,
      .preset-download,
      .media-row-action {
        flex: 1 1 0;
        width: 100%;
        text-align: center;
      }

      .topbar {
        flex-direction: column;
        align-items: flex-start;
      }

      .topbar-actions {
        width: 100%;
        justify-content: space-between;
      }

      .status-cluster {
        flex-wrap: wrap;
        row-gap: 8px;
      }

      .preview-stage {
        min-height: 0;
      }

      .preview-footer {
        left: 12px;
        right: 12px;
        bottom: 12px;
        flex-direction: row;
        align-items: flex-end;
        gap: 12px;
      }

      .time-card {
        flex: 1;
        min-width: 0;
        padding: 12px 14px;
      }

      .time-value {
        font-size: clamp(20px, 5vw, 26px);
      }

      .record-button {
        flex: 0 0 76px;
        width: 76px;
        height: 76px;
        box-shadow: 0 18px 36px rgba(220, 38, 38, 0.2);
      }

      .record-icon-dot {
        width: 30px;
        height: 30px;
      }

      .record-icon-stop {
        width: 28px;
        height: 28px;
        border-radius: 8px;
      }

      .pill-grid,
      .pill-grid.three-col,
      .telemetry-grid {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }

    @media (max-width: 480px) {
      .page {
        padding: 12px;
        gap: 16px;
      }

      .brand {
        align-items: flex-start;
      }

      .brand-icon,
      .brand-logo {
        width: 48px;
        height: 48px;
      }

      .brand-title {
        font-size: 18px;
      }

      .brand-subtitle {
        font-size: 13px;
        gap: 6px;
      }

      .date-time-pill,
      .status-pill {
        padding: 9px 14px;
        font-size: 13px;
      }

      .status-cluster {
        gap: 8px;
      }

      .icon-button {
        width: 42px;
        height: 42px;
      }

      .preview-badges {
        top: 12px;
        gap: 6px;
      }

      .preview-badges.left {
        left: 12px;
        right: 12px;
      }

      .preview-badges.right {
        top: 12px;
        right: 12px;
        left: auto;
      }

      .badge {
        padding: 6px 10px;
        font-size: 11px;
      }

      .time-card {
        padding: 11px 12px;
      }

      .time-label {
        font-size: 11px;
      }

      .time-value {
        font-size: 18px;
      }

      .record-button {
        flex-basis: 68px;
        width: 68px;
        height: 68px;
      }

      .record-icon-dot {
        width: 26px;
        height: 26px;
      }

      .record-icon-stop {
        width: 24px;
        height: 24px;
      }

      .section-head {
        gap: 10px;
        margin-bottom: 20px;
      }

      .section-title {
        font-size: 19px;
      }

      .settings-nav-item {
        min-width: 140px;
        min-height: 52px;
        font-size: 16px;
      }

      .settings-card-title {
        font-size: 20px;
      }

      .select-topline {
        gap: 8px;
        font-size: 14px;
      }

      .settings-summary-grid,
      .config-meta-grid,
      .telemetry-grid {
        grid-template-columns: 1fr;
      }

      .slider-labels {
        font-size: 11px;
      }

      .pill-grid,
      .pill-grid.three-col {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }

    @media (max-width: 560px) {
      .preview-stage {
        min-height: 0;
      }

      .preview-badges {
        top: 10px;
        gap: 6px;
      }

      .preview-badges.left {
        left: 10px;
        right: 120px;
      }

      .preview-badges.right {
        top: 10px;
        right: 10px;
        left: auto;
        display: block;
      }

      .badge {
        padding: 5px 9px;
        font-size: 11px;
      }

      .preview-footer {
        left: 10px;
        right: 10px;
        bottom: 10px;
        display: grid;
        grid-template-columns: minmax(0, 1fr) auto;
        align-items: end;
        gap: 10px;
      }

      .time-card {
        min-width: 0;
        max-width: 100%;
        padding: 10px 12px;
      }

      .time-label {
        font-size: 11px;
      }

      .time-value {
        margin-top: 3px;
        font-size: 16px;
        line-height: 1.05;
        white-space: nowrap;
      }

      .record-button {
        width: 62px;
        height: 62px;
        flex-basis: 62px;
      }

      .record-icon-dot {
        width: 22px;
        height: 22px;
      }

      .record-icon-stop {
        width: 20px;
        height: 20px;
        border-radius: 6px;
      }
    }

    @media (max-width: 420px) {
      .preview-stage {
        min-height: 0;
      }

      .time-card {
        padding: 9px 11px;
      }

      .time-value {
        font-size: 15px;
      }

      .record-button {
        width: 58px;
        height: 58px;
        flex-basis: 58px;
      }

      .record-icon-dot {
        width: 20px;
        height: 20px;
      }

      .record-icon-stop {
        width: 18px;
        height: 18px;
      }
    }
  </style>
</head>
<body>
  <div class="page">
    <header class="topbar">
      <div class="brand">
        <div class="brand-icon">
          <img class="brand-logo" src="/icons/apertar.png" alt="Apertar">
        </div>
        <div>
          <h1 class="brand-title">Apertar Control</h1>
          <div class="brand-subtitle">
            <span id="networkTransport">Wi-Fi / Ethernet</span>
            <span class="brand-divider">•</span>
            <span id="networkHost">Loading...</span>
          </div>
        </div>
      </div>

      <div class="topbar-actions">
        <div class="status-cluster">
          <div class="date-time-pill" id="webDateTimePill">--</div>
          <div class="status-pill">
          <span class="status-dot" id="statusDot">●</span>
            <span id="recordingStatus">READY</span>
          </div>
        </div>
        <button class="icon-button" id="mediaBrowserButton" aria-label="Open clip browser">
          <img class="icon-image" src="/icons/folder.png" alt="">
        </button>
        <button class="icon-button" id="settingsButton" aria-label="Open settings">
          <img class="icon-image" src="/icons/settings.png" alt="">
        </button>
      </div>
    </header>

    <main class="main-grid">
      <section class="column">
        <div class="card preview-card">
          <div class="preview-stage" id="previewStage">
            <img class="preview-stream" id="previewStream" alt="Live preview">
            <canvas class="preview-processed-canvas" id="previewProcessedCanvas"></canvas>
            <div id="stillExposureOverlay" style="display:none;position:absolute;inset:0;background:#000;z-index:20;align-items:center;justify-content:center;pointer-events:none">
              <div style="text-align:center"><div id="stillExposureCountdown" style="font-size:24px;color:white"></div><div style="font-size:11px;color:#999;letter-spacing:1.5px;margin-top:6px">EXPOSING</div></div>
            </div>
            <div class="preview-smpte" id="previewSmpte" aria-hidden="true">
              <div class="smpte-row top">
                <span style="background:#bfbfbf"></span>
                <span style="background:#bfbf00"></span>
                <span style="background:#00bfbf"></span>
                <span style="background:#00bf00"></span>
                <span style="background:#bf00bf"></span>
                <span style="background:#bf0000"></span>
                <span style="background:#0000bf"></span>
              </div>
              <div class="smpte-row middle">
                <span style="background:#0000bf"></span>
                <span style="background:#121212"></span>
                <span style="background:#bf00bf"></span>
                <span style="background:#121212"></span>
                <span style="background:#00bfbf"></span>
                <span style="background:#121212"></span>
                <span style="background:#bfbfbf"></span>
              </div>
              <div class="smpte-row bottom">
                <span style="flex:0.2;background:#002040"></span>
                <span style="flex:0.2;background:#ffffff"></span>
                <span style="flex:0.15;background:#33004d"></span>
                <span style="flex:0.15;background:#121212"></span>
                <span style="flex:0.1;background:#262626"></span>
                <span style="flex:0.1;background:#121212"></span>
                <span style="flex:0.1;background:#383838"></span>
              </div>
            </div>
            <div class="false-color-legend" id="falseColorLegend" aria-hidden="true">
              <div class="false-color-legend-row">
                <div class="false-color-legend-labels" id="falseColorLegendLabels"></div>
                <div class="false-color-legend-scale" id="falseColorLegendScale"></div>
              </div>
            </div>
            <div class="preview-grid"></div>

            <div class="preview-badges left">
              <span class="badge" id="resolutionBadge">4K</span>
              <span class="badge" id="fpsBadge">24 FPS</span>
              <span class="badge" id="isoBadge">ISO 400</span>
            </div>

            <div class="preview-badges right">
              <span class="badge" id="wbBadge">WB 5600K</span>
              <span class="badge hidden" id="timelapseShotCounter" aria-label="Saved timelapse shots">0 SHOTS</span>
            </div>

            <div class="preview-center" id="previewOverlay">
              <div class="preview-center-card" id="previewOverlayCard">
                <div class="preview-center-icon">▣</div>
                <div class="preview-center-label" id="previewOverlayLabel">Waiting for live preview</div>
              </div>
            </div>

            <div class="preview-footer">
              <div class="time-card" id="timecodeBlock">
                <div class="timecode-value" id="timecodeValue">00:00:00:00</div>
              </div>

              <button class="record-button" id="recordButton" aria-label="Toggle record">
                <div class="record-icon-dot" id="recordIcon"></div>
              </button>
            </div>
          </div>
        </div>

        <div class="card">
          <div class="card-body">
            <div class="section-head">
              <div class="section-icon">
                <img class="icon-image" src="/icons/settings.png" alt="">
              </div>
              <h2 class="section-title">Camera settings</h2>
            </div>

            <div class="stack">
              <section class="select-block">
                <div class="select-topline">
                  <span>Frame rate</span>
                </div>
                <div class="pill-grid" id="fpsOptions"></div>
              </section>

              <section class="select-block">
                <div class="select-topline">
                  <span>ISO</span>
                </div>
                <div class="pill-grid" id="isoOptions"></div>
              </section>

              <section class="select-block">
                <div class="select-topline">
                  <span id="shutterSectionLabel">Shutter angle</span>
                </div>
                <div class="pill-grid" id="shutterOptions"></div>
              </section>

              <section class="select-block">
                <div class="select-topline">
                  <span id="quickResolutionLabel">Resolution</span>
                </div>
                <div class="pill-grid three-col" id="resolutionOptions"></div>
              </section>

              <section class="select-block hidden" id="cropRatioBlock">
                <div class="select-topline">
                  <span id="quickCropLabel">Crop mode</span>
                </div>
                <div class="pill-grid three-col" id="cropRatioOptions"></div>
              </section>

              <section class="select-block">
                <div class="select-topline">
                  <span>White balance</span>
                  <span class="select-value select-value-pill" id="whiteBalanceValue">5600K</span>
                </div>
                <div class="pill-grid" id="whiteBalanceModeOptions"></div>
                <input class="slider" id="whiteBalanceRange" type="range" min="0" max="18" step="1" value="12">
                <div class="slider-labels">
                  <span>2500K</span>
                  <span>5600K</span>
                  <span>8000K</span>
                </div>
              </section>
            </div>
          </div>
        </div>
      </section>

      <aside class="column">
        <div class="card">
          <div class="card-body">
            <div class="system-head">
              <div>
                <h2 class="section-title" style="margin:0;">System</h2>
                <p class="system-eyebrow">Camera Performance & Status</p>
              </div>
              <div class="online-pill" id="onlinePill">Online</div>
            </div>

            <div class="telemetry-grid">
              <div class="telemetry-card">
                <div class="telemetry-inner">
                  <div class="telemetry-top">
                    <div class="section-icon">
                      <img class="icon-image" src="/icons/cpu.png" alt="">
                    </div>
                    <span class="telemetry-progress-text" id="cpuUsageProgressText">--%</span>
                  </div>
                  <div class="telemetry-value" id="cpuUsageValue">--</div>
                  <div class="telemetry-label">CPU usage</div>
                  <div class="telemetry-sublabel" id="cpuUsageSub">Sampling...</div>
                  <div class="progress-track"><div class="progress-fill" id="cpuUsageProgress"></div></div>
                </div>
              </div>

              <div class="telemetry-card">
                <div class="telemetry-inner">
                  <div class="telemetry-top">
                    <div class="section-icon">
                      <img class="icon-image" src="/icons/thermal.png" alt="">
                    </div>
                    <span class="telemetry-progress-text" id="cpuTempProgressText">--%</span>
                  </div>
                  <div class="telemetry-value" id="cpuTempValue">--</div>
                  <div class="telemetry-label">CPU temp</div>
                  <div class="telemetry-sublabel" id="cpuTempSub">Thermal zone</div>
                  <div class="progress-track"><div class="progress-fill" id="cpuTempProgress"></div></div>
                </div>
              </div>

              <div class="telemetry-card">
                <div class="telemetry-inner">
                  <div class="telemetry-top">
                    <div class="section-icon">
                      <img class="icon-image" src="/icons/info.png" alt="">
                    </div>
                    <span class="telemetry-progress-text" id="powerProgressText">--%</span>
                  </div>
                  <div class="telemetry-value" id="powerValue">--</div>
                  <div class="telemetry-label">Power draw</div>
                  <div class="telemetry-sublabel" id="powerSub">System input</div>
                  <div class="progress-track"><div class="progress-fill" id="powerProgress"></div></div>
                </div>
              </div>

              <div class="telemetry-card">
                <div class="telemetry-inner">
                  <div class="telemetry-top">
                    <div class="section-icon">
                      <img class="icon-image" src="/icons/power.png" alt="">
                    </div>
                    <span class="telemetry-progress-text" id="batteryProgressText">--%</span>
                  </div>
                  <div class="telemetry-value" id="batteryValue">--</div>
                  <div class="telemetry-label">Battery</div>
                  <div class="telemetry-sublabel" id="batterySub">Input rail</div>
                  <div class="progress-track"><div class="progress-fill" id="batteryProgress"></div></div>
                </div>
              </div>

              <div class="telemetry-card">
                <div class="telemetry-inner">
                  <div class="telemetry-top">
                    <div class="section-icon">
                      <img class="icon-image" src="/icons/media.png" alt="">
                    </div>
                    <span class="telemetry-progress-text" id="storageProgressText">--%</span>
                  </div>
                  <div class="telemetry-value" id="storageValue">--</div>
                  <div class="telemetry-label">Space left</div>
                  <div class="telemetry-sublabel" id="storageSub">Media root</div>
                  <div class="progress-track"><div class="progress-fill" id="storageProgress"></div></div>
                </div>
              </div>

              <div class="telemetry-card">
                <div class="telemetry-inner">
                  <div class="telemetry-top">
                    <div class="section-icon">
                      <img class="icon-image" src="/icons/time-date.png" alt="">
                    </div>
                    <span class="telemetry-progress-text" id="timeLeftProgressText">--%</span>
                  </div>
                  <div class="telemetry-value" id="timeLeftValue">--</div>
                  <div class="telemetry-label" id="timeLeftLabel">Time left</div>
                  <div class="telemetry-sublabel" id="timeLeftSub">Current format estimate</div>
                  <div class="progress-track"><div class="progress-fill" id="timeLeftProgress"></div></div>
                </div>
              </div>
            </div>
          </div>
        </div>

        <div class="card">
          <div class="card-body">
            <h2 class="section-title" style="margin:0 0 18px;">Current capture</h2>
            <div class="capture-list">
              <div class="capture-row">
                <span class="capture-key">Codec</span>
                <span class="capture-value" id="captureCodec">CinemaDNG RAW</span>
              </div>
              <div class="capture-row">
                <span class="capture-key">Sensor</span>
                <span class="capture-value" id="captureSensor">IMX585</span>
              </div>
              <div class="capture-row">
                <span class="capture-key">Storage</span>
                <span class="capture-value" id="captureStorage">RAW</span>
              </div>
              <div class="capture-row">
                <span class="capture-key">Preview</span>
                <span class="capture-value" id="capturePreview">720p stream</span>
              </div>
              <div class="capture-row">
                <span class="capture-key">Network</span>
                <span class="capture-value" id="captureNetwork">Ethernet / Wi-Fi</span>
              </div>
            </div>
          </div>
        </div>
      </aside>
    </main>
  </div>

  <div class="settings-overlay hidden" id="settingsOverlay">
    <div class="settings-shell">
      <aside class="settings-sidebar">
        <div class="settings-sidebar-head">
          <div class="section-icon">
            <img class="icon-image" src="/icons/settings.png" alt="">
          </div>
          <div>
            <h2 class="section-title" style="margin:0;">Settings</h2>
            <p class="settings-sidebar-copy">Adjust camera, monitoring, and system behavior from one place.</p>
          </div>
        </div>

        <nav class="settings-nav" id="settingsNav" aria-label="Settings sections">
          <button class="settings-nav-item active" data-panel="favorites" type="button">
            <span class="settings-nav-icon"><img class="settings-nav-icon-image" src="/icons/favorites.png" alt=""></span>
            <span>Favorites</span>
          </button>
          <button class="settings-nav-item" data-panel="monitoring" type="button">
            <span class="settings-nav-icon"><img class="settings-nav-icon-image" src="/icons/monitoring.png" alt=""></span>
            <span>Monitoring</span>
          </button>
          <button class="settings-nav-item" data-panel="record" type="button">
            <span class="settings-nav-icon"><img class="settings-nav-icon-image" src="/icons/record.png" alt=""></span>
            <span>Record</span>
          </button>
          <button class="settings-nav-item" data-panel="audio" type="button">
            <span class="settings-nav-icon"><img class="settings-nav-icon-image" src="/icons/audio.png" alt=""></span>
            <span>Audio</span>
          </button>
          <button class="settings-nav-item" data-panel="power" type="button">
            <span class="settings-nav-icon"><img class="settings-nav-icon-image" src="/icons/media.png" alt=""></span>
            <span>Media</span>
          </button>
          <button class="settings-nav-item" data-panel="system" type="button">
            <span class="settings-nav-icon"><img class="settings-nav-icon-image" src="/icons/system.png" alt=""></span>
            <span>System</span>
          </button>
          <button class="settings-nav-item" data-panel="presets" type="button">
            <span class="settings-nav-icon"><img class="settings-nav-icon-image" src="/icons/levels.png" alt=""></span>
            <span>Presets</span>
          </button>
          <button class="settings-nav-item" data-panel="config" type="button">
            <span class="settings-nav-icon"><img class="settings-nav-icon-image" src="/icons/config.png" alt=""></span>
            <span>Config</span>
          </button>
          <button class="settings-nav-item" data-panel="info" type="button">
            <span class="settings-nav-icon"><img class="settings-nav-icon-image" src="/icons/info.png" alt=""></span>
            <span>Info</span>
          </button>
        </nav>
      </aside>

      <section class="settings-content">
        <div class="settings-header">
          <div>
            <h2 class="section-title" id="settingsPageTitle" style="margin:0;">Favorites</h2>
            <p class="settings-sidebar-copy" id="settingsPageCopy">Manage your Quick Access favorites</p>
          </div>
          <button class="icon-button settings-back-button" id="closeSettingsButton" type="button" aria-label="Back">
            <span style="font-size:14px; font-weight:600; line-height:1;">Back</span>
          </button>
        </div>

        <div class="settings-panels">
          <section class="settings-panel" data-panel="record">
            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Still Mode</h3>
                  <p class="settings-card-copy">Enable still capture and configure timer & burst settings.</p>
                </div>
              </div>
              <div class="pill-grid" id="settingsStillModeOptions"></div>
              <div class="field-stack">
                <span class="input-label">Long Exposure</span>
                <div class="pill-grid two-col" id="settingsStillLongExposureOptions"></div>
                <p class="settings-card-copy">Still shutter speeds up to 30s, within sensor limits. Preview updates follow exposure time.</p>
              </div>
              <div class="field-stack">
                <span class="input-label">Bracketing</span>
                <div class="pill-grid two-col" id="settingsStillBracketingOptions"></div>
                <p class="settings-card-copy">Three shots below, at and above the base exposure using EV Offset, with fixed ISO and white balance. Replaces Burst; not used for timelapse.</p>
              </div>
              <div class="settings-field-grid">
                <label class="field-stack">
                  <span class="input-label">Format</span>
                  <select class="settings-select" id="settingsStillFormatSelect"></select>
                </label>
                <label class="field-stack">
                  <span class="input-label">Timer</span>
                  <select class="settings-select" id="settingsStillTimerSelect"></select>
                </label>
                <label class="field-stack">
                  <span class="input-label" id="settingsStillBurstLabel">Burst</span>
                  <select class="settings-select" id="settingsStillBurstSelect"></select>
                </label>
                <label class="field-stack field-span-2">
                  <span class="input-label">Timelapse</span>
                  <div class="pill-grid two-col" id="settingsStillTimelapseToggleOptions"></div>
                </label>
                <label class="field-stack field-span-2">
                  <span class="input-label">Interval</span>
                  <select class="settings-select" id="settingsStillTimelapseSelect"></select>
                </label>
                <div class="field-stack field-span-2 hidden" id="settingsStillCustomIntervalField">
                  <label class="input-label" for="settingsStillCustomIntervalInput">Custom interval (seconds, 0.1-86400)</label>
                  <input class="settings-input" id="settingsStillCustomIntervalInput" type="number" min="0.1" max="86400" step="0.1" value="5">
                  <button class="secondary-button" id="settingsStillCustomIntervalApply" type="button">Apply Interval</button>
                </div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Sound Recorder</h3>
                  <p class="settings-card-copy" id="settingsSoundRecorderNote">Record WAV only audio</p>
                </div>
                <div class="online-pill" id="settingsSoundRecorderValue">Off</div>
              </div>
              <div class="pill-grid" id="settingsSoundRecorderOptions"></div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Recording format</h3>
                  <p class="settings-card-copy">Choose the active capture resolution and raw container for the next clips.</p>
                </div>
                <div class="online-pill" id="settingsRecordFormatValue">CinemaDNG</div>
              </div>
              <div class="settings-field-grid">
                <label class="field-stack field-span-2" id="settingsRecordResolutionField">
                  <span class="input-label" id="settingsRecordResolutionLabel">Resolution</span>
                  <select class="settings-select" id="settingsRecordResolutionSelect"></select>
                </label>
                <label class="field-stack hidden" id="settingsRecordCropField">
                  <span class="input-label" id="settingsRecordCropLabel">Crop mode</span>
                  <select class="settings-select" id="settingsRecordCropSelect"></select>
                </label>
                <label class="field-stack field-span-2 hidden" id="settingsH264ResolutionField">
                  <span class="input-label">H.264 Size</span>
                  <select class="settings-select" id="settingsH264ResolutionSelect"></select>
                </label>
              </div>
              <div class="pill-grid" id="settingsRecordFormatOptions"></div>
              <div class="field-stack hidden" id="settingsProresColorField">
                <label class="input-label" for="settingsProresColorSelect">Color Profile</label>
                <select class="settings-select" id="settingsProresColorSelect">
                  <option value="standard">Rec709 / Gamma 2.4</option>
                  <option value="arri_logc4">ARRI Wide Gamut 4 / LogC4 (10-bit)</option>
                  <option value="sony_slog3">Sony S-Gamut3.Cine / S-Log3</option>
                  <option value="p3_d60_linear">P3-D60 / Linear</option>
                  <option value="davinci_intermediate">DaVinci Wide Gamut / Intermediate</option>
                </select>
                <p class="settings-card-copy" id="settingsProresColorNote"></p>
              </div>
              <div class="field-stack hidden" id="settingsCdngPreviewColorField">
                <label class="input-label" for="settingsCdngPreviewColorSelect">Preview Color Space / Gamma</label>
                <select class="settings-select" id="settingsCdngPreviewColorSelect">
                  <option value="standard">Rec709 / Gamma 2.4</option>
                  <option value="arri_logc4">ARRI Wide Gamut 4 / LogC4</option>
                  <option value="sony_slog3">Sony S-Gamut3.Cine / S-Log3</option>
                  <option value="p3_d60_linear">P3-D60 / Linear</option>
                  <option value="davinci_intermediate">DaVinci Wide Gamut / Intermediate</option>
                </select>
                <p class="settings-card-copy" id="settingsCdngPreviewColorNote">Preview only, including HDMI and WebUI. DNG stays sensor RAW; Resolve decoding is unchanged.</p>
              </div>
              <div class="settings-field-grid" id="settingsCdngCompressionField">
                <label class="field-stack field-span-2">
                  <span class="input-label">Compression</span>
                  <select class="settings-select" id="settingsCdngCompressionSelect">
                    <option value="none">Uncompressed</option>
                    <option value="lossless_jpeg">Lossless JPEG</option>
                    <option value="lossy_3to1">Lossy 3:1</option>
                    <option value="lossy_5to1">Lossy 5:1</option>
                  </select>
                </label>
              </div>
              <div class="settings-card-head compact-head">
                <div>
                  <h3 class="settings-card-title">Bit Depth</h3>
                  <p class="settings-card-copy" id="settingsRawBitDepthNote">Raw sensor bit depth for IMX585/IMX294 capture.</p>
                </div>
                <div class="online-pill" id="settingsRawBitDepthValue">12-bit</div>
              </div>
              <div class="pill-grid three-col" id="settingsRawBitDepthOptions"></div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Exposure Compensation</h3>
                  <p class="settings-card-copy">Auto ISO / shutter target. Minus is darker; plus is brighter.</p>
                </div>
              </div>
              <label class="field-stack">
                <span class="input-label">Compensation (EV)</span>
                <select class="settings-select" id="settingsExposureCompensationSelect"></select>
              </label>
              <label class="field-stack">
                <span class="input-label">Metering</span>
                <select class="settings-select" id="settingsExposureMeteringSelect"></select>
              </label>
              <p class="settings-card-copy" id="settingsExposureCompensationNote">Enable Auto ISO or Auto Shutter to adjust exposure.</p>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Gyro Data</h3>
                  <p class="settings-card-copy">Write IMU motion data beside each clip.</p>
                </div>
                <div class="online-pill" id="settingsGyroDataValue">Off</div>
              </div>
              <div class="pill-grid" id="settingsGyroDataOptions"></div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Gyro Calibration</h3>
                  <p class="settings-card-copy">Set IMU driver, orientation, sync offset, and still-camera bias.</p>
                </div>
                <div class="online-pill" id="settingsImuStatusPill">Idle</div>
              </div>

              <div class="settings-field-grid">
                <label class="field-stack">
                  <span class="input-label">IMU Driver</span>
                  <select class="settings-select" id="settingsImuDriverSelect"></select>
                </label>
                <label class="field-stack">
                  <span class="input-label">Orientation</span>
                  <select class="settings-select" id="settingsImuOrientationSelect"></select>
                </label>
              </div>

              <div class="imu-live-card">
                <div class="imu-axis-visual" aria-hidden="true">
                  <canvas class="imu-axis-canvas" id="settingsImuAxisCanvas" width="224" height="224"></canvas>
                </div>
                <div class="imu-live-copy">
                  <div class="imu-live-title">Live Gyro</div>
                  <div class="imu-live-status">
                    <span class="imu-live-dot" id="settingsImuLiveDot"></span>
                    <span id="settingsImuLiveStatus">Live gyro idle</span>
                  </div>
                  <div class="imu-readout" id="settingsImuOrientationReadout">Orientation XYZ | 500 Hz</div>
                  <div class="imu-readout"><strong>A</strong> <span id="settingsImuAccelReadout">+0.00 / +0.00 / +0.00 g</span></div>
                  <div class="imu-readout"><strong>G</strong> <span id="settingsImuGyroReadout">+0.00 / +0.00 / +0.00 dps</span></div>
                  <div class="imu-readout">Sync <span id="settingsImuSyncReadout">0 ms</span></div>
                  <div class="imu-readout">Level <span id="settingsImuLevelReadout">Not set</span></div>
                  <div class="imu-readout">Config <span id="settingsImuConfigReadout">/home/pi/apertar-hardware.conf</span></div>
                </div>
              </div>

              <div class="field-stack">
                <span class="input-label">Sync Offset</span>
                <div class="imu-stepper-row">
                  <button class="secondary-button" id="settingsImuSyncMinusButton" type="button">-1 ms</button>
                  <div class="imu-sync-value" id="settingsImuSyncOffsetValue">0 ms</div>
                  <button class="secondary-button" id="settingsImuSyncPlusButton" type="button">+1 ms</button>
                </div>
                <button class="secondary-button" id="settingsImuSyncResetButton" type="button">Reset Sync Offset</button>
                <span class="subtle-note">Set this to the sync offset that Gyroflow reports after a test clip. Positive values move the gyro timeline earlier against the video.</span>
              </div>

              <div class="action-grid two-col">
                <button class="secondary-button" id="settingsImuSetLevelButton" type="button">Set Level</button>
                <button class="secondary-button" id="settingsImuClearLevelButton" type="button">Clear Level</button>
              </div>
              <span class="subtle-note" id="settingsImuLevelNote">Place the camera in its neutral shooting position, then press Set Level.</span>

              <button class="secondary-button" id="settingsImuCalibrateButton" type="button">Calibrate Gyro Bias</button>
              <span class="subtle-note" id="settingsImuCalibrationStatus"></span>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Timecode</h3>
                  <p class="settings-card-copy">Timecode behavior.</p>
                </div>
                <div class="online-pill" id="settingsTimecodeValue">Free Run</div>
              </div>
              <div class="pill-grid" id="settingsTimecodeOptions"></div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Autofocus</h3>
                  <p class="settings-card-copy">Lens focus mode, range, speed, and metering window.</p>
                </div>
                <div class="online-pill" id="settingsAutofocusValue">Continuous</div>
              </div>
              <section class="select-block">
                <div class="select-topline"><span>Mode</span></div>
                <div class="pill-grid" id="settingsAutofocusModeOptions"></div>
              </section>
              <section class="select-block">
                <div class="select-topline"><span>Range</span></div>
                <div class="pill-grid" id="settingsAutofocusRangeOptions"></div>
              </section>
              <section class="select-block">
                <div class="select-topline"><span>Speed</span></div>
                <div class="pill-grid" id="settingsAutofocusSpeedOptions"></div>
              </section>
              <section class="select-block">
                <div class="select-topline"><span>Window</span></div>
                <div class="pill-grid" id="settingsAutofocusWindowOptions"></div>
              </section>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Current capture setup</h3>
                  <p class="settings-card-copy">A quick readout of the active camera profile.</p>
                </div>
              </div>
              <div class="settings-summary-grid">
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">Sensor</div>
                  <div class="settings-summary-value" id="settingsSensorValue">--</div>
                </div>
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">Resolution</div>
                  <div class="settings-summary-value" id="settingsResolutionSummary">--</div>
                </div>
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">Frame rate</div>
                  <div class="settings-summary-value" id="settingsFpsSummary">--</div>
                </div>
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">Storage</div>
                  <div class="settings-summary-value" id="settingsStorageSummary">--</div>
                </div>
              </div>
            </div>
          </section>

          <section class="settings-panel active" data-panel="favorites">
            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Quick Access</h3>
                  <p class="settings-card-copy">Choose which settings appear in the shared Favorites list used by the camera UI.</p>
                </div>
                <div class="online-pill" id="settingsFavoriteCountPill">0 items</div>
              </div>
              <div class="settings-action-row">
                <button class="secondary-button" id="clearFavoritesButton" type="button">Clear All</button>
                <span class="subtle-note">Changes here sync back to the DPI UI favorites tab.</span>
              </div>
            </div>

            <div class="stack" id="settingsFavoriteControls">
              <div class="settings-card">
                <div class="preset-empty">No favorites selected yet. Add some below to build your quick access tab.</div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Arrange Favorites</h3>
                  <p class="settings-card-copy">Reorder or remove the settings that appear in your quick access tab.</p>
                </div>
              </div>
              <div class="preset-list" id="settingsFavoriteList">
                <div class="preset-empty">No favorites selected yet.</div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Available Settings</h3>
                  <p class="settings-card-copy">Add any supported camera setting to your shared favorites list.</p>
                </div>
              </div>
              <div class="preset-list" id="settingsFavoriteCandidateList">
                <div class="preset-empty">Loading available favorite settings...</div>
              </div>
            </div>
          </section>

          <section class="settings-panel" data-panel="monitoring">
            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Live preview</h3>
                  <p class="settings-card-copy">Current preview transport and monitoring endpoint for the web feed.</p>
                </div>
                <div class="online-pill" id="settingsPreviewStatus">Live</div>
              </div>
              <div class="capture-list">
                <div class="capture-row">
                  <span class="capture-key">Preview stream</span>
                  <span class="capture-value" id="settingsPreviewStreamValue">MJPEG preview</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Transport</span>
                  <span class="capture-value" id="settingsPreviewTransportValue">Wi-Fi / Ethernet</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Control address</span>
                  <span class="capture-value" id="settingsPreviewHostValue">--</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Preview source</span>
                  <span class="capture-value" id="settingsPreviewSourceValue">--</span>
                </div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">External monitor</h3>
                  <p class="settings-card-copy">Configure HDMI monitoring controls.</p>
                </div>
              </div>

              <section class="select-block">
                <div class="select-topline">
                  <span>External monitor</span>
                </div>
                <div class="pill-grid" id="settingsExternalMonitorOptions"></div>
              </section>

              <div class="settings-field-grid">
                <div class="field-stack">
                  <label class="input-label" for="settingsExternalMonitorModeSelect">Mode</label>
                  <select class="settings-select" id="settingsExternalMonitorModeSelect"></select>
                </div>
                <div class="field-stack">
                  <label class="input-label" for="settingsExternalMonitorInfoOverlaySelect">Info Overlay</label>
                  <select class="settings-select" id="settingsExternalMonitorInfoOverlaySelect"></select>
                </div>
                <div class="field-stack">
                  <label class="input-label" for="settingsExternalMonitorOrientationSelect">Orientation</label>
                  <select class="settings-select" id="settingsExternalMonitorOrientationSelect"></select>
                </div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Assist tools</h3>
                  <p class="settings-card-copy">Manage monitoring overlays and assist modes.</p>
                </div>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>LUT Preview</span>
                  </div>
                  <div class="pill-grid" id="settingsLutPreviewOptions"></div>
                </section>
                <div class="settings-field-grid">
                  <div class="field-stack field-span-2">
                    <label class="input-label" for="settingsLutPreviewSelect">LUT</label>
                    <select class="settings-select" id="settingsLutPreviewSelect"></select>
                    <span class="subtle-note">Scans /home/pi/LUTs and recording media /LUTs.</span>
                  </div>
                </div>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>Focus peaking</span>
                  </div>
                  <div class="pill-grid" id="settingsFocusPeakingOptions"></div>
                </section>
                <div class="settings-field-grid">
                  <div class="field-stack">
                    <label class="input-label" for="settingsFocusPeakingThresholdSelect">Threshold</label>
                    <select class="settings-select" id="settingsFocusPeakingThresholdSelect"></select>
                  </div>
                  <div class="field-stack">
                    <label class="input-label" for="settingsFocusPeakingColorSelect">Color</label>
                    <select class="settings-select" id="settingsFocusPeakingColorSelect"></select>
                  </div>
                </div>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>Zebra</span>
                  </div>
                  <div class="pill-grid" id="settingsZebraOptions"></div>
                </section>
                <div class="settings-field-grid">
                  <div class="field-stack">
                    <label class="input-label" for="settingsZebraLevelSelect">Level</label>
                    <select class="settings-select" id="settingsZebraLevelSelect"></select>
                  </div>
                </div>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>False color</span>
                  </div>
                  <div class="pill-grid" id="settingsFalseColorOptions"></div>
                </section>
                <div class="settings-field-grid">
                  <div class="field-stack field-span-2">
                    <label class="input-label" for="settingsFalseColorModeSelect">Mode</label>
                    <select class="settings-select" id="settingsFalseColorModeSelect"></select>
                  </div>
                </div>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>Grayscale</span>
                  </div>
                  <div class="pill-grid" id="settingsGrayscaleOptions"></div>
                </section>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>SMPTE Bars</span>
                  </div>
                  <div class="pill-grid" id="settingsSmpteOptions"></div>
                </section>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Framing aids</h3>
                  <p class="settings-card-copy">Guides, center marker, and anamorphic desqueeze controls.</p>
                </div>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>Guides</span>
                  </div>
                  <div class="pill-grid" id="settingsGuidesOptions"></div>
                </section>
                <div class="settings-field-grid">
                  <div class="field-stack">
                    <label class="input-label" for="settingsGuidesTypeSelect">Type</label>
                    <select class="settings-select" id="settingsGuidesTypeSelect"></select>
                  </div>
                  <div class="field-stack">
                    <label class="input-label" for="settingsGuidesThicknessSelect">Thickness</label>
                    <select class="settings-select" id="settingsGuidesThicknessSelect"></select>
                  </div>
                </div>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>Center marker</span>
                  </div>
                  <div class="pill-grid" id="settingsCenterMarkerOptions"></div>
                </section>
                <div class="settings-field-grid">
                  <div class="field-stack field-span-2">
                    <label class="input-label" for="settingsCenterMarkerTypeSelect">Type</label>
                    <select class="settings-select" id="settingsCenterMarkerTypeSelect"></select>
                  </div>
                </div>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>Anamorphic desqueeze</span>
                  </div>
                  <div class="pill-grid" id="settingsAnamorphicOptions"></div>
                </section>
                <div class="settings-field-grid">
                  <div class="field-stack field-span-2">
                    <label class="input-label" for="settingsAnamorphicRatioSelect">Ratio</label>
                    <select class="settings-select" id="settingsAnamorphicRatioSelect"></select>
                  </div>
                </div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Live camera readout</h3>
                  <p class="settings-card-copy">Shows the current image settings.</p>
                </div>
              </div>
              <div class="settings-summary-grid">
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">ISO</div>
                  <div class="settings-summary-value" id="settingsIsoSummary">--</div>
                </div>
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">White balance</div>
                  <div class="settings-summary-value" id="settingsWbSummary">--</div>
                </div>
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">Shutter angle</div>
                  <div class="settings-summary-value" id="settingsShutterSummary">--</div>
                </div>
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">Codec</div>
                  <div class="settings-summary-value" id="settingsCodecSummary">--</div>
                </div>
              </div>
            </div>
          </section>

          <section class="settings-panel" data-panel="audio">
            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Audio</h3>
                  <p class="settings-card-copy">Configure input and recording options.</p>
                </div>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>Record Audio</span>
                  </div>
                  <div class="pill-grid" id="settingsRecordAudioOptions"></div>
                </section>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>Live Audio Monitoring</span>
                  </div>
                  <div class="pill-grid" id="settingsLiveAudioMonitoringOptions"></div>
                </section>
              </div>

              <div class="settings-option-group">
                <section class="select-block">
                  <div class="select-topline">
                    <span>VU Meter</span>
                  </div>
                  <div class="pill-grid" id="settingsAudioMeterOptions"></div>
                </section>
              </div>

              <div class="settings-field-grid">
                <div class="field-stack field-span-2">
                  <label class="input-label" for="settingsAudioInputDeviceSelect">Input Device</label>
                  <select class="settings-select" id="settingsAudioInputDeviceSelect"></select>
                  <span class="subtle-note" id="settingsAudioInputNote">No audio input device detected.</span>
                </div>
                <div class="field-stack">
                  <label class="input-label" for="settingsAudioChannelsSelect">Input Channels</label>
                  <select class="settings-select" id="settingsAudioChannelsSelect"></select>
                </div>
                <div class="field-stack">
                  <label class="input-label" for="settingsAudioRateSelect">Sample Rate</label>
                  <select class="settings-select" id="settingsAudioRateSelect"></select>
                </div>
                <div class="field-stack field-span-2">
                  <label class="input-label" for="settingsAudioFormatSelect">Sample Format</label>
                  <select class="settings-select" id="settingsAudioFormatSelect"></select>
                  <span class="subtle-note" id="settingsAudioCaptureNote"></span>
                  <button class="secondary-button" id="refreshAudioCapabilitiesButton" type="button">Refresh Audio Input</button>
                </div>
                <div class="field-stack field-span-2">
                  <label class="input-label" for="settingsAudioOutputDeviceSelect">Output Device</label>
                  <select class="settings-select" id="settingsAudioOutputDeviceSelect"></select>
                  <span class="subtle-note" id="settingsAudioOutputNote">Choose where live monitoring should play back.</span>
                </div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Levels</h3>
                  <p class="settings-card-copy">Adjust recording gain and headphone monitor level.</p>
                </div>
              </div>

              <section class="select-block">
                <div class="select-topline">
                  <span>Input Volume</span>
                  <span class="select-value" id="settingsInputVolumeValue">60</span>
                </div>
                <input class="slider" id="settingsInputVolumeRange" type="range" min="0" max="100" step="1" value="60">
                <div class="slider-labels">
                  <span>0</span>
                  <span>100</span>
                </div>
              </section>

              <section class="select-block">
                <div class="select-topline">
                  <span>Headphone Volume</span>
                  <span class="select-value" id="settingsHeadphoneVolumeValue">55</span>
                </div>
                <input class="slider" id="settingsHeadphoneVolumeRange" type="range" min="0" max="100" step="1" value="55">
                <div class="slider-labels">
                  <span>0</span>
                  <span>100</span>
                </div>
              </section>
            </div>
          </section>

          <section class="settings-panel" data-panel="system">
            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">UI Orientation</h3>
                  <p class="settings-card-copy">Rotate the full DPI interface and live preview.</p>
                </div>
              </div>
              <div class="field-stack">
                <label class="input-label" for="settingsUiOrientationSelect">Orientation</label>
                <select class="settings-select" id="settingsUiOrientationSelect">
                  <option value="Landscape">Landscape</option>
                  <option value="Left Side">Left Side</option>
                  <option value="Right Side">Right Side</option>
                  <option value="Upside Down">Upside Down</option>
                </select>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Controls Transparency</h3>
                  <p class="settings-card-copy">Fade only the camera page control backgrounds while keeping text readable.</p>
                </div>
              </div>
              <div class="settings-field-grid">
                <label class="field-stack">
                  <span class="input-label">Opacity</span>
                  <select class="settings-select" id="settingsControlsOpacitySelect"></select>
                </label>
                <label class="field-stack">
                  <span class="input-label">Controls</span>
                  <select class="settings-select" id="settingsControlsModeSelect"></select>
                </label>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Date &amp; Time</h3>
                  <p class="settings-card-copy">Set local time and date for clips and metadata.</p>
                </div>
              </div>
              <div class="settings-field-grid">
                <div class="field-stack field-span-2">
                  <label class="input-label" for="settingsTimeZoneSelect">Timezone</label>
                  <select class="settings-select" id="settingsTimeZoneSelect"></select>
                </div>
                <div class="field-stack field-span-2">
                  <label class="input-label" for="settingsDateTimeInput">Local Date &amp; Time</label>
                  <input class="settings-input" id="settingsDateTimeInput" type="datetime-local" />
                </div>
              </div>
              <section class="select-block">
                <div class="select-topline">
                  <span>Show on Camera Page</span>
                </div>
                <span class="subtle-note">Display date and time on camera page.</span>
                <div class="pill-grid" id="settingsDateTimeOverlayOptions"></div>
              </section>
              <div class="settings-action-row">
                <button class="primary-button" id="applyDateTimeButton" type="button">Apply Date &amp; Time</button>
                <span class="subtle-note">Writes the system clock and RTC with the selected timezone.</span>
              </div>
            </div>

            <div class="settings-card" id="settingsMediaCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Media</h3>
                  <p class="settings-card-copy">Recording readiness and mounted media details.</p>
                </div>
                <div class="online-pill" id="settingsMediaStatePill">Ready</div>
              </div>
              <div class="capture-list">
                <div class="capture-row">
                  <span class="capture-key">Status</span>
                  <span class="capture-value" id="settingsMediaStatusValue">--</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Destination</span>
                  <span class="capture-value" id="settingsMediaTypeValue">--</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Mount path</span>
                  <span class="capture-value" id="settingsMediaPathValue">--</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key" id="settingsMediaTimeLeftLabel">Record time left</span>
                  <span class="capture-value" id="settingsMediaTimeLeftValue">--</span>
                </div>
              </div>
              <section class="select-block">
                <div class="select-topline">
                  <span>Space Left</span>
                  <span class="select-value" id="settingsMediaSpaceLeftValue">--</span>
                </div>
                <div class="progress-track"><div class="progress-fill" id="settingsMediaUsageProgress"></div></div>
                <div class="slider-labels">
                  <span id="settingsMediaUsageDetailValue">--</span>
                  <span id="settingsMediaCapacityValue">--</span>
                </div>
              </section>
              <div class="field-stack">
                <label class="input-label" for="settingsMediaDriveSelect">Mount Drive</label>
                <select class="settings-select" id="settingsMediaDriveSelect">
                  <option value="">No available drives</option>
                </select>
                <span class="subtle-note" id="settingsMediaDriveNote">Supports exFAT, FAT32, NTFS, and other Linux-supported filesystems.</span>
              </div>
              <div class="action-grid two-col">
                <button class="secondary-button" id="refreshMediaDrivesButton" type="button">Refresh Drives</button>
                <button class="secondary-button" id="mountMediaDriveButton" type="button">Mount Selected</button>
              </div>
              <div class="settings-action-row">
                <button class="secondary-button" id="ejectMediaButton" type="button">Eject Media</button>
                <span class="subtle-note">Unmounts current media and prepares removable media for safe removal.</span>
              </div>
            </div>

            <div class="settings-card" id="settingsFormatMediaCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Format Media</h3>
                  <p class="settings-card-copy">Choose the filesystem for the mounted drive while keeping its current drive name where possible.</p>
                </div>
                <div class="online-pill" id="settingsFormatMediaStatePill">Ready</div>
              </div>
              <div class="field-stack">
                <label class="input-label" for="settingsMediaFormatSelect">Format As</label>
                <select class="settings-select" id="settingsMediaFormatSelect">
                  <option value="exfat">exFAT</option>
                  <option value="vfat">FAT32</option>
                  <option value="ntfs">NTFS</option>
                </select>
                <span class="subtle-note">exFAT is recommended for recording. FAT32 and NTFS are useful for USB/stills workflows when needed.</span>
              </div>
              <div class="settings-action-row">
                <button class="danger-button" id="formatMediaButton" type="button">Format Media</button>
                <span class="subtle-note" id="settingsFormatMediaNote">This erases the mounted media, keeps the label when the filesystem allows it, and mounts it again.</span>
              </div>
            </div>

            <div class="settings-card" id="settingsMediaSpeedCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Media Speed</h3>
                  <p class="settings-card-copy">Measure sequential write performance for the mounted recording media.</p>
                </div>
              </div>
              <div class="capture-list">
                <div class="capture-row">
                  <span class="capture-key">Write Test</span>
                  <span class="capture-value" id="settingsMediaSpeedResultValue">Not Tested</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Detail</span>
                  <div class="capture-value-stack">
                    <span class="capture-detail" id="settingsMediaSpeedDetailValue">Runs a temporary direct write test and removes the file afterwards.</span>
                  </div>
                </div>
              </div>
              <div class="settings-action-row">
                <button class="secondary-button" id="runMediaSpeedTestButton" type="button">Run Speed Test</button>
                <span class="subtle-note">Runs a temporary direct write test and removes the file afterwards.</span>
              </div>
            </div>

            <div class="settings-card" id="settingsThermalCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Thermal</h3>
                  <p class="settings-card-copy">Fan control and temperature management.</p>
                </div>
              </div>
              <div class="settings-summary-grid">
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">CPU Temperature</div>
                  <div class="settings-summary-value" id="settingsThermalCpuValue">--</div>
                </div>
                <div class="settings-summary-tile">
                  <div class="settings-summary-label">Fan Mode</div>
                  <div class="settings-summary-value" id="settingsThermalFanModeValue">Auto</div>
                </div>
              </div>
              <div class="field-stack">
                <label class="input-label" for="settingsFanModeSelect">Fan Mode</label>
                <select class="settings-select" id="settingsFanModeSelect">
                  <option value="Silent">Silent</option>
                  <option value="Auto">Auto</option>
                  <option value="Full Blast">Full Blast</option>
                </select>
                <span class="subtle-note" id="settingsFanModeNote">Fan mode control is available when supported by the board.</span>
              </div>
            </div>

            <div class="settings-card" id="settingsPowerActionsCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Power Actions</h3>
                  <p class="settings-card-copy">Safe device control actions from the web control page.</p>
                </div>
              </div>
              <div class="action-grid two-col power-actions-grid">
                <button class="secondary-button standby-button" id="settingsStandbyButton" type="button">Standby</button>
                <button class="secondary-button" id="restartSystemButton" type="button">Restart Camera</button>
                <button class="danger-button" id="powerOffSystemButton" type="button">Shutdown</button>
              </div>
              <div class="settings-action-row">
                <span class="subtle-note">Uses the same system-level permissions as the local UI.</span>
              </div>
            </div>
          </section>

          <section class="settings-panel" data-panel="power">
            <div class="settings-card" id="settingsBatteryCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Battery</h3>
                  <p class="settings-card-copy">Set the battery profile and capacity used for runtime estimates.</p>
                </div>
                <div class="online-pill" id="settingsBatteryTypeValue">V-Mount</div>
              </div>
              <div class="field-stack">
                <label class="input-label" for="settingsBatteryTypeSelect">Battery Type</label>
                <select class="settings-select" id="settingsBatteryTypeSelect"></select>
              </div>
              <div class="field-stack">
                <label class="input-label" for="settingsBatteryCapacitySelect">Battery Capacity</label>
                <select class="settings-select" id="settingsBatteryCapacitySelect"></select>
              </div>
              <div class="settings-action-row">
                <span class="subtle-note" id="settingsBatteryCapacityNote">Used for remaining runtime estimates.</span>
              </div>
            </div>

            <div class="settings-card hidden" id="settingsCustomBatteryCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Custom Capacity</h3>
                  <p class="settings-card-copy">Fine-tune custom capacity and voltage endpoints.</p>
                </div>
              </div>
              <div class="settings-summary-grid">
                <div class="settings-summary-tile" style="grid-column: 1 / -1;">
                  <div class="settings-summary-label">Battery Capacity</div>
                  <div class="settings-summary-value" id="settingsCustomBatteryWhValue">150Wh</div>
                </div>
                <div class="settings-summary-tile custom-battery-voltage">
                  <div class="settings-summary-label">Full Voltage</div>
                  <div class="settings-summary-value" id="settingsCustomBatteryFullVoltageValue">16.8V</div>
                </div>
                <div class="settings-summary-tile custom-battery-voltage">
                  <div class="settings-summary-label">Empty Voltage</div>
                  <div class="settings-summary-value" id="settingsCustomBatteryEmptyVoltageValue">13.2V</div>
                </div>
              </div>
              <div class="action-grid two-col">
                <button class="secondary-button" id="decreaseCustomBatteryWhButton" type="button">-5Wh</button>
                <button class="secondary-button" id="increaseCustomBatteryWhButton" type="button">+5Wh</button>
              </div>
              <div class="action-grid two-col custom-battery-voltage">
                <button class="secondary-button" id="decreaseCustomBatteryFullVoltageButton" type="button">Full -0.1V</button>
                <button class="secondary-button" id="increaseCustomBatteryFullVoltageButton" type="button">Full +0.1V</button>
                <button class="secondary-button" id="decreaseCustomBatteryEmptyVoltageButton" type="button">Empty -0.1V</button>
                <button class="secondary-button" id="increaseCustomBatteryEmptyVoltageButton" type="button">Empty +0.1V</button>
              </div>
            </div>

            <div class="settings-card" id="settingsBatteryInfoCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Battery Info</h3>
                  <p class="settings-card-copy">Live power telemetry and estimated remaining runtime.</p>
                </div>
              </div>
              <div class="capture-list">
                <div class="capture-row">
                  <span class="capture-key">Voltage / Current</span>
                  <div class="capture-value-stack">
                    <span class="capture-value" id="settingsBatteryInfoPrimary">--</span>
                    <span class="capture-detail" id="settingsBatteryInfoSecondary">--</span>
                  </div>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Runtime Estimate</span>
                  <span class="capture-value" id="settingsBatteryRuntimeValue">--</span>
                </div>
              </div>
            </div>

            <div class="settings-card" id="settingsAutoPowerOffCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Auto Power Off</h3>
                  <p class="settings-card-copy">Automatically shut down after inactivity.</p>
                </div>
                <div class="online-pill" id="settingsSleepModeValue">Off</div>
              </div>
              <div class="field-stack">
                <label class="input-label" for="settingsSleepModeSelect">Power Off</label>
                <select class="settings-select" id="settingsSleepModeSelect">
                  <option value="Off">Off</option>
                  <option value="1 min">1 min</option>
                  <option value="2 min">2 min</option>
                  <option value="5 min">5 min</option>
                  <option value="10 min">10 min</option>
                </select>
              </div>
            </div>
            <div class="settings-card" id="settingsStandbyCard">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Camera Standby</h3>
                  <p class="settings-card-copy">Wake with a hardware button or one power-button press, not the touchscreen. Networking stays available.</p>
                </div>
              </div>
              <div class="field-stack">
                <label class="input-label" for="settingsStandbySelect">After Inactivity</label>
                <select class="settings-select" id="settingsStandbySelect">
                  <option value="0">Off</option>
                  <option value="30">30 sec</option>
                  <option value="60">1 min</option>
                  <option value="120">2 min</option>
                  <option value="300">5 min</option>
                  <option value="600">10 min</option>
                  <option value="900">15 min</option>
                  <option value="1800">30 min</option>
                </select>
              </div>
              <div class="field-stack">
                <label class="input-label" for="settingsStandbyUsbSelect">USB Power in Standby</label>
                <select class="settings-select" id="settingsStandbyUsbSelect">
                  <option value="false">Keep On</option>
                  <option value="true">Off</option>
                </select>
                <p class="settings-card-copy">USB power-off is skipped when USB storage is mounted or a USB network adapter is present.</p>
              </div>
            </div>
          </section>

          <section class="settings-panel" data-panel="presets">
            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Save Current Preset</h3>
                  <p class="settings-card-copy">Capture the live Apertar setup into the same preset format used by the native UI.</p>
                </div>
              </div>
              <div class="settings-field-grid">
                <div class="field-stack field-span-2">
                  <label class="input-label" for="savePresetNameInput">Preset Name</label>
                  <input class="settings-input" id="savePresetNameInput" type="text" placeholder="Preset Name" maxlength="64" />
                </div>
              </div>
              <div class="settings-action-row">
                <button class="primary-button" id="saveCurrentPresetButton" type="button">Save Current Preset</button>
                <span class="subtle-note">Saved presets appear here and in the camera UI preset library.</span>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Preset Library</h3>
                  <p class="settings-card-copy">Download camera presets saved on the Apertar system and share them anywhere.</p>
                </div>
                <div class="online-pill" id="settingsPresetCountPill">0</div>
              </div>
              <div class="settings-action-row">
                <button class="primary-button" id="importPresetButton" type="button">Import Presets</button>
                <button class="secondary-button" id="refreshPresetsButton" type="button">Refresh Library</button>
                <span class="subtle-note">These are the same preset files used by the native UI.</span>
              </div>
              <input id="presetImportInput" type="file" accept=".apertar-preset.json,application/json" multiple style="display:none;" />
              <div class="preset-list" id="settingsPresetList">
                <div class="preset-empty">No presets saved on the camera yet.</div>
              </div>
            </div>
          </section>

          <section class="settings-panel" data-panel="config">
            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Apertar Config</h3>
                  <p class="settings-card-copy">Edit the hardware, display, input, media, and IMU config file used by ApertarCore and the native UI.</p>
                </div>
                <div class="online-pill" id="settingsConfigStatePill">Not Loaded</div>
              </div>

              <div class="config-meta-grid">
                <div class="config-meta-tile">
                  <div class="config-meta-label">Path</div>
                  <div class="config-meta-value" id="settingsConfigPathValue">/home/pi/apertar-hardware.conf</div>
                </div>
                <div class="config-meta-tile">
                  <div class="config-meta-label">Status</div>
                  <div class="config-meta-value" id="settingsConfigMetaValue">Open Config to load</div>
                </div>
              </div>

              <label class="field-stack">
                <span class="input-label">Config File</span>
                <textarea class="settings-input config-editor" id="settingsConfigEditor" spellcheck="false" wrap="off" placeholder="Loading Apertar config..."></textarea>
              </label>

              <div class="settings-action-row">
                <button class="secondary-button" id="reloadConfigButton" type="button">Reload Config</button>
                <button class="primary-button" id="saveConfigButton" type="button" disabled>Save Config</button>
                <span class="subtle-note">Hardware, input, display, and sensor changes may need ApertarCore or the UI restarted before they fully apply.</span>
              </div>
            </div>
          </section>

          <section class="settings-panel" data-panel="info">
            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">Camera</h3>
                  <p class="settings-card-copy">Detected camera backend and sensor information.</p>
                </div>
              </div>
              <div class="capture-list">
                <div class="capture-row">
                  <span class="capture-key">Sensor</span>
                  <div class="capture-value-stack">
                    <span class="capture-value" id="settingsInfoSensorValue">--</span>
                    <span class="capture-detail" id="settingsInfoSensorDetail">--</span>
                  </div>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Backend</span>
                  <div class="capture-value-stack">
                    <span class="capture-value" id="settingsInfoBackendValue">--</span>
                    <span class="capture-detail" id="settingsInfoBackendDetail">--</span>
                  </div>
                </div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">SBC</h3>
                  <p class="settings-card-copy">Board identity and system memory details.</p>
                </div>
              </div>
              <div class="capture-list">
                <div class="capture-row">
                  <span class="capture-key">Model</span>
                  <span class="capture-value" id="settingsInfoPiModelValue">--</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Serial Number</span>
                  <span class="capture-value" id="settingsInfoSerialValue">--</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Memory</span>
                  <div class="capture-value-stack">
                    <span class="capture-value" id="settingsInfoMemoryValue">--</span>
                    <span class="capture-detail" id="settingsInfoMemoryDetail">--</span>
                  </div>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Operating System</span>
                  <div class="capture-value-stack">
                    <span class="capture-value" id="settingsInfoOsValue">--</span>
                    <span class="capture-detail" id="settingsInfoHostnameDetail">--</span>
                  </div>
                </div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">System</h3>
                  <p class="settings-card-copy">Runtime and network information.</p>
                </div>
              </div>
              <div class="capture-list">
                <div class="capture-row">
                  <span class="capture-key">Kernel Version</span>
                  <span class="capture-value" id="settingsInfoKernelValue">--</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">Uptime</span>
                  <span class="capture-value" id="settingsInfoUptimeValue">--</span>
                </div>
                <div class="capture-row">
                  <span class="capture-key">IP Address</span>
                  <span class="capture-value" id="settingsInfoIpValue">--</span>
                </div>
              </div>
            </div>

            <div class="settings-card">
              <div class="settings-card-head">
                <div>
                  <h3 class="settings-card-title">System Drive</h3>
                  <p class="settings-card-copy">Internal system and boot storage information.</p>
                </div>
              </div>
              <div class="capture-list">
                <div class="capture-row">
                  <span class="capture-key">System / SD Storage</span>
                  <div class="capture-value-stack">
                    <span class="capture-value" id="settingsInfoSystemDriveValue">--</span>
                    <span class="capture-detail" id="settingsInfoSystemDriveDetail">--</span>
                  </div>
                </div>
              </div>
            </div>
          </section>
        </div>
      </section>
    </div>
  </div>

  <div class="media-browser-overlay hidden" id="mediaBrowserOverlay">
    <div class="media-browser-shell">
      <div class="media-browser-header">
        <div>
          <div class="media-browser-title-row">
            <div class="media-browser-icon">
              <img class="icon-image" src="/icons/folder.png" alt="">
            </div>
            <div>
              <h2 class="section-title" style="margin:0;">Clip Browser</h2>
              <p class="settings-sidebar-copy">Download or delete clips, stills, timelapses, and audio recordings from mounted media.</p>
            </div>
          </div>
        </div>
        <button class="icon-button settings-back-button" id="closeMediaBrowserButton" type="button" aria-label="Back">
          <span>Back</span>
        </button>
      </div>
      <div class="media-browser-body">
        <div class="media-browser-toolbar">
          <div class="media-browser-filters" id="mediaBrowserFilters">
            <button class="media-filter-button active" data-media-filter="clip" type="button">Clips</button>
            <button class="media-filter-button" data-media-filter="still" type="button">Stills</button>
            <button class="media-filter-button" data-media-filter="timelapse" type="button">Timelapse</button>
            <button class="media-filter-button" data-media-filter="audio" type="button">Audio</button>
          </div>
          <div class="media-browser-tools">
            <div class="media-browser-filters media-browser-tools-row">
              <div class="online-pill" id="mediaBrowserCountPill">0 items</div>
              <button class="secondary-button" id="refreshMediaBrowserButton" type="button">Refresh Media</button>
              <select class="settings-select media-sort-select" id="mediaBrowserSortSelect" aria-label="Sort media">
                <option value="newest">Newest First</option>
                <option value="oldest">Oldest First</option>
              </select>
            </div>
          </div>
        </div>
        <div class="media-selection-toolbar" id="mediaSelectionToolbar">
          <div class="media-selection-count" id="mediaSelectionCount">No items selected</div>
          <div class="media-selection-actions">
            <button class="secondary-button media-select-button" id="selectAllMediaButton" type="button">Select All</button>
            <button class="secondary-button media-select-button" id="clearMediaSelectionButton" type="button">Clear</button>
            <button class="primary-button media-select-button" id="downloadSelectedMediaButton" type="button" disabled>Download Selected</button>
            <button class="danger-button media-select-button" id="deleteSelectedMediaButton" type="button" disabled>Delete Selected</button>
          </div>
        </div>
        <div class="media-browser-list" id="mediaBrowserList">
          <div class="preset-empty">Open the browser to scan mounted media.</div>
        </div>
      </div>
    </div>
  </div>

  <div class="loading-overlay" id="loadingOverlay">
    <div class="loading-card">Connecting to ApertarCore...</div>
  </div>

  <div class="toast" id="toast"></div>

  <script>
    const fpsPresets = [24.000, 25.000, 30.000, 35.000, 40.000, 45.000, 50.000, 60.000, 70.000, 80.000, 90.000, 100.000, 110.000, 120.000, 130.000, 140.000, 150.000, 160.000, 170.000, 180.000, 190.000, 200.000, 210.000, 220.000, 230.000, 240.000, 280.000];
    const isoOptions = ["Auto", "100", "200", "400", "800", "1600", "3200", "6400", "12800", "25600", "51200"];
    const shutterOptions = ["Auto", "11.25°", "15°", "22.5°", "30°", "37.5°", "45°", "60°", "72°", "75°", "90°", "108°", "120°", "144°", "150°", "172.8°", "180°", "216°", "270°", "324°", "360°"];
    const fallbackResolutionOptions = ["1920x1080", "3840x2160"];
    const whiteBalanceOptions = [2500, 2800, 3000, 3200, 3400, 3600, 4000, 4500, 4800, 5000, 5200, 5400, 5600, 5800, 6000, 6500, 7000, 7500, 8000];
    const recordFormatOptions = [
      { value: "CinemaDNG", label: "CinemaDNG", enabled: true },
      { value: "MLV", label: "MLV", enabled: true },
      { value: "H.264", label: "H.264", enabled: true },
      { value: "ProRes Proxy", label: "ProRes Proxy", enabled: true },
      { value: "ProRes LT", label: "ProRes LT", enabled: true },
      { value: "ProRes 422", label: "ProRes 422", enabled: true },
      { value: "ProRes HQ", label: "ProRes HQ", enabled: true },
    ];
    const rawBitDepthOptions = ["12-bit", "10-bit", "8-bit"];
    const h264ResolutionOptions = ["Match Capture", "1920x1080", "2560x1440"];
    const h264BaseResolutionOptions = [
      { value: "1920x1080", label: "1920x1080" },
      { value: "2560x1440", label: "2560x1440" },
    ];
    const timecodeOptions = ["Free Run", "Rec Run"];
    const autofocusModeOptions = ["Manual", "Auto", "Continuous"];
    const autofocusRangeOptions = ["Normal", "Macro", "Full"];
    const autofocusSpeedOptions = ["Normal", "Fast"];
    const autofocusWindowOptions = ["Center", "Full Frame"];
    const imuDriverOptions = ["MPU6050", "BMI160", "BMI270", "ICM42688-P"];
    const imuOrientationOptions = ["XYZ", "xYz", "XyZ", "XYz", "xYZ", "Xyz", "xyZ", "xyz"];
    const toggleOptions = ["Off", "On"];
    const shutterSpeedOptions = ["Auto", "1/24", "1/30", "1/40", "1/48", "1/50", "1/60", "1/80", "1/100", "1/125", "1/160", "1/200", "1/250", "1/320", "1/500", "1/1000"];
    const longShutterSpeedOptions = ["Auto", "30s", "25s", "20s", "15s", "10s", "8s", "6s", "5s", "4s", "3s", "2s", "1s", "1/2", "1/4", "1/8", "1/12", "1/15", ...shutterSpeedOptions.slice(1)];
    const stillTimerOptions = ["Off", "2s", "5s", "10s", "15s", "20s", "25s", "30s"];
    const stillBurstOptions = ["Single", "3 Shots", "5 Shots", "10 Shots"];
    const bracketEvOptions = ["1/3 EV", "2/3 EV", "1 EV", "1 1/3 EV", "1 2/3 EV", "2 EV", "2 1/3 EV", "2 2/3 EV", "3 EV"];
    const stillFormatOptions = ["DNG", "JPEG", "PNG", "TIFF", "DNG + JPEG", "DNG + PNG", "DNG + TIFF"];
    const timelapseStillFormatOptions = ["DNG", "JPEG", "PNG", "TIFF"];
    const stillTimelapseIntervalOptions = ["1s", "2s", "3s", "5s", "10s", "15s", "30s", "60s", "2m", "3m", "4m", "5m", "6m", "7m", "8m", "9m", "10m"];
    const stillTimelapseOptions = ["Off", ...stillTimelapseIntervalOptions];
    const externalMonitorModeOptions = ["Clean Feed", "Assist Feed"];
    const externalMonitorInfoOverlayOptions = ["On", "Off"];
    const externalMonitorOrientationOptions = ["Landscape", "Upside Down"];
    const focusPeakingThresholdOptions = ["Low", "Medium", "High"];
    const focusPeakingColorOptions = ["Red", "Green", "Blue", "Yellow", "Pink"];
    const zebraLevelOptions = ["70%", "75%", "80%", "85%", "90%", "95%", "100%"];
    const falseColorModeOptions = ["Exposure Based", "Skin Tone", "Highlight Priority", "Shadow Priority"];
    const guidesTypeOptions = ["Thirds", "16:9", "2.39:1", "4:3", "1:1", "Academy 1.37:1", "5:4", "9:16", "14:9"];
    const guidesThicknessOptions = ["1", "2", "3", "4"];
    const centerMarkerTypeOptions = ["Circle/Dot", "Crosshair"];
    const anamorphicRatioOptions = ["1.33x", "1.5x", "1.8x", "2.0x"];
    const controlsOpacityOptions = ["100%", "85%", "70%", "55%", "40%", "25%", "15%"];
    const controlsModeOptions = ["Light", "Dark"];
    const batteryTypeOptions = ["V-Mount", "NP-F", "LP-E6", "USB-C / DC", "Custom"];
    const sleepModeOptions = ["Off", "1 min", "2 min", "5 min", "10 min"];
    const favoriteCandidateOrder = [
      "externalMonitor",
      "lutPreview",
      "focusPeaking",
      "zebra",
      "falseColor",
      "smpteBars",
      "guides",
      "centerMarker",
      "greyscale",
      "anamorphicDesqueeze",
      "stillMode",
      "soundRecorder",
      "format",
      "gyroData",
      "autofocus",
      "timecode",
      "audioOptions",
      "levels",
      "uiOrientation",
      "cameraControlsOpacity",
      "dateTime",
      "battery",
      "batteryInfo",
      "autoPowerOff",
      "thermal",
      "powerActions",
      "media",
      "mediaSpeed",
      "wifiToggle",
      "currentNetwork",
      "availableNetworks",
      "camera",
      "sbc",
      "systemInfo",
      "systemDrive",
    ];
    const favoriteItemMeta = {
      externalMonitor: { title: "External Monitor", section: "Monitoring" },
      lutPreview: { title: "LUT Preview", section: "Monitoring" },
      focusPeaking: { title: "Focus Peaking", section: "Monitoring" },
      zebra: { title: "Zebra", section: "Monitoring" },
      falseColor: { title: "False Color", section: "Monitoring" },
      smpteBars: { title: "SMPTE Bars", section: "Monitoring" },
      guides: { title: "Guides", section: "Monitoring" },
      centerMarker: { title: "Center Marker", section: "Monitoring" },
      greyscale: { title: "Greyscale", section: "Monitoring" },
      anamorphicDesqueeze: { title: "Anamorphic Desqueeze", section: "Monitoring" },
      stillMode: { title: "Still Mode", section: "Record" },
      soundRecorder: { title: "Sound Recorder", section: "Record" },
      format: { title: "Format", section: "Record" },
      gyroData: { title: "Gyro Data", section: "Record" },
      autofocus: { title: "Autofocus", section: "Record" },
      timecode: { title: "Timecode", section: "Record" },
      audioOptions: { title: "Audio", section: "Audio" },
      levels: { title: "Levels", section: "Audio" },
      uiOrientation: { title: "UI Orientation", section: "System" },
      cameraControlsOpacity: { title: "Controls Transparency", section: "System" },
      dateTime: { title: "Date & Time", section: "System" },
      battery: { title: "Battery", section: "System" },
      batteryInfo: { title: "Battery Info", section: "System" },
      autoPowerOff: { title: "Auto Power Off", section: "System" },
      thermal: { title: "Thermal", section: "System" },
      powerActions: { title: "Power Actions", section: "System" },
      media: { title: "Media", section: "Media" },
      mediaSpeed: { title: "Media Speed", section: "Media" },
      wifiToggle: { title: "Wi-Fi", section: "Wi-Fi" },
      currentNetwork: { title: "Current Network", section: "Wi-Fi" },
      availableNetworks: { title: "Available Networks", section: "Wi-Fi" },
      camera: { title: "Camera", section: "Info" },
      sbc: { title: "SBC", section: "Info" },
      systemInfo: { title: "System", section: "Info" },
      systemDrive: { title: "System Drive", section: "Info" },
    };
    const settingsPanelMeta = {
      record: {
        title: "Record",
        copy: "Still mode, recording format, timecode, and capture setup.",
      },
      favorites: {
        title: "Favorites",
        copy: "Manage the same quick access favorites list used by the on-device UI.",
      },
      monitoring: {
        title: "Monitoring",
        copy: "Configure live preview routing, monitoring tools, and overlays.",
      },
      audio: {
        title: "Audio",
        copy: "Audio capture, monitoring, device routing, and level control.",
      },
      system: {
        title: "System",
        copy: "UI orientation, controls transparency, date and time, battery, thermal, and power actions.",
      },
      power: {
        title: "Media",
        copy: "Mounted recording media, eject, format, and write speed tools.",
      },
      presets: {
        title: "Presets",
        copy: "Download the same preset files saved by the native camera UI.",
      },
      config: {
        title: "Config",
        copy: "Edit the Apertar hardware and display config file.",
      },
      info: {
        title: "Info",
        copy: "Capture summary and Raspberry Pi telemetry.",
      },
    };

    const state = {
      snapshot: null,
      fetchInFlight: false,
      fetchQueued: false,
      fetchQueuedSilent: true,
      fetchRefreshTimer: null,
      deferFetchUntilMs: 0,
      commandInFlightCount: 0,
      commandEpoch: 0,
      recordingBasePerfMs: 0,
      recordingBaseElapsedSeconds: 0,
      timecodeBasePerfMs: 0,
      timecodeBaseLocalMsOfDay: 0,
      liveReadoutTimerId: 0,
      wbCommitTimer: null,
      inputVolumeCommitTimer: null,
      headphoneVolumeCommitTimer: null,
      toastTimer: null,
      previewRetryTimer: null,
      photoDelayTimer: null,
      photoCountdownTimer: null,
      photoCapturePulseTimer: null,
      previewStreamNonce: 0,
      previewStreamStarted: false,
      settingsOpen: false,
      audioCaptureModes: [],
      audioCapabilityDevice: "",
      audioCapabilityError: "",
      audioCapabilitiesBusy: false,
      mediaBrowserOpen: false,
      activeSettingsPanel: "favorites",
      favoriteControlsSignature: "",
      presetsFetchInFlight: false,
      presetLibrary: [],
      configFetchInFlight: false,
      configSaveInFlight: false,
      configDirty: false,
      mediaFetchInFlight: false,
      mediaLibrary: [],
      mediaLibraryPayload: null,
      mediaBrowserFilter: "clip",
      mediaBrowserSort: "newest",
      mediaSelection: new Set(),
      photoCaptureInProgress: false,
      photoCountdownSecondsRemaining: 0,
      pendingPhotoShots: 0,
      photoCapturePulse: false,
      gyroCalibrationRunning: false,
      gyroCalibrationStatus: "",
      gyroCalibrationOk: false,
      previewProcessingBackend: "",
      previewProcessingGl: null,
      previewProcessingProgram: null,
      previewProcessingTexture: null,
      previewProcessingVertexBuffer: null,
      previewProcessingUniforms: null,
      previewProcessingPositionLoc: -1,
      previewProcessingUvLoc: -1,
      previewProcessingContext: null,
      previewProcessingCanvasWidth: 0,
      previewProcessingCanvasHeight: 0,
      previewProcessingLastFrameMs: 0,
    };

    function clamp(value, min, max) {
      return Math.max(min, Math.min(max, value));
    }

    function cloneSnapshot(snapshot) {
      return snapshot ? JSON.parse(JSON.stringify(snapshot)) : null;
    }

    function ensureSnapshotShape(snapshot) {
      if (!snapshot || typeof snapshot !== "object")
        return null;
      if (!snapshot.ui || typeof snapshot.ui !== "object")
        snapshot.ui = {};
      if (!snapshot.system || typeof snapshot.system !== "object")
        snapshot.system = {};
      if (!snapshot.capture || typeof snapshot.capture !== "object")
        snapshot.capture = {};
      if (!snapshot.audio || typeof snapshot.audio !== "object")
        snapshot.audio = {};
      if (!snapshot.device || typeof snapshot.device !== "object")
        snapshot.device = {};
      if (!snapshot.imu || typeof snapshot.imu !== "object")
        snapshot.imu = {};
      return snapshot;
    }

    function isFiniteNumber(value) {
      return typeof value === "number" && Number.isFinite(value);
    }

    function asToggleBool(value) {
      return value === true || value === "true" || value === "1" || value === 1;
    }

    function imuDriverLabel(value) {
      const normalized = String(value || "").toLowerCase().replace(/[^a-z0-9]/g, "");
      if (normalized === "bmi160") return "BMI160";
      if (normalized === "bmi270") return "BMI270";
      if (normalized === "icm42688p") return "ICM42688-P";
      return "MPU6050";
    }

    function imuDriverValue(label) {
      const normalized = String(label || "").toLowerCase().replace(/[^a-z0-9]/g, "");
      if (normalized === "bmi160") return "bmi160";
      if (normalized === "bmi270") return "bmi270";
      if (normalized === "icm42688p") return "icm42688p";
      return "mpu6050";
    }

    function formatSignedNumber(value, digits) {
      const number = Number(value);
      if (!Number.isFinite(number))
        return "+0.00";
      const fixed = number.toFixed(digits);
      return number >= 0 ? ("+" + fixed) : fixed;
    }

    function formatImuVector(x, y, z, unit, digits) {
      return formatSignedNumber(x, digits) + " / "
        + formatSignedNumber(y, digits) + " / "
        + formatSignedNumber(z, digits) + " " + unit;
    }

    function formatSyncOffsetText(value) {
      const number = Number(value);
      if (!Number.isFinite(number) || Math.abs(number) < 0.05)
        return "0 ms";
      const rounded = Math.round(number);
      const text = Math.abs(number - rounded) < 0.05 ? String(rounded) : number.toFixed(1);
      return (number > 0 ? "+" : "") + text + " ms";
    }

    function formatImuConfigPath(path) {
      const value = String(path || "");
      if (value.startsWith("/home/pi/"))
        return "~/" + value.slice("/home/pi/".length);
      return value || "/home/pi/apertar-hardware.conf";
    }

    function normalizeImuOrientation(value) {
      const text = String(value || "XYZ").replace(/\s/g, "");
      if (text.length !== 3)
        return "XYZ";

      const seen = {};
      for (const ch of text) {
        const axis = ch.toUpperCase();
        if (axis !== "X" && axis !== "Y" && axis !== "Z")
          return "XYZ";
        if (seen[axis])
          return "XYZ";
        seen[axis] = true;
      }
      return text;
    }

    function imuSourceAxisValue(imuState, axis, kind) {
      const upper = String(axis || "X").toUpperCase();
      const prefix = kind === "gyro" ? "gyro_" : "accel_";
      const suffix = kind === "gyro" ? "_dps" : "_g";
      const key = prefix + upper.toLowerCase() + suffix;
      const value = Number(imuState && imuState[key]);
      return Number.isFinite(value) ? value : 0;
    }

    function mappedImuAxisValue(imuState, index, kind) {
      const orientation = normalizeImuOrientation(imuState && imuState.orientation);
      const token = orientation.charAt(index);
      const axis = token.toUpperCase();
      const sign = token === axis ? 1 : -1;
      return sign * imuSourceAxisValue(imuState, axis, kind);
    }

    function displayImuAccelValue(imuState, index) {
      const mapped = mappedImuAxisValue(imuState, index, "accel");
      if (!imuState || !imuState.level_reference_set)
        return mapped;
      const key = index === 0 ? "level_accel_x_g" : (index === 1 ? "level_accel_y_g" : "level_accel_z_g");
      const level = Number(imuState[key]);
      return mapped - (Number.isFinite(level) ? level : 0);
    }

    function imuAxisColor(axis) {
      if (axis === "X") return "#ff4b4b";
      if (axis === "Y") return "#42d879";
      return "#4d9fff";
    }

    function imuAxisVector(axis, positive) {
      if (axis === "X")
        return { x: positive ? 1 : -1, y: 0 };
      if (axis === "Y")
        return { x: 0, y: positive ? 1 : -1 };
      return { x: positive ? 0.72 : -0.72, y: positive ? -0.72 : 0.72 };
    }

    function drawImuArrow(ctx, centerX, centerY, baseVec, axis, signedValue, radius, lineWidth, labelSize, liveAvailable) {
      const magnitude = Math.max(0, Math.min(1.25, Math.abs(Number(signedValue) || 0)));
      const sign = signedValue >= 0 ? 1 : -1;
      const minLength = liveAvailable ? radius * 0.16 : radius * 0.7;
      const length = liveAvailable ? Math.max(minLength, radius * Math.min(1, magnitude)) : radius * 0.78;
      const vecX = baseVec.x * sign;
      const vecY = baseVec.y * sign;
      const endX = centerX + vecX * length;
      const endY = centerY + vecY * length;
      const angle = Math.atan2(endY - centerY, endX - centerX);
      const head = Math.max(7, labelSize * 0.55);
      const color = imuAxisColor(axis);

      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = lineWidth;
      ctx.globalAlpha = liveAvailable ? (0.34 + Math.min(0.62, magnitude * 0.56)) : 0.38;
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(endX, endY);
      ctx.stroke();

      ctx.globalAlpha = liveAvailable ? 0.94 : 0.48;
      ctx.beginPath();
      ctx.moveTo(endX, endY);
      ctx.lineTo(endX - Math.cos(angle - 0.52) * head, endY - Math.sin(angle - 0.52) * head);
      ctx.lineTo(endX - Math.cos(angle + 0.52) * head, endY - Math.sin(angle + 0.52) * head);
      ctx.closePath();
      ctx.fill();

      ctx.globalAlpha = 1.0;
      ctx.font = "800 " + labelSize + "px Inter, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(axis, endX + vecX * (labelSize * 0.82), endY + vecY * (labelSize * 0.82));
    }

    function drawImuAxisVisual(imuState) {
      const canvas = document.getElementById("settingsImuAxisCanvas");
      if (!canvas)
        return;

      const rect = canvas.getBoundingClientRect();
      const cssWidth = Math.max(1, rect.width || 112);
      const cssHeight = Math.max(1, rect.height || 112);
      const dpr = Math.max(1, window.devicePixelRatio || 1);
      const pixelWidth = Math.round(cssWidth * dpr);
      const pixelHeight = Math.round(cssHeight * dpr);
      if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
        canvas.width = pixelWidth;
        canvas.height = pixelHeight;
      }

      const ctx = canvas.getContext("2d");
      if (!ctx)
        return;

      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, cssWidth, cssHeight);
      ctx.lineCap = "round";
      ctx.lineJoin = "round";

      const centerX = cssWidth * 0.5;
      const centerY = cssHeight * 0.54;
      const radius = Math.min(cssWidth, cssHeight) * 0.30;
      const liveAvailable = !!(imuState && imuState.live_available);
      const orientation = normalizeImuOrientation(imuState && imuState.orientation);
      const lineWidth = Math.max(3.5, radius * 0.09);
      const labelSize = Math.max(14, radius * 0.43);

      ctx.strokeStyle = "rgba(255,255,255,0.13)";
      ctx.lineWidth = 1;
      ctx.globalAlpha = 1;
      ctx.beginPath();
      ctx.arc(centerX, centerY, radius * 1.04, 0, Math.PI * 2);
      ctx.stroke();

      for (let i = 0; i < orientation.length; ++i) {
        const token = orientation.charAt(i);
        const axis = token.toUpperCase();
        const positive = token === axis;
        const baseVec = imuAxisVector(axis, positive);
        drawImuArrow(ctx, centerX, centerY, baseVec, axis, displayImuAccelValue(imuState, i), radius, lineWidth, labelSize, liveAvailable);
      }

      if (liveAvailable) {
        const dotLimit = radius * 0.82;
        const dotX = centerX + Math.max(-1, Math.min(1, displayImuAccelValue(imuState, 0))) * dotLimit;
        const dotY = centerY + Math.max(-1, Math.min(1, displayImuAccelValue(imuState, 1))) * dotLimit;
        ctx.strokeStyle = "rgba(255,255,255,0.72)";
        ctx.fillStyle = "rgba(255,255,255,0.97)";
        ctx.lineWidth = 1.4;
        ctx.beginPath();
        ctx.arc(dotX, dotY, Math.max(4.4, radius * 0.13), 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      } else {
        ctx.fillStyle = "rgba(255,255,255,0.42)";
        ctx.font = "800 " + Math.max(10, labelSize * 0.62) + "px Inter, sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("WAIT", centerX, centerY);
      }

      ctx.fillStyle = "rgba(255,255,255,0.95)";
      ctx.globalAlpha = 1.0;
      ctx.beginPath();
      ctx.arc(centerX, centerY, Math.max(3.4, radius * 0.10), 0, Math.PI * 2);
      ctx.fill();
    }

    function zebraLevelToThreshold(level) {
      if (level === "70%") return 0.70;
      if (level === "75%") return 0.75;
      if (level === "80%") return 0.80;
      if (level === "85%") return 0.85;
      if (level === "90%") return 0.90;
      if (level === "95%") return 0.95;
      if (level === "100%") return 0.99;
      return 0.70;
    }

    function thresholdToZebraLevel(threshold) {
      const value = Number(threshold);
      if (Math.abs(value - 0.70) < 0.001) return "70%";
      if (Math.abs(value - 0.75) < 0.001) return "75%";
      if (Math.abs(value - 0.80) < 0.001) return "80%";
      if (Math.abs(value - 0.85) < 0.001) return "85%";
      if (Math.abs(value - 0.90) < 0.001) return "90%";
      if (Math.abs(value - 0.95) < 0.001) return "95%";
      if (Math.abs(value - 0.99) < 0.001) return "100%";
      return "70%";
    }

    function focusThresholdToValue(label) {
      if (label === "Low") return 0.05;
      if (label === "Medium") return 0.08;
      if (label === "High") return 0.12;
      return 0.08;
    }

    function valueToFocusThreshold(value) {
      const numeric = Number(value);
      if (Math.abs(numeric - 0.05) < 0.01) return "Low";
      if (Math.abs(numeric - 0.08) < 0.01) return "Medium";
      if (Math.abs(numeric - 0.12) < 0.01) return "High";
      return "Medium";
    }

    function falseColorModeToInt(mode) {
      if (mode === "Exposure Based") return 0;
      if (mode === "Skin Tone") return 1;
      if (mode === "Highlight Priority") return 2;
      if (mode === "Shadow Priority") return 3;
      return 0;
    }

    function intToFalseColorMode(mode) {
      const numeric = Number(mode);
      if (numeric === 1) return "Skin Tone";
      if (numeric === 2) return "Highlight Priority";
      if (numeric === 3) return "Shadow Priority";
      return "Exposure Based";
    }

    function opacityLabelToValue(label) {
      const parsed = parseInt(String(label || "").replace("%", ""), 10);
      if (!Number.isFinite(parsed))
        return 1.0;
      return clamp(parsed / 100.0, 0.15, 1.0);
    }

    function opacityValueToLabel(value) {
      const clamped = clamp(Number(value) || 1.0, 0.15, 1.0);
      let bestLabel = controlsOpacityOptions[0];
      let bestDelta = Math.abs(opacityLabelToValue(bestLabel) - clamped);
      for (const candidate of controlsOpacityOptions.slice(1)) {
        const delta = Math.abs(opacityLabelToValue(candidate) - clamped);
        if (delta < bestDelta) {
          bestLabel = candidate;
          bestDelta = delta;
        }
      }
      return bestLabel;
    }

    function falseColorLegendEntries(mode) {
      if (mode === "Skin Tone") {
        return [
          { label: "75", color: "#ff3b30" },
          { label: "65", color: "#ffe24a" },
          { label: "45", color: "#34d15f" },
          { label: "30", color: "#36a8ff" },
          { label: "0", color: "#2350ff" },
        ];
      }

      if (mode === "Highlight Priority") {
        return [
          { label: "98", color: "#ff3b30" },
          { label: "92", color: "#ff6a30" },
          { label: "85", color: "#ffb030" },
          { label: "70", color: "#ffe24a" },
          { label: "0", color: "#4a4a4a" },
        ];
      }

      if (mode === "Shadow Priority") {
        return [
          { label: "20", color: "#36f3ff" },
          { label: "10", color: "#36a8ff" },
          { label: "5", color: "#2350ff" },
          { label: "2", color: "#8b2dff" },
          { label: "0", color: "#232323" },
        ];
      }

      return [
        { label: "95", color: "#ff3b30" },
        { label: "85", color: "#ff8a1f" },
        { label: "70", color: "#ffe24a" },
        { label: "55", color: "#96f59a" },
        { label: "45", color: "#5a5a5a" },
        { label: "35", color: "#34d15f" },
        { label: "20", color: "#36a8ff" },
        { label: "10", color: "#2350ff" },
        { label: "0", color: "#b13dff" },
      ];
    }

    function pad2(value) {
      const number = Math.max(0, Number(value) || 0);
      return number < 10 ? ("0" + number) : String(number);
    }

    function formatWebDateTimePill(value) {
      const monthNames = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
      const match = /^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2})/.exec(String(value || ""));
      if (match) {
        const monthIndex = Math.max(0, Math.min(11, Number(match[2]) - 1));
        return pad2(match[3]) + " " + monthNames[monthIndex] + " " + match[1] + " " + match[4] + ":" + match[5];
      }

      const now = new Date();
      return pad2(now.getDate())
        + " "
        + monthNames[now.getMonth()]
        + " "
        + now.getFullYear()
        + " "
        + pad2(now.getHours())
        + ":"
        + pad2(now.getMinutes());
    }

    function currentFpsInt(snapshot) {
      const parsed = Number(snapshot && snapshot.fps);
      if (!isFinite(parsed) || parsed <= 0)
        return 24;
      return Math.round(parsed);
    }

    function formatTime(seconds, fps) {
      const safeFps = Math.max(1, Number(fps) || 24);
      const totalFrames = Math.floor(Math.max(0, seconds) * safeFps);
      const frames = String(totalFrames % safeFps).padStart(2, "0");
      const totalSeconds = Math.floor(Math.max(0, seconds));
      const h = String(Math.floor(totalSeconds / 3600)).padStart(2, "0");
      const m = String(Math.floor((totalSeconds % 3600) / 60)).padStart(2, "0");
      const s = String(totalSeconds % 60).padStart(2, "0");
      return h + ":" + m + ":" + s + ":" + frames;
    }

    function formatTimecode(totalFrames, fpsInt) {
      const safeFps = Math.max(1, Number(fpsInt) || 24);
      const wholeFrames = Math.max(0, Math.floor(Number(totalFrames) || 0));
      const hours = Math.floor(wholeFrames / (safeFps * 3600));
      let remainder = wholeFrames % (safeFps * 3600);
      const minutes = Math.floor(remainder / (safeFps * 60));
      remainder %= (safeFps * 60);
      const seconds = Math.floor(remainder / safeFps);
      const frames = remainder % safeFps;
      return pad2(hours) + ":" + pad2(minutes) + ":" + pad2(seconds) + ":" + pad2(frames);
    }

    function formatAngle(us, fps) {
      const safeFps = Math.max(0.001, Number(fps) || 24);
      const frameDurationUs = 1000000 / safeFps;
      const angle = (Number(us) / frameDurationUs) * 360;
      let best = shutterOptions[0];
      let bestDiff = Infinity;
      for (const option of shutterOptions) {
        const value = parseFloat(option);
        const diff = Math.abs(value - angle);
        if (diff < bestDiff) {
          best = option;
          bestDiff = diff;
        }
      }
      return best;
    }

    function shutterAngleToUs(label, fps) {
      const safeFps = Math.max(0.001, Number(fps) || 24);
      const angle = parseFloat(String(label).replace("°", ""));
      return Math.round((angle / 360) * (1000000 / safeFps));
    }

    function shutterSpeedToUs(label) {
      const text = String(label || "").trim();
      if (text === "Auto")
        return 0;
      if (/^\d+(?:\.\d+)?s$/.test(text)) {
        const seconds = Number(text.slice(0, -1));
        return seconds > 0 && seconds <= 30 ? Math.round(seconds * 1000000) : 0;
      }
      const slashIndex = text.indexOf("/");
      if (slashIndex === -1)
        return 0;
      const denominator = Number(text.slice(slashIndex + 1));
      if (!isFinite(denominator) || denominator <= 0)
        return 0;
      return Math.round(1000000 / denominator);
    }

    function formatShutterSpeed(us) {
      const shutterUs = Number(us) || 0;
      let best = shutterSpeedOptions[0];
      let bestDiff = Infinity;
      for (const option of longShutterSpeedOptions) {
        if (option === "Auto")
          continue;
        const candidateUs = shutterSpeedToUs(option);
        const diff = Math.abs(candidateUs - shutterUs);
        if (diff < bestDiff) {
          best = option;
          bestDiff = diff;
        }
      }
      return best;
    }

    function availableStillShutterOptions(uiState) {
      const options = uiState.photo_long_exposure_enabled ? longShutterSpeedOptions : shutterSpeedOptions;
      const match = /^(\d+(?:\.\d+)?)(s|m)$/.exec(String(uiState.photo_timelapse || "Off"));
      if (!match) return options;
      const maximumUs = Math.round(Number(match[1]) * (match[2] === "m" ? 60 : 1) * 1000000);
      return options.filter((value) => value === "Auto" || shutterSpeedToUs(value) <= maximumUs);
    }

    function photoTimerDelayMs(timerValue) {
      if (timerValue === "2s")
        return 2000;
      if (timerValue === "5s")
        return 5000;
      if (timerValue === "10s")
        return 10000;
      if (timerValue === "15s")
        return 15000;
      if (timerValue === "20s")
        return 20000;
      if (timerValue === "25s")
        return 25000;
      if (timerValue === "30s")
        return 30000;
      return 0;
    }

    function photoBurstCount(burstValue) {
      if (burstValue === "3 Shots")
        return 3;
      if (burstValue === "5 Shots")
        return 5;
      if (burstValue === "10 Shots")
        return 10;
      return 1;
    }

    function stillTimelapseEnabled(value) {
      return !!value && value !== "Off";
    }

    function timelapseIntervalLabelFromSeconds(seconds) {
      const value = Math.max(0, Math.round((Number(seconds) || 0) * 10) / 10);
      if (value <= 0)
        return "5s";
      if (value >= 120 && value % 60 === 0)
        return String(value / 60) + "m";
      return String(value) + "s";
    }

    function defaultStillTimelapseInterval(snapshot, currentValue) {
      if (stillTimelapseEnabled(currentValue))
        return currentValue;
      return timelapseIntervalLabelFromSeconds(snapshot && snapshot.timelapse_interval_seconds);
    }

    function stillFormatOptionsForTimelapse(value) {
      return stillTimelapseEnabled(value) ? timelapseStillFormatOptions : stillFormatOptions;
    }

    function safeTimelapseStillFormat(value) {
      return timelapseStillFormatOptions.includes(value) ? value : "DNG";
    }

    function sensorNameLower(snapshot) {
      const sensor = snapshot && snapshot.capture ? snapshot.capture.sensor : "";
      return String(sensor || "").toLowerCase();
    }

    function piModelLower(snapshot) {
      const model = snapshot && snapshot.device ? snapshot.device.pi_model : "";
      return String(model || "").toLowerCase();
    }

    function fanModeSupportedForSnapshot(snapshot) {
      return piModelLower(snapshot).indexOf("raspberry pi 4") === -1;
    }

    function imx477MaximumSupportedFps(width, height, bitDepth) {
      const key = String(width) + "x" + String(height);
      const bits = rawBitDepthNumber(bitDepth);
      const limits = bits === 8 ? {
        "1332x990": 282.17,
        "2028x796": 242.48, "2028x812": 237.98, "2028x848": 228.41, "2028x864": 224.42,
        "2028x914": 212.72, "2028x922": 210.97, "2028x1014": 192.72, "2028x1068": 183.39,
        "2028x1080": 181.42, "2028x1096": 178.89, "2028x1140": 172.27, "2028x1520": 130.51,
        "4056x1592": 66.44, "4056x1624": 65.16, "4056x1696": 62.47, "4056x1728": 61.35,
        "4056x1828": 58.08, "4056x1844": 57.59, "4056x2028": 52.48,
        "4056x2136": 49.89, "4056x2160": 49.34, "4056x2192": 48.64, "4056x2280": 46.80,
        "4056x3040": 35.28
      } : (bits === 10 ? {
        "1332x990": 233.92,
        "2028x796": 198.85, "2028x812": 195.12, "2028x848": 187.30, "2028x864": 183.99,
        "2028x914": 174.43, "2028x922": 173.01, "2028x1014": 158.00, "2028x1068": 150.38,
        "2028x1080": 148.77, "2028x1096": 146.69, "2028x1140": 141.26, "2028x1520": 107.02,
        "4056x1592": 53.84, "4056x1624": 52.81, "4056x1696": 50.63,
        "4056x1728": 50.00, "4056x1828": 47.07, "4056x1844": 46.67, "4056x2028": 42.54,
        "4056x2136": 40.43, "4056x2160": 40.00, "4056x2192": 39.42, "4056x2280": 37.93,
        "4056x3040": 28.60
      } : {
        "1332x990": 136.43,
        "2028x796": 167.79, "2028x812": 164.66, "2028x848": 158.05, "2028x864": 155.28,
        "2028x914": 147.21, "2028x922": 145.99, "2028x1014": 133.35, "2028x1068": 126.89,
        "2028x1080": 125.55, "2028x1096": 123.79, "2028x1140": 119.20, "2028x1520": 90.31,
        "4056x1592": 45.26, "4056x1624": 44.40, "4056x1696": 42.56, "4056x1728": 41.80,
        "4056x1828": 39.57, "4056x1844": 39.24, "4056x2028": 35.76, "4056x2136": 33.99,
        "4056x2160": 33.62, "4056x2192": 33.14, "4056x2280": 31.89, "4056x3040": 24.04
      });
      return limits[key] !== undefined ? limits[key] : 120.0;
    }

    function imx477MaximumUncompressedFps(width, height, bitDepth) {
      const key = String(width) + "x" + String(height);
      const bits = rawBitDepthNumber(bitDepth);
      const limits = bits === 8 ? {
        "2028x796": 140.0, "2028x812": 140.0, "2028x848": 140.0, "2028x864": 120.0,
        "2028x914": 120.0, "2028x922": 120.0, "2028x1014": 100.0, "2028x1068": 100.0,
        "2028x1080": 100.0, "2028x1096": 90.0, "2028x1140": 90.0, "2028x1520": 90.0,
        "4056x1592": 30.0, "4056x1624": 30.0, "4056x1696": 30.0, "4056x1728": 30.0,
        "4056x1828": 30.0, "4056x1844": 30.0, "4056x2028": 30.0, "4056x2136": 30.0,
        "4056x2160": 30.0, "4056x2192": 30.0, "4056x2280": 30.0, "4056x3040": 25.0
      } : (bits === 10 ? {
        "2028x796": 120.0, "2028x812": 120.0, "2028x848": 120.0, "2028x864": 120.0,
        "2028x914": 130.0, "2028x922": 130.0, "2028x1014": 100.0, "2028x1068": 100.0,
        "2028x1080": 100.0, "2028x1096": 90.0, "2028x1140": 90.0, "2028x1520": 90.0,
        "4056x1592": 30.0, "4056x1624": 30.0, "4056x1696": 30.0, "4056x1728": 30.0,
        "4056x1828": 30.0, "4056x1844": 30.0, "4056x2028": 30.0, "4056x2136": 30.0,
        "4056x2160": 30.0, "4056x2192": 30.0, "4056x2280": 30.0, "4056x3040": 24.0
      } : {
        "2028x796": 120.0, "2028x812": 120.0, "2028x848": 120.0, "2028x864": 120.0,
        "2028x914": 120.0, "2028x922": 120.0, "2028x1014": 100.0, "2028x1068": 100.0,
        "2028x1080": 100.0, "2028x1096": 90.0, "2028x1140": 60.0, "2028x1520": 60.0,
        "4056x1592": 30.0, "4056x1624": 30.0, "4056x1696": 30.0, "4056x1728": 30.0,
        "4056x1828": 30.0, "4056x1844": 30.0, "4056x2028": 30.0, "4056x2136": 30.0,
        "4056x2160": 30.0, "4056x2192": 30.0, "4056x2280": 30.0, "4056x3040": 24.0
      });
      return limits[key] !== undefined ? limits[key] : 1000.0;
    }

    function isFourKCaptureForSnapshot(snapshot) {
      const width = Number(snapshot && snapshot.width) || 0;
      const sensorLower = sensorNameLower(snapshot);
      if (sensorLower.indexOf("imx477") !== -1 || sensorLower.indexOf("imx577") !== -1)
        return width === 4056;
      if (sensorLower.indexOf("imx585") !== -1)
        return width === 3840 || width === 3856;
      if (sensorLower.indexOf("imx294") !== -1)
        return width >= 3392;
      return width >= 3840;
    }

    function isApproximatelyFourThreeSnapshot(snapshot) {
      const width = Number(snapshot && snapshot.width) || 0;
      const height = Number(snapshot && snapshot.height) || 0;
      if (width <= 0 || height <= 0)
        return false;
      const widthTerm = width * 3;
      const heightTerm = height * 4;
      return Math.abs(widthTerm - heightTerm) <= Math.max(8, Math.max(widthTerm, heightTerm) / 50);
    }

    function isLosslessFourKSnapshot(snapshot) {
      return !stillCaptureForSnapshot(snapshot) && sensorNameLower(snapshot).indexOf("imx294") === -1
        && recordingFormatLabel(snapshot && snapshot.recording_format) === "CinemaDNG"
        && String(snapshot && snapshot.cdng_compression || "none") === "lossless_jpeg"
        && isFourKCaptureForSnapshot(snapshot);
    }

    function isLosslessTwoKSnapshot(snapshot) {
      const sensorLower = sensorNameLower(snapshot);
      return !stillCaptureForSnapshot(snapshot) && recordingFormatLabel(snapshot && snapshot.recording_format) === "CinemaDNG"
        && String(snapshot && snapshot.cdng_compression || "none") === "lossless_jpeg"
        && (sensorLower.indexOf("imx477") !== -1 || sensorLower.indexOf("imx577") !== -1)
        && Number(snapshot && snapshot.width) === 2028;
    }

    function isLosslessImx585Snapshot(snapshot) {
      return !stillCaptureForSnapshot(snapshot) && sensorNameLower(snapshot).indexOf("imx585") !== -1
        && recordingFormatLabel(snapshot && snapshot.recording_format) === "CinemaDNG"
        && String(snapshot && snapshot.cdng_compression || "none") === "lossless_jpeg";
    }

    function maximumSupportedFpsForSnapshot(snapshot) {
      const width = Number(snapshot.width) || 0;
      const height = Number(snapshot.height) || 0;
      const sensorLower = sensorNameLower(snapshot);

      if (sensorLower.indexOf("imx577") !== -1) {
        if (width === 1332 && height === 990)
          return 101.68;
        if ((width === 2028 && height === 1080)
            || (width === 1928 && height === 1090) || (width === 1920 && height === 1080))
          return 62.81;
        if (width === 2028 && height === 1520)
          return 45.19;
        if ((width === 4056 && height === 2160)
            || (width === 3856 && height === 2180) || (width === 3840 && height === 2160))
          return 16.39;
        if (width === 4056 && height === 3040)
          return 11.72;
      }

      if (sensorLower.indexOf("imx477") !== -1) {
        const normalized = normalizedResolutionForSnapshot(
          snapshot,
          (snapshot && snapshot.resolution) || formatResolutionValue(width, height)
        );
        const parts = String(normalized || "").split("x");
        const normalizedWidth = parts.length === 2 ? Number(parts[0]) || width : width;
        const normalizedHeight = parts.length === 2 ? Number(parts[1]) || height : height;
        return imx477MaximumSupportedFps(normalizedWidth, normalizedHeight, snapshot && snapshot.raw_bit_depth);
      }

      if (sensorLower.indexOf("imx294") !== -1) {
        if ((width === 3792 && height === 2824) || (width === 3792 && height === 2528)
            || (width === 3392 && height === 2824))
          return 34.70;
        if (width === 4176 && height === 2176)
          return 40.95;
        if ((width === 4096 && height === 2048) || (width === 4096 && height === 1844)
            || (width === 4096 && height === 1744) || (width === 4096 && height === 1716)
            || (width === 4096 && height === 1640) || (width === 2048 && height === 1080)
            || (width === 1920 && height === 1080))
          return 40.95;
        if (width === 4144 && height === 2176)
          return 41.08;
        if (width === 3872 && height === 2176)
          return 44.17;
      }

      if (sensorLower.indexOf("imx585") !== -1) {
        if ((width === 3856 && height === 2180) || (width === 3840 && height === 2160))
          return 30.0;
        if (width === 3840 && (height === 2076 || height === 2024 || height === 1920 || height === 1744
            || height === 1728 || height === 1632 || height === 1608 || height === 1536 || height === 1504))
          return 60.0;
        if (width === 1920 && (height === 1038 || height === 1012 || height === 960 || height === 872
            || height === 864 || height === 816 || height === 804 || height === 768 || height === 752))
          return 60.0;
        if ((width === 1928 && height === 1090) || (width === 1920 && height === 1080))
          return 60.0;
      }

      return 60.0;
    }

    function availableFpsOptionsForSnapshot(snapshot) {
      const sensorLower = sensorNameLower(snapshot);
      const presets = (sensorLower.indexOf("imx477") !== -1 || sensorLower.indexOf("imx577") !== -1)
        ? fpsPresets
        : fpsPresets.filter((value) => value <= 60.0005);
      let max = maximumSupportedFpsForSnapshot(snapshot);
      const h264MaxFps = Number(snapshot && snapshot.recording_capabilities && snapshot.recording_capabilities.h264_max_fps) || 0;
      if (processedVideoRecordFormatForSnapshot(snapshot) && h264MaxFps > 0)
        max = Math.min(max, h264MaxFps);
      if (!stillCaptureForSnapshot(snapshot) && sensorLower.indexOf("imx477") !== -1
          && recordingFormatLabel(snapshot && snapshot.recording_format) === "CinemaDNG"
          && String(snapshot && snapshot.cdng_compression || "none") === "none") {
        const normalized = normalizedResolutionForSnapshot(
          snapshot,
          (snapshot && snapshot.resolution) || formatResolutionValue(snapshot.width, snapshot.height)
        );
        const parts = String(normalized || "").split("x");
        const width = parts.length === 2 ? Number(parts[0]) || Number(snapshot.width) : Number(snapshot.width);
        const height = parts.length === 2 ? Number(parts[1]) || Number(snapshot.height) : Number(snapshot.height);
        max = Math.min(max, imx477MaximumUncompressedFps(width, height, snapshot && snapshot.raw_bit_depth));
      }
      if (isLosslessImx585Snapshot(snapshot))
        max = Math.min(max, 30.0);
      else if (isLosslessFourKSnapshot(snapshot))
        max = Math.min(max, isApproximatelyFourThreeSnapshot(snapshot) ? 24.0 : 30.0);
      else if (isLosslessTwoKSnapshot(snapshot))
        max = Math.min(max, 60.0);
      const filtered = presets.filter((value) => value <= max + 0.0005);
      return (filtered.length > 0 ? filtered : [max]).map((value) => Number(value).toFixed(3));
    }

    function availableIsoOptionsForSnapshot(snapshot) {
      return isLosslessImx585Snapshot(snapshot) || isLosslessFourKSnapshot(snapshot) || isLosslessTwoKSnapshot(snapshot)
        ? ["100", "200", "400", "800"]
        : isoOptions;
    }

    function formatResolutionValue(width, height) {
      const numericWidth = Number(width) || 0;
      const numericHeight = Number(height) || 0;
      if (numericWidth <= 0 || numericHeight <= 0)
        return "";
      return String(numericWidth) + "x" + String(numericHeight);
    }

    function imx294ResolutionOptionsForSnapshot(snapshot) {
      return rawBitDepthNumber(snapshot && snapshot.raw_bit_depth) === 10
        ? ["4144x2176", "4096x2048", "4096x1844", "4096x1744", "4096x1716", "4096x1640", "3872x2176", "3792x2824", "3792x2528", "3392x2824"]
        : ["4176x2176", "4096x2048", "4096x1844", "4096x1744", "4096x1716", "4096x1640", "3872x2176", "3792x2824", "3792x2528", "3392x2824"];
    }

    function imx294BaseResolutionOptions(snapshot) {
      return [
        { value: imx294ResolutionOptionsForSnapshot(snapshot)[0], label: "4K" },
      ];
    }

    function imx294CropRatioLabel(resolution) {
      const labels = {
        "4144x2176": "17:9", "4176x2176": "17:9", "2048x1080": "17:9",
        "1920x1080": "16:9", "3872x2176": "16:9",
        "3792x2824": "4:3", "3792x2528": "3:2", "3392x2824": "6:5",
        "4096x2048": "2.00:1", "4096x1844": "2.22:1",
        "4096x1744": "2.35:1", "4096x1716": "2.39:1", "4096x1640": "2.50:1",
      };
      return labels[resolution] || "";
    }

    function imx294CropOptionsForBase(snapshot, baseValue, enabled = true) {
      const modes = imx294ResolutionOptionsForSnapshot(snapshot);
      const order = ["4:3", "3:2", "6:5", "16:9", "17:9", "2.00:1", "2.22:1", "2.35:1", "2.39:1", "2.50:1"];
      return (baseValue === modes[0] ? modes : []).map(value => ({ value, label: imx294CropRatioLabel(value), enabled }))
        .sort((a, b) => order.indexOf(a.label) - order.indexOf(b.label));
    }

    function imx477ResolutionOptions() {
      return ["1332x990", "2028x1080", "2028x1140", "2028x1096", "2028x1068", "2028x1014", "2028x922", "2028x914", "2028x864", "2028x848", "2028x812", "2028x796", "2028x1520", "4056x2160", "4056x2280", "4056x2192", "4056x2136", "4056x2028", "4056x1844", "4056x1828", "4056x1728", "4056x1696", "4056x1624", "4056x1592", "4056x3040"];
    }

    function imx477BaseResolutionOptions() {
      return [
        { value: "2028x1080", label: "2K" },
        { value: "4056x2160", label: "4K" },
      ];
    }

    function imx477CropOptionsForBase(baseValue, enabled = true) {
      if (baseValue === "4056x2160") {
        return [
          { value: "4056x3040", label: "4:3", enabled },
          { value: "4056x2280", label: "16:9", enabled },
          { value: "4056x2160", label: "17:9", enabled },
          { value: "4056x2192", label: "1.85:1", enabled },
          { value: "4056x2136", label: "1.90:1", enabled },
          { value: "4056x2028", label: "2.00:1", enabled },
          { value: "4056x1844", label: "2.20:1", enabled },
          { value: "4056x1828", label: "2.22:1", enabled },
          { value: "4056x1728", label: "2.35:1", enabled },
          { value: "4056x1696", label: "2.39:1", enabled },
          { value: "4056x1624", label: "2.50:1", enabled },
          { value: "4056x1592", label: "2.55:1", enabled },
        ];
      }

      return [
        { value: "2028x1520", label: "4:3", enabled },
        { value: "2028x1140", label: "16:9", enabled },
        { value: "2028x1080", label: "17:9", enabled },
        { value: "2028x1096", label: "1.85:1", enabled },
        { value: "2028x1068", label: "1.90:1", enabled },
        { value: "2028x1014", label: "2.00:1", enabled },
        { value: "2028x922", label: "2.20:1", enabled },
        { value: "2028x914", label: "2.22:1", enabled },
        { value: "2028x864", label: "2.35:1", enabled },
        { value: "2028x848", label: "2.39:1", enabled },
        { value: "2028x812", label: "2.50:1", enabled },
        { value: "2028x796", label: "2.55:1", enabled },
      ];
    }

    function sensorResolutionOptions(sensorLower, snapshot) {
      if (sensorLower.indexOf("imx577") !== -1)
        return ["1332x990", "2028x1080", "2028x1520", "4056x2160", "4056x3040"];
      if (sensorLower.indexOf("imx477") !== -1)
        return imx477ResolutionOptions();
      if (sensorLower.indexOf("imx294") !== -1)
        return imx294ResolutionOptionsForSnapshot(snapshot);
      if (sensorLower.indexOf("imx585") !== -1)
        return ["1920x1080", "1920x1038", "1920x1012", "1920x960", "1920x872", "1920x864", "1920x816", "1920x804", "1920x768", "1920x752", "3840x2160", "3840x2076", "3840x2024", "3840x1920", "3840x1744", "3840x1728", "3840x1632", "3840x1608", "3840x1536", "3840x1504"];
      return fallbackResolutionOptions;
    }

    function normalizedResolutionForSnapshot(snapshot, resolutionValue) {
      const sensorLower = sensorNameLower(snapshot);
      const value = String(resolutionValue || "").trim();
      if (processedVideoRecordFormatForSnapshot(snapshot))
        return forcedH264CaptureResolutionForSnapshot(snapshot);

      if (sensorLower.indexOf("imx577") !== -1) {
        if (value === "1332x990" || value === "2028x1080" || value === "2028x1520"
            || value === "4056x2160" || value === "4056x3040")
          return value;
        if (value === "1928x1090" || value === "1920x1080")
          return "2028x1080";
        if (value === "3856x2180" || value === "3840x2160")
          return "4056x2160";
        return "2028x1080";
      }

      if (sensorLower.indexOf("imx477") !== -1) {
        const options = imx477ResolutionOptions();
        if (options.indexOf(value) !== -1)
          return value;
        if (value === "1928x1090" || value === "1920x1080")
          return "2028x1080";
        if (value === "3856x2180" || value === "3840x2160")
          return "4056x2160";
        if (value === "4056x3040")
          return "4056x3040";
        return "2028x1080";
      }

      if (sensorLower.indexOf("imx294") !== -1) {
        const options = imx294ResolutionOptionsForSnapshot(snapshot);
        if (options.indexOf(value) !== -1)
          return value;
        if (value === "4056x3040")
          return "3792x2824";
        if (value === "2028x1080")
          return options[0];
        if (value === "1920x1080")
          return "3872x2176";
        if (value === "1928x1090" || value === "1920x1080"
            || value === "3856x2180" || value === "3840x2160"
            || value === "4056x2160")
          return options[0] || "4144x2176";
        return options[0] || "4144x2176";
      }

      if (sensorLower.indexOf("imx585") !== -1) {
        if (value === "1928x1090" || value === "1920x1080")
          return "1920x1080";
        if (value === "1920x1038" || value === "1920x1012" || value === "1920x960" || value === "1920x872"
            || value === "1920x864" || value === "1920x816" || value === "1920x804" || value === "1920x768"
            || value === "1920x752")
          return value;
        if (value === "3856x2180" || value === "3840x2160")
          return "3840x2160";
        if (value === "3840x2076" || value === "3840x2024" || value === "3840x1920" || value === "3840x1744"
            || value === "3840x1728" || value === "3840x1632" || value === "3840x1608" || value === "3840x1536"
            || value === "3840x1504")
          return value;
        if (value === "1332x990" || value === "2028x1080" || value === "2028x1520")
          return "1920x1080";
        if (value === "4056x2160" || value === "4056x3040")
          return "3840x2160";
        return "1920x1080";
      }

      return value || fallbackResolutionOptions[0];
    }

    function forcedH264CaptureResolutionForSnapshot(snapshot) {
      const sensorLower = sensorNameLower(snapshot);
      if (sensorLower.indexOf("imx577") !== -1)
        return "4056x2160";
      if (sensorLower.indexOf("imx477") !== -1)
        return "2028x1080";
      if (sensorLower.indexOf("imx294") !== -1)
        return "4144x2176";
      return "3840x2160";
    }

    function applyForcedH264CaptureResolution(snapshot) {
      if (!snapshot || !processedVideoRecordFormatForSnapshot(snapshot))
        return;

      const resolution = forcedH264CaptureResolutionForSnapshot(snapshot);
      const parts = resolution.split("x");
      snapshot.resolution = resolution;
      if (parts.length === 2) {
        snapshot.width = Math.max(0, Math.round(Number(parts[0]) || 0));
        snapshot.height = Math.max(0, Math.round(Number(parts[1]) || 0));
      }
    }

    function availableResolutionOptionsForSnapshot(snapshot) {
      const sensorLower = sensorNameLower(snapshot);
      const options = sensorResolutionOptions(sensorLower, snapshot);
      if (sensorLower.length > 0)
        return options;

      const current = normalizedResolutionForSnapshot(
        snapshot,
        (snapshot && snapshot.resolution) || formatResolutionValue(snapshot && snapshot.width, snapshot && snapshot.height)
      );
      if (!current || options.indexOf(current) !== -1)
        return options;
      return [current].concat(options.filter((value) => value !== current));
    }

    function resolutionLabelFromSnapshot(snapshot) {
      const explicit = snapshot && typeof snapshot.resolution === "string" ? snapshot.resolution : "";
      const fallback = formatResolutionValue(snapshot && snapshot.width, snapshot && snapshot.height);
      return normalizedResolutionForSnapshot(snapshot, explicit || fallback);
    }

    function imx585CropControlsAvailableForSnapshot(snapshot) {
      return sensorNameLower(snapshot).indexOf("imx585") !== -1;
    }

    function sensorCropControlsAvailableForSnapshot(snapshot) {
      const sensorLower = sensorNameLower(snapshot);
      return sensorLower.indexOf("imx585") !== -1 || sensorLower.indexOf("imx477") !== -1
        || sensorLower.indexOf("imx294") !== -1;
    }

    function imx585CropRatioOptionsForSnapshot(snapshot) {
      const enabled = imx585CropControlsAvailableForSnapshot(snapshot);
      if (!isImx585FourKResolution(resolutionLabelFromSnapshot(snapshot))) {
        return [
          { value: "1920x1080", label: "16:9", enabled },
          { value: "1920x1038", label: "1.85:1", enabled },
          { value: "1920x1012", label: "1.90:1", enabled },
          { value: "1920x960", label: "2.00:1", enabled },
          { value: "1920x872", label: "2.20:1", enabled },
          { value: "1920x864", label: "2.22:1", enabled },
          { value: "1920x816", label: "2.35:1", enabled },
          { value: "1920x804", label: "2.39:1", enabled },
          { value: "1920x768", label: "2.50:1", enabled },
          { value: "1920x752", label: "2.55:1", enabled },
        ];
      }

      return [
        { value: "3840x2160", label: "16:9", enabled },
        { value: "3840x2076", label: "1.85:1", enabled },
        { value: "3840x2024", label: "1.90:1", enabled },
        { value: "3840x1920", label: "2.00:1", enabled },
        { value: "3840x1744", label: "2.20:1", enabled },
        { value: "3840x1728", label: "2.22:1", enabled },
        { value: "3840x1632", label: "2.35:1", enabled },
        { value: "3840x1608", label: "2.39:1", enabled },
        { value: "3840x1536", label: "2.50:1", enabled },
        { value: "3840x1504", label: "2.55:1", enabled },
      ];
    }

    function imx585BaseResolutionOptions() {
      return [
        { value: "1920x1080", label: "Full HD" },
        { value: "3840x2160", label: "4K" },
      ];
    }

    function sensorBaseResolutionOptionsForSnapshot(snapshot) {
      if (sensorNameLower(snapshot).indexOf("imx294") !== -1)
        return imx294BaseResolutionOptions(snapshot);
      return sensorNameLower(snapshot).indexOf("imx477") !== -1
        ? imx477BaseResolutionOptions()
        : imx585BaseResolutionOptions();
    }

    function isImx585FourKResolution(resolutionValue) {
      return resolutionValue === "3840x2160"
        || resolutionValue === "3856x2180"
        || resolutionValue === "3840x2076"
        || resolutionValue === "3840x2024"
        || resolutionValue === "3840x1920"
        || resolutionValue === "3840x1744"
        || resolutionValue === "3840x1728"
        || resolutionValue === "3840x1632"
        || resolutionValue === "3840x1608"
        || resolutionValue === "3840x1536"
        || resolutionValue === "3840x1504";
    }

    function isImx585FullHdResolution(resolutionValue) {
      return resolutionValue === "1920x1080"
        || resolutionValue === "1928x1090"
        || resolutionValue === "1920x1038"
        || resolutionValue === "1920x1012"
        || resolutionValue === "1920x960"
        || resolutionValue === "1920x872"
        || resolutionValue === "1920x864"
        || resolutionValue === "1920x816"
        || resolutionValue === "1920x804"
        || resolutionValue === "1920x768"
        || resolutionValue === "1920x752";
    }

    function isImx477FourKResolution(resolutionValue) {
      return resolutionValue === "4056x3040"
        || resolutionValue === "4056x2280"
        || resolutionValue === "4056x2192"
        || resolutionValue === "4056x2160"
        || resolutionValue === "4056x2136"
        || resolutionValue === "4056x2028"
        || resolutionValue === "4056x1844"
        || resolutionValue === "4056x1828"
        || resolutionValue === "4056x1728"
        || resolutionValue === "4056x1696"
        || resolutionValue === "4056x1624"
        || resolutionValue === "4056x1592";
    }

    function sensorBaseResolutionValueFromSnapshot(snapshot) {
      const resolution = resolutionLabelFromSnapshot(snapshot);
      if (sensorNameLower(snapshot).indexOf("imx294") !== -1)
        return imx294ResolutionOptionsForSnapshot(snapshot)[0];
      if (sensorNameLower(snapshot).indexOf("imx477") !== -1)
        return isImx477FourKResolution(resolution) ? "4056x2160" : "2028x1080";
      if (sensorNameLower(snapshot).indexOf("imx585") !== -1)
        return isImx585FourKResolution(resolution) ? "3840x2160" : "1920x1080";
      return resolution;
    }

    function sensorCropRatioOptionsForSnapshot(snapshot) {
      const enabled = sensorCropControlsAvailableForSnapshot(snapshot);
      const baseValue = sensorBaseResolutionValueFromSnapshot(snapshot);
      if (sensorNameLower(snapshot).indexOf("imx294") !== -1)
        return imx294CropOptionsForBase(snapshot, baseValue, enabled);
      if (sensorNameLower(snapshot).indexOf("imx477") !== -1)
        return imx477CropOptionsForBase(baseValue, enabled);
      return imx585CropRatioOptionsForSnapshot(snapshot);
    }

    function sensorResolutionForBaseAndRatio(snapshot, baseValue, ratioLabelOrValue) {
      const selected = String(ratioLabelOrValue || "");
      const sensorLower = sensorNameLower(snapshot);
      if (sensorLower.indexOf("imx294") !== -1) {
        const option = imx294CropOptionsForBase(snapshot, baseValue).find(
          item => item.value === selected || item.label === selected);
        return option ? option.value : baseValue;
      }
      const options = sensorLower.indexOf("imx477") !== -1
        ? imx477CropOptionsForBase(baseValue, true)
        : (baseValue === "3840x2160"
           ? [
             { value: "3840x2160", label: "16:9" },
             { value: "3840x2076", label: "1.85:1" },
             { value: "3840x2024", label: "1.90:1" },
             { value: "3840x1920", label: "2.00:1" },
             { value: "3840x1744", label: "2.20:1" },
             { value: "3840x1728", label: "2.22:1" },
             { value: "3840x1632", label: "2.35:1" },
             { value: "3840x1608", label: "2.39:1" },
             { value: "3840x1536", label: "2.50:1" },
             { value: "3840x1504", label: "2.55:1" },
           ]
           : [
             { value: "1920x1080", label: "16:9" },
             { value: "1920x1038", label: "1.85:1" },
             { value: "1920x1012", label: "1.90:1" },
             { value: "1920x960", label: "2.00:1" },
             { value: "1920x872", label: "2.20:1" },
             { value: "1920x864", label: "2.22:1" },
             { value: "1920x816", label: "2.35:1" },
             { value: "1920x804", label: "2.39:1" },
             { value: "1920x768", label: "2.50:1" },
             { value: "1920x752", label: "2.55:1" },
           ]);
      const match = options.find((option) => option.label === selected || option.value === selected);
      return match ? match.value : baseValue;
    }

    function resolutionBaseValueFromSnapshot(snapshot) {
      const resolution = resolutionLabelFromSnapshot(snapshot);
      return sensorCropControlsAvailableForSnapshot(snapshot)
        ? sensorBaseResolutionValueFromSnapshot(snapshot)
        : resolution;
    }

    function cropRatioValueFromSnapshot(snapshot) {
      return resolutionLabelFromSnapshot(snapshot);
    }

    function resolutionBaseCommandPayload(snapshot, value) {
      if (processedVideoRecordFormatForSnapshot(snapshot))
        return h264ResolutionCommandPayload(h264ResolutionFromBaseSelection(snapshot, value));

      if (sensorCropControlsAvailableForSnapshot(snapshot)) {
        const currentRatio = cropRatioLabelForResolutionValue(resolutionLabelFromSnapshot(snapshot));
        return resolutionCommandPayload(sensorResolutionForBaseAndRatio(snapshot, value, currentRatio));
      }

      return resolutionCommandPayload(value);
    }

    function cropRatioCommandPayload(snapshot, value) {
      if (processedVideoRecordFormatForSnapshot(snapshot))
        return h264ResolutionCommandPayload(value);

      return resolutionCommandPayload(value);
    }

    function selectEntriesFromChoices(options) {
      return normalizeChoiceOptions(options).map((option) => ({
        id: option.value,
        label: option.label,
      }));
    }

    function setPreviewAspectForSnapshot(snapshot) {
      const stage = document.getElementById("previewStage");
      if (!stage)
        return;

      const resolution = processedVideoRecordFormatForSnapshot(snapshot)
        ? h264OutputResolutionLabel(snapshot)
        : resolutionLabelFromSnapshot(snapshot);
      const parts = String(resolution || "").split("x");
      const width = parts.length === 2 ? Number(parts[0]) : 16;
      const height = parts.length === 2 ? Number(parts[1]) : 9;
      if (width > 0 && height > 0)
        stage.style.setProperty("--preview-aspect-ratio", String(width) + " / " + String(height));
      else
        stage.style.setProperty("--preview-aspect-ratio", "16 / 9");
    }

    function resolutionCommandPayload(label) {
      if (label === "1332x990")
        return { cmd: "set_resolution", width: 1332, height: 990 };
      if (label === "2028x1080")
        return { cmd: "set_resolution", width: 2028, height: 1080 };
      if (label === "2028x1520")
        return { cmd: "set_resolution", width: 2028, height: 1520 };
      if (label === "4056x2160")
        return { cmd: "set_resolution", width: 4056, height: 2160 };
      if (label === "4056x3040")
        return { cmd: "set_resolution", width: 4056, height: 3040 };
      if (label === "3856x2180" || label === "3840x2160" || label === "4K")
        return { cmd: "set_resolution", width: 3840, height: 2160 };
      if (label === "1920x1012")
        return { cmd: "set_resolution", width: 1920, height: 1012 };
      if (label === "1920x1038")
        return { cmd: "set_resolution", width: 1920, height: 1038 };
      if (label === "1920x960")
        return { cmd: "set_resolution", width: 1920, height: 960 };
      if (label === "1920x872")
        return { cmd: "set_resolution", width: 1920, height: 872 };
      if (label === "1920x864")
        return { cmd: "set_resolution", width: 1920, height: 864 };
      if (label === "1920x816")
        return { cmd: "set_resolution", width: 1920, height: 816 };
      if (label === "1920x804")
        return { cmd: "set_resolution", width: 1920, height: 804 };
      if (label === "1920x768")
        return { cmd: "set_resolution", width: 1920, height: 768 };
      if (label === "1920x752")
        return { cmd: "set_resolution", width: 1920, height: 752 };
      if (label === "3840x2076")
        return { cmd: "set_resolution", width: 3840, height: 2076 };
      if (label === "3840x2024")
        return { cmd: "set_resolution", width: 3840, height: 2024 };
      if (label === "3840x1920")
        return { cmd: "set_resolution", width: 3840, height: 1920 };
      if (label === "3840x1744")
        return { cmd: "set_resolution", width: 3840, height: 1744 };
      if (label === "3840x1728")
        return { cmd: "set_resolution", width: 3840, height: 1728 };
      if (label === "3840x1632")
        return { cmd: "set_resolution", width: 3840, height: 1632 };
      if (label === "3840x1608")
        return { cmd: "set_resolution", width: 3840, height: 1608 };
      if (label === "3840x1536")
        return { cmd: "set_resolution", width: 3840, height: 1536 };
      if (label === "3840x1504")
        return { cmd: "set_resolution", width: 3840, height: 1504 };
      const genericMatch = String(label || "").match(/^(\d{2,5})x(\d{2,5})$/);
      if (genericMatch)
        return {
          cmd: "set_resolution",
          width: Math.max(0, Math.round(Number(genericMatch[1]) || 0)),
          height: Math.max(0, Math.round(Number(genericMatch[2]) || 0)),
        };
      return { cmd: "set_resolution", width: 1920, height: 1080 };
    }

    function nearestWhiteBalance(kelvin) {
      const numericKelvin = Number(kelvin) || 5600;
      let best = whiteBalanceOptions[0];
      let bestDiff = Infinity;
      for (const option of whiteBalanceOptions) {
        const diff = Math.abs(option - numericKelvin);
        if (diff < bestDiff) {
          best = option;
          bestDiff = diff;
        }
      }
      return best;
    }

    function whiteBalanceIndexFromKelvin(kelvin) {
      const matched = nearestWhiteBalance(kelvin);
      const index = whiteBalanceOptions.indexOf(matched);
      return index >= 0 ? index : 0;
    }

    function whiteBalanceFromSliderValue(value) {
      const index = clamp(Number(value) || 0, 0, whiteBalanceOptions.length - 1);
      return whiteBalanceOptions[index];
    }

    function recordingFormatLabel(formatValue) {
      const normalized = String(formatValue || "").toLowerCase();
      if (normalized === "prores")
        return "ProRes " + ({proxy: "Proxy", lt: "LT", hq: "HQ", "422": "422"}[state.snapshot && state.snapshot.prores_profile] || "422");
      if (normalized === "lossydng" || normalized === "lossy_dng")
        return "CinemaDNG";
      if (normalized === "mlv")
        return "MLV";
      if (normalized === "h264" || normalized === "h.264" || normalized === "mp4")
        return "H.264";
      return "CinemaDNG";
    }

    function recordingFormatCommandValue(displayValue) {
      const normalized = String(displayValue || "").toLowerCase();
      if (normalized.startsWith("prores")) return "prores";
      if (normalized === "cinemadng 3:1" || normalized === "cinemadng 3to1" ||
          normalized === "cdng 3:1" || normalized === "cdng 3to1" ||
          normalized === "cinemadng 5:1" || normalized === "cinemadng 5to1" ||
          normalized === "cdng 5:1" || normalized === "cdng 5to1" ||
          normalized === "lossydng" || normalized === "lossy_dng")
        return "lossydng";
      if (normalized === "mlv")
        return "mlv";
      if (normalized === "h.264" || normalized === "h264" || normalized === "mp4")
        return "h264";
      return "cdng";
    }

    function proresProfileCommandValue(value) {
      const label = String(value).toLowerCase();
      return label.endsWith("proxy") ? "proxy" : label.endsWith("lt") ? "lt" : label.endsWith("hq") ? "hq" : "422";
    }

    function recordingFeatureSupported(snapshot, feature) {
      return !!(snapshot && snapshot.recording_capabilities && snapshot.recording_capabilities[feature] === true);
    }

    function availableRecordFormatOptions(snapshot) {
      const rawRecording = snapshot && snapshot.recording_capabilities
        && snapshot.recording_capabilities.raw_recording !== false;
      return recordFormatOptions.filter((option) => {
        if (option.value === "CinemaDNG" || option.value === "MLV") return rawRecording;
        return !option.value.startsWith("ProRes") || recordingFeatureSupported(snapshot, "prores");
      });
    }

    function availableCompressionOptions(snapshot) {
      const options = [{ value: "none", label: "Uncompressed" }];
      return recordingFeatureSupported(snapshot, "raw_compression") ? options.concat([
        { value: "lossless_jpeg", label: "Lossless JPEG" },
        { value: "lossy_3to1", label: "Lossy 3:1" },
        { value: "lossy_5to1", label: "Lossy 5:1" }
      ]) : options;
    }

    function lossyDngRatioCommandValue(displayValue) {
      const normalized = String(displayValue || "").toLowerCase();
      return normalized.indexOf("5") !== -1 ? "5to1" : "3to1";
    }

    function normalizeCdngCompressionValue(value) {
      const normalized = String(value || "").trim().toLowerCase().replace(/[\s_-]/g, "");
      if (normalized === "losslessjpeg" || normalized === "lossless")
        return "lossless_jpeg";
      if (normalized === "lossy3:1" || normalized === "lossy3to1" || normalized === "3:1" || normalized === "3to1")
        return "lossy_3to1";
      if (normalized === "lossy5:1" || normalized === "lossy5to1" || normalized === "5:1" || normalized === "5to1")
        return "lossy_5to1";
      return "none";
    }

    function cdngCompressionValueForSnapshot(snapshot) {
      const format = String(snapshot && snapshot.recording_format || "").toLowerCase();
      if (format === "lossydng" || format === "lossy_dng")
        return String(snapshot && snapshot.lossy_dng_ratio || "3to1") === "5to1" ? "lossy_5to1" : "lossy_3to1";
      return normalizeCdngCompressionValue(snapshot && snapshot.cdng_compression);
    }

    function lossyDngRatioFromCompression(value) {
      return normalizeCdngCompressionValue(value) === "lossy_5to1" ? "5to1" : "3to1";
    }

    async function dispatchCdngCompressionSelection(value, dispatchFn) {
      const normalized = normalizeCdngCompressionValue(value);
      if (normalized !== "none" && !recordingFeatureSupported(state.snapshot, "raw_compression"))
        throw new Error("RAW compression requires Raspberry Pi 5-class hardware.");
      if (normalized === "lossy_3to1" || normalized === "lossy_5to1") {
        await dispatchFn({ cmd: "set_record_format", value: "lossydng" });
        await dispatchFn({ cmd: "set_lossy_dng_ratio", value: lossyDngRatioFromCompression(normalized) });
        return;
      }
      await dispatchFn({ cmd: "set_record_format", value: "cdng" });
      await dispatchFn({ cmd: "set_cdng_compression", value: normalized });
    }

    function processedVideoRecordFormat(label) {
      return label === "H.264";
    }

    function processedVideoRecordFormatForSnapshot(snapshot) {
      return !stillCaptureForSnapshot(snapshot) && processedVideoRecordFormat(recordingFormatLabel(snapshot && snapshot.recording_format));
    }

    function stillCaptureForSnapshot(snapshot) {
      return !!(snapshot && snapshot.ui && snapshot.ui.photo_mode_enabled);
    }

    function processedVideoSizeLabel(recordFormatText) {
      return "H.264 Size";
    }

    function processedVideoCropLabel(recordFormatText) {
      return "H.264 Crop";
    }

    function fixedTenBitRecordingFormat(formatValue, snapshot) {
      const label = recordingFormatLabel(formatValue);
      return !stillCaptureForSnapshot(snapshot) && (processedVideoRecordFormat(label) || label.startsWith("ProRes"));
    }

    function rawBitDepthNumber(value) {
      const digits = String(value || "").replace(/[^0-9]/g, "");
      if (digits === "8")
        return 8;
      return digits === "10" ? 10 : 12;
    }

    function rawBitDepthLabel(value) {
      const bitDepth = rawBitDepthNumber(value);
      if (bitDepth === 8)
        return "8-bit";
      return bitDepth === 10 ? "10-bit" : "12-bit";
    }

    function rawBitDepthAvailableForSensor(sensorLower) {
      const name = String(sensorLower || "");
      return name.indexOf("imx585") !== -1
        || name.indexOf("imx294") !== -1
        || name.indexOf("imx477") !== -1
        || name.indexOf("imx577") !== -1;
    }

    function raw8BitAvailableForSensor(sensorLower) {
      const name = String(sensorLower || "");
      return name.indexOf("imx477") !== -1
        || name.indexOf("imx577") !== -1;
    }

    function rawBitDepthOptionEnabled(snapshot, value, fixedForH264) {
      if (fixedForH264)
        return false;
      const sensorLower = sensorNameLower(snapshot);
      if (value === "12-bit" || value === "10-bit")
        return true;
      if (value === "8-bit")
        return raw8BitAvailableForSensor(sensorLower) || !sensorLower || sensorLower.indexOf("unknown") !== -1;
      return false;
    }

    function h264ResolutionLabel(snapshot) {
      const width = Number(snapshot && snapshot.h264_width) || 0;
      const height = Number(snapshot && snapshot.h264_height) || 0;
      if (width <= 0 || height <= 0)
        return "Match Capture";
      return formatResolutionValue(width, height);
    }

    function h264ResolutionCommandPayload(value) {
      const normalized = String(value || "").trim().toLowerCase();
      if (!normalized || normalized === "match capture" || normalized === "match" || normalized === "capture")
        return { cmd: "set_h264_resolution", width: 0, height: 0 };
      const parts = normalized.split("x");
      if (parts.length === 2) {
        const width = Math.max(0, Math.round(Number(parts[0]) || 0));
        const height = Math.max(0, Math.round(Number(parts[1]) || 0));
        return { cmd: "set_h264_resolution", width, height };
      }
      return { cmd: "set_h264_resolution", width: 0, height: 0 };
    }

    function h264OutputResolutionLabel(snapshot) {
      const width = Number(snapshot && snapshot.h264_width) || 0;
      const height = Number(snapshot && snapshot.h264_height) || 0;
      if (width <= 0 || height <= 0)
        return recordingFeatureSupported(snapshot, "h264_hardware") ? "1920x1080" : "2560x1440";
      return formatResolutionValue(width, height);
    }

    function cropRatioLabelForResolutionValue(resolutionValue) {
      const imx294Label = imx294CropRatioLabel(resolutionValue);
      if (imx294Label)
        return imx294Label;
      const value = String(resolutionValue || "");
      if (value === "4056x3040" || value === "2028x1520" || value === "1332x990")
        return "4:3";
      if (value === "4056x2280" || value === "2028x1140")
        return "16:9";
      if (value === "3840x2076" || value === "2560x1384" || value === "1920x1038"
          || value === "4056x2192" || value === "2028x1096")
        return "1.85:1";
      if (value === "3840x2024" || value === "2560x1348" || value === "1920x1012"
          || value === "4056x2136" || value === "2028x1068")
        return "1.90:1";
      if (value === "3840x1920" || value === "2560x1280" || value === "1920x960"
          || value === "4056x2028" || value === "2028x1014")
        return "2.00:1";
      if (value === "3840x1744" || value === "2560x1164" || value === "1920x872"
          || value === "4056x1844" || value === "2028x922")
        return "2.20:1";
      if (value === "3840x1728" || value === "2560x1152" || value === "1920x864"
          || value === "4056x1828" || value === "2028x914")
        return "2.22:1";
      if (value === "3840x1632" || value === "2560x1088" || value === "1920x816"
          || value === "4056x1728" || value === "2028x864")
        return "2.35:1";
      if (value === "3840x1608" || value === "2560x1072" || value === "1920x804"
          || value === "4056x1696" || value === "2028x848")
        return "2.39:1";
      if (value === "3840x1536" || value === "2560x1024" || value === "1920x768"
          || value === "4056x1624" || value === "2028x812")
        return "2.50:1";
      if (value === "3840x1504" || value === "2560x1004" || value === "1920x752"
          || value === "4056x1592" || value === "2028x796")
        return "2.55:1";
      if (value === "4056x2160" || value === "2028x1080")
        return "17:9";
      return "16:9";
    }

    function h264BaseResolutionForValue(resolutionValue) {
      const value = String(resolutionValue || "");
      if (value.startsWith("1920x") || value.startsWith("2028x"))
        return "1920x1080";
      if (value.startsWith("2560x"))
        return "2560x1440";
      return "2560x1440";
    }

    function h264CropOptionsForBase(baseValue) {
      if (baseValue === "1920x1080") {
        return [
          { value: "1920x1080", label: "16:9" },
          { value: "1920x1038", label: "1.85:1" },
          { value: "1920x1012", label: "1.90:1" },
          { value: "1920x960", label: "2.00:1" },
          { value: "1920x872", label: "2.20:1" },
          { value: "1920x864", label: "2.22:1" },
          { value: "1920x816", label: "2.35:1" },
          { value: "1920x804", label: "2.39:1" },
          { value: "1920x768", label: "2.50:1" },
          { value: "1920x752", label: "2.55:1" },
        ];
      }
      if (baseValue === "2560x1440") {
        return [
          { value: "2560x1440", label: "16:9" },
          { value: "2560x1384", label: "1.85:1" },
          { value: "2560x1348", label: "1.90:1" },
          { value: "2560x1280", label: "2.00:1" },
          { value: "2560x1164", label: "2.20:1" },
          { value: "2560x1152", label: "2.22:1" },
          { value: "2560x1088", label: "2.35:1" },
          { value: "2560x1072", label: "2.39:1" },
          { value: "2560x1024", label: "2.50:1" },
          { value: "2560x1004", label: "2.55:1" },
        ];
      }
      return h264CropOptionsForBase("2560x1440");
    }

    function h264ResolutionForBaseAndRatio(baseValue, ratioLabelOrValue) {
      const options = h264CropOptionsForBase(baseValue);
      const selected = String(ratioLabelOrValue || "");
      const match = options.find((option) => option.label === selected || option.value === selected);
      return match ? match.value : baseValue;
    }

    function h264BaseResolutionValueFromSnapshot(snapshot) {
      return h264BaseResolutionForValue(h264OutputResolutionLabel(snapshot));
    }

    function h264CropRatioValueFromSnapshot(snapshot) {
      const output = h264OutputResolutionLabel(snapshot);
      return h264ResolutionForBaseAndRatio(
        h264BaseResolutionForValue(output),
        cropRatioLabelForResolutionValue(output)
      );
    }

    function h264ResolutionFromBaseSelection(snapshot, baseValue) {
      return h264ResolutionForBaseAndRatio(
        baseValue,
        cropRatioLabelForResolutionValue(h264OutputResolutionLabel(snapshot))
      );
    }

    function processedVideoBaseResolutionOptionsForSnapshot(snapshot) {
      const maxWidth = Number(snapshot && snapshot.recording_capabilities && snapshot.recording_capabilities.h264_max_width) || 2560;
      return h264BaseResolutionOptions.filter(option => Number(option.value.split("x")[0]) <= maxWidth);
    }

    function captureCodecLabel(formatValue) {
      const normalized = String(formatValue || "").toLowerCase();
      if (normalized === "prores") return recordingFormatLabel(formatValue);
      if (normalized === "lossydng" || normalized === "lossy_dng")
        return (state.snapshot && state.snapshot.lossy_dng_ratio === "5to1")
          ? "CinemaDNG 5:1 RAW"
          : "CinemaDNG 3:1 RAW";
      if (normalized === "mlv")
        return "MLV RAW";
      if (normalized === "h264" || normalized === "h.264" || normalized === "mp4")
        return "H.264";
      return "CinemaDNG RAW";
    }

    function timecodeModeLabel(modeValue) {
      return String(modeValue || "").toLowerCase() === "rec_run" ? "Rec Run" : "Free Run";
    }

    function autofocusModeLabel(modeValue) {
      const value = String(modeValue || "").toLowerCase();
      if (value === "manual") return "Manual";
      if (value === "auto") return "Auto";
      return "Continuous";
    }

    function autofocusRangeLabel(rangeValue) {
      const value = String(rangeValue || "").toLowerCase();
      if (value === "macro") return "Macro";
      if (value === "full") return "Full";
      return "Normal";
    }

    function autofocusSpeedLabel(speedValue) {
      return String(speedValue || "").toLowerCase() === "fast" ? "Fast" : "Normal";
    }

    function autofocusWindowLabel(snapshot) {
      const x = Number(snapshot && snapshot.af_window_x);
      const y = Number(snapshot && snapshot.af_window_y);
      const width = Number(snapshot && snapshot.af_window_w);
      const height = Number(snapshot && snapshot.af_window_h);
      if (Math.abs(x) < 0.001 && Math.abs(y) < 0.001 &&
          Math.abs(width - 1) < 0.001 && Math.abs(height - 1) < 0.001)
        return "Full Frame";
      if (Math.abs(x - 0.25) < 0.001 && Math.abs(y - 0.25) < 0.001 &&
          Math.abs(width - 0.5) < 0.001 && Math.abs(height - 0.5) < 0.001)
        return "Center";
      return "Custom";
    }

    function autofocusWindowPayload(label) {
      return label === "Full Frame"
        ? { cmd: "set_af_window", x: 0, y: 0, width: 1, height: 1 }
        : { cmd: "set_af_window", x: 0.25, y: 0.25, width: 0.5, height: 0.5 };
    }

    function formatFpsLabel(value) {
      const fps = Number(value);
      if (!isFinite(fps) || fps <= 0)
        return "--";
      return Number.isInteger(fps) ? (fps + " FPS") : (fps.toFixed(3) + " FPS");
    }

    function formatToggleLabel(value) {
      return value ? "On" : "Off";
    }

    function formatPercent(value) {
      if (typeof value !== "number" || !isFinite(value))
        return "--%";
      return Math.round(value) + "%";
    }

    function formatDegrees(value) {
      if (typeof value !== "number" || !isFinite(value))
        return "--";
      return value.toFixed(1).replace(".0", "") + "°C";
    }

    function formatWatts(value) {
      if (typeof value !== "number" || !isFinite(value))
        return "--";
      return value.toFixed(1) + " W";
    }

    function formatVolts(value) {
      if (typeof value !== "number" || !isFinite(value))
        return "--";
      return value.toFixed(1) + " V";
    }

    function formatStorage(bytes) {
      if (typeof bytes !== "number" || !isFinite(bytes) || bytes <= 0)
        return "--";
      const gb = bytes / (1024 * 1024 * 1024);
      if (gb >= 100)
        return Math.round(gb) + " GB";
      return gb.toFixed(1).replace(".0", "") + " GB";
    }

    function formatMinutes(minutes) {
      if (typeof minutes !== "number" || !isFinite(minutes) || minutes <= 0)
        return "--";
      if (minutes >= 60) {
        const hours = Math.floor(minutes / 60);
        const mins = Math.round(minutes % 60);
        return hours + "h " + String(mins).padStart(2, "0") + "m";
      }
      return Math.round(minutes) + " min";
    }

    function formatStillCount(count) {
      if (typeof count !== "number" || !isFinite(count) || count < 0)
        return "--";
      const stills = Math.floor(count);
      return stills === 1
        ? "1 still"
        : stills.toLocaleString() + " stills";
    }

    function mediaEstimateValue(systemState, stillModeEnabled) {
      if (stillModeEnabled)
        return formatStillCount(systemState && systemState.record_stills_left);
      return formatMinutes(systemState && systemState.record_time_left_minutes);
    }

    function mediaEstimateLabel(stillModeEnabled) {
      return stillModeEnabled ? "Stills left" : "Time left";
    }

    function mediaEstimateSubLabel(systemState, stillModeEnabled) {
      return stillModeEnabled
        ? "Current still estimate"
        : ((systemState && systemState.time_left_label) || "Current format estimate");
    }

    function activeBatteryType(uiState) {
      return uiState && uiState.battery_type ? uiState.battery_type : "V-Mount";
    }

    function batteryCapacityOptionsForType(type) {
      if (type === "NP-F")
        return ["15Wh", "24Wh", "37Wh", "50Wh", "Custom"];
      if (type === "LP-E6")
        return ["13Wh", "16Wh", "20Wh", "Custom"];
      if (type === "Custom" || type === "USB-C / DC")
        return ["Custom"];
      return ["50Wh", "95Wh", "150Wh", "190Wh", "Custom"];
    }

    function defaultBatteryCapacityForType(type) {
      if (type === "NP-F")
        return "37Wh";
      if (type === "LP-E6")
        return "16Wh";
      if (type === "Custom" || type === "USB-C / DC")
        return "Custom";
      return "150Wh";
    }

    function defaultBatteryFullVoltageForType(type) {
      if (type === "NP-F" || type === "LP-E6")
        return 8.4;
      return 16.8;
    }

    function defaultBatteryEmptyVoltageForType(type) {
      if (type === "NP-F" || type === "LP-E6")
        return 6.0;
      return 13.2;
    }

    function batterySupportsPercent(type) {
      return type !== "USB-C / DC";
    }

    function batteryCapacityForType(uiState) {
      const type = activeBatteryType(uiState);
      const batteryCapacity = uiState && uiState.battery_capacity ? uiState.battery_capacity : "150Wh";
      const options = batteryCapacityOptionsForType(type);
      return options.indexOf(batteryCapacity) === -1 ? defaultBatteryCapacityForType(type) : batteryCapacity;
    }

    function selectedBatteryCapacityWh(uiState) {
      const batteryCapacity = batteryCapacityForType(uiState);
      if (batteryCapacity === "Custom") {
        const customWh = Number(uiState && uiState.custom_battery_wh);
        return clamp(isFinite(customWh) ? customWh : 150, 10, 500);
      }

      const parsed = parseInt(String(batteryCapacity).replace("Wh", ""), 10);
      return isFinite(parsed) ? parsed : 150;
    }

    function batteryProfilePercent(uiState, systemState) {
      if (!systemState || !systemState.power_sensor_available)
        return NaN;

      const voltage = Number(systemState.battery_voltage_v);
      const current = Number(systemState.battery_current_a);
      if (!isFinite(voltage) || voltage <= 3.0)
        return NaN;

      const type = activeBatteryType(uiState);
      if (!batterySupportsPercent(type))
        return NaN;

      let full = defaultBatteryFullVoltageForType(type);
      let empty = defaultBatteryEmptyVoltageForType(type);
      let sag = type === "V-Mount" ? 0.26 : 0.08;
      if (type === "Custom") {
        full = Number(uiState && uiState.custom_battery_full_voltage);
        empty = Number(uiState && uiState.custom_battery_empty_voltage);
        if (!isFinite(full))
          full = 16.8;
        if (!isFinite(empty))
          empty = 13.2;
        sag = 0.0;
      }

      const compensatedVoltage = voltage + Math.max(0, isFinite(current) ? current : 0) * sag;
      return clamp(((compensatedVoltage - empty) / Math.max(0.1, full - empty)) * 100, 0, 100);
    }

    function formatBatteryCurrent(value) {
      if (typeof value !== "number" || !isFinite(value))
        return "--.-A";
      return value.toFixed(1) + "A";
    }

    function formatBatteryVoltage(value) {
      if (typeof value !== "number" || !isFinite(value))
        return "--.-V";
      return value.toFixed(1) + "V";
    }

    function formatBatteryRuntimeEstimate(uiState, systemState) {
      const powerDraw = Number(systemState.power_draw_w);
      const batteryPercent = batteryProfilePercent(uiState, systemState);
      if (!isFinite(powerDraw) || powerDraw <= 0.1 || !isFinite(batteryPercent))
        return "Runtime unavailable";

      const remainingWh = (selectedBatteryCapacityWh(uiState) * clamp(batteryPercent, 0, 100)) / 100;
      if (remainingWh <= 0.05)
        return "Almost empty";

      const totalMinutes = Math.floor((remainingWh / powerDraw) * 60);
      if (totalMinutes < 1)
        return "<1 min left";

      const hours = Math.floor(totalMinutes / 60);
      const minutes = totalMinutes % 60;
      if (hours > 0)
        return hours + "h " + String(minutes).padStart(2, "0") + "m left";
      return minutes + " min left";
    }

    function currentAudioOptions(snapshot) {
      const audioState = snapshot && snapshot.audio && typeof snapshot.audio === "object"
        ? snapshot.audio
        : {};
      return {
        inputDevices: normalizeSelectEntries(audioState.input_devices),
        outputDevices: normalizeSelectEntries(audioState.output_devices),
        hasInputDevices: !!audioState.has_input_devices,
        hasOutputDevices: !!audioState.has_output_devices,
      };
    }

    function moveSettingsCards(panel, ids, beforeNode) {
      if (!panel)
        return;
      const fragment = document.createDocumentFragment();
      ids.forEach((id) => {
        const card = document.getElementById(id);
        if (card)
          fragment.appendChild(card);
      });
      panel.insertBefore(fragment, beforeNode || null);
    }

    function arrangeSettingsPanels() {
      const systemPanel = document.querySelector('.settings-panel[data-panel="system"]');
      const mediaPanel = document.querySelector('.settings-panel[data-panel="power"]');
      moveSettingsCards(mediaPanel, ["settingsMediaCard", "settingsFormatMediaCard", "settingsMediaSpeedCard"], null);
      moveSettingsCards(
        systemPanel,
        ["settingsBatteryCard", "settingsCustomBatteryCard", "settingsBatteryInfoCard", "settingsAutoPowerOffCard"],
        document.getElementById("settingsThermalCard") || document.getElementById("settingsPowerActionsCard")
      );
    }

    function setSettingsOpen(open) {
      state.settingsOpen = !!open;
      if (state.settingsOpen && state.mediaBrowserOpen)
        setMediaBrowserOpen(false);
      document.body.classList.toggle("settings-open", state.settingsOpen);
      document.getElementById("settingsOverlay").classList.toggle("hidden", !state.settingsOpen);
      if (state.settingsOpen && state.activeSettingsPanel === "audio") void refreshAudioCaptureCapabilities();
    }

    function setMediaBrowserOpen(open) {
      state.mediaBrowserOpen = !!open;
      if (state.mediaBrowserOpen)
        setSettingsOpen(false);
      document.body.classList.toggle("media-browser-open", state.mediaBrowserOpen);
      document.getElementById("mediaBrowserOverlay").classList.toggle("hidden", !state.mediaBrowserOpen);
      if (state.mediaBrowserOpen)
        void fetchMediaLibrary(false);
    }

    function setSettingsPanel(panelName) {
      const normalized = settingsPanelMeta[panelName] ? panelName : "record";
      state.activeSettingsPanel = normalized;

      document.querySelectorAll(".settings-nav-item").forEach((item) => {
        item.classList.toggle("active", item.dataset.panel === normalized);
      });
      document.querySelectorAll(".settings-panel").forEach((panel) => {
        panel.classList.toggle("active", panel.dataset.panel === normalized);
      });

      setText("settingsPageTitle", settingsPanelMeta[normalized].title);
      setText("settingsPageCopy", settingsPanelMeta[normalized].copy);
      if (normalized === "audio" && state.settingsOpen) void refreshAudioCaptureCapabilities();

      if (normalized === "presets")
        fetchPresetLibrary(true);
      if (normalized === "config" && !state.configDirty)
        fetchConfigFile(true);
      if (normalized === "record" && state.snapshot)
        window.requestAnimationFrame(() => drawImuAxisVisual(state.snapshot.imu || {}));
    }

    function setText(id, value) {
      const element = document.getElementById(id);
      if (element && element.textContent !== value)
        element.textContent = value;
    }

    function setFormValue(id, value) {
      const element = document.getElementById(id);
      if (!element || document.activeElement === element)
        return;
      element.value = value;
    }

    function setElementDisabled(id, disabled) {
      const element = document.getElementById(id);
      if (element)
        element.disabled = !!disabled;
    }

    function ensureSelectOptions(id, options) {
      const element = document.getElementById(id);
      if (!element)
        return;

      const values = Array.isArray(options) ? options : [];
      const existing = Array.from(element.options).map((option) => option.value);
      if (existing.length === values.length && existing.every((value, index) => value === values[index]))
        return;

      element.innerHTML = "";
      for (const value of values) {
        const option = document.createElement("option");
        option.value = value;
        option.textContent = value;
        element.appendChild(option);
      }
    }

    function normalizeSelectEntries(entries) {
      return Array.isArray(entries)
        ? entries
            .filter((entry) => entry && typeof entry.label === "string")
            .map((entry) => ({
              value: String(entry.id !== undefined ? entry.id : (entry.value !== undefined ? entry.value : "")),
              label: String(entry.label),
            }))
        : [];
    }

    function ensureSelectEntries(id, entries) {
      const element = document.getElementById(id);
      if (!element)
        return;

      const normalized = normalizeSelectEntries(entries);
      const existing = Array.from(element.options).map((option) => ({
        value: option.value,
        label: option.textContent || "",
      }));
      if (existing.length === normalized.length
          && existing.every((option, index) => option.value === normalized[index].value
            && option.label === normalized[index].label)) {
        return;
      }

      element.innerHTML = "";
      for (const entry of normalized) {
        const option = document.createElement("option");
        option.value = entry.value;
        option.textContent = entry.label;
        element.appendChild(option);
      }
    }

    function lutPreviewLabel(path) {
      const value = String(path || "");
      if (!value)
        return "No LUT";
      const slash = Math.max(value.lastIndexOf("/"), value.lastIndexOf("\\"));
      return slash >= 0 ? value.slice(slash + 1) : value;
    }

    function lutPreviewEntries(uiState) {
      const selected = String(uiState && uiState.lut_preview_file || "");
      const options = Array.isArray(uiState && uiState.lut_preview_options)
        ? uiState.lut_preview_options
        : [];
      const values = [""];
      for (const option of options) {
        const value = String(option || "");
        if (value && !values.includes(value))
          values.push(value);
      }
      if (selected && !values.includes(selected))
        values.push(selected);
      return values.map((value) => ({ id: value, label: lutPreviewLabel(value) }));
    }

    function renderMediaDriveOptions(systemState, hasMedia) {
      const drives = Array.isArray(systemState.media_drives) ? systemState.media_drives : [];
      const entries = drives.length > 0
        ? drives.map((drive) => ({
            id: String(drive.device_path || ""),
            label: String(drive.display_name || drive.device_path || "Unnamed drive"),
          })).filter((entry) => entry.id.length > 0)
        : [{ id: "", label: "No available drives" }];

      const select = document.getElementById("settingsMediaDriveSelect");
      const previousValue = select ? select.value : "";
      ensureSelectEntries("settingsMediaDriveSelect", entries);
      if (select && entries.some((entry) => entry.id === previousValue))
        select.value = previousValue;

      const hasMountableDrives = drives.length > 0 && !hasMedia;
      setElementDisabled("settingsMediaDriveSelect", !hasMountableDrives);
      setElementDisabled("mountMediaDriveButton", !hasMountableDrives);
      setText(
        "settingsMediaDriveNote",
        hasMedia
          ? "Eject current media before mounting another drive."
          : "Supports exFAT, FAT32, NTFS, and other Linux-supported filesystems."
      );
    }

    function resolveSelectEntryLabel(entries, selectedValue, placeholder) {
      const normalized = normalizeSelectEntries(entries);
      const valueText = String(selectedValue === undefined || selectedValue === null ? "" : selectedValue);
      const exactMatch = normalized.find((entry) => entry.value === valueText);
      if (exactMatch)
        return exactMatch.label;
      if (!valueText && normalized.length > 0)
        return normalized[0].label;
      return placeholder;
    }

    function formatPresetCountLabel(count) {
      const numeric = Number(count) || 0;
      return numeric + (numeric === 1 ? " preset" : " presets");
    }

    function favoriteCountLabel(count) {
      const numeric = Number(count) || 0;
      return numeric + (numeric === 1 ? " item" : " items");
    }

    function normalizeChoiceOptions(options) {
      return Array.isArray(options)
        ? options.map((option) => {
            if (option && typeof option === "object") {
              const value = option.value !== undefined ? option.value : (option.label !== undefined ? option.label : "");
              const label = option.label !== undefined ? option.label : String(value);
              return {
                value: String(value),
                label: String(label),
                enabled: option.enabled !== false,
              };
            }
            return {
              value: String(option),
              label: String(option),
              enabled: true,
            };
          })
        : [];
    }

    function favoriteItemTitle(itemId) {
      return favoriteItemMeta[itemId] ? favoriteItemMeta[itemId].title : itemId;
    }

    function favoriteItemSection(itemId) {
      return favoriteItemMeta[itemId] ? favoriteItemMeta[itemId].section : "";
    }

    function sanitizeFavoriteOrder(values) {
      const input = Array.isArray(values) ? values : [];
      const out = [];
      for (const entry of input) {
        const itemId = String(entry || "").trim();
        if (!favoriteItemMeta[itemId] || out.includes(itemId))
          continue;
        out.push(itemId);
      }
      return out;
    }

    function favoriteItemsFromSnapshot(snapshot) {
      const uiState = snapshot && snapshot.ui && typeof snapshot.ui === "object" ? snapshot.ui : {};
      return sanitizeFavoriteOrder(uiState.favorites_order);
    }

    function favoriteValueSummary(itemId, snapshot) {
      const uiState = snapshot && snapshot.ui && typeof snapshot.ui === "object" ? snapshot.ui : {};
      const systemState = snapshot && snapshot.system && typeof snapshot.system === "object" ? snapshot.system : {};
      const deviceState = snapshot && snapshot.device && typeof snapshot.device === "object" ? snapshot.device : {};
      const displayWhiteBalance = clamp(Number(snapshot && snapshot.wb_kelvin) || 5600, 2500, 8000);
      const focusPeakingEnabled = !!uiState.focus_peaking_enabled;
      const focusPeakingThreshold = valueToFocusThreshold(uiState.focus_peaking_threshold);
      const focusPeakingColor = uiState.focus_peaking_color || "Red";
      const zebraEnabled = !!uiState.zebra_enabled;
      const zebraLevel = thresholdToZebraLevel(uiState.zebra_threshold);
      const falseColorEnabled = !!uiState.false_color_enabled;
      const falseColorMode = intToFalseColorMode(uiState.false_color_mode);
      const lutPreviewEnabled = !!uiState.lut_preview_enabled;
      const lutPreviewFile = String(uiState.lut_preview_file || "");
      const smpteEnabled = !!uiState.smpte_enabled;
      const guidesEnabled = uiState.guides_enabled !== false;
      const guidesType = uiState.guides_type || "Thirds";
      const guidesThickness = String(clamp(Number(uiState.guides_thickness) || 1, 1, 4));
      const centerMarkerEnabled = uiState.center_marker_enabled !== false;
      const centerMarkerType = uiState.center_marker_type || "Circle/Dot";
      const anamorphicEnabled = !!uiState.anamorphic_desqueeze_enabled;
      const anamorphicRatio = uiState.anamorphic_ratio || "1.33x";
      const stillModeEnabled = !!uiState.photo_mode_enabled;
      const soundRecorderEnabled = !!uiState.sound_recorder_enabled;
      const recordFormatText = recordingFormatLabel(snapshot && snapshot.recording_format);
      const resolutionLabel = snapshot ? resolutionLabelFromSnapshot(snapshot) : fallbackResolutionOptions[0];
      const timecodeText = timecodeModeLabel(uiState.timecode_mode || (snapshot && snapshot.timecode_mode));
      const autofocusModeText = autofocusModeLabel(snapshot && snapshot.af_mode);
      const autofocusWindowText = autofocusWindowLabel(snapshot);
      const inputVolume = clamp(Number(uiState.input_volume) || 60, 0, 100);
      const headphoneVolume = clamp(Number(uiState.headphone_volume) || 55, 0, 100);
      const batteryType = activeBatteryType(uiState);
      const batteryCapacity = batteryCapacityForType(uiState);
      const sleepMode = uiState.sleep_mode || "Off";
      const uiOrientation = uiState.ui_orientation || "Landscape";
      const controlsOpacityLabel = opacityValueToLabel(uiState.camera_controls_opacity);
      const controlsMode = uiState.camera_controls_mode || "Light";
      const dateTimeOverlayEnabled = !!uiState.date_time_overlay_enabled;
      const fanMode = systemState.fan_mode || "Auto";
      const mediaStatus = snapshot && snapshot.media_mounted
        ? ((snapshot.recording) ? "Recording active" : "Media mounted")
        : "No media mounted";

      switch (itemId) {
        case "externalMonitor":
          return formatToggleLabel(!!uiState.external_monitor_enabled)
            + " • "
            + (uiState.external_monitor_mode || "Clean Feed");
        case "focusPeaking":
          return formatToggleLabel(focusPeakingEnabled) + " • " + focusPeakingThreshold + " • " + focusPeakingColor;
        case "zebra":
          return formatToggleLabel(zebraEnabled) + " • " + zebraLevel;
        case "falseColor":
          return formatToggleLabel(falseColorEnabled) + " • " + falseColorMode;
        case "lutPreview":
          return formatToggleLabel(lutPreviewEnabled) + " • " + lutPreviewLabel(lutPreviewFile);
        case "smpteBars":
          return formatToggleLabel(smpteEnabled);
        case "guides":
          return formatToggleLabel(guidesEnabled) + " • " + guidesType + " • " + guidesThickness;
        case "centerMarker":
          return formatToggleLabel(centerMarkerEnabled) + " • " + centerMarkerType;
        case "greyscale":
          return formatToggleLabel(!!uiState.grayscale_enabled);
        case "anamorphicDesqueeze":
          return formatToggleLabel(anamorphicEnabled) + " • " + anamorphicRatio;
        case "stillMode":
          return formatToggleLabel(stillModeEnabled)
            + " • "
            + (uiState.photo_format || "DNG")
            + " • "
            + (uiState.photo_timer || "Off")
            + " • "
            + (uiState.photo_bracketing_enabled ? "Bracket 3" : (uiState.photo_burst || "Single"));
        case "soundRecorder":
          return formatToggleLabel(soundRecorderEnabled);
        case "format":
          return recordFormatText
            + " • "
            + (processedVideoRecordFormatForSnapshot(snapshot) ? h264OutputResolutionLabel(snapshot) : resolutionLabel);
        case "gyroData":
          return formatToggleLabel(!!uiState.record_gyro_enabled);
        case "timecode":
          return timecodeText;
        case "autofocus":
          return autofocusModeText + " • " + autofocusWindowText;
        case "audioOptions":
          return "Record "
            + formatToggleLabel(!!uiState.record_audio_enabled)
            + " • Monitor "
            + formatToggleLabel(!!uiState.live_audio_monitoring_enabled);
        case "levels":
          return "Input " + inputVolume + " • Headphones " + headphoneVolume;
        case "battery":
          return batteryType + " • " + batteryCapacity;
        case "batteryInfo":
          return formatBatteryRuntimeEstimate(uiState, systemState);
        case "autoPowerOff":
          return sleepMode;
        case "uiOrientation":
          return uiOrientation;
        case "cameraControlsOpacity":
          return controlsOpacityLabel + " • " + controlsMode;
        case "dateTime":
          return (systemState.time_zone || "UTC") + " â€¢ " + formatToggleLabel(dateTimeOverlayEnabled);
        case "media":
          return mediaStatus + " • " + ((snapshot && snapshot.capture && snapshot.capture.storage) || "No Media");
        case "mediaSpeed":
          return systemState.write_speed_result_text || "Not Tested";
        case "thermal":
          return fanMode + " • " + formatDegrees(systemState.cpu_temp_c);
        case "powerActions":
          return "Restart and shutdown controls";
        case "wifiToggle":
          return "Managed from the DPI UI";
        case "currentNetwork":
          return (snapshot && snapshot.network_host) || "Managed from the DPI UI";
        case "availableNetworks":
          return "Managed from the DPI UI";
        case "camera":
          return ((snapshot && snapshot.capture && snapshot.capture.sensor) || "Camera")
            + " • "
            + ((snapshot && snapshot.capture && snapshot.capture.codec) || "RAW");
        case "sbc":
          return deviceState.pi_model || "Raspberry Pi";
        case "systemInfo":
          return (deviceState.hostname || "--") + " • " + (deviceState.uptime_text || "--");
        case "systemDrive":
          return (deviceState.system_storage_total_text || "--")
            + " total • "
            + (deviceState.system_storage_free_text || "--")
            + " free";
        default:
          return "Quick access setting";
      }
    }

    async function applyFavoritesOrder(order) {
      const sanitized = sanitizeFavoriteOrder(order);
      await sendCommand({ cmd: "set_favorites_order", value: sanitized.join(", ") });
    }

    async function moveFavoriteItem(itemId, direction) {
      const favorites = favoriteItemsFromSnapshot(state.snapshot);
      const currentIndex = favorites.indexOf(itemId);
      if (currentIndex < 0)
        return;

      const targetIndex = currentIndex + direction;
      if (targetIndex < 0 || targetIndex >= favorites.length)
        return;

      const next = favorites.slice();
      const [item] = next.splice(currentIndex, 1);
      next.splice(targetIndex, 0, item);

      try {
        await applyFavoritesOrder(next);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function removeFavoriteItem(itemId) {
      const favorites = favoriteItemsFromSnapshot(state.snapshot).filter((entry) => entry !== itemId);
      try {
        await applyFavoritesOrder(favorites);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function toggleFavoriteItem(itemId) {
      const favorites = favoriteItemsFromSnapshot(state.snapshot);
      const next = favorites.includes(itemId)
        ? favorites.filter((entry) => entry !== itemId)
        : favorites.concat(itemId);
      try {
        await applyFavoritesOrder(next);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function clearFavorites() {
      try {
        await applyFavoritesOrder([]);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    function createDomElement(tagName, className, text) {
      const element = document.createElement(tagName);
      if (className)
        element.className = className;
      if (text !== undefined)
        element.textContent = text;
      return element;
    }

    function createFavoriteCardFrame(itemId, snapshot) {
      const card = createDomElement("div", "settings-card");
      const head = createDomElement("div", "settings-card-head");
      const copyColumn = createDomElement("div");
      copyColumn.appendChild(createDomElement("h3", "settings-card-title", favoriteItemTitle(itemId)));
      copyColumn.appendChild(createDomElement("p", "settings-card-copy", favoriteValueSummary(itemId, snapshot)));
      head.appendChild(copyColumn);

      const section = favoriteItemSection(itemId);
      if (section) {
        head.appendChild(createDomElement("div", "online-pill", section));
      }

      card.appendChild(head);
      return card;
    }

    function createFavoritePlaceholderCard(itemId, snapshot, detail) {
      const card = createFavoriteCardFrame(itemId, snapshot);
      card.appendChild(createDomElement("div", "preset-empty", detail));
      return card;
    }

    function createFavoriteActionButton(label, className, onClick, disabled) {
      const button = createDomElement("button", className, label);
      button.type = "button";
      button.disabled = !!disabled;
      button.addEventListener("click", () => {
        void onClick();
      });
      return button;
    }

    function createFavoritePillGrid(options, currentValue, onSelect, columns) {
      const grid = createDomElement("div", "pill-grid" + (columns === 3 ? " three-col" : ""));
      normalizeChoiceOptions(options).forEach((option) => {
        const button = createDomElement(
          "button",
          "pill-button"
            + (option.value === String(currentValue) ? " active" : "")
            + (option.enabled ? "" : " disabled"),
          option.label
        );
        button.type = "button";
        button.disabled = !option.enabled;
        button.addEventListener("click", () => {
          if (!option.enabled)
            return;
          void onSelect(option.value);
        });
        grid.appendChild(button);
      });
      return grid;
    }

    function createFavoriteSelectBlock(label, valueText, options, currentValue, onSelect, columns) {
      const block = createDomElement("section", "select-block");
      const topline = createDomElement("div", "select-topline");
      topline.appendChild(createDomElement("span", "", label));
      if (valueText !== undefined && valueText !== null && valueText !== "")
        topline.appendChild(createDomElement("span", "select-value", valueText));
      block.appendChild(topline);
      block.appendChild(createFavoritePillGrid(options, currentValue, onSelect, columns));
      return block;
    }

    function createFavoriteFieldGrid() {
      return createDomElement("div", "settings-field-grid");
    }

    function createFavoriteSelectField(label, options, currentValue, onChange, spanTwo, disabled) {
      const stack = createDomElement("label", "field-stack" + (spanTwo ? " field-span-2" : ""));
      stack.appendChild(createDomElement("span", "input-label", label));
      const select = createDomElement("select", "settings-select");
      options.forEach((option) => {
        const item = document.createElement("option");
        item.value = String(option);
        item.textContent = String(option);
        select.appendChild(item);
      });
      select.value = String(currentValue);
      select.disabled = !!disabled;
      select.addEventListener("change", () => {
        if (select.disabled)
          return;
        void onChange(select.value);
      });
      stack.appendChild(select);
      return stack;
    }

    function createFavoriteSelectEntryField(label, entries, currentValue, onChange, spanTwo, disabled) {
      const stack = createDomElement("label", "field-stack" + (spanTwo ? " field-span-2" : ""));
      stack.appendChild(createDomElement("span", "input-label", label));
      const select = createDomElement("select", "settings-select");
      const normalized = normalizeSelectEntries(entries);
      normalized.forEach((entry) => {
        const item = document.createElement("option");
        item.value = entry.value;
        item.textContent = entry.label;
        select.appendChild(item);
      });
      select.value = String(currentValue || "");
      select.disabled = !!disabled || normalized.length === 0;
      select.addEventListener("change", () => {
        if (select.disabled)
          return;
        void onChange(select.value);
      });
      stack.appendChild(select);
      return stack;
    }

    function createFavoriteCaptureList(rows) {
      const list = createDomElement("div", "capture-list");
      rows.forEach((row) => {
        const captureRow = createDomElement("div", "capture-row");
        captureRow.appendChild(createDomElement("span", "capture-key", row.key));
        if (row.detail !== undefined) {
          const stack = createDomElement("div", "capture-value-stack");
          stack.appendChild(createDomElement("span", "capture-value", row.value));
          stack.appendChild(createDomElement("span", "capture-detail", row.detail));
          captureRow.appendChild(stack);
        } else {
          captureRow.appendChild(createDomElement("span", "capture-value", row.value));
        }
        list.appendChild(captureRow);
      });
      return list;
    }

    function createFavoriteActionRow(buttons, noteText) {
      const row = createDomElement("div", "settings-action-row");
      buttons.forEach((button) => row.appendChild(button));
      if (noteText)
        row.appendChild(createDomElement("span", "subtle-note", noteText));
      return row;
    }

    function createFavoriteSliderBlock(label, value, onCommit) {
      const block = createDomElement("section", "select-block");
      const topline = createDomElement("div", "select-topline");
      const valueText = createDomElement("span", "select-value", String(value));
      topline.appendChild(createDomElement("span", "", label));
      topline.appendChild(valueText);
      block.appendChild(topline);

      const input = createDomElement("input", "slider");
      input.type = "range";
      input.min = "0";
      input.max = "100";
      input.step = "1";
      input.value = String(value);
      input.addEventListener("input", () => {
        valueText.textContent = input.value;
      });
      input.addEventListener("change", () => {
        void onCommit(Number(input.value));
      });
      block.appendChild(input);

      const labels = createDomElement("div", "slider-labels");
      labels.appendChild(createDomElement("span", "", "0"));
      labels.appendChild(createDomElement("span", "", "100"));
      block.appendChild(labels);
      return block;
    }

    function favoriteControlsInputActive(signature) {
      const container = document.getElementById("settingsFavoriteControls");
      const active = document.activeElement;
      return state.favoriteControlsSignature === signature
        && container
        && active
        && container.contains(active)
        && (active.tagName === "INPUT" || active.tagName === "SELECT" || active.tagName === "TEXTAREA");
    }

    function renderFavoriteControls(snapshot) {
      const container = document.getElementById("settingsFavoriteControls");
      if (!container)
        return;

      const favorites = favoriteItemsFromSnapshot(snapshot);
      const signature = favorites.join("|");
      if (favoriteControlsInputActive(signature))
        return;

      state.favoriteControlsSignature = signature;
      container.innerHTML = "";

      if (favorites.length === 0) {
        const card = createDomElement("div", "settings-card");
        card.appendChild(createDomElement("div", "preset-empty", "No favorites selected yet. Add some below to build your quick access tab."));
        container.appendChild(card);
        return;
      }

      const uiState = snapshot && snapshot.ui && typeof snapshot.ui === "object" ? snapshot.ui : {};
      const systemState = snapshot && snapshot.system && typeof snapshot.system === "object" ? snapshot.system : {};
      const deviceState = snapshot && snapshot.device && typeof snapshot.device === "object" ? snapshot.device : {};
      const audioOptions = currentAudioOptions(snapshot);
      const hasMedia = !!(snapshot && snapshot.media_mounted);
      const recordFormatText = recordingFormatLabel(snapshot && snapshot.recording_format);
      const h264RecordFormat = processedVideoRecordFormatForSnapshot(snapshot);
      const fixedTenBitRecordFormat = fixedTenBitRecordingFormat(snapshot && snapshot.recording_format, snapshot);
      const snapshotSensorLower = sensorNameLower(snapshot);
      const rawBitDepthAvailable = rawBitDepthAvailableForSensor(snapshotSensorLower);
      const rawBitDepthFixedForH264 = fixedTenBitRecordFormat;
      const rawBitDepthText = rawBitDepthFixedForH264 ? (recordingFeatureSupported(snapshot, "h264_hardware") ? "8-bit" : "10-bit") : rawBitDepthLabel(snapshot && snapshot.raw_bit_depth);
      const rawBitDepthSelectOptions = rawBitDepthOptions.map((value) => ({
        value,
        label: value,
        enabled: rawBitDepthOptionEnabled(snapshot, value, rawBitDepthFixedForH264),
      }));
      const resolutionLabel = snapshot ? resolutionLabelFromSnapshot(snapshot) : fallbackResolutionOptions[0];
      const resolutionOptions = availableResolutionOptionsForSnapshot(snapshot);
      const timecodeText = timecodeModeLabel(uiState.timecode_mode || (snapshot && snapshot.timecode_mode));
      const autofocusModeText = autofocusModeLabel(snapshot && snapshot.af_mode);
      const autofocusRangeText = autofocusRangeLabel(snapshot && snapshot.af_range);
      const autofocusSpeedText = autofocusSpeedLabel(snapshot && snapshot.af_speed);
      const autofocusWindowText = autofocusWindowLabel(snapshot);
      const focusPeakingEnabled = !!uiState.focus_peaking_enabled;
      const focusPeakingThreshold = valueToFocusThreshold(uiState.focus_peaking_threshold);
      const focusPeakingColor = uiState.focus_peaking_color || "Red";
      const zebraEnabled = !!uiState.zebra_enabled;
      const zebraLevel = thresholdToZebraLevel(uiState.zebra_threshold);
      const falseColorEnabled = !!uiState.false_color_enabled;
      const falseColorMode = intToFalseColorMode(uiState.false_color_mode);
      const lutPreviewEnabled = !!uiState.lut_preview_enabled;
      const lutPreviewFile = String(uiState.lut_preview_file || "");
      const lutPreviewOptionEntries = lutPreviewEntries(uiState);
      const smpteEnabled = !!uiState.smpte_enabled;
      const guidesEnabled = uiState.guides_enabled !== false;
      const guidesType = uiState.guides_type || "Thirds";
      const guidesThickness = String(clamp(Number(uiState.guides_thickness) || 1, 1, 4));
      const centerMarkerEnabled = uiState.center_marker_enabled !== false;
      const centerMarkerType = uiState.center_marker_type || "Circle/Dot";
      const grayscaleEnabled = !!uiState.grayscale_enabled;
      const anamorphicEnabled = !!uiState.anamorphic_desqueeze_enabled;
      const anamorphicRatio = uiState.anamorphic_ratio || "1.33x";
      const cropControlsAvailable = sensorCropControlsAvailableForSnapshot(snapshot);
      const cropControlsVisible = h264RecordFormat || cropControlsAvailable;
      const resolutionBaseOptions = h264RecordFormat
        ? processedVideoBaseResolutionOptionsForSnapshot(snapshot)
        : (cropControlsAvailable ? sensorBaseResolutionOptionsForSnapshot(snapshot) : selectEntriesFromChoices(resolutionOptions));
      const resolutionBaseValue = h264RecordFormat ? h264BaseResolutionValueFromSnapshot(snapshot) : resolutionBaseValueFromSnapshot(snapshot);
      const cropRatioOptions = h264RecordFormat ? h264CropOptionsForBase(resolutionBaseValue) : sensorCropRatioOptionsForSnapshot(snapshot);
      const cropRatioValue = h264RecordFormat ? h264CropRatioValueFromSnapshot(snapshot) : cropRatioValueFromSnapshot(snapshot);
      const cropRatioEnabled = h264RecordFormat || cropControlsAvailable;
      const externalMonitorEnabled = !!uiState.external_monitor_enabled;
      const externalMonitorMode = uiState.external_monitor_mode || "Clean Feed";
      const externalMonitorInfoOverlay = uiState.external_monitor_info_overlay || "On";
      const externalMonitorOrientation = uiState.external_monitor_orientation || "Landscape";
      const stillModeEnabled = !!uiState.photo_mode_enabled;
      const stillTimerValue = uiState.photo_timer || "Off";
      const stillBurstValue = uiState.photo_burst || "Single";
      const stillFormatValue = uiState.photo_format || "DNG";
      const stillTimelapseValue = uiState.photo_timelapse || "Off";
      const soundRecorderEnabled = !!uiState.sound_recorder_enabled && audioOptions.hasInputDevices;
      const recordAudioEnabled = !!uiState.record_audio_enabled && audioOptions.hasInputDevices;
      const recordGyroEnabled = !!uiState.record_gyro_enabled;
      const liveAudioMonitoringEnabled = !!uiState.live_audio_monitoring_enabled;
      const audioMeterEnabled = !!uiState.audio_meter_enabled;
      const inputVolume = clamp(Number(uiState.input_volume) || 60, 0, 100);
      const headphoneVolume = clamp(Number(uiState.headphone_volume) || 55, 0, 100);
      const batteryType = activeBatteryType(uiState);
      const batteryCapacity = batteryCapacityForType(uiState);
      const customBatteryWh = clamp(Number(uiState.custom_battery_wh) || 150, 10, 500);
      const customBatteryFullVoltage = clamp(Number(uiState.custom_battery_full_voltage) || 16.8, 0.5, 60);
      const customBatteryEmptyVoltage = clamp(Number(uiState.custom_battery_empty_voltage) || 13.2, 0.5, customBatteryFullVoltage - 0.1);
      const uiOrientation = uiState.ui_orientation || "Landscape";
      const controlsOpacityLabel = opacityValueToLabel(uiState.camera_controls_opacity);
      const controlsMode = uiState.camera_controls_mode || "Light";
      const dateTimeOverlayEnabled = !!uiState.date_time_overlay_enabled;
      const timeZoneValue = systemState.time_zone || "UTC";
      const localDateTimeValue = systemState.local_time_input || "";
      const fanModeValue = systemState.fan_mode || "Auto";
      const sleepModeValue = uiState.sleep_mode || "Off";
      const fanModeSupported = fanModeSupportedForSnapshot(snapshot);

      favorites.forEach((itemId) => {
        const card = createFavoriteCardFrame(itemId, snapshot);

        switch (itemId) {
          case "externalMonitor": {
            card.appendChild(createFavoriteSelectBlock("External Monitor", null, toggleOptions, formatToggleLabel(externalMonitorEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_external_monitor_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Mode", externalMonitorModeOptions, externalMonitorMode, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_external_monitor_mode", value });
            }));
            grid.appendChild(createFavoriteSelectField("Info Overlay", externalMonitorInfoOverlayOptions, externalMonitorInfoOverlay, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_external_monitor_info_overlay", value });
            }));
            grid.appendChild(createFavoriteSelectField("Orientation", externalMonitorOrientationOptions, externalMonitorOrientation, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_external_monitor_orientation", value });
            }));
            card.appendChild(grid);
            break;
          }
          case "focusPeaking": {
            card.appendChild(createFavoriteSelectBlock("Focus Peaking", null, toggleOptions, formatToggleLabel(focusPeakingEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_focus_peaking_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Threshold", focusPeakingThresholdOptions, focusPeakingThreshold, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_focus_peaking_threshold", value: focusThresholdToValue(value) });
            }));
            grid.appendChild(createFavoriteSelectField("Color", focusPeakingColorOptions, focusPeakingColor, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_focus_peaking_color", value });
            }));
            card.appendChild(grid);
            break;
          }
          case "zebra": {
            card.appendChild(createFavoriteSelectBlock("Zebra", null, toggleOptions, formatToggleLabel(zebraEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_zebra_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Level", zebraLevelOptions, zebraLevel, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_zebra_threshold", value: zebraLevelToThreshold(value) });
            }));
            card.appendChild(grid);
            break;
          }
          case "falseColor": {
            card.appendChild(createFavoriteSelectBlock("False Color", null, toggleOptions, formatToggleLabel(falseColorEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_false_color_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Mode", falseColorModeOptions, falseColorMode, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_false_color_mode", value });
            }, true));
            card.appendChild(grid);
            break;
          }
          case "lutPreview": {
            card.appendChild(createFavoriteSelectBlock("LUT Preview", null, toggleOptions, formatToggleLabel(lutPreviewEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_lut_preview_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectEntryField("LUT", lutPreviewOptionEntries, lutPreviewFile, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_lut_preview_file", value });
            }, true));
            card.appendChild(grid);
            break;
          }
          case "smpteBars": {
            card.appendChild(createFavoriteSelectBlock("SMPTE Bars", null, toggleOptions, formatToggleLabel(smpteEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_smpte_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            break;
          }
          case "guides": {
            card.appendChild(createFavoriteSelectBlock("Guides", null, toggleOptions, formatToggleLabel(guidesEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_guides_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Type", guidesTypeOptions, guidesType, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_guides_type", value });
            }));
            grid.appendChild(createFavoriteSelectField("Thickness", guidesThicknessOptions, guidesThickness, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_guides_thickness", value: Number(value) });
            }));
            card.appendChild(grid);
            break;
          }
          case "centerMarker": {
            card.appendChild(createFavoriteSelectBlock("Center Marker", null, toggleOptions, formatToggleLabel(centerMarkerEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_center_marker_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Type", centerMarkerTypeOptions, centerMarkerType, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_center_marker_type", value });
            }, true));
            card.appendChild(grid);
            break;
          }
          case "greyscale": {
            card.appendChild(createFavoriteSelectBlock("Greyscale", null, toggleOptions, formatToggleLabel(grayscaleEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_grayscale_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            break;
          }
          case "anamorphicDesqueeze": {
            card.appendChild(createFavoriteSelectBlock("Anamorphic Desqueeze", null, toggleOptions, formatToggleLabel(anamorphicEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_anamorphic_desqueeze_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Ratio", anamorphicRatioOptions, anamorphicRatio, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_anamorphic_ratio", value });
            }, true));
            card.appendChild(grid);
            break;
          }
          case "stillMode": {
            card.appendChild(createFavoriteSelectBlock("Still Mode", null, toggleOptions, formatToggleLabel(stillModeEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_photo_mode_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Format", stillFormatOptionsForTimelapse(stillTimelapseValue), stillFormatValue, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_photo_format", value });
            }));
            grid.appendChild(createFavoriteSelectField("Timer", stillTimerOptions, stillTimerValue, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_photo_timer", value });
            }));
            grid.appendChild(createFavoriteSelectField(uiState.photo_bracketing_enabled ? "EV Offset" : "Burst",
              uiState.photo_bracketing_enabled ? bracketEvOptions : stillBurstOptions,
              uiState.photo_bracketing_enabled ? bracketEvOptions[clamp(Math.round(Number(uiState.photo_bracketing_ev || 1) * 3) - 1, 0, 8)] : stillBurstValue, async (value) => {
              await dispatchFavoriteCommand(uiState.photo_bracketing_enabled
                ? {cmd: "set_photo_bracketing_ev", value: (bracketEvOptions.indexOf(value) + 1) / 3}
                : {cmd: "set_photo_burst", value});
            }));
            grid.appendChild(createFavoriteSelectField("Timelapse", stillTimelapseOptions.includes(stillTimelapseValue)
              ? stillTimelapseOptions : [...stillTimelapseOptions, stillTimelapseValue], stillTimelapseValue, async (value) => {
              await dispatchFavoriteCommand({
                cmd: "set_photo_timelapse",
                value,
                format: value === "Off" ? stillFormatValue : safeTimelapseStillFormat(stillFormatValue),
              });
            }));
            card.appendChild(grid);
            break;
          }
          case "soundRecorder": {
            card.appendChild(createFavoriteSelectBlock("Sound Recorder", null, toggleOptions, formatToggleLabel(soundRecorderEnabled), async (value) => {
              if (!audioOptions.hasInputDevices) {
                showToast("No audio input device detected.", true);
                return;
              }
              await dispatchFavoriteCommand({ cmd: "set_sound_recorder_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            break;
          }
          case "format": {
            const grid = createFavoriteFieldGrid();
            if (cropControlsVisible) {
              grid.appendChild(createFavoriteSelectEntryField(h264RecordFormat ? processedVideoSizeLabel(recordFormatText) : "Resolution", resolutionBaseOptions, resolutionBaseValue, async (value) => {
                await dispatchFavoriteCommand(resolutionBaseCommandPayload(snapshot, value));
              }, false, false));
              grid.appendChild(createFavoriteSelectEntryField(h264RecordFormat ? processedVideoCropLabel(recordFormatText) : "Crop Mode", cropRatioOptions, cropRatioValue, async (value) => {
                await dispatchFavoriteCommand(cropRatioCommandPayload(snapshot, value));
              }, false, !cropRatioEnabled));
            } else {
              grid.appendChild(createFavoriteSelectField("Resolution", resolutionOptions, resolutionLabel, async (value) => {
                await dispatchFavoriteCommand(resolutionCommandPayload(value));
              }, true, false));
            }
            card.appendChild(grid);
            card.appendChild(createFavoriteSelectBlock("Record Format", recordFormatText, availableRecordFormatOptions(snapshot), recordFormatText, async (value) => {
              const formatValue = recordingFormatCommandValue(value);
              await dispatchFavoriteCommand({ cmd: "set_record_format", value: formatValue });
              if (formatValue === "prores")
                await dispatchFavoriteCommand({ cmd: "set_prores_profile", value: proresProfileCommandValue(value) });
              if (formatValue === "lossydng")
                await dispatchFavoriteCommand({ cmd: "set_lossy_dng_ratio", value: lossyDngRatioCommandValue(value) });
            }));
            if (recordFormatText === "CinemaDNG") {
              card.appendChild(createFavoriteSelectEntryField("Compression", availableCompressionOptions(snapshot), cdngCompressionValueForSnapshot(snapshot), async (value) => {
                await dispatchCdngCompressionSelection(value, dispatchFavoriteCommand);
              }, true, !!snapshot.recording || !recordingFeatureSupported(snapshot, "raw_compression")));
            }
            card.appendChild(createFavoriteSelectBlock("Bit Depth", rawBitDepthText, rawBitDepthSelectOptions, rawBitDepthText, async (value) => {
              if (rawBitDepthFixedForH264)
                return;
              await dispatchFavoriteCommand({ cmd: "set_raw_bit_depth", value: rawBitDepthNumber(value) });
            }));
            break;
          }
          case "gyroData": {
            card.appendChild(createFavoriteSelectBlock("Record Gyro Data", null, toggleOptions, formatToggleLabel(!!uiState.record_gyro_enabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_record_gyro_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            break;
          }
          case "timecode": {
            card.appendChild(createFavoriteSelectBlock("Timecode Mode", timecodeText, timecodeOptions, timecodeText, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_timecode_mode", value: value === "Rec Run" ? "rec_run" : "free_run" });
            }));
            break;
          }
          case "autofocus": {
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Mode", autofocusModeOptions, autofocusModeText, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_af_mode", value: value.toLowerCase() });
            }));
            grid.appendChild(createFavoriteSelectField("Window", autofocusWindowOptions, autofocusWindowText === "Custom" ? "Center" : autofocusWindowText, async (value) => {
              await dispatchFavoriteCommand(autofocusWindowPayload(value));
            }));
            grid.appendChild(createFavoriteSelectField("Range", autofocusRangeOptions, autofocusRangeText, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_af_range", value: value.toLowerCase() });
            }));
            grid.appendChild(createFavoriteSelectField("Speed", autofocusSpeedOptions, autofocusSpeedText, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_af_speed", value: value.toLowerCase() });
            }));
            card.appendChild(grid);
            break;
          }
          case "audioOptions": {
            card.appendChild(createFavoriteSelectBlock("Record Audio", null, toggleOptions, formatToggleLabel(recordAudioEnabled), async (value) => {
              if (!audioOptions.hasInputDevices) {
                showToast("No audio input device detected.", true);
                return;
              }
              await dispatchFavoriteCommand({ cmd: "set_record_audio_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            card.appendChild(createFavoriteSelectBlock("Live Audio Monitoring", null, toggleOptions, formatToggleLabel(liveAudioMonitoringEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_live_audio_monitoring_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            card.appendChild(createFavoriteSelectBlock("VU Meter", null, toggleOptions, formatToggleLabel(audioMeterEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_audio_meter_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectEntryField("Input Device", audioOptions.hasInputDevices ? audioOptions.inputDevices : [{ id: "", label: "No audio input device detected" }], uiState.audio_input_device || "", async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_audio_input_device", value });
            }, true));
            grid.appendChild(createFavoriteSelectEntryField("Output Device", audioOptions.hasOutputDevices ? audioOptions.outputDevices : [{ id: "", label: "None" }], uiState.audio_output_device || "", async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_audio_output_device", value });
            }, true));
            card.appendChild(grid);
            break;
          }
          case "levels": {
            card.appendChild(createFavoriteSliderBlock("Input Volume", inputVolume, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_input_volume", value });
            }));
            card.appendChild(createFavoriteSliderBlock("Headphone Volume", headphoneVolume, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_headphone_volume", value });
            }));
            break;
          }
          case "battery": {
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Battery Type", batteryTypeOptions, batteryType, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_battery_type", value });
            }));
            grid.appendChild(createFavoriteSelectField("Battery Capacity", batteryCapacityOptionsForType(batteryType), batteryCapacity, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_battery_capacity", value });
            }, false, batteryType === "USB-C / DC"));
            card.appendChild(grid);
            if (batteryCapacity === "Custom" || batteryType === "Custom") {
              card.appendChild(createFavoriteActionRow([
                createFavoriteActionButton("-5Wh", "secondary-button", async () => {
                  await dispatchFavoriteCommand({ cmd: "set_custom_battery_wh", value: clamp(customBatteryWh - 5, 10, 500) });
                }),
                createFavoriteActionButton(customBatteryWh + "Wh", "secondary-button", async () => {}),
                createFavoriteActionButton("+5Wh", "secondary-button", async () => {
                  await dispatchFavoriteCommand({ cmd: "set_custom_battery_wh", value: clamp(customBatteryWh + 5, 10, 500) });
                }),
              ], "Custom battery size used for runtime estimates."));
              card.lastChild.children[1].disabled = true;
            }
            if (batteryType === "Custom") {
              card.appendChild(createFavoriteActionRow([
                createFavoriteActionButton("Full -0.1V", "secondary-button", async () => {
                  await dispatchFavoriteCommand({ cmd: "set_custom_battery_voltage", full: clamp(customBatteryFullVoltage - 0.1, customBatteryEmptyVoltage + 0.1, 60) });
                }),
                createFavoriteActionButton("Full +0.1V", "secondary-button", async () => {
                  await dispatchFavoriteCommand({ cmd: "set_custom_battery_voltage", full: clamp(customBatteryFullVoltage + 0.1, customBatteryEmptyVoltage + 0.1, 60) });
                }),
                createFavoriteActionButton("Empty -0.1V", "secondary-button", async () => {
                  await dispatchFavoriteCommand({ cmd: "set_custom_battery_voltage", empty: clamp(customBatteryEmptyVoltage - 0.1, 0.5, customBatteryFullVoltage - 0.1) });
                }),
                createFavoriteActionButton("Empty +0.1V", "secondary-button", async () => {
                  await dispatchFavoriteCommand({ cmd: "set_custom_battery_voltage", empty: clamp(customBatteryEmptyVoltage + 0.1, 0.5, customBatteryFullVoltage - 0.1) });
                }),
              ], "Custom full/empty voltage endpoints."));
            }
            break;
          }
          case "batteryInfo": {
            const profilePercent = batteryProfilePercent(uiState, systemState);
            card.appendChild(createFavoriteCaptureList([
              { key: "Battery", value: formatBatteryVoltage(systemState.battery_voltage_v), detail: formatBatteryCurrent(systemState.battery_current_a) + " • " + (isFinite(profilePercent) ? formatPercent(profilePercent) : activeBatteryType(uiState)) },
              { key: "Power Draw", value: formatWatts(systemState.power_draw_w), detail: formatBatteryRuntimeEstimate(uiState, systemState) },
            ]));
            break;
          }
          case "autoPowerOff": {
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Power Off", sleepModeOptions, sleepModeValue, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_sleep_mode", value });
            }, true));
            card.appendChild(grid);
            break;
          }
          case "uiOrientation": {
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Orientation", ["Landscape", "Left Side", "Right Side", "Upside Down"], uiOrientation, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_ui_orientation", value });
            }, true));
            card.appendChild(grid);
            break;
          }
          case "cameraControlsOpacity": {
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Opacity", controlsOpacityOptions, controlsOpacityLabel, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_camera_controls_opacity", value: opacityLabelToValue(value) });
            }, true));
            grid.appendChild(createFavoriteSelectField("Controls", controlsModeOptions, controlsMode, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_camera_controls_mode", value });
            }, true));
            card.appendChild(grid);
            break;
          }
          case "dateTime": {
            card.appendChild(createFavoriteSelectBlock("Show on Camera Page", null, toggleOptions, formatToggleLabel(dateTimeOverlayEnabled), async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_date_time_overlay_enabled", enabled: value === "On" ? "true" : "false" });
            }));
            const grid = createFavoriteFieldGrid();
            const timezoneField = createFavoriteSelectField(
              "Timezone",
              Array.isArray(systemState.time_zone_options) ? systemState.time_zone_options : ["UTC"],
              timeZoneValue,
              async () => {
                if (timezoneSelect)
                  await applyTimeZoneSelection(timezoneSelect);
              },
              true
            );
            grid.appendChild(timezoneField);
            const inputStack = createDomElement("label", "field-stack field-span-2");
            inputStack.appendChild(createDomElement("span", "input-label", "Local Date & Time"));
            const dateInput = createDomElement("input", "settings-input");
            dateInput.type = "datetime-local";
            dateInput.value = localDateTimeValue;
            inputStack.appendChild(dateInput);
            grid.appendChild(inputStack);
            card.appendChild(grid);
            const timezoneSelect = timezoneField.querySelector("select");
            card.appendChild(createFavoriteActionRow([
              createFavoriteActionButton("Apply Date & Time", "primary-button", async () => {
                const parts = parseDateTimeInputParts(dateInput.value);
                if (!parts) {
                  showToast("Choose a valid local date and time first.", true);
                  return;
                }
                await dispatchFavoriteCommand({
                  cmd: "apply_date_time",
                  year: parts.year,
                  month: parts.month,
                  day: parts.day,
                  hour: parts.hour,
                  minute: parts.minute,
                  time_zone: timezoneSelect.value,
                });
              }),
            ], "Writes the system clock and RTC with the selected timezone."));
            break;
          }
          case "media": {
            card.appendChild(createFavoriteCaptureList([
              { key: "Status", value: hasMedia ? (snapshot.recording ? "Recording active" : "Media mounted") : "No media mounted" },
              { key: "Type", value: (snapshot.capture && snapshot.capture.storage) || "No Media" },
              { key: "Path", value: systemState.media_label || "--" },
              { key: mediaEstimateLabel(stillModeEnabled), value: hasMedia ? mediaEstimateValue(systemState, stillModeEnabled) : "N/A" },
            ]));
            card.appendChild(createFavoriteActionRow([
              createFavoriteActionButton("Format Media", "primary-button", async () => { await requestFormatMedia(); }, !hasMedia),
              createFavoriteActionButton("Eject Media", "secondary-button", async () => { await requestEjectMedia(); }, !hasMedia),
            ]));
            break;
          }
          case "mediaSpeed": {
            card.appendChild(createFavoriteCaptureList([
              { key: "Result", value: systemState.write_speed_result_text || "Not Tested" },
              { key: "Detail", value: systemState.write_speed_detail_text || "--" },
            ]));
            card.appendChild(createFavoriteActionRow([
              createFavoriteActionButton(systemState.write_speed_running ? "Testing..." : "Run Speed Test", "primary-button", async () => { await requestMediaSpeedTest(); }, !hasMedia || !!systemState.write_speed_running),
            ], "Runs a temporary direct write test and removes the file afterwards."));
            break;
          }
          case "thermal": {
            card.appendChild(createFavoriteCaptureList([
              { key: "CPU Temp", value: formatDegrees(systemState.cpu_temp_c) },
            ]));
            const grid = createFavoriteFieldGrid();
            grid.appendChild(createFavoriteSelectField("Fan Mode", ["Auto", "Full Blast"], fanModeValue, async (value) => {
              await dispatchFavoriteCommand({ cmd: "set_fan_mode", value });
            }, true, !fanModeSupported));
            card.appendChild(grid);
            if (!fanModeSupported)
              card.appendChild(createDomElement("div", "subtle-note", "Fan mode is unavailable on Raspberry Pi 4."));
            break;
          }
          case "powerActions": {
            const actions = createDomElement("div", "action-grid two-col power-actions-grid");
            actions.appendChild(createFavoriteActionButton(snapshot.state === "standby" ? "Wake Camera" : "Standby",
              "secondary-button standby-button", requestStandby, !!snapshot.recording || !!snapshot.timelapse_active));
            actions.appendChild(createFavoriteActionButton("Restart Camera", "secondary-button", requestRestartSystem));
            actions.appendChild(createFavoriteActionButton("Shutdown", "danger-button", requestPowerOffSystem));
            card.appendChild(actions);
            card.appendChild(createDomElement("div", "subtle-note", "Uses the same system-level permissions as the local UI."));
            break;
          }
          case "wifiToggle":
          case "currentNetwork":
          case "availableNetworks": {
            container.appendChild(createFavoritePlaceholderCard(itemId, snapshot, "Wi-Fi control is still handled in the native UI. This web app currently mirrors the network readout only."));
            return;
          }
          case "camera": {
            card.appendChild(createFavoriteCaptureList([
              { key: "Sensor", value: (snapshot.capture && snapshot.capture.sensor) || "--" },
              { key: "Codec", value: (snapshot.capture && snapshot.capture.codec) || "--" },
              { key: "Preview", value: (snapshot.capture && snapshot.capture.preview) || "--" },
            ]));
            break;
          }
          case "sbc": {
            card.appendChild(createFavoriteCaptureList([
              { key: "Model", value: deviceState.pi_model || "--" },
              { key: "Serial", value: deviceState.serial_number || "--" },
              { key: "Memory", value: deviceState.ram_total_text || "--", detail: (deviceState.ram_used_text || "--") + " used" },
            ]));
            break;
          }
          case "systemInfo": {
            card.appendChild(createFavoriteCaptureList([
              { key: "Kernel", value: deviceState.kernel_version || "--" },
              { key: "Uptime", value: deviceState.uptime_text || "--" },
              { key: "IP Address", value: deviceState.ip_address || "--" },
            ]));
            break;
          }
          case "systemDrive": {
            card.appendChild(createFavoriteCaptureList([
              { key: "System / SD Storage", value: deviceState.system_storage_total_text || "--", detail: (deviceState.system_storage_free_text || "--") + " free • Mounted at " + (deviceState.system_storage_mount_point || "/") },
            ]));
            break;
          }
          default:
            card.appendChild(createDomElement("div", "preset-empty", "This favorite does not have a dedicated web control yet."));
            break;
        }

        container.appendChild(card);
      });
    }

    async function dispatchFavoriteCommand(payload) {
      try {
        await sendCommand(payload);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    function renderFavoritesPanel(snapshot) {
      const favorites = favoriteItemsFromSnapshot(snapshot);
      const currentList = document.getElementById("settingsFavoriteList");
      const candidateList = document.getElementById("settingsFavoriteCandidateList");
      setText("settingsFavoriteCountPill", favoriteCountLabel(favorites.length));
      setElementDisabled("clearFavoritesButton", favorites.length === 0);
      renderFavoriteControls(snapshot);

      currentList.innerHTML = "";
      if (favorites.length === 0) {
        const empty = document.createElement("div");
        empty.className = "preset-empty";
        empty.textContent = "No favorites selected yet. Add some below to build your quick access tab.";
        currentList.appendChild(empty);
      } else {
        favorites.forEach((itemId, index) => {
          const row = document.createElement("div");
          row.className = "preset-row";

          const meta = document.createElement("div");
          meta.className = "preset-meta";

          const sectionPill = document.createElement("span");
          sectionPill.className = "favorite-section-pill";
          sectionPill.textContent = favoriteItemSection(itemId);
          meta.appendChild(sectionPill);

          const name = document.createElement("div");
          name.className = "preset-name";
          name.textContent = favoriteItemTitle(itemId);
          meta.appendChild(name);

          const detail = document.createElement("div");
          detail.className = "favorite-summary";
          detail.textContent = favoriteValueSummary(itemId, snapshot);
          meta.appendChild(detail);

          row.appendChild(meta);

          const actions = document.createElement("div");
          actions.className = "preset-actions";

          const up = document.createElement("button");
          up.className = "secondary-button preset-action";
          up.type = "button";
          up.textContent = "Up";
          up.disabled = index === 0;
          up.addEventListener("click", () => {
            void moveFavoriteItem(itemId, -1);
          });
          actions.appendChild(up);

          const down = document.createElement("button");
          down.className = "secondary-button preset-action";
          down.type = "button";
          down.textContent = "Down";
          down.disabled = index === favorites.length - 1;
          down.addEventListener("click", () => {
            void moveFavoriteItem(itemId, 1);
          });
          actions.appendChild(down);

          const remove = document.createElement("button");
          remove.className = "danger-button preset-action";
          remove.type = "button";
          remove.textContent = "Remove";
          remove.addEventListener("click", () => {
            void removeFavoriteItem(itemId);
          });
          actions.appendChild(remove);

          row.appendChild(actions);
          currentList.appendChild(row);
        });
      }

      candidateList.innerHTML = "";
      favoriteCandidateOrder.forEach((itemId) => {
        const selected = favorites.includes(itemId);
        const row = document.createElement("div");
        row.className = "preset-row";

        const meta = document.createElement("div");
        meta.className = "preset-meta";

        const sectionPill = document.createElement("span");
        sectionPill.className = "favorite-section-pill";
        sectionPill.textContent = favoriteItemSection(itemId);
        meta.appendChild(sectionPill);

        const name = document.createElement("div");
        name.className = "preset-name";
        name.textContent = favoriteItemTitle(itemId);
        meta.appendChild(name);

        const detail = document.createElement("div");
        detail.className = "preset-detail";
        detail.textContent = selected
          ? "Already in the shared Favorites list."
          : "Add this setting to your quick access tab.";
        meta.appendChild(detail);

        row.appendChild(meta);

        const actions = document.createElement("div");
        actions.className = "preset-actions";

        const toggle = document.createElement("button");
        toggle.className = (selected ? "primary-button" : "secondary-button") + " preset-action";
        toggle.type = "button";
        toggle.textContent = selected ? "Added" : "Add";
        toggle.addEventListener("click", () => {
          void toggleFavoriteItem(itemId);
        });
        actions.appendChild(toggle);

        row.appendChild(actions);
        candidateList.appendChild(row);
      });
    }

    function renderPresetLibrary(presets) {
      const list = document.getElementById("settingsPresetList");
      const items = Array.isArray(presets) ? presets : [];
      state.presetLibrary = items;
      setText("settingsPresetCountPill", formatPresetCountLabel(items.length));

      list.innerHTML = "";
      if (items.length === 0) {
        const empty = document.createElement("div");
        empty.className = "preset-empty";
        empty.textContent = "No presets saved on the camera yet.";
        list.appendChild(empty);
        return;
      }

      for (const preset of items) {
        const row = document.createElement("div");
        row.className = "preset-row";

        const meta = document.createElement("div");
        meta.className = "preset-meta";

        const name = document.createElement("div");
        name.className = "preset-name";
        name.textContent = preset.name || preset.file_name || "Preset";
        meta.appendChild(name);

        const detail = document.createElement("div");
        detail.className = "preset-detail";
        detail.textContent = (preset.size_text || "--")
          + " • Updated "
          + (preset.modified_text || "Unknown");
        meta.appendChild(detail);

        const actions = document.createElement("div");
        actions.className = "preset-actions";

        const load = document.createElement("button");
        load.className = "primary-button preset-action";
        load.type = "button";
        load.textContent = "Load";
        load.addEventListener("click", () => {
          void loadPresetFromLibrary(preset);
        });
        actions.appendChild(load);

        const download = document.createElement("a");
        download.className = "secondary-button preset-download preset-action";
        download.href = preset.download_url || "#";
        download.textContent = "Download";
        download.setAttribute("download", preset.file_name || "preset.apertar-preset.json");
        if (preset.download_url) {
          download.addEventListener("click", () => {
            showToast("Downloading " + (preset.name || "preset") + ".", false);
          });
        } else {
          download.setAttribute("aria-disabled", "true");
        }
        actions.appendChild(download);

        const remove = document.createElement("button");
        remove.className = "danger-button preset-action";
        remove.type = "button";
        remove.textContent = "Delete";
        remove.addEventListener("click", () => {
          void deletePresetFromLibrary(preset);
        });
        actions.appendChild(remove);

        row.appendChild(meta);
        row.appendChild(actions);
        list.appendChild(row);
      }
    }

    function mediaBrowserFilterLabel(filter) {
      if (filter === "still")
        return "stills";
      if (filter === "timelapse")
        return "timelapses";
      if (filter === "audio")
        return "audio recordings";
      return "clips";
    }

    function mediaItemCategory(item) {
      if (item && item.category)
        return item.category;
      if (item && item.kind === "audio")
        return "audio";
      if (item && item.kind === "still")
        return "still";
      const type = String(item && item.type || "").toLowerCase();
      if (type.includes("timelapse"))
        return "timelapse";
      if (type.includes("audio") || type.includes("wav"))
        return "audio";
      return "clip";
    }

    function formatMediaBrowserCountLabel(count, filter) {
      const label = mediaBrowserFilterLabel(filter);
      return count === 1 ? "1 " + label.replace(/s$/, "") : count + " " + label;
    }

    function setMediaBrowserFilter(filter) {
      const normalized = ["clip", "still", "timelapse", "audio"].includes(filter) ? filter : "clip";
      state.mediaBrowserFilter = normalized;
      document.querySelectorAll(".media-filter-button").forEach((button) => {
        button.classList.toggle("active", button.dataset.mediaFilter === normalized);
      });
      renderMediaLibrary(state.mediaLibraryPayload || { ok: true, mounted: false, items: [] });
    }

    function setMediaBrowserSort(sortMode) {
      state.mediaBrowserSort = sortMode === "oldest" ? "oldest" : "newest";
      const select = document.getElementById("mediaBrowserSortSelect");
      if (select)
        select.value = state.mediaBrowserSort;
      renderMediaLibrary(state.mediaLibraryPayload || { ok: true, mounted: false, items: [] });
    }

    function sortedMediaItems(items) {
      const sorted = Array.isArray(items) ? [...items] : [];
      const direction = state.mediaBrowserSort === "oldest" ? 1 : -1;
      sorted.sort((left, right) => {
        const leftTime = Number(left && left.modified_ms) || 0;
        const rightTime = Number(right && right.modified_ms) || 0;
        if (leftTime !== rightTime)
          return (leftTime - rightTime) * direction;
        return String(left && left.name || "").localeCompare(String(right && right.name || "")) * direction;
      });
      return sorted;
    }

    function mediaFrameCountLabel(item) {
      const frameCount = Number(item && item.frame_count) || 0;
      if (mediaItemCategory(item) === "audio")
        return "WAV";
      if (mediaItemCategory(item) === "still")
        return "1 still";
      if (frameCount === 0) {
        const fileCount = Number(item && item.file_count) || 0;
        return fileCount === 1 ? "1 file" : fileCount + " files";
      }
      return frameCount === 1 ? "1 frame" : frameCount + " frames";
    }

    function mediaFpsLabel(item) {
      if (mediaItemCategory(item) === "audio")
        return String(item && item.fps_text || "Audio").toUpperCase();
      return String(item && item.fps_text || "-- FPS").toUpperCase();
    }

    function mediaSelectionKey(item) {
      if (!item || !item.kind || !item.id)
        return "";
      return String(item.kind) + ":" + String(item.id);
    }

    function selectedMediaItems() {
      const allItems = state.mediaLibraryPayload && Array.isArray(state.mediaLibraryPayload.items)
        ? state.mediaLibraryPayload.items
        : [];
      return allItems.filter((item) => state.mediaSelection.has(mediaSelectionKey(item)));
    }

    function pruneMediaSelection(allItems) {
      const validKeys = new Set((Array.isArray(allItems) ? allItems : []).map(mediaSelectionKey).filter(Boolean));
      for (const key of Array.from(state.mediaSelection)) {
        if (!validKeys.has(key))
          state.mediaSelection.delete(key);
      }
    }

    function updateMediaSelectionControls() {
      const selectedCount = state.mediaSelection.size;
      const visibleCount = Array.isArray(state.mediaLibrary) ? state.mediaLibrary.length : 0;
      setText("mediaSelectionCount", selectedCount === 0
        ? "No items selected"
        : (selectedCount === 1 ? "1 item selected" : selectedCount + " items selected"));

      const selectAllButton = document.getElementById("selectAllMediaButton");
      if (selectAllButton)
        selectAllButton.disabled = visibleCount === 0;

      const clearButton = document.getElementById("clearMediaSelectionButton");
      if (clearButton)
        clearButton.disabled = selectedCount === 0;

      const downloadButton = document.getElementById("downloadSelectedMediaButton");
      if (downloadButton)
        downloadButton.disabled = selectedCount === 0;

      const deleteButton = document.getElementById("deleteSelectedMediaButton");
      if (deleteButton)
        deleteButton.disabled = selectedCount === 0;
    }

    function toggleMediaSelection(item) {
      const key = mediaSelectionKey(item);
      if (!key)
        return;
      if (state.mediaSelection.has(key))
        state.mediaSelection.delete(key);
      else
        state.mediaSelection.add(key);
      renderMediaLibrary(state.mediaLibraryPayload || { ok: true, mounted: false, items: [] });
    }

    function selectAllVisibleMedia() {
      const visibleItems = Array.isArray(state.mediaLibrary) ? state.mediaLibrary : [];
      for (const item of visibleItems) {
        const key = mediaSelectionKey(item);
        if (key)
          state.mediaSelection.add(key);
      }
      renderMediaLibrary(state.mediaLibraryPayload || { ok: true, mounted: false, items: [] });
    }

    function clearMediaSelection() {
      if (state.mediaSelection.size === 0)
        return;
      state.mediaSelection.clear();
      renderMediaLibrary(state.mediaLibraryPayload || { ok: true, mounted: false, items: [] });
    }

    function mediaSelectionQuery(items) {
      return items
        .map((item) => "item=" + encodeURIComponent(mediaSelectionKey(item)))
        .join("&");
    }

    function renderMediaLibrary(payload) {
      const list = document.getElementById("mediaBrowserList");
      if (!list)
        return;

      state.mediaLibraryPayload = payload;
      const allItems = payload && Array.isArray(payload.items) ? payload.items : [];
      const filter = state.mediaBrowserFilter || "clip";
      const items = sortedMediaItems(allItems.filter((item) => mediaItemCategory(item) === filter));
      state.mediaLibrary = items;
      pruneMediaSelection(allItems);
      updateMediaSelectionControls();
      setText("mediaBrowserCountPill", formatMediaBrowserCountLabel(items.length, filter));
      const sortSelect = document.getElementById("mediaBrowserSortSelect");
      if (sortSelect)
        sortSelect.value = state.mediaBrowserSort || "newest";
      document.querySelectorAll(".media-filter-button").forEach((button) => {
        button.classList.toggle("active", button.dataset.mediaFilter === filter);
      });

      list.innerHTML = "";
      if (!payload || !payload.mounted) {
        const empty = document.createElement("div");
        empty.className = "preset-empty";
        empty.textContent = "No media mounted. Mount a drive to browse clips and stills.";
        list.appendChild(empty);
        return;
      }

      if (items.length === 0) {
        const empty = document.createElement("div");
        empty.className = "preset-empty";
        empty.textContent = "No " + mediaBrowserFilterLabel(filter) + " found on this media.";
        list.appendChild(empty);
        return;
      }

      for (const item of items) {
        const selectionKey = mediaSelectionKey(item);
        const selected = state.mediaSelection.has(selectionKey);
        const row = document.createElement("div");
        row.className = "media-row";
        row.classList.toggle("selected", selected);
        row.addEventListener("click", (event) => {
          if (event.target.closest("a, button"))
            return;
          toggleMediaSelection(item);
        });

        const select = document.createElement("button");
        select.className = "media-select-control" + (selected ? " selected" : "");
        select.type = "button";
        select.setAttribute("aria-label", selected ? "Deselect media item" : "Select media item");
        select.setAttribute("aria-pressed", selected ? "true" : "false");
        select.addEventListener("click", (event) => {
          event.stopPropagation();
          toggleMediaSelection(item);
        });
        row.appendChild(select);

        const type = document.createElement("div");
        type.className = "media-row-type";
        type.textContent = item.type || "Media";
        row.appendChild(type);

        const meta = document.createElement("div");
        meta.className = "media-row-meta";

        const name = document.createElement("div");
        name.className = "media-row-name";
        name.textContent = item.name || item.id || "Apertar media";
        meta.appendChild(name);

        const detail = document.createElement("div");
        detail.className = "media-row-detail";
        detail.textContent = "Updated " + (item.modified_text || "Unknown");
        meta.appendChild(detail);

        const stats = document.createElement("div");
        stats.className = "media-row-stats";
        const statValues = [
          mediaFrameCountLabel(item),
          "Duration " + (item.duration_text || "--"),
          mediaFpsLabel(item),
          item.size_text || "--",
        ];
        for (const value of statValues) {
          const stat = document.createElement("span");
          stat.className = "media-stat";
          stat.textContent = value;
          stats.appendChild(stat);
        }
        meta.appendChild(stats);
        row.appendChild(meta);

        const actions = document.createElement("div");
        actions.className = "media-row-actions";

        const download = document.createElement("a");
        download.className = "secondary-button media-row-action";
        download.href = item.download_url || "#";
        download.textContent = "Download";
        if (item.download_url) {
          download.setAttribute("download", "");
          download.addEventListener("click", () => {
            showToast("Downloading " + (item.name || "media") + ".", false);
          });
        } else {
          download.setAttribute("aria-disabled", "true");
        }
        actions.appendChild(download);

        const remove = document.createElement("button");
        remove.className = "danger-button media-row-action";
        remove.type = "button";
        remove.textContent = "Delete";
        remove.addEventListener("click", () => {
          void deleteMediaLibraryItem(item);
        });
        actions.appendChild(remove);

        row.appendChild(actions);
        list.appendChild(row);
      }
    }

    function setMetric(prefix, valueText, sublabel, progress) {
      setText(prefix + "Value", valueText);
      setText(prefix + "Sub", sublabel);
      setText(prefix + "ProgressText", formatPercent(progress));
      const fill = document.getElementById(prefix + "Progress");
      if (fill)
        fill.style.width = clamp(typeof progress === "number" ? progress : 0, 0, 100) + "%";
    }

    function showToast(message, isError) {
      const toast = document.getElementById("toast");
      toast.textContent = message;
      toast.classList.toggle("error", !!isError);
      toast.classList.add("visible");
      clearTimeout(state.toastTimer);
      state.toastTimer = window.setTimeout(() => {
        toast.classList.remove("visible");
      }, isError ? 4600 : 2600);
    }

    function setPreviewOverlay(visible, label, isError) {
      const overlay = document.getElementById("previewOverlay");
      const overlayCard = document.getElementById("previewOverlayCard");
      const smpteActive = !!(state.snapshot && state.snapshot.ui && state.snapshot.ui.smpte_enabled);
      setText("previewOverlayLabel", label);
      overlay.classList.toggle("hidden", !visible || (smpteActive && !isError));
      overlayCard.classList.toggle("error", !!isError);
    }

    function setPreviewSmpteEnabled(enabled) {
      const stage = document.getElementById("previewStage");
      if (!stage)
        return;
      stage.classList.toggle("smpte-active", !!enabled);
    }

    function setFalseColorLegend(mode, visible) {
      const stage = document.getElementById("previewStage");
      const labels = document.getElementById("falseColorLegendLabels");
      const scale = document.getElementById("falseColorLegendScale");
      if (!stage || !labels || !scale)
        return;

      stage.classList.toggle("false-color-legend-active", !!visible);
      if (!visible)
        return;

      const entries = falseColorLegendEntries(mode);
      labels.innerHTML = "";
      scale.innerHTML = "";
      for (const entry of entries) {
        const label = document.createElement("div");
        label.className = "false-color-legend-label";
        label.textContent = entry.label;
        labels.appendChild(label);

        const swatch = document.createElement("div");
        swatch.className = "false-color-legend-swatch";
        swatch.style.background = entry.color;
        scale.appendChild(swatch);
      }
    }

    function setPreviewProcessingEnabled(enabled) {
      const stage = document.getElementById("previewStage");
      if (!stage)
        return;
      stage.classList.toggle("processed-active", !!enabled);
    }

    function previewProcessingEnabled(snapshot) {
      if (!snapshot || !snapshot.ui)
        return false;

      const uiState = snapshot.ui;
      if (uiState.smpte_enabled)
        return false;

      return !!uiState.false_color_enabled
        || !!uiState.zebra_enabled
        || !!uiState.grayscale_enabled
        || !!uiState.focus_peaking_enabled;
    }

    function webPreviewZebraThreshold(uiThreshold) {
      const threshold = clamp(Number(uiThreshold) || 0.70, 0, 1);
      if (threshold >= 0.99)
        return 0.88;
      if (threshold >= 0.95)
        return 0.90;
      return threshold;
    }

    function webPreviewFocusPeakingThreshold(uiThreshold) {
      const threshold = clamp(Number(uiThreshold) || 0.08, 0, 1);
      return clamp((threshold * 1.65) + 0.02, 0.06, 0.45);
    }

    function compilePreviewProcessingShader(gl, type, source) {
      const shader = gl.createShader(type);
      gl.shaderSource(shader, source);
      gl.compileShader(shader);
      if (gl.getShaderParameter(shader, gl.COMPILE_STATUS))
        return shader;

      console.warn("Preview shader compile failed:", gl.getShaderInfoLog(shader) || "");
      gl.deleteShader(shader);
      return null;
    }

    function ensurePreviewProcessingWebGl() {
      const canvas = document.getElementById("previewProcessedCanvas");
      if (!canvas)
        return null;

      if (state.previewProcessingBackend === "2d")
        return null;

      if (state.previewProcessingGl
          && state.previewProcessingProgram
          && state.previewProcessingTexture
          && state.previewProcessingVertexBuffer) {
        return state.previewProcessingGl;
      }

      const gl = canvas.getContext("webgl", {
        alpha: false,
        antialias: false,
        depth: false,
        stencil: false,
        premultipliedAlpha: false,
        preserveDrawingBuffer: false,
      }) || canvas.getContext("experimental-webgl");
      if (!gl) {
        state.previewProcessingBackend = "2d";
        return null;
      }

      const vertexSource = [
        "precision mediump float;",
        "attribute vec2 aPos;",
        "attribute vec2 aUv;",
        "varying vec2 vUv;",
        "void main() {",
        "  vUv = aUv;",
        "  gl_Position = vec4(aPos, 0.0, 1.0);",
        "}",
      ].join("\n");

      const fragmentSource = [
        "#ifdef GL_FRAGMENT_PRECISION_HIGH",
        "precision highp float;",
        "#else",
        "precision mediump float;",
        "#endif",
        "varying vec2 vUv;",
        "uniform sampler2D uTex;",
        "uniform float uTime;",
        "uniform int uZebraEnabled;",
        "uniform float uZebraThreshold;",
        "uniform int uFocusPeakingEnabled;",
        "uniform float uFocusPeakingThreshold;",
        "uniform vec3 uFocusPeakingColor;",
        "uniform int uGrayscaleEnabled;",
        "uniform int uFalseColorEnabled;",
        "uniform int uFalseColorMode;",
        "uniform int uExposureReferenceEnabled;",
        "uniform sampler2D uExposureReferenceTex;",
        "uniform mat3 uExposureTo709;",
        "float exposureLookup(float value,float row) {",
        "  vec2 encodedPair=texture2D(uExposureReferenceTex,vec2((clamp(value,0.0,1.0)*1023.0+0.5)/1024.0,row)).rg;",
        "  return dot(encodedPair,vec2(65280.0/65535.0,255.0/65535.0));",
        "}",
        "vec3 exposureReferenceRgb(vec3 color) {",
        "  vec3 linear=vec3(exposureLookup(color.r,0.25),exposureLookup(color.g,0.25),exposureLookup(color.b,0.25));",
        "  vec3 rgb=uExposureTo709*linear;",
        "  return vec3(exposureLookup(rgb.r,0.75),exposureLookup(rgb.g,0.75),exposureLookup(rgb.b,0.75));",
        "}",
        "uniform vec2 uTexelSize;",
        "float luminance(vec3 c) {",
        "  return dot(c, vec3(0.299, 0.587, 0.114));",
        "}",
        "float peakingLuminance(vec3 color) {",
        "  return luminance(uExposureReferenceEnabled == 1 ? exposureReferenceRgb(color) : color);",
        "}",
        "float edgeStrength(vec2 uv) {",
        "  vec2 step = uTexelSize * 2.0;",
        "  float l = peakingLuminance(texture2D(uTex, uv - vec2(step.x, 0.0)).rgb);",
        "  float r = peakingLuminance(texture2D(uTex, uv + vec2(step.x, 0.0)).rgb);",
        "  float u = peakingLuminance(texture2D(uTex, uv - vec2(0.0, step.y)).rgb);",
        "  float d = peakingLuminance(texture2D(uTex, uv + vec2(0.0, step.y)).rgb);",
        "  return (abs(r - l) + abs(d - u)) * 0.5;",
        "}",
        "vec3 falseColorExposure(float l) {",
        "  if (l > 0.92) return vec3(1.0, 0.0, 0.0);",
        "  if (l > 0.85) return vec3(1.0, 0.5, 0.0);",
        "  if (l > 0.70) return vec3(1.0, 1.0, 0.0);",
        "  if (l > 0.55) return vec3(0.6, 1.0, 0.6);",
        "  if (l > 0.45) return vec3(0.18, 0.18, 0.18);",
        "  if (l > 0.35) return vec3(0.0, 1.0, 0.0);",
        "  if (l > 0.20) return vec3(0.0, 0.5, 1.0);",
        "  if (l > 0.13) return vec3(0.0, 0.0, 1.0);",
        "  return vec3(0.5, 0.0, 0.5);",
        "}",
        "vec3 falseColorSkin(float l) {",
        "  if (l > 0.75) return vec3(1.0, 0.0, 0.0);",
        "  if (l > 0.65) return vec3(1.0, 1.0, 0.0);",
        "  if (l > 0.45) return vec3(0.0, 1.0, 0.0);",
        "  if (l > 0.30) return vec3(0.0, 0.5, 1.0);",
        "  return vec3(0.0, 0.0, 1.0);",
        "}",
        "vec3 falseColorHighlight(float l) {",
        "  if (l > 0.95) return vec3(1.0, 0.0, 0.0);",
        "  if (l > 0.92) return vec3(1.0, 0.3, 0.0);",
        "  if (l > 0.85) return vec3(1.0, 0.7, 0.0);",
        "  if (l > 0.70) return vec3(1.0, 1.0, 0.0);",
        "  return vec3(l);",
        "}",
        "vec3 falseColorShadow(float l) {",
        "  if (l < 0.03) return vec3(0.5, 0.0, 0.5);",
        "  if (l < 0.05) return vec3(0.0, 0.0, 1.0);",
        "  if (l < 0.10) return vec3(0.0, 0.5, 1.0);",
        "  if (l < 0.20) return vec3(0.0, 1.0, 1.0);",
        "  return vec3(l);",
        "}",
        "void main() {",
        "  vec4 src = texture2D(uTex, vUv);",
        "  float luma = luminance(src.rgb);",
        "  vec3 exposureRgb=src.rgb;",
        "  if (uExposureReferenceEnabled==1 && (uZebraEnabled==1 || uFalseColorEnabled==1)) exposureRgb=exposureReferenceRgb(src.rgb);",
        "  float exposureLuma=luminance(exposureRgb);",
        "  vec4 outColor = src;",
        "  if (uFalseColorEnabled == 1) {",
        "    if (uFalseColorMode == 0)",
        "      outColor.rgb = falseColorExposure(exposureLuma);",
        "    else if (uFalseColorMode == 1)",
        "      outColor.rgb = falseColorSkin(exposureLuma);",
        "    else if (uFalseColorMode == 2)",
        "      outColor.rgb = falseColorHighlight(exposureLuma);",
        "    else if (uFalseColorMode == 3)",
        "      outColor.rgb = falseColorShadow(exposureLuma);",
        "  } else if (uGrayscaleEnabled == 1) {",
        "    outColor.rgb = vec3(luma);",
        "  }",
        "  float zebraMetric = exposureLuma;",
        "  if (uZebraThreshold >= 0.95) {",
        "    zebraMetric = max(max(exposureRgb.r, exposureRgb.g), exposureRgb.b);",
        "  }",
        "  if (uZebraEnabled == 1 && zebraMetric >= uZebraThreshold) {",
        "    float stripe = fract((vUv.x + vUv.y) * 40.0 + uTime * 0.8);",
        "    vec3 zebraColor = (stripe < 0.5) ? vec3(0.03) : vec3(1.0);",
        "    outColor.rgb = mix(outColor.rgb, zebraColor, 0.72);",
        "  }",
        "  if (uFocusPeakingEnabled == 1) {",
        "    float edge = edgeStrength(vUv);",
        "    if (edge > uFocusPeakingThreshold) {",
        "      outColor.rgb = mix(outColor.rgb, uFocusPeakingColor, 0.88);",
        "    }",
        "  }",
        "  gl_FragColor = outColor;",
        "}",
      ].join("\n");

      const vertexShader = compilePreviewProcessingShader(gl, gl.VERTEX_SHADER, vertexSource);
      const fragmentShader = compilePreviewProcessingShader(gl, gl.FRAGMENT_SHADER, fragmentSource);
      if (!vertexShader || !fragmentShader) {
        if (vertexShader)
          gl.deleteShader(vertexShader);
        if (fragmentShader)
          gl.deleteShader(fragmentShader);
        state.previewProcessingBackend = "2d";
        return null;
      }

      const program = gl.createProgram();
      gl.attachShader(program, vertexShader);
      gl.attachShader(program, fragmentShader);
      gl.linkProgram(program);
      gl.deleteShader(vertexShader);
      gl.deleteShader(fragmentShader);

      if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
        console.warn("Preview shader link failed:", gl.getProgramInfoLog(program) || "");
        gl.deleteProgram(program);
        state.previewProcessingBackend = "2d";
        return null;
      }

      const texture = gl.createTexture();
      const vertexBuffer = gl.createBuffer();
      if (!texture || !vertexBuffer) {
        if (texture)
          gl.deleteTexture(texture);
        if (vertexBuffer)
          gl.deleteBuffer(vertexBuffer);
        gl.deleteProgram(program);
        state.previewProcessingBackend = "2d";
        return null;
      }

      gl.bindTexture(gl.TEXTURE_2D, texture);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
      gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);

      state.previewProcessingBackend = "webgl";
      state.previewProcessingGl = gl;
      state.previewExposureTexture = null;
      state.previewExposureTextureKey = "";
      state.previewProcessingProgram = program;
      state.previewProcessingTexture = texture;
      state.previewProcessingVertexBuffer = vertexBuffer;
      state.previewProcessingPositionLoc = gl.getAttribLocation(program, "aPos");
      state.previewProcessingUvLoc = gl.getAttribLocation(program, "aUv");
      state.previewProcessingUniforms = {
        tex: gl.getUniformLocation(program, "uTex"),
        time: gl.getUniformLocation(program, "uTime"),
        zebraEnabled: gl.getUniformLocation(program, "uZebraEnabled"),
        zebraThreshold: gl.getUniformLocation(program, "uZebraThreshold"),
        focusPeakingEnabled: gl.getUniformLocation(program, "uFocusPeakingEnabled"),
        focusPeakingThreshold: gl.getUniformLocation(program, "uFocusPeakingThreshold"),
        focusPeakingColor: gl.getUniformLocation(program, "uFocusPeakingColor"),
        grayscaleEnabled: gl.getUniformLocation(program, "uGrayscaleEnabled"),
        falseColorEnabled: gl.getUniformLocation(program, "uFalseColorEnabled"),
        falseColorMode: gl.getUniformLocation(program, "uFalseColorMode"),
        exposureEnabled: gl.getUniformLocation(program, "uExposureReferenceEnabled"),
        exposureTexture: gl.getUniformLocation(program, "uExposureReferenceTex"),
        exposureMatrix: gl.getUniformLocation(program, "uExposureTo709"),
        texelSize: gl.getUniformLocation(program, "uTexelSize"),
      };

      return gl;
    }

    function previewExposureReference(snapshot) {
      const reference = snapshot && snapshot.preview_exposure_reference;
      if (!reference) return null;
      if (state.previewExposureObject === reference) return state.previewExposureData;
      state.previewExposureObject = reference;
      const key = JSON.stringify(reference);
      if (state.previewExposureKey === key) return state.previewExposureData;
      state.previewExposureKey = key;
      state.previewExposureData = null;
      const validCurve = points => Array.isArray(points) && points.length >= 2 && points.length <= 1000
        && points.every((p, i) => Array.isArray(p) && p.length === 2 && p.every(Number.isFinite)
          && (!i || p[0] > points[i - 1][0]));
      if (!validCurve(reference.decode_curve) || !validCurve(reference.display_curve)
          || !Array.isArray(reference.to_rec709) || reference.to_rec709.length !== 9
          || !reference.to_rec709.every(Number.isFinite)) return null;
      const sample = (points, x) => {
        if (x <= points[0][0]) return points[0][1];
        let i = 1;
        while (i < points.length && points[i][0] < x) ++i;
        if (i === points.length) return points[i - 1][1];
        const a = points[i - 1], b = points[i];
        return a[1] + (x - a[0]) / (b[0] - a[0]) * (b[1] - a[1]);
      };
      const decode = new Float32Array(1024), display = new Float32Array(1024);
      const pixels = new Uint8Array(1024 * 2 * 4), matrix = new Float32Array(9);
      for (let row = 0; row < 2; ++row) for (let i = 0; i < 1024; ++i) {
        const value = Math.round(clamp(sample(row ? reference.display_curve : reference.decode_curve, i / 1023), 0, 1) * 65535);
        (row ? display : decode)[i] = value / 65535;
        const offset = (row * 1024 + i) * 4;
        pixels[offset] = value >> 8; pixels[offset + 1] = value & 255; pixels[offset + 3] = 255;
      }
      for (let r = 0; r < 3; ++r) for (let c = 0; c < 3; ++c) matrix[c * 3 + r] = reference.to_rec709[r * 3 + c];
      return state.previewExposureData = { decode, display, pixels, matrix, rows: reference.to_rec709, key };
    }

    function previewExposureLookup(table, value) {
      const x = clamp(value, 0, 1) * 1023, i = Math.floor(x), j = Math.min(i + 1, 1023);
      return table[i] + (table[j] - table[i]) * (x - i);
    }

    function previewExposureRgb(red, green, blue, reference, output) {
      const r = previewExposureLookup(reference.decode, red), g = previewExposureLookup(reference.decode, green), b = previewExposureLookup(reference.decode, blue);
      const m = reference.rows;
      output[0] = previewExposureLookup(reference.display, m[0] * r + m[1] * g + m[2] * b);
      output[1] = previewExposureLookup(reference.display, m[3] * r + m[4] * g + m[5] * b);
      output[2] = previewExposureLookup(reference.display, m[6] * r + m[7] * g + m[8] * b);
    }

    function bindPreviewExposureReference(gl, uniforms, snapshot, needed) {
      const reference = needed ? previewExposureReference(snapshot) : null;
      if (!state.previewExposureTexture) {
        state.previewExposureTexture = gl.createTexture();
        gl.activeTexture(gl.TEXTURE1);
        gl.bindTexture(gl.TEXTURE_2D, state.previewExposureTexture);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, 1, 1, 0, gl.RGBA, gl.UNSIGNED_BYTE, new Uint8Array([0, 0, 0, 255]));
      }
      gl.activeTexture(gl.TEXTURE1);
      gl.bindTexture(gl.TEXTURE_2D, state.previewExposureTexture);
      if (reference && state.previewExposureTextureKey !== reference.key) {
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, 1024, 2, 0, gl.RGBA, gl.UNSIGNED_BYTE, reference.pixels);
        state.previewExposureTextureKey = reference.key;
      }
      gl.uniform1i(uniforms.exposureEnabled, reference ? 1 : 0);
      gl.uniform1i(uniforms.exposureTexture, 1);
      if (reference) gl.uniformMatrix3fv(uniforms.exposureMatrix, false, reference.matrix);
    }

    function ensurePreviewProcessingContext() {
      const canvas = document.getElementById("previewProcessedCanvas");
      if (!canvas)
        return null;

      if (!state.previewProcessingContext) {
        state.previewProcessingContext = canvas.getContext("2d", { willReadFrequently: true });
        if (state.previewProcessingContext && !state.previewProcessingBackend)
          state.previewProcessingBackend = "2d";
      }

      return state.previewProcessingContext;
    }

    function resizePreviewProcessingCanvas(canvas, width, height) {
      if (!canvas)
        return;

      const safeWidth = Math.max(1, Math.round(width));
      const safeHeight = Math.max(1, Math.round(height));
      if (state.previewProcessingCanvasWidth === safeWidth
          && state.previewProcessingCanvasHeight === safeHeight
          && canvas.width === safeWidth
          && canvas.height === safeHeight) {
        return;
      }

      canvas.width = safeWidth;
      canvas.height = safeHeight;
      state.previewProcessingCanvasWidth = safeWidth;
      state.previewProcessingCanvasHeight = safeHeight;
    }

    function drawPreviewImageCover(ctx, image, targetWidth, targetHeight) {
      const sourceWidth = Number(image.naturalWidth || image.videoWidth || image.width || 0);
      const sourceHeight = Number(image.naturalHeight || image.videoHeight || image.height || 0);
      if (!sourceWidth || !sourceHeight || !targetWidth || !targetHeight)
        return false;

      const scale = Math.max(targetWidth / sourceWidth, targetHeight / sourceHeight);
      const drawWidth = sourceWidth * scale;
      const drawHeight = sourceHeight * scale;
      const offsetX = (targetWidth - drawWidth) * 0.5;
      const offsetY = (targetHeight - drawHeight) * 0.5;

      ctx.clearRect(0, 0, targetWidth, targetHeight);
      ctx.drawImage(image, offsetX, offsetY, drawWidth, drawHeight);
      return true;
    }

    function focusPeakingColorVec(label) {
      if (label === "Green") return [0.1, 1.0, 0.1];
      if (label === "Blue") return [63 / 255, 208 / 255, 1.0];
      if (label === "Yellow") return [1.0, 1.0, 0.1];
      if (label === "Pink") return [1.0, 0.2, 0.7];
      return [1.0, 0.1, 0.1];
    }

    function previewImageUvRect(image, targetWidth, targetHeight) {
      const sourceWidth = Number(image.naturalWidth || image.videoWidth || image.width || 0);
      const sourceHeight = Number(image.naturalHeight || image.videoHeight || image.height || 0);
      if (!sourceWidth || !sourceHeight || !targetWidth || !targetHeight) {
        return { u0: 0, v0: 0, u1: 1, v1: 1 };
      }

      const sourceAspect = sourceWidth / sourceHeight;
      const targetAspect = targetWidth / targetHeight;
      let u0 = 0.0;
      let v0 = 0.0;
      let u1 = 1.0;
      let v1 = 1.0;

      if (sourceAspect > targetAspect) {
        const visibleWidth = targetAspect / sourceAspect;
        const inset = (1.0 - visibleWidth) * 0.5;
        u0 = inset;
        u1 = 1.0 - inset;
      } else if (sourceAspect < targetAspect) {
        const visibleHeight = sourceAspect / targetAspect;
        const inset = (1.0 - visibleHeight) * 0.5;
        v0 = inset;
        v1 = 1.0 - inset;
      }

      return { u0, v0, u1, v1 };
    }

    function falseColorExposureRgb(luma) {
      if (luma > 0.92) return 0xff0000;
      if (luma > 0.85) return 0xff8000;
      if (luma > 0.70) return 0xffff00;
      if (luma > 0.55) return 0x99ff99;
      if (luma > 0.45) return 0x2e2e2e;
      if (luma > 0.35) return 0x00ff00;
      if (luma > 0.20) return 0x0080ff;
      if (luma > 0.13) return 0x0000ff;
      return 0x800080;
    }

    function falseColorSkinRgb(luma) {
      if (luma > 0.75) return 0xff0000;
      if (luma > 0.65) return 0xffff00;
      if (luma > 0.45) return 0x00ff00;
      if (luma > 0.30) return 0x0080ff;
      return 0x0000ff;
    }

    function falseColorHighlightRgb(luma) {
      if (luma > 0.95) return 0xff0000;
      if (luma > 0.92) return 0xff4d00;
      if (luma > 0.85) return 0xffb300;
      if (luma > 0.70) return 0xffff00;
      const gray = clamp(Math.round(luma * 255), 0, 255);
      return (gray << 16) | (gray << 8) | gray;
    }

    function falseColorShadowRgb(luma) {
      if (luma < 0.03) return 0x800080;
      if (luma < 0.05) return 0x0000ff;
      if (luma < 0.10) return 0x0080ff;
      if (luma < 0.20) return 0x00ffff;
      const gray = clamp(Math.round(luma * 255), 0, 255);
      return (gray << 16) | (gray << 8) | gray;
    }

    function falseColorRgb(luma, mode) {
      if (mode === 1)
        return falseColorSkinRgb(luma);
      if (mode === 2)
        return falseColorHighlightRgb(luma);
      if (mode === 3)
        return falseColorShadowRgb(luma);
      return falseColorExposureRgb(luma);
    }

    function renderProcessedPreviewFrameWebGl(canvas, preview, snapshot, now) {
      const gl = ensurePreviewProcessingWebGl();
      if (!gl || !state.previewProcessingProgram || !state.previewProcessingTexture || !state.previewProcessingVertexBuffer)
        return false;

      const uiState = snapshot && snapshot.ui ? snapshot.ui : {};
      const falseColorEnabled = !!uiState.false_color_enabled;
      const falseColorMode = clamp(Number(uiState.false_color_mode) || 0, 0, 3);
      const grayscaleEnabled = !falseColorEnabled && !!uiState.grayscale_enabled;
      const zebraEnabled = !!uiState.zebra_enabled;
      const zebraThreshold = webPreviewZebraThreshold(uiState.zebra_threshold);
      const focusPeakingEnabled = !!uiState.focus_peaking_enabled;
      const focusPeakingThreshold = webPreviewFocusPeakingThreshold(uiState.focus_peaking_threshold);
      const focusPeakingColor = focusPeakingColorVec(uiState.focus_peaking_color || "Red");
      const sourceWidth = Math.max(1, Number(preview.naturalWidth || preview.width || 1));
      const sourceHeight = Math.max(1, Number(preview.naturalHeight || preview.height || 1));
      const uv = previewImageUvRect(preview, canvas.width, canvas.height);
      const quad = new Float32Array([
        -1.0, -1.0, uv.u0, uv.v1,
         1.0, -1.0, uv.u1, uv.v1,
        -1.0,  1.0, uv.u0, uv.v0,
         1.0,  1.0, uv.u1, uv.v0,
      ]);

      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(0.0, 0.0, 0.0, 1.0);
      gl.clear(gl.COLOR_BUFFER_BIT);
      gl.useProgram(state.previewProcessingProgram);

      gl.bindBuffer(gl.ARRAY_BUFFER, state.previewProcessingVertexBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, quad, gl.STREAM_DRAW);

      gl.enableVertexAttribArray(state.previewProcessingPositionLoc);
      gl.vertexAttribPointer(state.previewProcessingPositionLoc, 2, gl.FLOAT, false, 16, 0);
      gl.enableVertexAttribArray(state.previewProcessingUvLoc);
      gl.vertexAttribPointer(state.previewProcessingUvLoc, 2, gl.FLOAT, false, 16, 8);

      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, state.previewProcessingTexture);
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, preview);

      const uniforms = state.previewProcessingUniforms;
      gl.uniform1i(uniforms.tex, 0);
      gl.uniform1f(uniforms.time, now / 1000.0);
      gl.uniform1i(uniforms.zebraEnabled, zebraEnabled ? 1 : 0);
      gl.uniform1f(uniforms.zebraThreshold, zebraThreshold);
      gl.uniform1i(uniforms.focusPeakingEnabled, focusPeakingEnabled ? 1 : 0);
      gl.uniform1f(uniforms.focusPeakingThreshold, focusPeakingThreshold);
      gl.uniform3f(uniforms.focusPeakingColor, focusPeakingColor[0], focusPeakingColor[1], focusPeakingColor[2]);
      gl.uniform1i(uniforms.grayscaleEnabled, grayscaleEnabled ? 1 : 0);
      gl.uniform1i(uniforms.falseColorEnabled, falseColorEnabled ? 1 : 0);
      gl.uniform1i(uniforms.falseColorMode, falseColorMode);
      gl.uniform2f(uniforms.texelSize, 1.0 / sourceWidth, 1.0 / sourceHeight);
      bindPreviewExposureReference(gl, uniforms, snapshot, zebraEnabled || falseColorEnabled || focusPeakingEnabled);

      gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
      return true;
    }

    function renderProcessedPreviewFrame() {
      const snapshot = state.snapshot;
      const canvas = document.getElementById("previewProcessedCanvas");
      const preview = document.getElementById("previewStream");
      const stage = document.getElementById("previewStage");
      if (!canvas || !preview || !stage) {
        return;
      }

      const processingEnabled = previewProcessingEnabled(snapshot);
      setPreviewProcessingEnabled(processingEnabled);
      if (!processingEnabled) {
        state.previewPeakingLuma = null;
        state.previewProcessingLastFrameMs = 0;
        return;
      }

      if (!preview.complete || !preview.naturalWidth || !preview.naturalHeight) {
        return;
      }

      const now = performance.now();
      const targetFps = clamp(currentFpsInt(snapshot), 1, 30);
      const targetIntervalMs = 1000 / targetFps;
      if (state.previewProcessingLastFrameMs > 0
          && (now - state.previewProcessingLastFrameMs) < targetIntervalMs) {
        return;
      }

      const rawWidth = Math.max(1, Math.round(stage.clientWidth || preview.clientWidth || 1));
      const rawHeight = Math.max(1, Math.round(stage.clientHeight || preview.clientHeight || 1));
      const processScale = Math.min(1, 1280 / rawWidth, 720 / rawHeight);
      const targetWidth = Math.max(1, Math.round(rawWidth * processScale));
      const targetHeight = Math.max(1, Math.round(rawHeight * processScale));
      resizePreviewProcessingCanvas(canvas, targetWidth, targetHeight);

      if (renderProcessedPreviewFrameWebGl(canvas, preview, snapshot, now)) {
        state.previewPeakingLuma = null;
        state.previewProcessingLastFrameMs = now;
        return;
      }

      const ctx = ensurePreviewProcessingContext();
      if (!ctx) {
        return;
      }

      if (!drawPreviewImageCover(ctx, preview, canvas.width, canvas.height)) {
        return;
      }

      const uiState = snapshot.ui || {};
      const falseColorEnabled = !!uiState.false_color_enabled;
      const falseColorMode = clamp(Number(uiState.false_color_mode) || 0, 0, 3);
      const grayscaleEnabled = !falseColorEnabled && !!uiState.grayscale_enabled;
      const zebraEnabled = !!uiState.zebra_enabled;
      const zebraThreshold = webPreviewZebraThreshold(uiState.zebra_threshold);
      const focusPeakingEnabled = !!uiState.focus_peaking_enabled;
      const focusPeakingThreshold = webPreviewFocusPeakingThreshold(uiState.focus_peaking_threshold);
      const focusPeakingColor = focusPeakingColorVec(uiState.focus_peaking_color || "Red");
      const focusPeakingColorRgb = [
        Math.round(focusPeakingColor[0] * 255),
        Math.round(focusPeakingColor[1] * 255),
        Math.round(focusPeakingColor[2] * 255),
      ];
      const zebraMix = 0.72;
      const zebraDark = 8;
      const zebraLight = 255;
      const focusPeakingMix = 0.88;
      const phase = Math.floor(performance.now() * 0.04);

      const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
      const data = imageData.data;
      const width = canvas.width;
      const height = canvas.height;

      const exposureReference = (zebraEnabled || falseColorEnabled || focusPeakingEnabled) ? previewExposureReference(snapshot) : null;
      const exposureRgb = new Float32Array(3);
      // Freeze source luminance before overlays change data in place. Reuse the
      // buffer and one reference conversion per pixel, not four per edge.
      let peakingLuma = null;
      if (focusPeakingEnabled) {
        if (!state.previewPeakingLuma || state.previewPeakingLuma.length !== width * height)
          state.previewPeakingLuma = new Float32Array(width * height);
        peakingLuma = state.previewPeakingLuma;
        for (let p = 0; p < peakingLuma.length; ++p) {
          const i = p * 4;
          let r = data[i] / 255, g = data[i + 1] / 255, b = data[i + 2] / 255;
          if (exposureReference) {
            previewExposureRgb(r, g, b, exposureReference, exposureRgb);
            r = exposureRgb[0]; g = exposureRgb[1]; b = exposureRgb[2];
          }
          peakingLuma[p] = r * 0.299 + g * 0.587 + b * 0.114;
        }
      } else {
        state.previewPeakingLuma = null;
      }

      for (let y = 0; y < height; y += 1) {
        const rowOffset = y * width * 4;
        for (let x = 0; x < width; x += 1) {
          const index = rowOffset + (x * 4);
          let red = data[index];
          let green = data[index + 1];
          let blue = data[index + 2];

          const luma = ((red * 0.299) + (green * 0.587) + (blue * 0.114)) / 255;
          let exposureLuma = luma;
          let exposurePeak = Math.max(red, green, blue) / 255;
          if (exposureReference && (zebraEnabled || falseColorEnabled)) {
            previewExposureRgb(red / 255, green / 255, blue / 255, exposureReference, exposureRgb);
            exposureLuma = exposureRgb[0] * 0.299 + exposureRgb[1] * 0.587 + exposureRgb[2] * 0.114;
            exposurePeak = Math.max(exposureRgb[0], exposureRgb[1], exposureRgb[2]);
          }
          let edgeStrength = 0;
          if (peakingLuma) {
            const leftLuma = peakingLuma[y * width + Math.max(0, x - 2)];
            const rightLuma = peakingLuma[y * width + Math.min(width - 1, x + 2)];
            const upLuma = peakingLuma[Math.max(0, y - 2) * width + x];
            const downLuma = peakingLuma[Math.min(height - 1, y + 2) * width + x];
            edgeStrength = (Math.abs(rightLuma - leftLuma) + Math.abs(downLuma - upLuma)) * 0.5;
          }

          if (falseColorEnabled) {
            const mapped = falseColorRgb(exposureLuma, falseColorMode);
            red = (mapped >> 16) & 0xff;
            green = (mapped >> 8) & 0xff;
            blue = mapped & 0xff;
          } else if (grayscaleEnabled) {
            const gray = clamp(Math.round(luma * 255), 0, 255);
            red = gray;
            green = gray;
            blue = gray;
          }

          let zebraMetric = exposureLuma;
          if (zebraThreshold >= 0.95)
            zebraMetric = exposurePeak;

          if (zebraEnabled && zebraMetric >= zebraThreshold) {
            const stripeOn = (((x + y + phase) >> 3) & 1) === 0;
            const zebraValue = stripeOn ? zebraDark : zebraLight;
            red = Math.round((red * (1 - zebraMix)) + (zebraValue * zebraMix));
            green = Math.round((green * (1 - zebraMix)) + (zebraValue * zebraMix));
            blue = Math.round((blue * (1 - zebraMix)) + (zebraValue * zebraMix));
          }

          if (focusPeakingEnabled && edgeStrength > focusPeakingThreshold) {
            red = Math.round((red * (1 - focusPeakingMix)) + (focusPeakingColorRgb[0] * focusPeakingMix));
            green = Math.round((green * (1 - focusPeakingMix)) + (focusPeakingColorRgb[1] * focusPeakingMix));
            blue = Math.round((blue * (1 - focusPeakingMix)) + (focusPeakingColorRgb[2] * focusPeakingMix));
          }

          data[index] = red;
          data[index + 1] = green;
          data[index + 2] = blue;
        }
      }

      ctx.putImageData(imageData, 0, 0);
      state.previewProcessingLastFrameMs = now;
    }

    function loadPreviewStream() {
      const preview = document.getElementById("previewStream");
      clearTimeout(state.previewRetryTimer);
      state.previewStreamNonce += 1;
      preview.src = "/preview.mjpeg?stream=" + state.previewStreamNonce;
    }

    function installPreviewStream() {
      if (state.previewStreamStarted)
        return;

      const preview = document.getElementById("previewStream");
      preview.addEventListener("load", () => {
        clearTimeout(state.previewRetryTimer);
        setPreviewOverlay(false, "", false);
      });
      preview.addEventListener("error", () => {
        setPreviewOverlay(true, "Preview unavailable. Retrying...", true);
        clearTimeout(state.previewRetryTimer);
        state.previewRetryTimer = window.setTimeout(loadPreviewStream, 1500);
      });

      state.previewStreamStarted = true;
      setPreviewOverlay(true, "Waiting for live preview", false);
      loadPreviewStream();
    }

    function scheduleStateRefresh(delayMs, silent) {
      const safeDelay = Math.max(0, Number(delayMs) || 0);
      clearTimeout(state.fetchRefreshTimer);
      state.fetchRefreshTimer = window.setTimeout(() => {
        state.fetchRefreshTimer = null;
        void fetchState(silent);
      }, safeDelay);
    }

    function mergeCameraStateFields(snapshot, source) {
      if (!snapshot || !source || typeof source !== "object")
        return;

      if (typeof source.state === "string")
        snapshot.state = source.state;
      if (typeof source.resolution === "string")
        snapshot.resolution = source.resolution;
      if (isFiniteNumber(source.width))
        snapshot.width = Number(source.width);
      if (isFiniteNumber(source.height))
        snapshot.height = Number(source.height);
      if (isFiniteNumber(source.fps))
        snapshot.fps = Number(source.fps);
      if (isFiniteNumber(source.iso))
        snapshot.iso = Math.round(Number(source.iso));
      if (typeof source.iso_auto === "boolean")
        snapshot.iso_auto = source.iso_auto;
      if (isFiniteNumber(source.shutter_us))
        snapshot.shutter_us = Math.max(1, Math.round(Number(source.shutter_us)));
      if (typeof source.shutter_auto === "boolean")
        snapshot.shutter_auto = source.shutter_auto;
      if (isFiniteNumber(source.exposure_compensation))
        snapshot.exposure_compensation = Number(source.exposure_compensation);
      if (typeof source.exposure_compensation_available === "boolean")
        snapshot.exposure_compensation_available = source.exposure_compensation_available;
      if (typeof source.exposure_metering === "string")
        snapshot.exposure_metering = source.exposure_metering;
      if (Array.isArray(source.exposure_metering_modes))
        snapshot.exposure_metering_modes = source.exposure_metering_modes;
      if (isFiniteNumber(source.wb_kelvin))
        snapshot.wb_kelvin = Math.round(Number(source.wb_kelvin));
      if (typeof source.recording_format === "string") {
        snapshot.recording_format = source.recording_format;
        if (snapshot.capture)
          snapshot.capture.codec = captureCodecLabel(source.recording_format);
      }
      if (typeof source.cdng_compression === "string")
        snapshot.cdng_compression = source.cdng_compression;
      if (source.recording_capabilities)
        snapshot.recording_capabilities = source.recording_capabilities;
      if (typeof source.lossy_dng_ratio === "string")
        snapshot.lossy_dng_ratio = source.lossy_dng_ratio === "5to1" ? "5to1" : "3to1";
      if (typeof source.recording_format === "string" && snapshot.capture)
        snapshot.capture.codec = captureCodecLabel(source.recording_format);
      if (isFiniteNumber(source.raw_bit_depth))
        snapshot.raw_bit_depth = rawBitDepthNumber(source.raw_bit_depth);
      if (isFiniteNumber(source.h264_width))
        snapshot.h264_width = Math.max(0, Math.round(Number(source.h264_width)));
      if (isFiniteNumber(source.h264_height))
        snapshot.h264_height = Math.max(0, Math.round(Number(source.h264_height)));
      if (typeof source.timecode_mode === "string")
        snapshot.timecode_mode = source.timecode_mode;
      if (typeof source.af_mode === "string")
        snapshot.af_mode = source.af_mode;
      if (typeof source.af_range === "string")
        snapshot.af_range = source.af_range;
      if (typeof source.af_speed === "string")
        snapshot.af_speed = source.af_speed;
      if (isFiniteNumber(source.af_window_x))
        snapshot.af_window_x = Number(source.af_window_x);
      if (isFiniteNumber(source.af_window_y))
        snapshot.af_window_y = Number(source.af_window_y);
      if (isFiniteNumber(source.af_window_w))
        snapshot.af_window_w = Number(source.af_window_w);
      if (isFiniteNumber(source.af_window_h))
        snapshot.af_window_h = Number(source.af_window_h);
      if (typeof source.recording === "boolean")
        snapshot.recording = source.recording;
      if (typeof source.media_writes_pending === "boolean")
        snapshot.media_writes_pending = source.media_writes_pending;
      if (typeof source.timelapse_active === "boolean")
        snapshot.timelapse_active = source.timelapse_active;
      if (isFiniteNumber(source.timelapse_shot_count))
        snapshot.timelapse_shot_count = Math.max(0, Math.floor(Number(source.timelapse_shot_count)));
      if (isFiniteNumber(source.timelapse_interval_seconds))
        snapshot.timelapse_interval_seconds = Math.max(0, Number(source.timelapse_interval_seconds));
      if (isFiniteNumber(source.interval_seconds))
        snapshot.timelapse_interval_seconds = Math.max(0, Number(source.interval_seconds));
      if (isFiniteNumber(source.recording_elapsed_seconds))
        snapshot.recording_elapsed_seconds = Number(source.recording_elapsed_seconds);
    }

    function applyCommandSnapshotPatch(payload, result, optimistic) {
      const snapshot = ensureSnapshotShape(state.snapshot);
      if (!snapshot || !payload || typeof payload !== "object")
        return false;

      mergeCameraStateFields(snapshot, result);
      if (result && result.state && typeof result.state === "object")
        mergeCameraStateFields(snapshot, result.state);

      const uiState = snapshot.ui;
      const systemState = snapshot.system;
      const imuState = snapshot.imu;
      const cmd = String(payload.cmd || "");
      const boolValue = asToggleBool(payload.enabled);

      if (result && result.imu && typeof result.imu === "object")
        Object.assign(imuState, result.imu);

      switch (cmd) {
        case "set_iso":
          if (isFiniteNumber(payload.value)) {
            snapshot.iso = Math.round(Number(payload.value));
            snapshot.iso_auto = false;
          }
          break;
        case "set_iso_auto":
          snapshot.iso_auto = true;
          break;
        case "set_fps":
          if (isFiniteNumber(payload.value))
            snapshot.fps = Number(payload.value);
          break;
        case "set_shutter_us":
          if (isFiniteNumber(payload.value)) {
            snapshot.shutter_us = Math.max(1, Math.round(Number(payload.value)));
            snapshot.shutter_auto = false;
          }
          break;
        case "set_shutter_auto":
          snapshot.shutter_auto = true;
          break;
        case "set_wb":
          if (isFiniteNumber(payload.kelvin)) {
            snapshot.wb_kelvin = Math.round(Number(payload.kelvin));
            snapshot.wb_auto = false;
          }
          break;
        case "set_wb_auto":
          snapshot.wb_auto = true;
          break;
        case "set_resolution":
          if (isFiniteNumber(payload.width) && isFiniteNumber(payload.height)) {
            snapshot.width = Math.round(Number(payload.width));
            snapshot.height = Math.round(Number(payload.height));
            snapshot.resolution = normalizedResolutionForSnapshot(
              snapshot,
              formatResolutionValue(snapshot.width, snapshot.height)
            );
          }
          break;
        case "set_record_format":
          snapshot.recording_format = recordingFormatCommandValue(payload.value);
          snapshot.capture.codec = captureCodecLabel(snapshot.recording_format);
          if (fixedTenBitRecordingFormat(snapshot.recording_format, snapshot))
            snapshot.raw_bit_depth = 10;
          applyForcedH264CaptureResolution(snapshot);
          break;
        case "set_cdng_compression":
          snapshot.cdng_compression = payload.value;
          break;
        case "set_prores_profile":
          snapshot.prores_profile = payload.value;
          snapshot.capture.codec = captureCodecLabel(snapshot.recording_format);
          break;
        case "set_prores_color_profile":
          snapshot.prores_color_profile = payload.value;
          break;
        case "set_cdng_preview_color_profile":
          snapshot.cdng_preview_color_profile = payload.value;
          break;
        case "set_lossy_dng_ratio":
          snapshot.lossy_dng_ratio = payload.value === "5to1" ? "5to1" : "3to1";
          snapshot.capture.codec = captureCodecLabel(snapshot.recording_format);
          break;
        case "set_raw_bit_depth":
          snapshot.raw_bit_depth = fixedTenBitRecordingFormat(snapshot.recording_format, snapshot)
            ? 10
            : rawBitDepthNumber(payload.value);
          snapshot.resolution = normalizedResolutionForSnapshot(
            snapshot,
            snapshot.resolution || formatResolutionValue(snapshot.width, snapshot.height)
          );
          {
            const parts = String(snapshot.resolution || "").split("x");
            if (parts.length === 2) {
              snapshot.width = Math.max(0, Math.round(Number(parts[0]) || 0));
              snapshot.height = Math.max(0, Math.round(Number(parts[1]) || 0));
            }
          }
          break;
        case "set_h264_resolution":
          if (isFiniteNumber(payload.width) && isFiniteNumber(payload.height)) {
            snapshot.h264_width = Math.max(0, Math.round(Number(payload.width)));
            snapshot.h264_height = Math.max(0, Math.round(Number(payload.height)));
          }
          break;
        case "set_timecode_mode":
          snapshot.timecode_mode = payload.value === "rec_run" ? "rec_run" : "free_run";
          uiState.timecode_mode = timecodeModeLabel(snapshot.timecode_mode);
          break;
        case "set_af_mode":
          snapshot.af_mode = String(payload.value || "continuous").toLowerCase();
          break;
        case "set_af_range":
          snapshot.af_range = String(payload.value || "normal").toLowerCase();
          break;
        case "set_af_speed":
          snapshot.af_speed = String(payload.value || "normal").toLowerCase();
          break;
        case "set_af_window":
          if (isFiniteNumber(payload.x))
            snapshot.af_window_x = Number(payload.x);
          if (isFiniteNumber(payload.y))
            snapshot.af_window_y = Number(payload.y);
          if (isFiniteNumber(payload.width))
            snapshot.af_window_w = Number(payload.width);
          if (isFiniteNumber(payload.height))
            snapshot.af_window_h = Number(payload.height);
          break;
        case "set_photo_mode_enabled":
          uiState.photo_mode_enabled = boolValue;
          if (boolValue) {
            uiState.sound_recorder_enabled = false;
          } else {
            uiState.photo_timelapse = "Off";
          }
          break;
        case "set_sound_recorder_enabled":
          uiState.sound_recorder_enabled = boolValue;
          if (boolValue) {
            uiState.photo_mode_enabled = false;
            uiState.photo_timelapse = "Off";
            snapshot.timelapse_active = false;
          }
          break;
        case "set_photo_timer":
          uiState.photo_timer = String(payload.value || "Off");
          break;
        case "set_photo_long_exposure_enabled":
          uiState.photo_long_exposure_enabled = boolValue;
          break;
        case "set_photo_bracketing_enabled":
          uiState.photo_bracketing_enabled = boolValue;
          break;
        case "set_photo_bracketing_ev":
          uiState.photo_bracketing_ev = Math.round(Number(payload.value) * 3) / 3;
          break;
        case "set_photo_burst":
          uiState.photo_burst = String(payload.value || "Single");
          break;
        case "set_photo_format":
          uiState.photo_format = String(payload.value || "DNG");
          break;
        case "set_photo_timelapse":
          uiState.photo_timelapse = String(payload.value || "Off");
          if (uiState.photo_timelapse !== "Off") {
            uiState.photo_mode_enabled = true;
            uiState.sound_recorder_enabled = false;
            uiState.photo_format = safeTimelapseStillFormat(String(payload.format || uiState.photo_format || "DNG"));
          } else {
            snapshot.timelapse_active = false;
          }
          break;
        case "timelapse_start":
          if (optimistic || result && result.timelapse_active === true) {
            snapshot.timelapse_active = true;
            uiState.photo_mode_enabled = true;
            uiState.sound_recorder_enabled = false;
            uiState.photo_timelapse = String(payload.value || result && result.photo_timelapse || uiState.photo_timelapse || "5s");
            uiState.photo_format = safeTimelapseStillFormat(String(payload.format || result && result.photo_format || uiState.photo_format || "DNG"));
          }
          break;
        case "timelapse_stop":
          if (optimistic || result && result.timelapse_active === false)
            snapshot.timelapse_active = false;
          break;
        case "set_focus_peaking_enabled":
          uiState.focus_peaking_enabled = boolValue;
          break;
        case "set_focus_peaking_threshold":
          uiState.focus_peaking_threshold = isFiniteNumber(result && result.focus_peaking_threshold)
            ? Number(result.focus_peaking_threshold)
            : Number(payload.value);
          break;
        case "set_focus_peaking_color":
          uiState.focus_peaking_color = String(payload.value || "Red");
          break;
        case "set_zebra_enabled":
          uiState.zebra_enabled = boolValue;
          break;
        case "set_zebra_threshold":
          uiState.zebra_threshold = isFiniteNumber(result && result.zebra_threshold)
            ? Number(result.zebra_threshold)
            : Number(payload.value);
          break;
        case "set_false_color_enabled":
          uiState.false_color_enabled = boolValue;
          break;
        case "set_false_color_mode":
          uiState.false_color_mode = isFiniteNumber(result && result.false_color_mode)
            ? Math.round(Number(result.false_color_mode))
            : falseColorModeValueFromLabel(payload.value);
          break;
        case "set_lut_preview_enabled":
          uiState.lut_preview_enabled = boolValue;
          break;
        case "set_lut_preview_file":
          uiState.lut_preview_file = String(result && result.lut_preview_file !== undefined
            ? result.lut_preview_file
            : (payload.value || ""));
          break;
        case "set_guides_enabled":
          uiState.guides_enabled = boolValue;
          break;
        case "set_guides_type":
          uiState.guides_type = String(payload.value || "Thirds");
          break;
        case "set_guides_thickness":
          if (isFiniteNumber(payload.value))
            uiState.guides_thickness = clamp(Math.round(Number(payload.value)), 1, 4);
          break;
        case "set_center_marker_enabled":
          uiState.center_marker_enabled = boolValue;
          break;
        case "set_center_marker_type":
          uiState.center_marker_type = String(payload.value || "Circle/Dot");
          break;
        case "set_grayscale_enabled":
          uiState.grayscale_enabled = boolValue;
          break;
        case "set_smpte_enabled":
          uiState.smpte_enabled = boolValue;
          break;
        case "set_anamorphic_desqueeze_enabled":
          uiState.anamorphic_desqueeze_enabled = boolValue;
          break;
        case "set_anamorphic_ratio":
          uiState.anamorphic_ratio = String(payload.value || "1.33x");
          break;
        case "set_external_monitor_enabled":
          uiState.external_monitor_enabled = boolValue;
          break;
        case "set_external_monitor_mode":
          uiState.external_monitor_mode = String(payload.value || "Clean Feed");
          break;
        case "set_external_monitor_info_overlay":
          uiState.external_monitor_info_overlay = String(payload.value || "On");
          break;
        case "set_external_monitor_orientation":
          uiState.external_monitor_orientation = String(payload.value || "Landscape");
          break;
        case "set_ui_orientation":
          uiState.ui_orientation = String(payload.value || "Landscape");
          break;
        case "set_camera_controls_opacity":
          if (isFiniteNumber(payload.value))
            uiState.camera_controls_opacity = clamp(Number(payload.value), 0.15, 1.0);
          break;
        case "set_camera_controls_mode":
          uiState.camera_controls_mode = String(payload.value || "Light") === "Dark" ? "Dark" : "Light";
          break;
        case "set_date_time_overlay_enabled":
          uiState.date_time_overlay_enabled = boolValue;
          break;
        case "set_record_audio_enabled":
          uiState.record_audio_enabled = boolValue;
          break;
        case "set_record_gyro_enabled":
          uiState.record_gyro_enabled = boolValue;
          break;
        case "set_imu_driver":
          imuState.driver = imuDriverValue(payload.value);
          imuState.driver_label = imuDriverLabel(payload.value);
          imuState.live_available = false;
          imuState.live_status_text = "Waiting for " + imuState.driver_label;
          break;
        case "set_imu_orientation":
          imuState.orientation = String(payload.value || "XYZ");
          break;
        case "set_imu_sync_offset":
          if (isFiniteNumber(payload.value)) {
            imuState.sync_offset_ms = Number(payload.value);
            imuState.sync_offset_text = formatSyncOffsetText(imuState.sync_offset_ms);
          }
          break;
        case "adjust_imu_sync_offset": {
          const currentOffset = Number(imuState.sync_offset_ms) || 0;
          const delta = Number(payload.delta) || 0;
          imuState.sync_offset_ms = currentOffset + delta;
          imuState.sync_offset_text = formatSyncOffsetText(imuState.sync_offset_ms);
          break;
        }
        case "reset_imu_sync_offset":
          imuState.sync_offset_ms = 0;
          imuState.sync_offset_text = "0 ms";
          break;
        case "set_imu_level_reference":
          if (!optimistic) {
            imuState.level_reference_set = true;
            if (!imuState.level_reference_text || imuState.level_reference_text === "Not set")
              imuState.level_reference_text = "Set";
          }
          break;
        case "clear_imu_level_reference":
          imuState.level_reference_set = false;
          imuState.level_reference_text = "Not set";
          break;
        case "set_live_audio_monitoring_enabled":
          uiState.live_audio_monitoring_enabled = boolValue;
          break;
        case "set_audio_meter_enabled":
          uiState.audio_meter_enabled = boolValue;
          break;
        case "set_audio_input_device":
          uiState.audio_input_device = String(payload.value || "");
          break;
        case "set_audio_output_device":
          uiState.audio_output_device = String(payload.value || "");
          break;
        case "set_audio_capture_options":
          if (typeof payload.format === "string") uiState.audio_sample_format = payload.format;
          if (isFiniteNumber(payload.rate)) uiState.audio_sample_rate = Number(payload.rate);
          if (typeof payload.channels === "string") uiState.audio_channels = payload.channels;
          break;
        case "set_input_volume":
          if (isFiniteNumber(payload.value))
            uiState.input_volume = clamp(Math.round(Number(payload.value)), 0, 100);
          break;
        case "set_headphone_volume":
          if (isFiniteNumber(payload.value))
            uiState.headphone_volume = clamp(Math.round(Number(payload.value)), 0, 100);
          break;
        case "set_battery_type":
          uiState.battery_type = String(payload.value || "V-Mount");
          if (batteryCapacityOptionsForType(uiState.battery_type).indexOf(uiState.battery_capacity || "") === -1)
            uiState.battery_capacity = defaultBatteryCapacityForType(uiState.battery_type);
          if (uiState.battery_type !== "Custom") {
            uiState.custom_battery_full_voltage = defaultBatteryFullVoltageForType(uiState.battery_type);
            uiState.custom_battery_empty_voltage = defaultBatteryEmptyVoltageForType(uiState.battery_type);
          }
          break;
        case "set_battery_capacity":
          uiState.battery_capacity = String(payload.value || "150Wh");
          break;
        case "set_custom_battery_wh":
          if (isFiniteNumber(payload.value))
            uiState.custom_battery_wh = clamp(Math.round(Number(payload.value)), 10, 500);
          break;
        case "set_custom_battery_voltage":
          if (isFiniteNumber(payload.full))
            uiState.custom_battery_full_voltage = clamp(Number(payload.full), 0.5, 60);
          if (isFiniteNumber(payload.empty))
            uiState.custom_battery_empty_voltage = clamp(Number(payload.empty), 0.5, Math.max(0.6, Number(uiState.custom_battery_full_voltage || 16.8) - 0.1));
          break;
        case "set_sleep_mode":
          uiState.sleep_mode = String(payload.value || "Off");
          break;
        case "set_favorites_order":
          uiState.favorites_order = String(payload.value || "")
            .split(",")
            .map((entry) => entry.trim())
            .filter((entry) => entry.length > 0);
          break;
        case "set_fan_mode":
          systemState.fan_mode = String(payload.value || "Auto");
          break;
        case "apply_time_zone":
          systemState.time_zone = String(
            result && typeof result.time_zone === "string"
              ? result.time_zone
              : (payload.time_zone || systemState.time_zone || "UTC")
          );
          if (result && typeof result.local_time_input === "string")
            systemState.local_time_input = result.local_time_input;
          break;
        case "apply_date_time":
          systemState.time_zone = String(payload.time_zone || systemState.time_zone || "UTC");
          if (isFiniteNumber(payload.year) &&
              isFiniteNumber(payload.month) &&
              isFiniteNumber(payload.day) &&
              isFiniteNumber(payload.hour) &&
              isFiniteNumber(payload.minute)) {
            const pad2 = (value) => String(value).padStart(2, "0");
            systemState.local_time_input =
              String(Math.round(Number(payload.year))).padStart(4, "0")
              + "-" + pad2(Math.round(Number(payload.month)))
              + "-" + pad2(Math.round(Number(payload.day)))
              + "T" + pad2(Math.round(Number(payload.hour)))
              + ":" + pad2(Math.round(Number(payload.minute)));
          }
          break;
        case "record_start":
          if (optimistic || result && result.recording === true) {
            snapshot.recording = true;
            snapshot.sound_recording = payload.sound_only === "true" || !!(result && result.sound_recording);
            snapshot.state = "recording";
            snapshot.recording_elapsed_seconds = 0;
          }
          break;
        case "record_stop":
          if (optimistic) snapshot.media_writes_pending = true;
          if (optimistic || result && result.recording === false) {
            snapshot.recording = false;
            snapshot.sound_recording = false;
            snapshot.state = "ready";
            snapshot.recording_elapsed_seconds = 0;
          }
          break;
        case "eject_media":
          if (!optimistic && result && result.media_ejected) {
            snapshot.media_mounted = false;
            snapshot.capture.storage = "No Media Mounted";
          }
          break;
        default:
          break;
      }

      return true;
    }

    async function sendCommand(payload) {
      const previousSnapshot = cloneSnapshot(state.snapshot);
      const hadSnapshot = !!state.snapshot;
      const refreshDelayMs = payload && payload.cmd === "record_start" ? 80 : 140;
      const gateUntilMs = performance.now() + refreshDelayMs;

      state.commandEpoch += 1;
      state.commandInFlightCount += 1;
      state.deferFetchUntilMs = Math.max(state.deferFetchUntilMs, gateUntilMs);

      try {
        if (hadSnapshot && applyCommandSnapshotPatch(payload, null, true))
          renderSnapshot(state.snapshot);

        const response = await fetch("/api/command", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
        });
        const result = await response.json();
        if (!result.ok) {
          if (previousSnapshot)
            renderSnapshot(previousSnapshot);
          throw new Error(result.error || "Command failed.");
        }

        if (hadSnapshot && applyCommandSnapshotPatch(payload, result, false))
          renderSnapshot(state.snapshot);

        state.deferFetchUntilMs = performance.now() + refreshDelayMs;
        scheduleStateRefresh(refreshDelayMs, true);
        return result;
      } finally {
        state.commandInFlightCount = Math.max(0, state.commandInFlightCount - 1);
      }
    }

    function buildOptionButtons(containerId, options, currentValue, onClick, columns) {
      const container = document.getElementById(containerId);
      if (!container)
        return;
      container.classList.toggle("three-col", columns === 3);
      container.innerHTML = "";
      for (const option of normalizeChoiceOptions(options)) {
        const button = document.createElement("button");
        button.className = "pill-button"
          + (option.value === String(currentValue) ? " active" : "")
          + (option.enabled ? "" : " disabled");
        button.textContent = option.label;
        button.disabled = !option.enabled;
        button.addEventListener("click", () => {
          if (!option.enabled)
            return;
          onClick(option.value);
        });
        container.appendChild(button);
      }
    }

    function updateTimecodeDisplay() {
      const block = document.getElementById("timecodeBlock");
      if (!block) {
        return;
      }

      if (!state.snapshot) {
        block.classList.remove("hidden");
        setText("timecodeValue", "00:00:00:00");
        return;
      }

      const uiState = state.snapshot.ui || {};
      const stillModeEnabled = !!uiState.photo_mode_enabled;
      block.classList.toggle("hidden", stillModeEnabled);
      block.classList.toggle("recording", !!state.snapshot.recording && !stillModeEnabled);
      if (stillModeEnabled) {
        return;
      }

      const modeLabel = uiState.timecode_mode || timecodeModeLabel(state.snapshot.timecode_mode);

      const fpsInt = currentFpsInt(state.snapshot);
      if (modeLabel === "Free Run") {
        const elapsedMs = performance.now() - state.timecodeBasePerfMs;
        const dayMs = 24 * 60 * 60 * 1000;
        let localMsOfDay = Math.floor(state.timecodeBaseLocalMsOfDay + elapsedMs);
        if (!isFinite(localMsOfDay))
          localMsOfDay = 0;
        localMsOfDay = ((localMsOfDay % dayMs) + dayMs) % dayMs;

        const totalSeconds = Math.floor(localMsOfDay / 1000);
        const frame = Math.min(
          fpsInt - 1,
          Math.floor(((localMsOfDay % 1000) / 1000) * fpsInt)
        );
        const hours = Math.floor(totalSeconds / 3600);
        const minutes = Math.floor((totalSeconds % 3600) / 60);
        const seconds = totalSeconds % 60;
        setText("timecodeValue", pad2(hours) + ":" + pad2(minutes) + ":" + pad2(seconds) + ":" + pad2(frame));
        return;
      }

      let elapsedSeconds = Number(state.snapshot.recording_elapsed_seconds || 0);
      if (state.snapshot.recording) {
        const elapsedMs = performance.now() - state.recordingBasePerfMs;
        elapsedSeconds = state.recordingBaseElapsedSeconds + elapsedMs / 1000;
      }
      setText("timecodeValue", formatTimecode(elapsedSeconds * fpsInt, fpsInt));
    }

    function updateLiveReadouts() {
      updateTimecodeDisplay();
      renderProcessedPreviewFrame();
    }

    function startLiveReadoutLoop() {
      if (state.liveReadoutTimerId)
        return;

      if (typeof window.requestAnimationFrame === "function") {
        const tick = () => {
          updateLiveReadouts();
          state.liveReadoutTimerId = window.requestAnimationFrame(tick);
        };
        state.liveReadoutTimerId = window.requestAnimationFrame(tick);
        return;
      }

      state.liveReadoutTimerId = window.setInterval(updateLiveReadouts, 16);
    }

    function clearPhotoSequenceTimers() {
      clearTimeout(state.photoDelayTimer);
      clearTimeout(state.photoCountdownTimer);
      clearTimeout(state.photoCapturePulseTimer);
      state.photoDelayTimer = null;
      state.photoCountdownTimer = null;
      state.photoCapturePulseTimer = null;
    }

    function cancelPhotoSequence() {
      clearPhotoSequenceTimers();
      state.photoCaptureInProgress = false;
      state.photoCountdownSecondsRemaining = 0;
      state.pendingPhotoShots = 0;
      state.photoCapturePulse = false;
      updateLiveReadouts();
    }

    function triggerPhotoCapturePulse() {
      clearTimeout(state.photoCapturePulseTimer);
      state.photoCapturePulse = true;
      updateLiveReadouts();
      state.photoCapturePulseTimer = window.setTimeout(() => {
        state.photoCapturePulse = false;
        updateLiveReadouts();
      }, 220);
    }

    async function queueNextPhotoInSequence() {
      if (!state.snapshot || state.pendingPhotoShots <= 0) {
        state.photoCaptureInProgress = false;
        updateLiveReadouts();
        await fetchState(true);
        return;
      }

      try {
        await sendCommand({ cmd: "capture_photo", count: state.pendingPhotoShots });
      } catch (error) {
        cancelPhotoSequence();
        showToast(error.message, true);
        return;
      }

      triggerPhotoCapturePulse();
      state.pendingPhotoShots = 0;
      state.photoCaptureInProgress = false;
      updateLiveReadouts();
      await fetchState(true);
    }

    async function beginPhotoCaptureSequence() {
      if (!state.snapshot || state.photoCaptureInProgress || state.snapshot.media_writes_pending)
        return;

      const uiState = state.snapshot.ui || {};
      state.photoCaptureInProgress = true;
      state.pendingPhotoShots = uiState.photo_bracketing_enabled ? 1 : photoBurstCount(uiState.photo_burst || "Single");
      state.photoCountdownSecondsRemaining = 0;
      updateLiveReadouts();

      const delayMs = photoTimerDelayMs(uiState.photo_timer || "Off");
      if (delayMs > 0) {
        state.photoCountdownSecondsRemaining = Math.max(1, Math.ceil(delayMs / 1000));
        updateLiveReadouts();
        state.photoDelayTimer = window.setTimeout(() => {
          state.photoCountdownSecondsRemaining = 0;
          clearTimeout(state.photoCountdownTimer);
          state.photoCountdownTimer = null;
          void queueNextPhotoInSequence();
        }, delayMs);
        state.photoCountdownTimer = window.setInterval(() => {
          if (state.photoCountdownSecondsRemaining > 1) {
            state.photoCountdownSecondsRemaining -= 1;
          } else {
            state.photoCountdownSecondsRemaining = 1;
            clearInterval(state.photoCountdownTimer);
            state.photoCountdownTimer = null;
          }
          updateLiveReadouts();
        }, 1000);
        return;
      }

      await queueNextPhotoInSequence();
    }

    function renderSnapshot(snapshot) {
      const previousPhotoError = state.snapshot && state.snapshot.photo_write_error;
      if (snapshot.photo_write_error && snapshot.photo_write_error !== previousPhotoError)
        showToast("Still not saved: " + snapshot.photo_write_error, true);
      const previousError = state.snapshot && state.snapshot.recording_error;
      if (snapshot.recording_error && snapshot.recording_error !== previousError)
        showToast("Recording stopped: " + snapshot.recording_error, true);
      state.snapshot = snapshot;
      document.getElementById("stillExposureOverlay").style.display = snapshot.still_exposure_active ? "flex" : "none";
      const exposureSeconds = Math.max(0, Math.ceil(Number(snapshot.still_exposure_remaining_seconds) || 0));
      setText("stillExposureCountdown", exposureSeconds > 0 ? exposureSeconds + "s" : "Finishing...");
      const loading = document.getElementById("loadingOverlay");
      loading.classList.add("hidden");

      const resolutionLabel = resolutionLabelFromSnapshot(snapshot);
      const resolutionOptions = availableResolutionOptionsForSnapshot(snapshot);
      const recordFormatText = recordingFormatLabel(snapshot.recording_format);
      const h264RecordFormat = processedVideoRecordFormatForSnapshot(snapshot);
      const fixedTenBitRecordFormat = fixedTenBitRecordingFormat(snapshot.recording_format, snapshot);
      const cropControlsAvailable = sensorCropControlsAvailableForSnapshot(snapshot);
      const cropControlsVisible = h264RecordFormat || cropControlsAvailable;
      const resolutionBaseOptions = h264RecordFormat
        ? processedVideoBaseResolutionOptionsForSnapshot(snapshot)
        : (cropControlsAvailable ? sensorBaseResolutionOptionsForSnapshot(snapshot) : selectEntriesFromChoices(resolutionOptions));
      const resolutionBaseValue = h264RecordFormat ? h264BaseResolutionValueFromSnapshot(snapshot) : resolutionBaseValueFromSnapshot(snapshot);
      const cropRatioOptions = h264RecordFormat ? h264CropOptionsForBase(resolutionBaseValue) : sensorCropRatioOptionsForSnapshot(snapshot);
      const cropRatioValue = h264RecordFormat ? h264CropRatioValueFromSnapshot(snapshot) : cropRatioValueFromSnapshot(snapshot);
      const cropRatioEnabled = h264RecordFormat || cropControlsAvailable;
      const isRecording = !!snapshot.recording;
      const isSoundRecording = !!snapshot.sound_recording;
      const isTimelapse = !!snapshot.timelapse_active;
      const hasMedia = !!snapshot.media_mounted;
      const whiteBalanceAuto = !!snapshot.wb_auto;
      const whiteBalanceAvailable = snapshot.wb_available === true;
      const displayWhiteBalance = nearestWhiteBalance(snapshot.wb_kelvin);
      const whiteBalanceLabel = whiteBalanceAuto ? "Auto" : (String(displayWhiteBalance) + "K");
      const fpsOptions = availableFpsOptionsForSnapshot(snapshot);
      const snapshotSensorLower = sensorNameLower(snapshot);
      const rawBitDepthAvailable = rawBitDepthAvailableForSensor(snapshotSensorLower);
      const rawBitDepthFixedForH264 = fixedTenBitRecordFormat;
      const rawBitDepthText = rawBitDepthFixedForH264 ? (recordingFeatureSupported(snapshot, "h264_hardware") ? "8-bit" : "10-bit") : rawBitDepthLabel(snapshot.raw_bit_depth);
      const h264ResolutionText = h264ResolutionLabel(snapshot);
      const timecodeText = timecodeModeLabel(snapshot.timecode_mode);
      const autofocusModeText = autofocusModeLabel(snapshot.af_mode);
      const autofocusRangeText = autofocusRangeLabel(snapshot.af_range);
      const autofocusSpeedText = autofocusSpeedLabel(snapshot.af_speed);
      const autofocusWindowText = autofocusWindowLabel(snapshot);
      const autofocusSummaryText = autofocusModeText + " / " + autofocusWindowText;
      const uiState = snapshot.ui || {};
      const imuState = snapshot.imu || {};
      const audioOptions = currentAudioOptions(snapshot);
      const systemState = snapshot.system || {};
      const stillModeEnabled = !!uiState.photo_mode_enabled || isTimelapse;
      const stillTimerValue = uiState.photo_timer || "Off";
      const stillBurstValue = uiState.photo_burst || "Single";
      const stillFormatValue = uiState.photo_format || "DNG";
      const stillTimelapseValue = uiState.photo_timelapse || "Off";
      const stillTimelapseArmed = stillTimelapseEnabled(stillTimelapseValue) || isTimelapse;
      const stillFormatDisplayValue = stillTimelapseArmed ? safeTimelapseStillFormat(stillFormatValue) : stillFormatValue;
      const stillTimelapseIntervalValue = defaultStillTimelapseInterval(snapshot, stillTimelapseValue);
      const shutterLabel = snapshot.shutter_auto
        ? "Auto"
        : (stillModeEnabled ? formatShutterSpeed(snapshot.shutter_us) : formatAngle(snapshot.shutter_us, snapshot.fps));
      const shutterOptionSet = stillModeEnabled
        ? availableStillShutterOptions(uiState) : shutterOptions;
      const focusPeakingEnabled = !!uiState.focus_peaking_enabled;
      const focusPeakingThreshold = valueToFocusThreshold(uiState.focus_peaking_threshold);
      const focusPeakingColor = uiState.focus_peaking_color || "Red";
      const zebraEnabled = !!uiState.zebra_enabled;
      const zebraLevel = thresholdToZebraLevel(uiState.zebra_threshold);
      const anamorphicDesqueezeEnabled = !!uiState.anamorphic_desqueeze_enabled;
      const anamorphicRatio = uiState.anamorphic_ratio || "1.33x";
      const falseColorEnabled = !!uiState.false_color_enabled;
      const falseColorMode = intToFalseColorMode(uiState.false_color_mode);
      const lutPreviewEnabled = !!uiState.lut_preview_enabled;
      const lutPreviewFile = String(uiState.lut_preview_file || "");
      const lutPreviewOptionEntries = lutPreviewEntries(uiState);
      const smpteEnabled = !!uiState.smpte_enabled;
      const guidesEnabled = uiState.guides_enabled !== false;
      const guidesType = uiState.guides_type || "Thirds";
      const guidesThickness = String(clamp(Number(uiState.guides_thickness) || 1, 1, 4));
      const centerMarkerEnabled = uiState.center_marker_enabled !== false;
      const centerMarkerType = uiState.center_marker_type || "Circle/Dot";
      const grayscaleEnabled = !!uiState.grayscale_enabled;
      const externalMonitorEnabled = !!uiState.external_monitor_enabled;
      const externalMonitorMode = uiState.external_monitor_mode || "Clean Feed";
      const externalMonitorInfoOverlay = uiState.external_monitor_info_overlay || "On";
      const externalMonitorOrientation = uiState.external_monitor_orientation || "Landscape";
      const uiOrientation = uiState.ui_orientation || "Landscape";
      const controlsOpacityLabel = opacityValueToLabel(uiState.camera_controls_opacity);
      const controlsMode = uiState.camera_controls_mode || "Light";
      const dateTimeOverlayEnabled = !!uiState.date_time_overlay_enabled;
      const soundRecorderEnabled = !!uiState.sound_recorder_enabled && audioOptions.hasInputDevices;
      const recordAudioEnabled = !!uiState.record_audio_enabled && audioOptions.hasInputDevices;
      const recordGyroEnabled = !!uiState.record_gyro_enabled;
      const imuDriverText = imuDriverLabel(imuState.driver || imuState.driver_label);
      const imuOrientation = imuState.orientation || "XYZ";
      const imuLiveAvailable = !!imuState.live_available;
      const imuCanCalibrate = !isRecording && !isTimelapse && !state.gyroCalibrationRunning;
      const imuDisplayXG = displayImuAccelValue(imuState, 0);
      const imuDisplayYG = displayImuAccelValue(imuState, 1);
      const imuDisplayZG = displayImuAccelValue(imuState, 2);
      const imuMappedGyroXDps = mappedImuAxisValue(imuState, 0, "gyro");
      const imuMappedGyroYDps = mappedImuAxisValue(imuState, 1, "gyro");
      const imuMappedGyroZDps = mappedImuAxisValue(imuState, 2, "gyro");
      const liveAudioMonitoringEnabled = !!uiState.live_audio_monitoring_enabled;
      const audioMeterEnabled = !!uiState.audio_meter_enabled;
      const audioInputDevice = uiState.audio_input_device || "";
      const audioOutputDevice = uiState.audio_output_device || "";
      const inputVolume = clamp(Number(uiState.input_volume) || 60, 0, 100);
      const headphoneVolume = clamp(Number(uiState.headphone_volume) || 55, 0, 100);
      const batteryType = activeBatteryType(uiState);
      const batteryCapacity = batteryCapacityForType(uiState);
      const customBatteryWh = clamp(Number(uiState.custom_battery_wh) || 150, 10, 500);
      const customBatteryFullVoltage = clamp(Number(uiState.custom_battery_full_voltage) || 16.8, 0.5, 60);
      const customBatteryEmptyVoltage = clamp(Number(uiState.custom_battery_empty_voltage) || 13.2, 0.5, customBatteryFullVoltage - 0.1);
      const sleepMode = uiState.sleep_mode || "Off";
      const timeZoneOptions = Array.isArray(systemState.time_zone_options) ? systemState.time_zone_options : ["UTC"];
      const timeZoneValue = systemState.time_zone || "UTC";
      const localDateTimeValue = systemState.local_time_input || "";
      const fanModeValue = systemState.fan_mode || "Auto";
      const fanModeSupported = fanModeSupportedForSnapshot(snapshot);
      const batteryRuntimeText = formatBatteryRuntimeEstimate(uiState, systemState);
      const mediaUsedPercent = hasMedia ? clamp(Number(systemState.media_used_percent) || 0, 0, 100) : 0;
      const mediaUsedBytes = hasMedia
        ? Math.max(0, (Number(systemState.media_total_bytes) || 0) - (Number(systemState.media_free_bytes) || 0))
        : 0;
      const mediaFreeText = hasMedia ? formatStorage(systemState.media_free_bytes) : "No Media";
      const mediaTotalText = hasMedia ? formatStorage(systemState.media_total_bytes) : "--";
      const mediaUsedText = hasMedia ? (formatStorage(mediaUsedBytes) + " used") : "Insert media to record";
      const mediaRemainingLabel = mediaEstimateLabel(stillModeEnabled);
      const mediaRemainingText = hasMedia ? mediaEstimateValue(systemState, stillModeEnabled) : "N/A";
      const mediaRemainingSub = hasMedia
        ? mediaEstimateSubLabel(systemState, stillModeEnabled)
        : "Mount /media/RAW to record";
      const powerSensorAvailable = !!systemState.power_sensor_available;
      const batteryInputPresent = powerSensorAvailable && Number(systemState.battery_voltage_v) > 3.0;
      const profileBatteryPercent = batteryProfilePercent(uiState, systemState);
      const batteryInfoPrimary = !powerSensorAvailable
        ? "No power sensor"
        : (!batteryInputPresent
           ? "External Input"
           : (formatBatteryVoltage(systemState.battery_voltage_v)
              + " • "
              + formatBatteryCurrent(systemState.battery_current_a)
              + (isFinite(profileBatteryPercent) ? (" • " + formatPercent(profileBatteryPercent)) : "")));
      const batteryInfoSecondary = !powerSensorAvailable
        ? "Battery telemetry is unavailable."
        : (!batteryInputPresent
           ? "DC input active"
           : (batteryType + " • " + formatWatts(systemState.power_draw_w) + " draw • " + batteryRuntimeText));

      setPreviewSmpteEnabled(smpteEnabled);
      setFalseColorLegend(falseColorMode, falseColorEnabled && !smpteEnabled);
      setPreviewAspectForSnapshot(snapshot);
      if (smpteEnabled)
        setPreviewOverlay(false, "", false);

      if (!stillModeEnabled && (state.photoCaptureInProgress || state.photoCountdownSecondsRemaining > 0 || state.photoCapturePulse))
        cancelPhotoSequence();

      setText("networkHost", snapshot.network_host || window.location.host || window.location.hostname);
      setText("networkTransport", snapshot.network_transport || "Wi-Fi / Ethernet");
      setText("webDateTimePill", formatWebDateTimePill(localDateTimeValue));
      setText("recordingStatus", !hasMedia ? "NO MEDIA" : (isTimelapse ? "TIMELAPSE" : (isSoundRecording ? "SOUND REC" : (isRecording ? "REC" : snapshot.media_writes_pending ? "WRITING" : "READY"))));
      setText("resolutionBadge", h264RecordFormat ? h264OutputResolutionLabel(snapshot) : resolutionLabel);
      setText("fpsBadge", String(Math.round(snapshot.fps)) + " FPS");
      const isoLabel = snapshot.iso_auto ? "Auto" : String(snapshot.iso);
      setText("isoBadge", snapshot.iso_auto ? "ISO Auto" : ("ISO " + isoLabel));
      setText("wbBadge", "WB " + whiteBalanceLabel);
      const timelapseShots = Math.max(0, Math.floor(Number(snapshot.timelapse_shot_count) || 0));
      setText("timelapseShotCounter", timelapseShots + (timelapseShots === 1 ? " SHOT" : " SHOTS"));
      document.getElementById("timelapseShotCounter").classList.toggle("hidden", !stillModeEnabled || !stillTimelapseArmed);

      setText("shutterSectionLabel", stillModeEnabled ? "Shutter speed" : "Shutter angle");
      setText("quickResolutionLabel", h264RecordFormat ? processedVideoSizeLabel(recordFormatText) : "Resolution");
      setText("quickCropLabel", h264RecordFormat ? processedVideoCropLabel(recordFormatText) : "Crop mode");
      setText("whiteBalanceValue", whiteBalanceLabel);
      setText("settingsRecordResolutionLabel", h264RecordFormat ? processedVideoSizeLabel(recordFormatText) : "Resolution");
      setText("settingsRecordCropLabel", h264RecordFormat ? processedVideoCropLabel(recordFormatText) : "Crop mode");
      setText("settingsRecordFormatValue", recordFormatText);
      document.getElementById("settingsProresColorField").classList.toggle("hidden", snapshot.recording_format !== "prores" || stillModeEnabled);
      setFormValue("settingsProresColorSelect", snapshot.prores_color_profile || "standard");
      setElementDisabled("settingsProresColorSelect", isRecording || !!snapshot.timelapse_active);
      document.getElementById("settingsCdngPreviewColorField").classList.toggle("hidden", recordFormatText !== "CinemaDNG" || stillModeEnabled);
      setFormValue("settingsCdngPreviewColorSelect", snapshot.cdng_preview_color_profile || "standard");
      setElementDisabled("settingsCdngPreviewColorSelect", isRecording || !!snapshot.timelapse_active);
      setText("settingsCdngPreviewColorNote", snapshot.cdng_preview_color_profile === "p3_d60_linear"
        ? "Preview only. Linear has no display gamma and looks darker; 8-bit preview precision is limited. DNG RAW is unchanged."
        : "Preview only, including HDMI and WebUI. DNG stays sensor RAW; Resolve decoding is unchanged.");
      setText("settingsProresColorNote", snapshot.prores_color_profile === "arri_logc4"
        ? "LogC4 is designed for at least 12-bit. This ProRes is 10-bit. Use matching Resolve input color space/gamma and Video data levels."
        : snapshot.prores_color_profile === "p3_d60_linear"
          ? "P3-D60 / Linear preview and recording. Linear has less shadow precision than log in 10-bit. Use P3-D60 / Linear and Video data levels in Resolve."
        : "Log profiles affect preview and recording. Use the matching Resolve input color space/gamma and Video data levels.");
      document.getElementById("settingsCdngCompressionField").classList.toggle("hidden", recordFormatText !== "CinemaDNG");
      setFormValue("settingsCdngCompressionSelect", cdngCompressionValueForSnapshot(snapshot));
      const rawCompressionSupported = recordingFeatureSupported(snapshot, "raw_compression");
      for (const option of document.getElementById("settingsCdngCompressionSelect").options) {
        option.hidden = option.value !== "none" && !rawCompressionSupported;
        option.disabled = option.hidden;
      }
      setElementDisabled("settingsCdngCompressionSelect", isRecording || !rawCompressionSupported);
      setText("settingsRawBitDepthValue", rawBitDepthText);
      setText("settingsRawBitDepthNote", rawBitDepthFixedForH264
        ? recordFormatText + (recordingFeatureSupported(snapshot, "h264_hardware") ? " uses 8-bit YUV420 hardware encoding." : " uses processed video output, so capture is fixed to 10-bit here.")
        : (rawBitDepthAvailable
          ? "Raw sensor bit depth for the active capture mode."
          : "Bit-depth selection is only available on supported RAW sensors."));
      setText("settingsGyroDataValue", formatToggleLabel(recordGyroEnabled));
      setText("settingsTimecodeValue", timecodeText);
      setText("settingsAutofocusValue", autofocusSummaryText);
      setText("settingsSensorValue", snapshot.capture.sensor);
      setText("settingsResolutionSummary", snapshot.resolution);
      setText("settingsFpsSummary", formatFpsLabel(snapshot.fps));
      setText("settingsStorageSummary", snapshot.capture.storage);
      setText("settingsPreviewStreamValue", snapshot.capture.preview);
      setText("settingsPreviewTransportValue", snapshot.network_transport || "Wi-Fi / Ethernet");
      setText("settingsPreviewHostValue", snapshot.network_host || window.location.host || window.location.hostname);
      setText("settingsPreviewSourceValue",
              smpteEnabled
                ? "SMPTE Bars"
                : (resolutionLabel + " / " + formatFpsLabel(snapshot.fps)));
      setText("settingsIsoSummary", snapshot.iso_auto ? "ISO Auto" : ("ISO " + isoLabel));
      setText("settingsWbSummary", whiteBalanceLabel);
      setText("settingsShutterSummary", shutterLabel);
      setText("settingsCodecSummary", snapshot.capture.codec);
      setText("settingsInputVolumeValue", String(inputVolume));
      setText("settingsHeadphoneVolumeValue", String(headphoneVolume));
      setText("settingsImuStatusPill", imuDriverText);
      setText("settingsImuLiveStatus",
              imuLiveAvailable
                ? (imuState.level_reference_set ? "Live motion | Level set" : "Live motion")
                : (imuState.live_status_text || "Live gyro idle"));
      setText("settingsImuOrientationReadout", "Orientation " + imuOrientation + " | " + (Number(imuState.sample_rate_hz) || 500) + " Hz");
      setText("settingsImuAccelReadout", formatImuVector(imuDisplayXG, imuDisplayYG, imuDisplayZG, "g", 2));
      setText("settingsImuGyroReadout", formatImuVector(imuMappedGyroXDps, imuMappedGyroYDps, imuMappedGyroZDps, "dps", 2));
      setText("settingsImuSyncReadout", imuState.sync_offset_text || "0 ms");
      setText("settingsImuSyncOffsetValue", imuState.sync_offset_text || "0 ms");
      setText("settingsImuLevelReadout", imuState.level_reference_text || "Not set");
      setText("settingsImuConfigReadout", formatImuConfigPath(imuState.config_path));
      setText("settingsImuLevelNote",
              imuState.level_reference_set
                ? ("Neutral level: " + (imuState.level_reference_text || "Set"))
                : "Place the camera in its neutral shooting position, then press Set Level.");
      setText("settingsImuCalibrationStatus", state.gyroCalibrationStatus);
      setText("settingsBatteryTypeValue", batteryType);
      setText("settingsCustomBatteryWhValue", customBatteryWh + "Wh");
      setText("settingsCustomBatteryFullVoltageValue", customBatteryFullVoltage.toFixed(1) + "V");
      setText("settingsCustomBatteryEmptyVoltageValue", customBatteryEmptyVoltage.toFixed(1) + "V");
      setText("settingsBatteryInfoPrimary", batteryInfoPrimary);
      setText("settingsBatteryInfoSecondary", batteryInfoSecondary);
      setText("settingsBatteryRuntimeValue", batteryRuntimeText);
      setText("settingsSleepModeValue", sleepMode);
      setText("settingsStandbyButton", snapshot.state === "standby" ? "Wake Camera" : "Standby");
      setFormValue("settingsStandbySelect", String(snapshot.standby_timeout_seconds || 0));
      setFormValue("settingsStandbyUsbSelect", snapshot.standby_usb_power_off ? "true" : "false");
      setElementDisabled("settingsStandbyButton", isRecording || isTimelapse);
      setText("settingsMediaStatusValue", hasMedia ? (isTimelapse ? "Timelapse active" : (isRecording ? "Recording active" : "Media mounted")) : "No media mounted");
      setText("settingsMediaTypeValue", snapshot.capture.storage);
      setText("settingsMediaPathValue", snapshot.system.media_label);
      setText("settingsMediaTimeLeftLabel", stillModeEnabled ? "Stills left" : "Record time left");
      setText("settingsMediaTimeLeftValue", mediaRemainingText);
      setText("settingsMediaSpaceLeftValue", mediaFreeText);
      setText("settingsMediaUsageDetailValue", mediaUsedText);
      setText("settingsMediaCapacityValue", hasMedia ? (mediaTotalText + " total") : "--");
      renderMediaDriveOptions(systemState, hasMedia);
      setText("settingsMediaSpeedResultValue", systemState.write_speed_result_text || "Not Tested");
      setText("settingsMediaSpeedDetailValue", systemState.write_speed_detail_text || "--");
      setText("settingsThermalCpuValue", formatDegrees(systemState.cpu_temp_c));
      setText("settingsThermalFanModeValue", fanModeValue);
      const device = snapshot.device || {};
      setText("settingsInfoSensorValue", snapshot.capture.sensor);
      setText("settingsInfoSensorDetail", snapshot.capture.preview || "Preview unavailable");
      setText("settingsInfoBackendValue", snapshot.backend_name || "Apertar-Core / libcamera");
      setText("settingsInfoBackendDetail", snapshot.capture.network || "Local Network");
      setText("settingsInfoPiModelValue", device.pi_model || "--");
      setText("settingsInfoSerialValue", device.serial_number || "--");
      setText("settingsInfoMemoryValue", (device.ram_total_text || "--") + " total");
      setText("settingsInfoMemoryDetail", (device.ram_used_text || "--") + " used");
      setText("settingsInfoOsValue", device.os_version || "--");
      setText("settingsInfoHostnameDetail", "Hostname " + (device.hostname || "--"));
      setText("settingsInfoKernelValue", device.kernel_version || "--");
      setText("settingsInfoUptimeValue", device.uptime_text || "--");
      setText("settingsInfoIpValue", device.ip_address || "--");
      setText("settingsInfoSystemDriveValue", (device.system_storage_total_text || "--") + " total");
      setText(
        "settingsInfoSystemDriveDetail",
        (device.system_storage_free_text || "--")
        + " free • Mounted at "
        + (device.system_storage_mount_point || "/")
      );

      ensureSelectOptions("settingsTimeZoneSelect", timeZoneOptions);
      ensureSelectOptions("settingsStillFormatSelect", stillTimelapseArmed ? timelapseStillFormatOptions : stillFormatOptions);
      ensureSelectOptions("settingsStillTimerSelect", stillTimerOptions);
      ensureSelectOptions("settingsStillBurstSelect", uiState.photo_bracketing_enabled ? bracketEvOptions : stillBurstOptions);
      setText("settingsStillBurstLabel", uiState.photo_bracketing_enabled ? "EV Offset" : "Burst");
      ensureSelectOptions("settingsStillTimelapseSelect", [...stillTimelapseIntervalOptions,
        ...(stillTimelapseIntervalOptions.includes(stillTimelapseIntervalValue) ? [] : [stillTimelapseIntervalValue]), "Custom..."]);
      ensureSelectEntries("settingsRecordResolutionSelect", resolutionBaseOptions);
      ensureSelectEntries("settingsRecordCropSelect", cropRatioOptions);
      ensureSelectOptions("settingsH264ResolutionSelect", h264ResolutionOptions.filter(value => {
        const maxWidth = Number(snapshot.recording_capabilities && snapshot.recording_capabilities.h264_max_width) || 2560;
        return value !== "Match Capture" ? Number(value.split("x")[0]) <= maxWidth : !recordingFeatureSupported(snapshot, "h264_hardware");
      }));
      ensureSelectOptions("settingsImuDriverSelect", imuDriverOptions);
      ensureSelectOptions("settingsImuOrientationSelect", imuOrientationOptions);
      ensureSelectOptions("settingsExternalMonitorModeSelect", externalMonitorModeOptions);
      ensureSelectOptions("settingsExternalMonitorInfoOverlaySelect", externalMonitorInfoOverlayOptions);
      ensureSelectOptions("settingsExternalMonitorOrientationSelect", externalMonitorOrientationOptions);
      ensureSelectOptions("settingsFocusPeakingThresholdSelect", focusPeakingThresholdOptions);
      ensureSelectOptions("settingsFocusPeakingColorSelect", focusPeakingColorOptions);
      ensureSelectOptions("settingsZebraLevelSelect", zebraLevelOptions);
      ensureSelectOptions("settingsFalseColorModeSelect", falseColorModeOptions);
      ensureSelectEntries("settingsLutPreviewSelect", lutPreviewOptionEntries);
      ensureSelectOptions("settingsGuidesTypeSelect", guidesTypeOptions);
      ensureSelectOptions("settingsGuidesThicknessSelect", guidesThicknessOptions);
      ensureSelectOptions("settingsCenterMarkerTypeSelect", centerMarkerTypeOptions);
      ensureSelectOptions("settingsAnamorphicRatioSelect", anamorphicRatioOptions);
      ensureSelectOptions("settingsControlsOpacitySelect", controlsOpacityOptions);
      ensureSelectOptions("settingsControlsModeSelect", controlsModeOptions);
      ensureSelectOptions("settingsBatteryTypeSelect", batteryTypeOptions);
      ensureSelectOptions("settingsBatteryCapacitySelect", batteryCapacityOptionsForType(batteryType));
      ensureSelectEntries(
        "settingsAudioInputDeviceSelect",
        audioOptions.hasInputDevices
          ? audioOptions.inputDevices
          : [{ id: "", label: "No audio input device detected" }]
      );
      ensureSelectEntries(
        "settingsAudioOutputDeviceSelect",
        audioOptions.hasOutputDevices
          ? audioOptions.outputDevices
          : [{ id: "", label: "None" }]
      );
      setFormValue("settingsTimeZoneSelect", timeZoneValue);
      setFormValue("settingsDateTimeInput", localDateTimeValue);
      setFormValue("settingsStillFormatSelect", stillFormatDisplayValue);
      setFormValue("settingsStillTimerSelect", stillTimerValue);
      setFormValue("settingsStillBurstSelect", uiState.photo_bracketing_enabled
        ? bracketEvOptions[clamp(Math.round(Number(uiState.photo_bracketing_ev || 1) * 3) - 1, 0, 8)] : stillBurstValue);
      if (!state.customIntervalEditing)
        setFormValue("settingsStillTimelapseSelect", stillTimelapseIntervalValue);
      setFormValue("settingsRecordResolutionSelect", resolutionBaseValue);
      const evOptions = [];
      for (let step = -9; step <= 9; step++) {
        const ev = step / 3;
        evOptions.push({value:ev.toFixed(3),label:(step > 0 ? "+" : "") + (step % 3 === 0 ? ev.toFixed(0) : ev.toFixed(2)) + " EV"});
      }
      ensureSelectEntries("settingsExposureCompensationSelect", evOptions);
      setFormValue("settingsExposureCompensationSelect", (Math.round(Number(snapshot.exposure_compensation || 0) * 3) / 3).toFixed(3));
      const evEnabled = snapshot.exposure_compensation_available && (snapshot.iso_auto || snapshot.shutter_auto);
      setElementDisabled("settingsExposureCompensationSelect", !evEnabled);
      const meteringLabels = {"centre-weighted":"Center-weighted",matrix:"Average",spot:"Spot"};
      const meteringOptions = (snapshot.exposure_metering_modes || [])
        .filter(mode => Object.prototype.hasOwnProperty.call(meteringLabels, mode))
        .map(mode => ({value:mode,label:meteringLabels[mode]}));
      ensureSelectEntries("settingsExposureMeteringSelect", meteringOptions.length ? meteringOptions : [{value:"",label:"Unavailable"}]);
      setFormValue("settingsExposureMeteringSelect", meteringOptions.length ? snapshot.exposure_metering : "");
      setElementDisabled("settingsExposureMeteringSelect", !(snapshot.iso_auto || snapshot.shutter_auto) || meteringOptions.length < 2);
      setText("settingsExposureCompensationNote", !snapshot.exposure_compensation_available ? "Not supported by this camera pipeline." : evEnabled ? "Applies to auto exposure only, within the sensor's exposure/gain limits." : "Enable Auto ISO or Auto Shutter to adjust exposure.");
      setFormValue("settingsRecordCropSelect", cropRatioValue);
      setFormValue("settingsH264ResolutionSelect", h264ResolutionText);
      setFormValue("settingsImuDriverSelect", imuDriverText);
      setFormValue("settingsImuOrientationSelect", imuOrientation);
      setFormValue("settingsExternalMonitorModeSelect", externalMonitorMode);
      setFormValue("settingsExternalMonitorInfoOverlaySelect", externalMonitorInfoOverlay);
      setFormValue("settingsExternalMonitorOrientationSelect", externalMonitorOrientation);
      setFormValue("settingsFocusPeakingThresholdSelect", focusPeakingThreshold);
      setFormValue("settingsFocusPeakingColorSelect", focusPeakingColor);
      setFormValue("settingsZebraLevelSelect", zebraLevel);
      setFormValue("settingsFalseColorModeSelect", falseColorMode);
      setFormValue("settingsLutPreviewSelect", lutPreviewFile);
      setFormValue("settingsGuidesTypeSelect", guidesType);
      setFormValue("settingsGuidesThicknessSelect", guidesThickness);
      setFormValue("settingsCenterMarkerTypeSelect", centerMarkerType);
      setFormValue("settingsAnamorphicRatioSelect", anamorphicRatio);
      setFormValue("settingsUiOrientationSelect", uiOrientation);
      setFormValue("settingsControlsOpacitySelect", controlsOpacityLabel);
      setFormValue("settingsControlsModeSelect", controlsMode);
      setFormValue("settingsFanModeSelect", fanModeValue);
      setElementDisabled("settingsFanModeSelect", !fanModeSupported);
      setElementDisabled("settingsStillTimerSelect", stillTimelapseArmed);
      setElementDisabled("settingsStillBurstSelect", stillTimelapseArmed);
      setElementDisabled("settingsStillTimelapseSelect", !stillTimelapseArmed);
      setElementDisabled("settingsRecordResolutionSelect", false);
      setElementDisabled("settingsRecordCropSelect", !cropRatioEnabled);
      const recordResolutionField = document.getElementById("settingsRecordResolutionField");
      const recordCropField = document.getElementById("settingsRecordCropField");
      const h264ResolutionField = document.getElementById("settingsH264ResolutionField");
      if (recordResolutionField)
        recordResolutionField.classList.toggle("field-span-2", !cropControlsVisible);
      if (recordCropField)
        recordCropField.classList.toggle("hidden", !cropControlsVisible);
      if (h264ResolutionField)
        h264ResolutionField.classList.add("hidden");
      setElementDisabled("settingsImuDriverSelect", state.gyroCalibrationRunning);
      setElementDisabled("settingsImuOrientationSelect", state.gyroCalibrationRunning);
      setElementDisabled("settingsImuSetLevelButton", state.gyroCalibrationRunning || !imuLiveAvailable);
      setElementDisabled("settingsImuClearLevelButton", state.gyroCalibrationRunning || !imuState.level_reference_set);
      setElementDisabled("settingsImuSyncMinusButton", state.gyroCalibrationRunning);
      setElementDisabled("settingsImuSyncPlusButton", state.gyroCalibrationRunning);
      setElementDisabled("settingsImuSyncResetButton", state.gyroCalibrationRunning);
      setElementDisabled("settingsImuCalibrateButton", !imuCanCalibrate);
      const imuCalibrateButton = document.getElementById("settingsImuCalibrateButton");
      if (imuCalibrateButton)
        imuCalibrateButton.textContent = state.gyroCalibrationRunning ? "Calibrating..." : "Calibrate Gyro Bias";
      const imuLiveDot = document.getElementById("settingsImuLiveDot");
      if (imuLiveDot)
        imuLiveDot.classList.toggle("live", imuLiveAvailable);
      const imuCalibrationStatus = document.getElementById("settingsImuCalibrationStatus");
      if (imuCalibrationStatus)
        imuCalibrationStatus.style.color = state.gyroCalibrationOk ? "#a7f2bd" : "var(--muted)";
      drawImuAxisVisual(imuState);
      setText("settingsFanModeNote",
              fanModeSupported
                ? "Choose the thermal profile written into boot configuration."
                : "Fan mode is unavailable on Raspberry Pi 4.");
      setFormValue("settingsAudioInputDeviceSelect", audioInputDevice);
      setFormValue("settingsAudioOutputDeviceSelect", audioOutputDevice);
      setFormValue("settingsBatteryTypeSelect", batteryType);
      setFormValue("settingsBatteryCapacitySelect", batteryCapacity);
      setFormValue("settingsSleepModeSelect", sleepMode);
      setElementDisabled("settingsAudioInputDeviceSelect", !audioOptions.hasInputDevices);
      setElementDisabled("settingsAudioOutputDeviceSelect", !audioOptions.hasOutputDevices);
      renderAudioCaptureOptions();
      const inputVolumeRange = document.getElementById("settingsInputVolumeRange");
      if (document.activeElement !== inputVolumeRange)
        inputVolumeRange.value = String(inputVolume);
      const headphoneVolumeRange = document.getElementById("settingsHeadphoneVolumeRange");
      if (document.activeElement !== headphoneVolumeRange)
        headphoneVolumeRange.value = String(headphoneVolume);
      setElementDisabled("settingsBatteryCapacitySelect", batteryType === "USB-C / DC");
      document.getElementById("settingsCustomBatteryCard").classList.toggle("hidden", batteryCapacity !== "Custom" && batteryType !== "Custom");
      document.querySelectorAll(".custom-battery-voltage").forEach((element) => {
        element.classList.toggle("hidden", batteryType !== "Custom");
      });
      setText(
        "settingsAudioInputNote",
        audioOptions.hasInputDevices
          ? ("Selected: " + resolveSelectEntryLabel(audioOptions.inputDevices, audioInputDevice, "USB audio input"))
          : "No audio input device detected."
      );
      setText(
        "settingsAudioOutputNote",
        audioOptions.hasOutputDevices
          ? ("Selected: " + resolveSelectEntryLabel(audioOptions.outputDevices, audioOutputDevice, "None"))
          : "No audio output device detected."
      );
      setText(
        "settingsBatteryCapacityNote",
        batteryType === "USB-C / DC"
          ? "DC input is shown without battery percentage."
          : (powerSensorAvailable
             ? "Used for remaining runtime estimates."
             : "Used once battery telemetry becomes available.")
      );

      const wbRange = document.getElementById("whiteBalanceRange");
      if (document.activeElement !== wbRange)
        wbRange.value = String(whiteBalanceIndexFromKelvin(displayWhiteBalance));
      setElementDisabled("whiteBalanceRange", whiteBalanceAuto || !whiteBalanceAvailable);
      document.getElementById("whiteBalanceValue").title = whiteBalanceAvailable
        ? "" : "White balance unavailable: this tuning has no calibrated AWB/CT curve.";
      if (!whiteBalanceAvailable) clearTimeout(state.wbCommitTimer);

      const statusPill = document.querySelector(".status-pill");
      const statusDot = document.getElementById("statusDot");
      statusPill.classList.toggle("recording", (isRecording || isTimelapse) && hasMedia);
      statusPill.classList.toggle("timelapse", isTimelapse && hasMedia);
      statusPill.classList.toggle("no-media", !hasMedia);
      statusDot.classList.toggle("recording", (isRecording || isTimelapse) && hasMedia);
      statusDot.classList.toggle("timelapse", isTimelapse && hasMedia);
      statusDot.classList.toggle("no-media", !hasMedia);
      statusDot.classList.toggle("writing", !!snapshot.media_writes_pending && !isRecording && !isTimelapse);

      const recordButton = document.getElementById("recordButton");
      recordButton.disabled = !(isRecording || isTimelapse || isSoundRecording) &&
        (!!snapshot.media_writes_pending || state.photoCaptureInProgress || state.commandInFlightCount > 0);
      const recordIcon = document.getElementById("recordIcon");
      recordButton.classList.toggle("recording", isRecording && hasMedia && !stillModeEnabled);
      recordButton.classList.toggle("photo-mode", stillModeEnabled);
      recordButton.classList.toggle("sound-recorder", soundRecorderEnabled && !stillModeEnabled);
      recordButton.classList.toggle("timelapse", isTimelapse && hasMedia);
      recordButton.classList.toggle("photo-pulse", stillModeEnabled && state.photoCapturePulse);
      recordButton.setAttribute("aria-label", stillModeEnabled
        ? (stillTimelapseArmed ? (isTimelapse ? "Stop timelapse" : "Start timelapse") : "Capture photo")
        : (soundRecorderEnabled ? (isSoundRecording ? "Stop sound recorder" : "Start sound recorder") : "Toggle record"));
      const showTimelapseIcon = stillModeEnabled && (isTimelapse || stillTimelapseValue !== "Off");
      if (showTimelapseIcon)
        recordIcon.className = "record-icon-timelapse";
      else if (stillModeEnabled)
        recordIcon.className = "record-icon-camera";
      else if (soundRecorderEnabled)
        recordIcon.className = "record-icon-microphone";
      else
        recordIcon.className = (isRecording && hasMedia) ? "record-icon-stop" : "record-icon-dot";

      const onlinePill = document.getElementById("onlinePill");
      onlinePill.textContent = snapshot.state === "error" ? "Error" : "Online";
      onlinePill.style.background = snapshot.state === "error" ? "rgba(239,68,68,0.12)" : "rgba(52,211,153,0.1)";
      onlinePill.style.color = snapshot.state === "error" ? "#f87171" : "#34d399";

      const previewStatePill = document.getElementById("settingsPreviewStatus");
      previewStatePill.textContent = snapshot.state === "error" ? "Offline" : "Live";
      previewStatePill.style.background = snapshot.state === "error" ? "rgba(239,68,68,0.12)" : "rgba(52,211,153,0.1)";
      previewStatePill.style.color = snapshot.state === "error" ? "#f87171" : "#34d399";

      const mediaStatePill = document.getElementById("settingsMediaStatePill");
      mediaStatePill.textContent = !hasMedia ? "No Media" : (isRecording ? "Recording" : "Ready");
      mediaStatePill.style.background = !hasMedia
        ? "rgba(250,204,21,0.14)"
        : (isRecording ? "rgba(239,68,68,0.14)" : "rgba(52,211,153,0.1)");
      mediaStatePill.style.color = !hasMedia
        ? "#facc15"
        : (isRecording ? "#f87171" : "#34d399");

      const formatMediaStatePill = document.getElementById("settingsFormatMediaStatePill");
      if (formatMediaStatePill) {
        formatMediaStatePill.textContent = hasMedia ? "Mounted" : "No Media";
        formatMediaStatePill.style.background = hasMedia ? "rgba(52,211,153,0.1)" : "rgba(250,204,21,0.14)";
        formatMediaStatePill.style.color = hasMedia ? "#34d399" : "#facc15";
      }

      const formatMediaButton = document.getElementById("formatMediaButton");
      const ejectMediaButton = document.getElementById("ejectMediaButton");
      const runMediaSpeedTestButton = document.getElementById("runMediaSpeedTestButton");
      if (formatMediaButton)
        formatMediaButton.disabled = !hasMedia;
      setElementDisabled("settingsMediaFormatSelect", !hasMedia);
      if (ejectMediaButton)
        ejectMediaButton.disabled = !hasMedia;
      if (runMediaSpeedTestButton) {
        runMediaSpeedTestButton.disabled = !hasMedia || !!systemState.write_speed_running;
        runMediaSpeedTestButton.textContent = systemState.write_speed_running ? "Testing..." : "Run Speed Test";
      }

      const mediaUsageProgress = document.getElementById("settingsMediaUsageProgress");
      if (mediaUsageProgress)
        mediaUsageProgress.style.width = mediaUsedPercent + "%";

      state.recordingBasePerfMs = performance.now();
      state.recordingBaseElapsedSeconds = Number(snapshot.recording_elapsed_seconds || 0);
      state.timecodeBasePerfMs = state.recordingBasePerfMs;
      state.timecodeBaseLocalMsOfDay = Number(snapshot.timecode_local_ms_of_day || 0);
      updateLiveReadouts();

      buildOptionButtons("fpsOptions", fpsOptions, Number(snapshot.fps).toFixed(3), async (value) => {
        try {
          await sendCommand({ cmd: "set_fps", value: Number(value) });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("isoOptions", availableIsoOptionsForSnapshot(snapshot), snapshot.iso_auto ? "Auto" : String(snapshot.iso), async (value) => {
        try {
          if (value === "Auto")
            await sendCommand({ cmd: "set_iso_auto" });
          else
            await sendCommand({ cmd: "set_iso", value: Number(value) });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("shutterOptions", shutterOptionSet, shutterLabel, async (value) => {
        try {
          if (value === "Auto")
            await sendCommand({ cmd: "set_shutter_auto" });
          else
            await sendCommand({ cmd: "set_shutter_us", value: stillModeEnabled ? shutterSpeedToUs(value) : shutterAngleToUs(value, snapshot.fps) });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("whiteBalanceModeOptions", ["Auto", "Manual"].map((value) => ({
        value: value, enabled: whiteBalanceAvailable
      })), whiteBalanceAuto ? "Auto" : "Manual", async (value) => {
        try {
          if (value === "Auto")
            await sendCommand({ cmd: "set_wb_auto" });
          else
            await sendCommand({ cmd: "set_wb", kelvin: displayWhiteBalance });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("resolutionOptions", resolutionBaseOptions, resolutionBaseValue, async (value) => {
        try {
          await sendCommand(resolutionBaseCommandPayload(snapshot, value));
        } catch (error) {
          showToast(error.message, true);
        }
      }, 3);

      const cropRatioBlock = document.getElementById("cropRatioBlock");
      if (cropRatioBlock)
        cropRatioBlock.classList.toggle("hidden", !cropControlsVisible);
      buildOptionButtons("cropRatioOptions", cropRatioOptions, cropRatioValue, async (value) => {
        try {
          await sendCommand(cropRatioCommandPayload(snapshot, value));
        } catch (error) {
          showToast(error.message, true);
        }
      }, 3);

      buildOptionButtons("settingsRecordFormatOptions", availableRecordFormatOptions(snapshot), recordFormatText, async (value) => {
        try {
          const formatValue = recordingFormatCommandValue(value);
          await sendCommand({ cmd: "set_record_format", value: formatValue });
          if (formatValue === "prores")
            await sendCommand({ cmd: "set_prores_profile", value: proresProfileCommandValue(value) });
          if (formatValue === "lossydng")
            await sendCommand({ cmd: "set_lossy_dng_ratio", value: lossyDngRatioCommandValue(value) });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsRawBitDepthOptions", rawBitDepthOptions.map((value) => ({
        value,
        label: value,
        enabled: rawBitDepthOptionEnabled(snapshot, value, rawBitDepthFixedForH264),
      })), rawBitDepthText, async (value) => {
        try {
          if (rawBitDepthFixedForH264)
            return;
          await sendCommand({ cmd: "set_raw_bit_depth", value: rawBitDepthNumber(value) });
        } catch (error) {
          showToast(error.message, true);
        }
      }, 3);

      buildOptionButtons("settingsGyroDataOptions", toggleOptions, formatToggleLabel(recordGyroEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_record_gyro_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsStillModeOptions", ["Off", "On"], formatToggleLabel(stillModeEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_photo_mode_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsStillLongExposureOptions", [
        {label:"Off",value:"Off",enabled:stillModeEnabled},
        {label:"On",value:"On",enabled:stillModeEnabled},
      ], formatToggleLabel(!!uiState.photo_long_exposure_enabled), async (value) => {
        try {
          await sendCommand({cmd:"set_photo_long_exposure_enabled",enabled:value === "On" ? "true" : "false"});
        } catch (error) { showToast(error.message, true); }
      }, 2);
      buildOptionButtons("settingsStillBracketingOptions", [
        {label:"Off",value:"Off",enabled:stillModeEnabled && !stillTimelapseArmed},
        {label:"On",value:"On",enabled:stillModeEnabled && !stillTimelapseArmed},
      ], formatToggleLabel(!!uiState.photo_bracketing_enabled), async (value) => {
        try {
          await sendCommand({cmd:"set_photo_bracketing_enabled",enabled:value === "On" ? "true" : "false"});
        } catch (error) { showToast(error.message, true); }
      }, 2);

      setText("settingsSoundRecorderValue", formatToggleLabel(soundRecorderEnabled));
      setText("settingsSoundRecorderNote",
              audioOptions.hasInputDevices
                ? "Record WAV only audio"
                : "No audio input device detected.");
      buildOptionButtons("settingsSoundRecorderOptions", [
        { label: "Off", value: "Off", enabled: true },
        { label: "On", value: "On", enabled: audioOptions.hasInputDevices },
      ], formatToggleLabel(soundRecorderEnabled), async (value) => {
        try {
          if (!audioOptions.hasInputDevices) {
            showToast("No audio input device detected.", true);
            return;
          }
          await sendCommand({ cmd: "set_sound_recorder_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      }, 2);

      buildOptionButtons("settingsStillTimelapseToggleOptions", ["Off", "On"], stillTimelapseArmed ? "On" : "Off", async (value) => {
        await applyStillTimelapseToggle(value === "On");
      }, 2);

      buildOptionButtons("settingsTimecodeOptions", timecodeOptions, timecodeText, async (value) => {
        try {
          await sendCommand({ cmd: "set_timecode_mode", value: value === "Rec Run" ? "rec_run" : "free_run" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsAutofocusModeOptions", autofocusModeOptions, autofocusModeText, async (value) => {
        try {
          await sendCommand({ cmd: "set_af_mode", value: value.toLowerCase() });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsAutofocusRangeOptions", autofocusRangeOptions, autofocusRangeText, async (value) => {
        try {
          await sendCommand({ cmd: "set_af_range", value: value.toLowerCase() });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsAutofocusSpeedOptions", autofocusSpeedOptions, autofocusSpeedText, async (value) => {
        try {
          await sendCommand({ cmd: "set_af_speed", value: value.toLowerCase() });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsAutofocusWindowOptions", autofocusWindowOptions, autofocusWindowText === "Custom" ? "Center" : autofocusWindowText, async (value) => {
        try {
          await sendCommand(autofocusWindowPayload(value));
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsDateTimeOverlayOptions", toggleOptions, formatToggleLabel(dateTimeOverlayEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_date_time_overlay_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsExternalMonitorOptions", toggleOptions, formatToggleLabel(externalMonitorEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_external_monitor_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsFocusPeakingOptions", toggleOptions, formatToggleLabel(focusPeakingEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_focus_peaking_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsZebraOptions", toggleOptions, formatToggleLabel(zebraEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_zebra_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsFalseColorOptions", toggleOptions, formatToggleLabel(falseColorEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_false_color_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsLutPreviewOptions", toggleOptions, formatToggleLabel(lutPreviewEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_lut_preview_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsSmpteOptions", toggleOptions, formatToggleLabel(smpteEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_smpte_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsGuidesOptions", toggleOptions, formatToggleLabel(guidesEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_guides_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsCenterMarkerOptions", toggleOptions, formatToggleLabel(centerMarkerEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_center_marker_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsGrayscaleOptions", toggleOptions, formatToggleLabel(grayscaleEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_grayscale_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsAnamorphicOptions", toggleOptions, formatToggleLabel(anamorphicDesqueezeEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_anamorphic_desqueeze_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsRecordAudioOptions", toggleOptions, formatToggleLabel(recordAudioEnabled), async (value) => {
        try {
          if (!audioOptions.hasInputDevices) {
            showToast("No audio input device detected.", true);
            return;
          }
          await sendCommand({ cmd: "set_record_audio_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsLiveAudioMonitoringOptions", toggleOptions, formatToggleLabel(liveAudioMonitoringEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_live_audio_monitoring_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      buildOptionButtons("settingsAudioMeterOptions", toggleOptions, formatToggleLabel(audioMeterEnabled), async (value) => {
        try {
          await sendCommand({ cmd: "set_audio_meter_enabled", enabled: value === "On" ? "true" : "false" });
        } catch (error) {
          showToast(error.message, true);
        }
      });

      setMetric("cpuUsage", formatPercent(snapshot.system.cpu_usage_percent), snapshot.system.cpu_usage_label, snapshot.system.cpu_usage_percent);
      setMetric("cpuTemp", formatDegrees(snapshot.system.cpu_temp_c), snapshot.system.cpu_temp_label, snapshot.system.cpu_temp_percent);
      setMetric("power", formatWatts(snapshot.system.power_draw_w), snapshot.system.power_draw_label, snapshot.system.power_draw_percent);
      setMetric("battery",
                formatVolts(snapshot.system.battery_voltage_v),
                batteryInputPresent ? batteryType : snapshot.system.battery_label,
                isFinite(profileBatteryPercent) ? profileBatteryPercent : 0);
      setMetric(
        "storage",
        hasMedia ? formatStorage(snapshot.system.media_free_bytes) : "No Media",
        snapshot.system.media_label,
        hasMedia ? snapshot.system.media_used_percent : 0
      );
      setMetric(
        "timeLeft",
        mediaRemainingText,
        mediaRemainingSub,
        hasMedia ? snapshot.system.time_left_percent : 0
      );
      setText("timeLeftLabel", mediaRemainingLabel);

      renderFavoritesPanel(snapshot);

      setText("captureCodec", snapshot.capture.codec);
      setText("captureSensor", snapshot.capture.sensor);
      setText("captureStorage", snapshot.capture.storage);
      setText("capturePreview", snapshot.capture.preview);
      setText("captureNetwork", snapshot.capture.network);
    }

    async function fetchState(silent, options) {
      const forceImmediate = !!(options && options.forceImmediate);
      const now = performance.now();
      if (!forceImmediate && state.deferFetchUntilMs > now) {
        scheduleStateRefresh(state.deferFetchUntilMs - now, silent);
        return;
      }

      if (state.fetchInFlight) {
        state.fetchQueued = true;
        state.fetchQueuedSilent = state.fetchQueuedSilent && !!silent;
        return;
      }

      state.fetchInFlight = true;
      const requestEpoch = state.commandEpoch;
      try {
        const response = await fetch("/api/state", { cache: "no-store" });
        const snapshot = await response.json();
        if (state.commandInFlightCount > 0 || requestEpoch !== state.commandEpoch) {
          state.fetchQueued = true;
          state.fetchQueuedSilent = state.fetchQueuedSilent && !!silent;
          return;
        }
        renderSnapshot(snapshot);
      } catch (error) {
        if (!silent)
          showToast("Could not reach ApertarCore web control.", true);
      } finally {
        state.fetchInFlight = false;
        if (state.fetchQueued) {
          const queuedSilent = state.fetchQueuedSilent;
          state.fetchQueued = false;
          state.fetchQueuedSilent = true;
          void fetchState(queuedSilent);
        }
      }
    }

    async function fetchPresetLibrary(silent) {
      if (state.presetsFetchInFlight)
        return;

      state.presetsFetchInFlight = true;
      try {
        const response = await fetch("/api/presets", { cache: "no-store" });
        const payload = await response.json();
        if (!payload.ok)
          throw new Error(payload.error || "Could not load presets.");
        renderPresetLibrary(payload.presets || []);
      } catch (error) {
        if (!silent)
          showToast(error.message || "Could not load presets.", true);
      } finally {
        state.presetsFetchInFlight = false;
      }
    }

    function updateConfigEditorControls() {
      const loading = state.configFetchInFlight;
      const saving = state.configSaveInFlight;
      const editor = document.getElementById("settingsConfigEditor");
      const reloadButton = document.getElementById("reloadConfigButton");
      const saveButton = document.getElementById("saveConfigButton");
      if (reloadButton)
        reloadButton.disabled = loading || saving;
      if (saveButton)
        saveButton.disabled = loading || saving || !state.configDirty;
      if (editor)
        editor.disabled = loading || saving;

      let status = "Ready";
      if (loading)
        status = "Loading";
      else if (saving)
        status = "Saving";
      else if (state.configDirty)
        status = "Unsaved";
      setText("settingsConfigStatePill", status);
    }

    function renderHardwareConfig(payload) {
      const editor = document.getElementById("settingsConfigEditor");
      const text = String(payload && payload.text || "");
      if (editor)
        editor.value = text;
      state.configDirty = false;

      const displayPath = payload && (payload.display_path || payload.path) ? (payload.display_path || payload.path) : "/home/pi/apertar-hardware.conf";
      const exists = !!(payload && payload.exists);
      const sizeText = payload && payload.size_text ? payload.size_text : (text.length + " B");
      const modifiedText = payload && payload.modified_text ? payload.modified_text : "Unknown";
      setText("settingsConfigPathValue", displayPath);
      setText("settingsConfigMetaValue", (exists ? "Saved" : "New file") + " • " + sizeText + " • " + modifiedText);
      updateConfigEditorControls();
    }

    async function fetchConfigFile(silent, forceReload) {
      if (state.configFetchInFlight)
        return;
      if (state.configDirty && forceReload && !window.confirm("Reload Apertar config and discard unsaved edits?"))
        return;

      state.configFetchInFlight = true;
      updateConfigEditorControls();
      try {
        const response = await fetch("/api/config", { cache: "no-store" });
        const payload = await response.json();
        if (!payload.ok)
          throw new Error(payload.error || "Could not load config.");
        renderHardwareConfig(payload);
        if (!silent)
          showToast("Config loaded.", false);
      } catch (error) {
        if (!silent)
          showToast(error.message || "Could not load config.", true);
        updateConfigEditorControls();
      } finally {
        state.configFetchInFlight = false;
        updateConfigEditorControls();
      }
    }

    async function saveConfigFile() {
      const editor = document.getElementById("settingsConfigEditor");
      const text = editor ? editor.value : "";
      if (!text.trim()) {
        showToast("Config file cannot be empty.", true);
        return;
      }

      state.configSaveInFlight = true;
      updateConfigEditorControls();
      try {
        const response = await fetch("/api/config", {
          method: "POST",
          headers: { "Content-Type": "text/plain; charset=utf-8" },
          body: text,
        });
        const payload = await response.json();
        if (!payload.ok)
          throw new Error(payload.error || "Could not save config.");
        state.configDirty = false;
        const backupNote = payload.backup_display_path ? " Backup: " + payload.backup_display_path + "." : "";
        showToast("Config saved." + backupNote, false);
        await fetchConfigFile(true);
      } catch (error) {
        showToast(error.message || "Could not save config.", true);
      } finally {
        state.configSaveInFlight = false;
        updateConfigEditorControls();
      }
    }

    async function fetchMediaLibrary(silent) {
      if (state.mediaFetchInFlight)
        return;

      state.mediaFetchInFlight = true;
      try {
        const response = await fetch("/api/media", { cache: "no-store" });
        const payload = await response.json();
        if (!payload.ok)
          throw new Error(payload.error || "Could not load media.");
        renderMediaLibrary(payload);
      } catch (error) {
        if (!silent)
          showToast(error.message || "Could not load media.", true);
      } finally {
        state.mediaFetchInFlight = false;
      }
    }

    async function deleteMediaLibraryItem(item) {
      if (!item || !item.id || !item.kind)
        return;

      const name = item.name || item.id || "this media item";
      if (!window.confirm("Delete " + name + " from media? This cannot be undone."))
        return;

      try {
        const response = await fetch(
          "/api/media/delete?kind=" + encodeURIComponent(item.kind) + "&id=" + encodeURIComponent(item.id),
          { method: "POST" }
        );
        const payload = await response.json();
        if (!payload.ok)
          throw new Error(payload.error || "Could not delete media.");
        showToast("Deleted " + name + ".", false);
        await fetchMediaLibrary(true);
        await fetchState(true, { forceImmediate: true });
      } catch (error) {
        showToast(error.message || "Could not delete media.", true);
      }
    }

    function downloadSelectedMediaItems() {
      const items = selectedMediaItems();
      if (items.length === 0) {
        showToast("Select media to download.", true);
        return;
      }

      const query = mediaSelectionQuery(items);
      if (!query) {
        showToast("Selected media could not be prepared.", true);
        return;
      }

      const link = document.createElement("a");
      link.href = "/api/media/download-selected?" + query;
      link.download = "Apertar-Media-Selection.zip";
      document.body.appendChild(link);
      link.click();
      link.remove();
      showToast("Preparing " + (items.length === 1 ? "1 item" : items.length + " items") + " for download.", false);
    }

    async function deleteSelectedMediaItems() {
      const items = selectedMediaItems();
      if (items.length === 0) {
        showToast("Select media to delete.", true);
        return;
      }

      const label = items.length === 1 ? (items[0].name || "this media item") : (items.length + " selected media items");
      if (!window.confirm("Delete " + label + "? This cannot be undone."))
        return;

      const query = mediaSelectionQuery(items);
      if (!query) {
        showToast("Selected media could not be prepared.", true);
        return;
      }

      try {
        const response = await fetch("/api/media/delete-selected?" + query, { method: "POST" });
        const payload = await response.json();
        if (!payload.ok)
          throw new Error(payload.error || "Could not delete selected media.");
        state.mediaSelection.clear();
        showToast("Deleted " + (items.length === 1 ? "1 item" : items.length + " items") + ".", false);
        await fetchMediaLibrary(true);
        await fetchState(true, { forceImmediate: true });
      } catch (error) {
        showToast(error.message || "Could not delete selected media.", true);
      }
    }

    async function importPresetFiles(fileList) {
      const files = Array.from(fileList || []).filter((file) => file && file.size > 0);
      if (files.length === 0)
        return;

      let importedCount = 0;
      let existingCount = 0;

      for (const file of files) {
        try {
          const content = await file.text();
          const response = await fetch("/api/presets/import?name=" + encodeURIComponent(file.name), {
            method: "POST",
            headers: {
              "Content-Type": "application/json",
              "X-Preset-Filename": file.name,
            },
            body: content,
          });
          const payload = await response.json();
          if (!payload.ok)
            throw new Error(payload.error || ("Could not import " + file.name + "."));

          if (payload.already_exists)
            existingCount += 1;
          else
            importedCount += 1;
        } catch (error) {
          showToast(error.message || ("Could not import " + file.name + "."), true);
          await fetchPresetLibrary(true);
          return;
        }
      }

      await fetchPresetLibrary(true);

      if (importedCount > 0 && existingCount > 0) {
        showToast("Imported " + importedCount + " preset" + (importedCount === 1 ? "" : "s")
          + ". " + existingCount + " already existed.", false);
      } else if (importedCount > 0) {
        showToast("Imported " + importedCount + " preset" + (importedCount === 1 ? "" : "s") + ".", false);
      } else if (existingCount > 0) {
        showToast(existingCount === 1 ? "Preset already exists in the camera library."
                                      : "All selected presets already exist in the camera library.", false);
      }
    }

    function scalarPresetValue(value) {
      if (Array.isArray(value) && value.every(item => typeof item === "string"))
        return value.join(", ");
      if (typeof value === "string")
        return value;
      if (typeof value === "number" && isFinite(value))
        return String(value);
      if (typeof value === "boolean")
        return value ? "true" : "false";
      return null;
    }

    function buildPresetApplyPayload(presetDocument) {
      const root = presetDocument && typeof presetDocument === "object" ? presetDocument : {};
      const settings = root.settings && typeof root.settings === "object" && !Array.isArray(root.settings)
        ? root.settings
        : root;
      const payload = { cmd: "apply_preset" };

      if (typeof root.name === "string" && root.name.trim())
        payload.presetName = root.name.trim();

      const allowedKeys = [
        "zebraEnabled",
        "zebraThreshold",
        "focusPeakingEnabled",
        "focusPeakingThreshold",
        "focusPeakingColor",
        "grayscaleEnabled",
        "smpteEnabled",
        "anamorphicDesqueezeEnabled",
        "anamorphicRatio",
        "falseColorEnabled",
        "falseColorMode",
        "lutPreviewEnabled",
        "lutPreviewFile",
        "guidesEnabled",
        "guidesType",
        "guidesThickness",
        "centerMarkerEnabled",
        "centerMarkerType",
        "uiOrientation",
        "externalMonitorEnabled",
        "externalMonitorMode",
        "externalMonitorInfoOverlay",
        "externalMonitorOrientation",
        "timecodeMode",
        "photoModeEnabled",
        "photoLongExposureEnabled",
        "photoBracketingEnabled",
        "photoBracketingEv",
        "photoTimer",
        "photoBurst",
        "soundRecorderEnabled",
        "recordAudioEnabled",
        "recordGyroEnabled",
        "liveAudioMonitoringEnabled",
        "audioMeterEnabled",
        "audioInputDevice",
        "audioOutputDevice",
        "audioSampleFormat",
        "audioSampleRate",
        "audioChannels",
        "inputVolume",
        "headphoneVolume",
        "batteryType",
        "batteryCapacity",
        "customBatteryWh",
        "customBatteryFullVoltage",
        "customBatteryEmptyVoltage",
        "sleepMode",
        "standbyTimeoutSeconds",
        "standbyUsbPowerOff",
        "cameraFps",
        "cameraIso",
        "cameraShutterMode",
        "cameraShutterAngle",
        "cameraShutterSpeed",
        "cameraWhiteBalance",
        "cameraResolution",
        "cameraResolutionWidth",
        "cameraResolutionHeight",
        "cameraRecordingFormat",
        "cameraCdngCompression",
        "cameraLossyDngRatio",
        "cameraProresProfile",
        "cameraProresColorProfile",
        "cameraCdngPreviewColorProfile",
        "cameraTuningFile",
        "cameraH264Resolution",
        "cameraAutofocusMode",
        "cameraAutofocusRange",
        "cameraAutofocusSpeed",
        "cameraAutofocusWindowX",
        "cameraAutofocusWindowY",
        "cameraAutofocusWindowWidth",
        "cameraAutofocusWindowHeight",
        "horizontalLevelEnabled",
        "horizontalLevelColor",
        "monitoringOrder", "recordOrder", "audioOrder", "powerOrder", "systemOrder",
        "wifiOrder", "infoOrder", "sidebarOrder", "favoritesOrder",
        "cameraRawBitDepth",
      ];

      for (const key of allowedKeys) {
        if (!Object.prototype.hasOwnProperty.call(settings, key))
          continue;
        const value = scalarPresetValue(settings[key]);
        if (value !== null)
          payload[key] = value;
      }

      return payload;
    }

    async function saveCurrentPreset() {
      const input = document.getElementById("savePresetNameInput");
      const presetName = String(input.value || "").trim();
      if (!presetName) {
        showToast("Enter a preset name first.", true);
        input.focus();
        return;
      }

      try {
        const response = await fetch("/api/presets/save?name=" + encodeURIComponent(presetName), {
          method: "POST",
        });
        const payload = await response.json();
        if (!payload.ok)
          throw new Error(payload.error || "Could not save the preset.");

        input.value = "";
        await fetchPresetLibrary(true);
        showToast(
          (payload.overwritten ? "Updated " : "Saved ")
          + (payload.saved_name || presetName)
          + ".",
          false
        );
      } catch (error) {
        showToast(error.message || "Could not save the preset.", true);
      }
    }

    async function loadPresetFromLibrary(preset) {
      if (!preset || !preset.download_url) {
        showToast("Preset file is unavailable.", true);
        return;
      }

      try {
        const response = await fetch(preset.download_url, { cache: "no-store" });
        const presetDocument = await response.json();
        const payload = buildPresetApplyPayload(presetDocument);
        await sendCommand(payload);
        showToast("Loaded " + (preset.name || "preset") + ".", false);
      } catch (error) {
        showToast(error.message || "Could not load the preset.", true);
      }
    }

    async function deletePresetFromLibrary(preset) {
      const presetName = preset && (preset.name || preset.file_name) ? (preset.name || preset.file_name) : "this preset";
      if (!preset || !preset.file_name) {
        showToast("Preset file is unavailable.", true);
        return;
      }
      if (!window.confirm("Delete " + presetName + " from the camera preset library?"))
        return;

      try {
        const response = await fetch("/api/presets/delete?file=" + encodeURIComponent(preset.file_name), {
          method: "POST",
        });
        const payload = await response.json();
        if (!payload.ok)
          throw new Error(payload.error || "Could not delete the preset.");

        await fetchPresetLibrary(true);
        showToast("Deleted " + (payload.deleted_name || presetName) + ".", false);
      } catch (error) {
        showToast(error.message || "Could not delete the preset.", true);
      }
    }

    async function toggleRecord() {
      if (!state.snapshot)
        return;
      if (!state.snapshot.recording && !state.snapshot.timelapse_active &&
          (state.snapshot.media_writes_pending || state.photoCaptureInProgress || state.commandInFlightCount > 0))
        return;
      try {
        const uiState = state.snapshot.ui || {};
        const stillModeEnabled = !!uiState.photo_mode_enabled;
        if (state.snapshot.timelapse_active) {
          await sendCommand({ cmd: "timelapse_stop" });
          showToast("Timelapse stopped.", false);
          await fetchState(true);
          return;
        }

        if (stillModeEnabled) {
          const timelapseValue = uiState.photo_timelapse || "Off";
          if (timelapseValue !== "Off") {
            if (!state.snapshot.media_mounted) {
              showToast("No media mounted.", true);
              return;
            }
            await sendCommand({
              cmd: "timelapse_start",
              value: timelapseValue,
              format: safeTimelapseStillFormat(uiState.photo_format || "DNG"),
            });
            showToast("Timelapse started.", false);
            await fetchState(true);
            return;
          }

          if (!state.snapshot.media_mounted) {
            showToast("No media mounted.", true);
            return;
          }
          await beginPhotoCaptureSequence();
          return;
        }

        if (state.snapshot.recording) {
          await sendCommand({ cmd: "record_stop" });
          showToast(state.snapshot.sound_recording ? "Sound recording stopped." : "Recording stopped.", false);
        } else {
          if (!state.snapshot.media_mounted) {
            showToast("No media mounted.", true);
            return;
          }
          const audioOptions = currentAudioOptions(state.snapshot);
          const inputDevice = String(uiState.audio_input_device || "").trim();
          const outputDevice = String(uiState.audio_output_device || "").trim();
          const inputVolume = clamp(Number(uiState.input_volume) || 60, 0, 100);
          const outputVolume = clamp(Number(uiState.headphone_volume) || 55, 0, 100);
          const soundRecorderEnabled = !!uiState.sound_recorder_enabled && audioOptions.hasInputDevices;
          if (uiState.sound_recorder_enabled && !audioOptions.hasInputDevices) {
            showToast("No audio input device detected.", true);
            return;
          }
          const audioEnabled = soundRecorderEnabled || (!!uiState.record_audio_enabled
            && audioOptions.hasInputDevices
            && inputDevice.length > 0);
          const monitoringEnabled = audioEnabled
            && !!uiState.live_audio_monitoring_enabled
            && outputDevice.length > 0
            && outputVolume > 0;
          const payload = {
            cmd: "record_start",
            sound_only: soundRecorderEnabled ? "true" : "false",
            audio_enabled: audioEnabled ? "true" : "false",
            gyro_enabled: (!soundRecorderEnabled && uiState.record_gyro_enabled) ? "true" : "false",
            audio_monitoring_enabled: monitoringEnabled ? "true" : "false",
            audio_input_volume: inputVolume,
            audio_output_volume: outputVolume,
          };
          if (audioEnabled)
            payload.audio_input_device = inputDevice;
          if (monitoringEnabled)
            payload.audio_output_device = outputDevice;

          await sendCommand(payload);
          showToast(soundRecorderEnabled ? "Sound recording started." : "Recording started.", false);
        }
      } catch (error) {
        showToast(error.message, true);
      }
    }

    function parseDateTimeInputParts(value) {
      const match = /^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2})$/.exec(String(value || ""));
      if (!match)
        return null;
      return {
        year: Number(match[1]),
        month: Number(match[2]),
        day: Number(match[3]),
        hour: Number(match[4]),
        minute: Number(match[5]),
      };
    }

    async function applySelectCommand(selectId, commandName, transformValue) {
      const select = document.getElementById(selectId);
      const value = typeof transformValue === "function" ? transformValue(select.value) : select.value;
      try {
        await sendCommand({ cmd: commandName, value });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyUiOrientationSelection() {
      const select = document.getElementById("settingsUiOrientationSelect");
      try {
        await sendCommand({ cmd: "set_ui_orientation", value: select.value });
        showToast("UI orientation updated.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyControlsOpacitySelection() {
      const select = document.getElementById("settingsControlsOpacitySelect");
      try {
        await sendCommand({ cmd: "set_camera_controls_opacity", value: opacityLabelToValue(select.value) });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyControlsModeSelection() {
      const select = document.getElementById("settingsControlsModeSelect");
      try {
        await sendCommand({ cmd: "set_camera_controls_mode", value: select.value });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyStillTimerSelection() {
      const select = document.getElementById("settingsStillTimerSelect");
      try {
        await sendCommand({ cmd: "set_photo_timer", value: select.value });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyStillFormatSelection() {
      const select = document.getElementById("settingsStillFormatSelect");
      const uiState = state.snapshot && state.snapshot.ui ? state.snapshot.ui : {};
      const timelapseArmed = stillTimelapseEnabled(uiState.photo_timelapse) || !!(state.snapshot && state.snapshot.timelapse_active);
      const selectedFormat = timelapseArmed ? safeTimelapseStillFormat(select.value) : select.value;
      if (select.value !== selectedFormat)
        select.value = selectedFormat;

      try {
        if (timelapseArmed) {
          await sendCommand({
            cmd: "set_photo_timelapse",
            value: defaultStillTimelapseInterval(state.snapshot, uiState.photo_timelapse),
            format: selectedFormat,
          });
        } else {
          await sendCommand({ cmd: "set_photo_format", value: selectedFormat });
        }
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyStillBurstSelection() {
      const select = document.getElementById("settingsStillBurstSelect");
      try {
        if (state.snapshot && state.snapshot.ui.photo_bracketing_enabled)
          await sendCommand({cmd: "set_photo_bracketing_ev", value: (bracketEvOptions.indexOf(select.value) + 1) / 3});
        else await sendCommand({ cmd: "set_photo_burst", value: select.value });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyStillTimelapseSelection() {
      const select = document.getElementById("settingsStillTimelapseSelect");
      state.customIntervalEditing = select.value === "Custom...";
      document.getElementById("settingsStillCustomIntervalField").classList.toggle("hidden", !state.customIntervalEditing);
      if (state.customIntervalEditing)
        return;
      const formatSelect = document.getElementById("settingsStillFormatSelect");
      const uiState = state.snapshot && state.snapshot.ui ? state.snapshot.ui : {};
      if (!stillTimelapseEnabled(uiState.photo_timelapse) && !(state.snapshot && state.snapshot.timelapse_active))
        return;

      let selectedFormat = formatSelect ? formatSelect.value : "DNG";
      selectedFormat = safeTimelapseStillFormat(selectedFormat);
      if (formatSelect && formatSelect.value !== selectedFormat)
        formatSelect.value = selectedFormat;

      try {
        await sendCommand({ cmd: "set_photo_timelapse", value: select.value, format: selectedFormat });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyStillTimelapseToggle(enabled) {
      const select = document.getElementById("settingsStillTimelapseSelect");
      const formatSelect = document.getElementById("settingsStillFormatSelect");
      const uiState = state.snapshot && state.snapshot.ui ? state.snapshot.ui : {};
      const interval = enabled
        ? defaultStillTimelapseInterval(state.snapshot, select && select.value && select.value !== "Custom..." ? select.value : uiState.photo_timelapse)
        : "Off";
      let selectedFormat = formatSelect ? formatSelect.value : (uiState.photo_format || "DNG");
      if (enabled)
        selectedFormat = safeTimelapseStillFormat(selectedFormat);
      if (formatSelect && formatSelect.value !== selectedFormat)
        formatSelect.value = selectedFormat;

      try {
        await sendCommand({ cmd: "set_photo_timelapse", value: interval, format: selectedFormat });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyRecordResolutionSelection() {
      const select = document.getElementById("settingsRecordResolutionSelect");
      try {
        await sendCommand(resolutionBaseCommandPayload(state.snapshot, select.value));
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyRecordCropSelection() {
      const select = document.getElementById("settingsRecordCropSelect");
      try {
        await sendCommand(cropRatioCommandPayload(state.snapshot, select.value));
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyH264ResolutionSelection() {
      const select = document.getElementById("settingsH264ResolutionSelect");
      if (!select)
        return;

      try {
        await sendCommand(h264ResolutionCommandPayload(select.value));
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyImuDriverSelection() {
      const select = document.getElementById("settingsImuDriverSelect");
      try {
        state.gyroCalibrationStatus = "";
        await sendCommand({ cmd: "set_imu_driver", value: imuDriverValue(select.value) });
        await fetchState(true, { forceImmediate: true });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyImuOrientationSelection() {
      const select = document.getElementById("settingsImuOrientationSelect");
      try {
        state.gyroCalibrationStatus = "";
        await sendCommand({ cmd: "set_imu_orientation", value: select.value });
        await fetchState(true, { forceImmediate: true });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function adjustImuSyncOffset(delta) {
      try {
        state.gyroCalibrationStatus = "";
        await sendCommand({ cmd: "adjust_imu_sync_offset", delta });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function resetImuSyncOffset() {
      try {
        state.gyroCalibrationStatus = "";
        await sendCommand({ cmd: "reset_imu_sync_offset" });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function setImuLevelReference() {
      try {
        state.gyroCalibrationStatus = "";
        await sendCommand({ cmd: "set_imu_level_reference" });
        showToast("Gyro level reference saved.", false);
        await fetchState(true, { forceImmediate: true });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function clearImuLevelReference() {
      try {
        state.gyroCalibrationStatus = "";
        await sendCommand({ cmd: "clear_imu_level_reference" });
        showToast("Gyro level reference cleared.", false);
        await fetchState(true, { forceImmediate: true });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function calibrateImuBias() {
      if (state.gyroCalibrationRunning)
        return;

      state.gyroCalibrationRunning = true;
      state.gyroCalibrationOk = false;
      state.gyroCalibrationStatus = "Keep the camera still. Calibrating gyro bias...";
      if (state.snapshot)
        renderSnapshot(state.snapshot);

      try {
        const result = await sendCommand({ cmd: "calibrate_gyro", seconds: 2.5 });
        state.gyroCalibrationOk = true;
        state.gyroCalibrationStatus = "Gyro bias saved"
          + (result && result.samples ? (" from " + result.samples + " samples.") : ".");
        showToast("Gyro calibration saved.", false);
        await fetchState(true, { forceImmediate: true });
      } catch (error) {
        state.gyroCalibrationOk = false;
        state.gyroCalibrationStatus = error.message || "Gyro calibration failed.";
        showToast(state.gyroCalibrationStatus, true);
      } finally {
        state.gyroCalibrationRunning = false;
        if (state.snapshot)
          renderSnapshot(state.snapshot);
      }
    }

    async function applyDateTimeSettings() {
      const timeZoneSelect = document.getElementById("settingsTimeZoneSelect");
      const dateTimeInput = document.getElementById("settingsDateTimeInput");
      const parts = parseDateTimeInputParts(dateTimeInput.value);
      if (!parts) {
        showToast("Choose a valid local date and time first.", true);
        return;
      }

      try {
        await sendCommand({
          cmd: "apply_date_time",
          year: parts.year,
          month: parts.month,
          day: parts.day,
          hour: parts.hour,
          minute: parts.minute,
          time_zone: timeZoneSelect.value,
        });
        showToast("Date and time applied.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyTimeZoneSelection(selectOrElement) {
      const select = typeof selectOrElement === "string"
        ? document.getElementById(selectOrElement)
        : selectOrElement;
      if (!select)
        return;

      const previousValue = state.snapshot && state.snapshot.system
        ? String(state.snapshot.system.time_zone || "UTC")
        : String(select.value || "UTC");

      try {
        const result = await sendCommand({
          cmd: "apply_time_zone",
          time_zone: select.value,
        });

        if (result && typeof result.local_time_input === "string") {
          const dateTimeInput = document.getElementById("settingsDateTimeInput");
          if (dateTimeInput && document.activeElement !== dateTimeInput)
            dateTimeInput.value = result.local_time_input;
        }
      } catch (error) {
        select.value = previousValue;
        showToast(error.message, true);
      }
    }

    async function applyFanModeSelection() {
      const select = document.getElementById("settingsFanModeSelect");
      if (select.disabled)
        return;
      try {
        await sendCommand({ cmd: "set_fan_mode", value: select.value });
        showToast("Fan mode updated.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function refreshAudioCaptureCapabilities(device) {
      if (state.audioCapabilitiesBusy) return;
      device = device || ((state.snapshot || {}).ui || {}).audio_input_device || document.getElementById("settingsAudioInputDeviceSelect").value;
      state.audioCapabilityDevice = device;
      state.audioCapabilitiesBusy = true;
      state.audioCaptureModes = [];
      state.audioCapabilityError = "";
      renderAudioCaptureOptions();
      try {
        const reply = await sendCommand({cmd: "audio_capabilities", device});
        state.audioCaptureModes = reply.modes || [];
      } catch (error) { state.audioCapabilityError = error.message; }
      finally { state.audioCapabilitiesBusy = false; renderAudioCaptureOptions(); }
    }

    function renderAudioCaptureOptions() {
      const ui = (state.snapshot || {}).ui || {};
      const format = ui.audio_sample_format || "auto";
      const rate = Number(ui.audio_sample_rate) || 0;
      const channels = ui.audio_channels || "auto";
      const modes = state.audioCapabilityDevice === (ui.audio_input_device || "") ? state.audioCaptureModes : [];
      const labels = {s16: "16-bit PCM", s24: "24-bit PCM", s32: "32-bit PCM", float32: "32-bit float (native)"};
      const formats = [{id: "auto", label: "Auto"}];
      for (const id of Object.keys(labels)) if (modes.some(m => m.format === id)) formats.push({id, label: labels[id]});
      const matching = modes.filter(m => format === "auto" || m.format === format);
      const rates = [{id: "0", label: "Auto"}, ...[...new Set(matching.map(m => m.rate))].sort((a,b) => a-b).map(r => ({id: String(r), label: (r/1000) + " kHz"}))];
      const available = matching.filter(m => !rate || m.rate === rate);
      const maxChannels = Math.max(0, ...available.map(m => m.channels));
      const inputs = [{id: "auto", label: "Auto"}];
      for (let ch = 1; ch <= maxChannels; ++ch) inputs.push({id: String(ch), label: "Input " + ch});
      for (let ch = 1; ch < maxChannels; ch += 2) inputs.push({id: ch + "-" + (ch+1), label: "Inputs " + ch + "-" + (ch+1)});
      if (maxChannels > 2) inputs.push({id: "all", label: "All inputs"});
      for (const [id, entries, value] of [["settingsAudioFormatSelect", formats, format], ["settingsAudioRateSelect", rates, String(rate)], ["settingsAudioChannelsSelect", inputs, channels]]) {
        ensureSelectEntries(id, entries);
        setFormValue(id, value);
        setElementDisabled(id, state.audioCapabilitiesBusy || !modes.length || !!(state.snapshot || {}).recording);
      }
      setElementDisabled("refreshAudioCapabilitiesButton", state.audioCapabilitiesBusy || !!(state.snapshot || {}).recording);
      let note = "Auto prefers native 48 kHz / 16-bit, up to two channels. Float requires device support.";
      const inputCount = channels === "all" ? maxChannels : channels === "auto" ? Math.min(2, maxChannels) : channels.includes("-") ? Number(channels.split("-")[1]) - Number(channels.split("-")[0]) + 1 : 1;
      if (rate && format !== "auto") note = "PCM bitrate: " + (rate * inputCount * (format === "s16" ? 16 : format === "s24" ? 24 : 32) / 1000) + " kbit/s. Native device support only.";
      setText("settingsAudioCaptureNote", state.audioCapabilitiesBusy ? "Checking native input capabilities..." : state.audioCapabilityError || note);
    }

    async function applyAudioCaptureOptions(kind) {
      const payload = {cmd: "set_audio_capture_options"};
      if (kind === "format") { payload.format = document.getElementById("settingsAudioFormatSelect").value; payload.rate = 0; payload.channels = "auto"; }
      if (kind === "rate") { payload.rate = Number(document.getElementById("settingsAudioRateSelect").value); payload.channels = "auto"; }
      if (kind === "channels") payload.channels = document.getElementById("settingsAudioChannelsSelect").value;
      try { await sendCommand(payload); } catch (error) { showToast(error.message, true); }
    }

    async function applyAudioInputDeviceSelection() {
      const select = document.getElementById("settingsAudioInputDeviceSelect");
      try {
        await sendCommand({ cmd: "set_audio_input_device", value: select.value });
        await refreshAudioCaptureCapabilities(select.value);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyAudioOutputDeviceSelection() {
      const select = document.getElementById("settingsAudioOutputDeviceSelect");
      try {
        await sendCommand({ cmd: "set_audio_output_device", value: select.value });
      } catch (error) {
        showToast(error.message, true);
      }
    }

    function scheduleInputVolumeCommit() {
      const input = document.getElementById("settingsInputVolumeRange");
      const volume = clamp(Number(input.value) || 0, 0, 100);
      setText("settingsInputVolumeValue", String(volume));
      clearTimeout(state.inputVolumeCommitTimer);
      state.inputVolumeCommitTimer = window.setTimeout(async () => {
        try {
          await sendCommand({ cmd: "set_input_volume", value: volume });
        } catch (error) {
          showToast(error.message, true);
        }
      }, 140);
    }

    function scheduleHeadphoneVolumeCommit() {
      const input = document.getElementById("settingsHeadphoneVolumeRange");
      const volume = clamp(Number(input.value) || 0, 0, 100);
      setText("settingsHeadphoneVolumeValue", String(volume));
      clearTimeout(state.headphoneVolumeCommitTimer);
      state.headphoneVolumeCommitTimer = window.setTimeout(async () => {
        try {
          await sendCommand({ cmd: "set_headphone_volume", value: volume });
        } catch (error) {
          showToast(error.message, true);
        }
      }, 140);
    }

    async function applyBatteryCapacitySelection() {
      const select = document.getElementById("settingsBatteryCapacitySelect");
      try {
        await sendCommand({ cmd: "set_battery_capacity", value: select.value });
        showToast("Battery capacity updated.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applyBatteryTypeSelection() {
      const select = document.getElementById("settingsBatteryTypeSelect");
      try {
        await sendCommand({ cmd: "set_battery_type", value: select.value });
        showToast("Battery profile updated.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function adjustCustomBatteryWh(delta) {
      const currentSnapshot = state.snapshot && state.snapshot.ui ? state.snapshot.ui : {};
      const currentValue = clamp(Number(currentSnapshot.custom_battery_wh) || 150, 10, 500);
      const nextValue = clamp(currentValue + delta, 10, 500);
      if (nextValue === currentValue)
        return;

      try {
        await sendCommand({ cmd: "set_custom_battery_wh", value: nextValue });
        showToast("Custom battery capacity updated.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function adjustCustomBatteryVoltage(kind, delta) {
      const currentSnapshot = state.snapshot && state.snapshot.ui ? state.snapshot.ui : {};
      const fullVoltage = clamp(Number(currentSnapshot.custom_battery_full_voltage) || 16.8, 0.5, 60);
      const emptyVoltage = clamp(Number(currentSnapshot.custom_battery_empty_voltage) || 13.2, 0.5, fullVoltage - 0.1);
      const payload = { cmd: "set_custom_battery_voltage" };
      if (kind === "full")
        payload.full = clamp(Math.round((fullVoltage + delta) * 10) / 10, emptyVoltage + 0.1, 60);
      else
        payload.empty = clamp(Math.round((emptyVoltage + delta) * 10) / 10, 0.5, fullVoltage - 0.1);

      try {
        await sendCommand(payload);
        showToast("Custom battery voltage updated.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function applySleepModeSelection() {
      const select = document.getElementById("settingsSleepModeSelect");
      try {
        await sendCommand({ cmd: "set_sleep_mode", value: select.value });
        showToast("Auto power off updated.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function requestFormatMedia() {
      const formatSelect = document.getElementById("settingsMediaFormatSelect");
      const filesystem = formatSelect && formatSelect.value ? formatSelect.value : "exfat";
      const formatLabel = filesystem === "vfat" ? "FAT32" : (filesystem === "ntfs" ? "NTFS" : "exFAT");
      if (!window.confirm("Format the mounted media as " + formatLabel + " now? This will erase it and keep the current drive name where possible."))
        return;

      try {
        await sendCommand({ cmd: "format_media", filesystem });
        showToast("Media formatted as " + formatLabel + ".", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function requestEjectMedia() {
      if (!window.confirm("Eject the mounted media now?"))
        return;

      try {
        await sendCommand({ cmd: "eject_media" });
        showToast("Media ejected.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function requestMountMediaDrive() {
      const select = document.getElementById("settingsMediaDriveSelect");
      const devicePath = select ? select.value : "";
      if (!devicePath) {
        showToast("No available non-system drive selected.", true);
        return;
      }

      try {
        await sendCommand({ cmd: "mount_media", device_path: devicePath });
        showToast("Media mounted.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function requestMediaSpeedTest() {
      try {
        await sendCommand({ cmd: "run_media_speed_test" });
        showToast("Write speed test started.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function requestStandby() {
      try {
        const result = await sendCommand({cmd: state.snapshot && state.snapshot.state === "standby" ? "wake" : "enter_standby"});
        if (result.warning)
          showToast(result.warning, true);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function requestRestartSystem() {
      if (!window.confirm("Restart the Apertar system now?"))
        return;

      try {
        await sendCommand({ cmd: "restart_system" });
        showToast("Restart requested.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function requestPowerOffSystem() {
      if (!window.confirm("Shut down the Apertar system now?"))
        return;

      try {
        await sendCommand({ cmd: "power_off_system" });
        showToast("Shutdown requested.", false);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    function installInteractions() {
      arrangeSettingsPanels();

      document.getElementById("recordButton").addEventListener("click", toggleRecord);
      document.getElementById("settingsButton").addEventListener("click", () => setSettingsOpen(true));
      document.getElementById("mediaBrowserButton").addEventListener("click", () => setMediaBrowserOpen(true));
      document.getElementById("closeMediaBrowserButton").addEventListener("click", () => setMediaBrowserOpen(false));
      document.getElementById("refreshMediaBrowserButton").addEventListener("click", () => fetchMediaLibrary(false));
      document.getElementById("mediaBrowserSortSelect").addEventListener("change", (event) => {
        setMediaBrowserSort(event.target.value);
      });
      document.getElementById("selectAllMediaButton").addEventListener("click", selectAllVisibleMedia);
      document.getElementById("clearMediaSelectionButton").addEventListener("click", clearMediaSelection);
      document.getElementById("downloadSelectedMediaButton").addEventListener("click", downloadSelectedMediaItems);
      document.getElementById("deleteSelectedMediaButton").addEventListener("click", () => {
        void deleteSelectedMediaItems();
      });
      document.getElementById("closeSettingsButton").addEventListener("click", () => setSettingsOpen(false));
      document.querySelectorAll(".media-filter-button").forEach((button) => {
        button.addEventListener("click", () => setMediaBrowserFilter(button.dataset.mediaFilter));
      });
      document.getElementById("clearFavoritesButton").addEventListener("click", () => {
        void clearFavorites();
      });
      document.getElementById("settingsStillFormatSelect").addEventListener("change", applyStillFormatSelection);
      document.getElementById("settingsStillTimerSelect").addEventListener("change", applyStillTimerSelection);
      document.getElementById("settingsStillBurstSelect").addEventListener("change", applyStillBurstSelection);
      document.getElementById("settingsStillTimelapseSelect").addEventListener("change", applyStillTimelapseSelection);
      document.getElementById("settingsStillCustomIntervalApply").addEventListener("click", async () => {
        const input = document.getElementById("settingsStillCustomIntervalInput");
        const seconds = Number(input.value);
        if (!input.checkValidity() || !Number.isFinite(seconds) || seconds < 0.1 || seconds > 86400) {
          showToast("Enter an interval from 0.1 to 86400 seconds, with one decimal place.", true);
          return;
        }
        try {
          const ui = state.snapshot && state.snapshot.ui || {};
          await sendCommand({cmd:"set_photo_timelapse",value:seconds + "s",format:safeTimelapseStillFormat(ui.photo_format || "DNG")});
          state.customIntervalEditing = false;
          document.getElementById("settingsStillCustomIntervalField").classList.add("hidden");
        } catch (error) { showToast(error.message, true); }
      });
      document.getElementById("settingsRecordResolutionSelect").addEventListener("change", applyRecordResolutionSelection);
      document.getElementById("settingsExposureCompensationSelect").addEventListener("change", async (event) => {
        try { await sendCommand({cmd:"set_exposure_compensation",value:Number(event.target.value)}); }
        catch (error) { showToast(error.message, true); }
      });
      document.getElementById("settingsExposureMeteringSelect").addEventListener("change", async (event) => {
        try { await sendCommand({cmd:"set_exposure_metering",value:event.target.value}); }
        catch (error) { showToast(error.message, true); }
      });
      document.getElementById("settingsProresColorSelect").addEventListener("change", async (event) => {
        try {
          await sendCommand({ cmd: "set_prores_color_profile", value: event.target.value });
        } catch (error) { showToast(error.message, true); }
      });
      document.getElementById("settingsCdngPreviewColorSelect").addEventListener("change", async (event) => {
        try {
          await sendCommand({ cmd: "set_cdng_preview_color_profile", value: event.target.value });
        } catch (error) { showToast(error.message, true); }
      });
      document.getElementById("settingsCdngCompressionSelect").addEventListener("change", async (event) => {
        try {
          await dispatchCdngCompressionSelection(event.target.value, sendCommand);
        } catch (error) {
          showToast(error.message, true);
        }
      });
      document.getElementById("settingsRecordCropSelect").addEventListener("change", applyRecordCropSelection);
      document.getElementById("settingsH264ResolutionSelect").addEventListener("change", applyH264ResolutionSelection);
      document.getElementById("settingsImuDriverSelect").addEventListener("change", applyImuDriverSelection);
      document.getElementById("settingsImuOrientationSelect").addEventListener("change", applyImuOrientationSelection);
      document.getElementById("settingsImuSyncMinusButton").addEventListener("click", () => {
        void adjustImuSyncOffset(-1);
      });
      document.getElementById("settingsImuSyncPlusButton").addEventListener("click", () => {
        void adjustImuSyncOffset(1);
      });
      document.getElementById("settingsImuSyncResetButton").addEventListener("click", resetImuSyncOffset);
      document.getElementById("settingsImuSetLevelButton").addEventListener("click", setImuLevelReference);
      document.getElementById("settingsImuClearLevelButton").addEventListener("click", clearImuLevelReference);
      document.getElementById("settingsImuCalibrateButton").addEventListener("click", calibrateImuBias);
      document.getElementById("settingsExternalMonitorModeSelect").addEventListener("change", () => {
        void applySelectCommand("settingsExternalMonitorModeSelect", "set_external_monitor_mode");
      });
      document.getElementById("settingsExternalMonitorInfoOverlaySelect").addEventListener("change", () => {
        void applySelectCommand("settingsExternalMonitorInfoOverlaySelect", "set_external_monitor_info_overlay");
      });
      document.getElementById("settingsExternalMonitorOrientationSelect").addEventListener("change", () => {
        void applySelectCommand("settingsExternalMonitorOrientationSelect", "set_external_monitor_orientation");
      });
      document.getElementById("settingsFocusPeakingThresholdSelect").addEventListener("change", () => {
        void applySelectCommand("settingsFocusPeakingThresholdSelect", "set_focus_peaking_threshold", focusThresholdToValue);
      });
      document.getElementById("settingsFocusPeakingColorSelect").addEventListener("change", () => {
        void applySelectCommand("settingsFocusPeakingColorSelect", "set_focus_peaking_color");
      });
      document.getElementById("settingsZebraLevelSelect").addEventListener("change", () => {
        void applySelectCommand("settingsZebraLevelSelect", "set_zebra_threshold", zebraLevelToThreshold);
      });
      document.getElementById("settingsFalseColorModeSelect").addEventListener("change", () => {
        void applySelectCommand("settingsFalseColorModeSelect", "set_false_color_mode");
      });
      document.getElementById("settingsLutPreviewSelect").addEventListener("change", () => {
        void applySelectCommand("settingsLutPreviewSelect", "set_lut_preview_file");
      });
      document.getElementById("settingsGuidesTypeSelect").addEventListener("change", () => {
        void applySelectCommand("settingsGuidesTypeSelect", "set_guides_type");
      });
      document.getElementById("settingsGuidesThicknessSelect").addEventListener("change", () => {
        void applySelectCommand("settingsGuidesThicknessSelect", "set_guides_thickness", (value) => Number(value));
      });
      document.getElementById("settingsCenterMarkerTypeSelect").addEventListener("change", () => {
        void applySelectCommand("settingsCenterMarkerTypeSelect", "set_center_marker_type");
      });
      document.getElementById("settingsAnamorphicRatioSelect").addEventListener("change", () => {
        void applySelectCommand("settingsAnamorphicRatioSelect", "set_anamorphic_ratio");
      });
      document.getElementById("settingsUiOrientationSelect").addEventListener("change", applyUiOrientationSelection);
      document.getElementById("settingsControlsOpacitySelect").addEventListener("change", applyControlsOpacitySelection);
      document.getElementById("settingsControlsModeSelect").addEventListener("change", applyControlsModeSelection);
      document.getElementById("settingsTimeZoneSelect").addEventListener("change", () => {
        void applyTimeZoneSelection("settingsTimeZoneSelect");
      });
      document.getElementById("applyDateTimeButton").addEventListener("click", applyDateTimeSettings);
      document.getElementById("settingsFanModeSelect").addEventListener("change", applyFanModeSelection);
      document.getElementById("settingsAudioInputDeviceSelect").addEventListener("change", applyAudioInputDeviceSelection);
      document.getElementById("settingsAudioFormatSelect").addEventListener("change", () => applyAudioCaptureOptions("format"));
      document.getElementById("settingsAudioRateSelect").addEventListener("change", () => applyAudioCaptureOptions("rate"));
      document.getElementById("settingsAudioChannelsSelect").addEventListener("change", () => applyAudioCaptureOptions("channels"));
      document.getElementById("refreshAudioCapabilitiesButton").addEventListener("click", () => refreshAudioCaptureCapabilities());
      document.getElementById("settingsAudioOutputDeviceSelect").addEventListener("change", applyAudioOutputDeviceSelection);
      document.getElementById("settingsInputVolumeRange").addEventListener("input", scheduleInputVolumeCommit);
      document.getElementById("settingsHeadphoneVolumeRange").addEventListener("input", scheduleHeadphoneVolumeCommit);
      document.getElementById("settingsBatteryTypeSelect").addEventListener("change", applyBatteryTypeSelection);
      document.getElementById("settingsBatteryCapacitySelect").addEventListener("change", applyBatteryCapacitySelection);
      document.getElementById("decreaseCustomBatteryWhButton").addEventListener("click", () => {
        void adjustCustomBatteryWh(-5);
      });
      document.getElementById("increaseCustomBatteryWhButton").addEventListener("click", () => {
        void adjustCustomBatteryWh(5);
      });
      document.getElementById("decreaseCustomBatteryFullVoltageButton").addEventListener("click", () => {
        void adjustCustomBatteryVoltage("full", -0.1);
      });
      document.getElementById("increaseCustomBatteryFullVoltageButton").addEventListener("click", () => {
        void adjustCustomBatteryVoltage("full", 0.1);
      });
      document.getElementById("decreaseCustomBatteryEmptyVoltageButton").addEventListener("click", () => {
        void adjustCustomBatteryVoltage("empty", -0.1);
      });
      document.getElementById("increaseCustomBatteryEmptyVoltageButton").addEventListener("click", () => {
        void adjustCustomBatteryVoltage("empty", 0.1);
      });
      document.getElementById("settingsSleepModeSelect").addEventListener("change", applySleepModeSelection);
      let lastStandbyActivity = 0;
      const notifyStandbyActivity = () => {
        const now = performance.now();
        if (now - lastStandbyActivity < 1000)
          return;
        lastStandbyActivity = now;
        fetch("/api/command", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({cmd: "user_activity"})
        }).catch(() => {});
      };
      document.addEventListener("pointerdown", notifyStandbyActivity, {passive: true});
      document.addEventListener("keydown", notifyStandbyActivity);
      document.getElementById("settingsStandbySelect").addEventListener("change", async (event) => {
        try {
          await sendCommand({cmd: "set_standby_timeout", value: Number(event.target.value)});
          showToast("Camera standby updated.", false);
        } catch (error) {
          showToast(error.message, true);
        }
      });
      document.getElementById("settingsStandbyUsbSelect").addEventListener("change", async (event) => {
        try {
          await sendCommand({cmd: "set_standby_usb_power_off", enabled: event.target.value === "true"});
        } catch (error) {
          showToast(error.message, true);
        }
      });
      document.getElementById("settingsStandbyButton").addEventListener("click", requestStandby);
      document.getElementById("formatMediaButton").addEventListener("click", requestFormatMedia);
      document.getElementById("ejectMediaButton").addEventListener("click", requestEjectMedia);
      document.getElementById("refreshMediaDrivesButton").addEventListener("click", () => fetchState(false));
      document.getElementById("mountMediaDriveButton").addEventListener("click", requestMountMediaDrive);
      document.getElementById("runMediaSpeedTestButton").addEventListener("click", requestMediaSpeedTest);
      document.getElementById("restartSystemButton").addEventListener("click", requestRestartSystem);
      document.getElementById("powerOffSystemButton").addEventListener("click", requestPowerOffSystem);
      document.getElementById("saveCurrentPresetButton").addEventListener("click", saveCurrentPreset);
      document.getElementById("savePresetNameInput").addEventListener("keydown", (event) => {
        if (event.key === "Enter") {
          event.preventDefault();
          void saveCurrentPreset();
        }
      });
      document.getElementById("importPresetButton").addEventListener("click", () => {
        document.getElementById("presetImportInput").click();
      });
      document.getElementById("refreshPresetsButton").addEventListener("click", () => fetchPresetLibrary(false));
      document.getElementById("presetImportInput").addEventListener("change", async (event) => {
        await importPresetFiles(event.target.files);
        event.target.value = "";
      });
      document.getElementById("reloadConfigButton").addEventListener("click", () => fetchConfigFile(false, true));
      document.getElementById("saveConfigButton").addEventListener("click", saveConfigFile);
      document.getElementById("settingsConfigEditor").addEventListener("input", () => {
        state.configDirty = true;
        updateConfigEditorControls();
      });
      updateConfigEditorControls();

      document.getElementById("settingsOverlay").addEventListener("click", (event) => {
        if (event.target === event.currentTarget)
          setSettingsOpen(false);
      });
      document.getElementById("mediaBrowserOverlay").addEventListener("click", (event) => {
        if (event.target === event.currentTarget)
          setMediaBrowserOpen(false);
      });

      document.querySelectorAll(".settings-nav-item").forEach((item) => {
        item.addEventListener("click", () => setSettingsPanel(item.dataset.panel));
      });

      document.addEventListener("keydown", (event) => {
        if (event.key === "Escape" && state.mediaBrowserOpen)
          setMediaBrowserOpen(false);
        else if (event.key === "Escape" && state.settingsOpen)
          setSettingsOpen(false);
      });
      window.addEventListener("resize", () => {
        if (state.snapshot)
          window.requestAnimationFrame(() => drawImuAxisVisual(state.snapshot.imu || {}));
      });

      const whiteBalanceRange = document.getElementById("whiteBalanceRange");
      whiteBalanceRange.addEventListener("input", () => {
        if (!state.snapshot || state.snapshot.wb_available !== true) return;
        const kelvin = whiteBalanceFromSliderValue(whiteBalanceRange.value);
        setText("whiteBalanceValue", kelvin + "K");
        setText("wbBadge", "WB " + kelvin + "K");
        clearTimeout(state.wbCommitTimer);
        state.wbCommitTimer = window.setTimeout(async () => {
          try {
            await sendCommand({ cmd: "set_wb", kelvin: kelvin });
          } catch (error) {
            showToast(error.message, true);
          }
        }, 160);
      });

      setSettingsPanel(state.activeSettingsPanel);
    }

    installInteractions();
    installPreviewStream();
    startLiveReadoutLoop();
    window.setInterval(() => fetchState(true), 1000);
    fetchState(false);
  </script>
</body>
</html>
)APERTAR";
    return std::string_view(page, sizeof(page) - 1);
}

} // namespace apertar
