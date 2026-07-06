# LLL Lanes Dashboard — Installation Guide

A live, per-lane control panel for LLL buffers, displayed **inside Mainsail** as an interactive webcam card. No Mainsail fork, no plugin, no extra service: one static HTML file served by the nginx instance you already have.

---

## 1. What it is

- **A single self-contained HTML page** (`dashboard/index.html`). Zero dependencies, no CDN, no build step. Dark glassmorphism theme that blends into Mainsail.
- **Talks to Moonraker over its WebSocket** (`/websocket`), through Mainsail's existing nginx proxy. Same origin as Mainsail → **no CORS configuration needed anywhere**.
- **Auto-discovers every lane**: on connect it calls `printer.objects.list`, subscribes to every `lll_buffer *` object it finds, and builds one card per lane. Declare a new `[lll_buffer <lane>]` in `printer.cfg` and it appears on the page by itself — the HTML never needs editing.
- **Per-lane card** shows live: role (`active` / `parked` / `standby`), tension-arm position with zone labels (slack / balanced / taut), filament presence, feed mode, motor activity, error state (with a one-click *clear error* button), and the filament registry entry (material, color swatch, temp, Spoolman spool). A ✎ form edits the registry via `LLL_FILAMENT`.
- **VCR-style transport** (`◂◂ ▶ ▸▸`) per lane:
  - **Hold** a jog button = the motor moves while you hold (rewind / feed).
  - **Double-click** = continuous mode (latch). Implemented as a stream of **10-second-bounded jog orders refreshed every 3 s** — the UI never sends an unbounded "run forever" command (see [Safety](#6-safety-why-continuous-mode-is-burst-bounded)).
  - **Single click** while latched = stop.
  - **▶ play/pause** = role control: activate a parked lane, pause an active lane (`standby` = freeze), resume a paused one.
- **Responsive grid**: cards reflow to the size of the webcam card; overflow scrolls inside the iframe.

> **Prerequisite:** the LLL firmware + Klipper host module from this repo must already be installed and your lanes declared as `[lll_buffer <lane>]` sections in `printer.cfg`. See the main [README](../README.md).
>
> **Note:** the ▶ button on a *parked* lane sends `SET_ACTIVE_LANE LANE=<lane>`. That macro is provided in [`config/lll_macros.cfg.example`](../config/lll_macros.cfg.example) (it parks the other lanes, then runs `LLL_ROLE LANE=<lane> ROLE=active`). Either install the example macros (recommended — the dashboard and the command manuals use them) or edit that line in `index.html` to `LLL_ROLE LANE=<lane> ROLE=active`.

## 2. Install (serve the page with nginx)

All commands run on the printer host (paths assume the default `pi` user — adapt if yours differs).

**a. Copy the dashboard:**

```bash
cd ~/lll-firmware        # your clone of this repository — adjust if you cloned it elsewhere
mkdir -p /home/pi/lll_dashboard
cp dashboard/index.html /home/pi/lll_dashboard/
```

**b. Add a location to Mainsail's nginx site.** Back up first, then edit `/etc/nginx/sites-available/mainsail` and insert this exact line **inside the existing `server { ... }` block** (next to the other `location` blocks):

```nginx
location /lll/ { alias /home/pi/lll_dashboard/; index index.html; add_header Cache-Control "no-store"; }
```

`Cache-Control "no-store"` means any update to the page is visible on a simple browser refresh — no cache fighting.

```bash
sudo cp /etc/nginx/sites-available/mainsail /tmp/mainsail.nginx.bak
sudo nano /etc/nginx/sites-available/mainsail   # insert the line above
sudo nginx -t && sudo systemctl reload nginx
```

**c. Check:**

```bash
curl -s -o /dev/null -w '%{http_code}\n' http://localhost/lll/   # expect: 200
```

Then open `http://<printer-ip>/lll/` in a browser: the page should show *live* in the top-right corner and one card per lane.

## 3. Show it inside Mainsail (webcam iframe)

Mainsail natively supports an `iframe` webcam type that embeds an interactive page as a dashboard card — clicks pass through into the page.

**Via the Mainsail UI:** *Settings → Webcams → Add webcam*, then:

- **Name:** `LLL Lanes` (or anything you like)
- **Service:** `iframe` (listed as *Adaptive/iframe* in some versions)
- **URL / Stream URL:** `/lll/`

Keep the URL **relative** (`/lll/`, not `http://.../lll/`): it resolves against whatever hostname or IP you used to open Mainsail, so it works from any device.

**Or via the Moonraker API** (from the printer host):

```bash
curl -X POST http://localhost:7125/server/webcams/item -H 'Content-Type: application/json' \
  -d '{"name":"LLL Lanes","service":"iframe","stream_url":"/lll/","snapshot_url":"/lll/","enabled":true}'
```

The card then appears in Mainsail's webcam panel — use the webcam selector to switch to **LLL Lanes**.

## 4. Language

Once connected to the WebSocket, the page follows **Mainsail's own language setting**: it reads the Moonraker database (`namespace: mainsail`, `key: general` → `value.language`). If that lookup fails (fresh install, item not set), it falls back to the **browser language**. Nothing to configure — change the language in Mainsail's settings and reload the card.

One caveat, stated honestly: the **pre-connection screen** (shown before the WebSocket is up) and a few empty-field fallback strings are in **French**. The shipped `index.html` is byte-identical to the validated production deployment and is left as-is — see the Language note in the main [README](../README.md).

Currently bundled: English and French. Adding a language is a matter of extending the `I18N` object at the top of `index.html`.

## 5. Optional: Spoolman integration

With Spoolman connected, the color swatch on each card shows the **real color of the spool** loaded in that lane, and Moonraker's filament-usage tracking follows whichever lane is active.

**a. Install Spoolman** (skip if already running) — see the [Spoolman project](https://github.com/Donkie/Spoolman) for details:

```bash
mkdir -p ~/spoolman && cd ~/spoolman
curl -sL https://github.com/Donkie/Spoolman/releases/latest/download/spoolman.zip -o spoolman.zip
unzip -oq spoolman.zip
echo y | bash ./scripts/install.sh      # installs the systemd service
curl -s http://localhost:7912/api/v1/info   # sanity check
```

**b. Connect Moonraker** — back up `moonraker.conf`, then add:

```ini
[spoolman]
server: http://localhost:7912
sync_rate: 5
```

```bash
sudo systemctl restart moonraker
```

Mainsail's native Spoolman panel appears automatically. The full Spoolman UI lives at `http://<printer-ip>:7912`.

**c. Link a lane to a spool:**

```gcode
LLL_FILAMENT LANE=lane1 TYPE=ASA-CF COLOR=black SPOOL_ID=3
```

(or use the ✎ form on the lane's card — same command underneath). From then on:

- The card's swatch uses the spool's `color_hex` from Spoolman (fetched through Moonraker's `server.spoolman.proxy`, so still no CORS). Without a `SPOOL_ID`, the swatch falls back to a color-name→hex mapping from the `COLOR=` field.
- **When a lane becomes active, its spool becomes Moonraker's active spool** — filament consumption is tracked per spool, per lane, automatically.

Remember to update the lane's `SPOOL_ID` whenever you physically swap spools; a lane without one leaves usage counting on the last known spool.

## 6. Safety: why continuous mode is burst-bounded

The dashboard **never issues an unbounded motor order**. Both hold-to-jog and double-click continuous mode work the same way under the hood:

- every order is `LLL_JOG ... MS=10000` — a jog **hard-limited to 10 seconds by the firmware itself**;
- while the button is held (or latched), the page re-sends that bounded order every 3 seconds to keep the motion seamless;
- releasing the button (or a single click on a latched button) sends an explicit `DIR=stop`.

Consequence: if the browser tab crashes, the phone goes to sleep, or Wi-Fi drops in the middle of a continuous run, the refresh stream stops and **the motor stops on its own in less than 10 seconds** — no runaway, no host-side watchdog needed. The stop path never depends on the UI staying alive.

---

## Quick checklist

1. `index.html` copied to `/home/pi/lll_dashboard/` ✔
2. `location /lll/ { ... }` inside the Mainsail `server{}` block, `nginx -t` clean, nginx reloaded ✔
3. `http://<printer-ip>/lll/` returns the page, status shows *live* ✔
4. Webcam item `LLL Lanes` (service `iframe`, URL `/lll/`) added in Mainsail ✔
5. Clicks work inside the card (try a short hold on `▸▸`) ✔
6. *(Optional)* Spoolman on port 7912 + `[spoolman]` in `moonraker.conf`, `LLL_FILAMENT ... SPOOL_ID=` set per lane ✔
