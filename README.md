# FityP's Opinionated H2 Firmware

A [Rockbox](https://www.rockbox.org/) build for the HIFI WALKER H2 (Rockbox
target `erosqnative`, "Eros Q / K native"), with Bluetooth audio and a handful
of changes made for how I use the player. Everything else is stock Rockbox.

Companion tool: [H2Sync](https://github.com/fityp/h2sync) keeps the player in
sync with a Navidrome (Subsonic) server: playlists to the device, plays back
to the server as scrobbles.

## What's different from stock Rockbox

### Bluetooth audio

Stock Rockbox has no Bluetooth on this player. This build sends audio to
Bluetooth headphones, speakers and car stereos.

- **Codecs:** SBC, aptX, aptX HD and LDAC, picked by what the other device
  supports.
- **High-resolution audio over LDAC.** LDAC runs at up to 96 kHz. Tracks above
  what the connection supports are sent at its highest rate of the same family
  (a 192 kHz track goes out at 96 kHz, not 48 kHz).
- **Settings > Bluetooth Settings:** turn Bluetooth on or off, pair new
  devices, and connect, disconnect or forget paired ones.
- **Reconnects on its own.** At start-up it reconnects to the last device, and
  paired headphones can connect to the player themselves (e.g. when taken out
  of their case).
- **Controls and display on the other device.** Play, pause, next and previous
  from headphones and car stereos. Play and pause are separate commands, so a
  headset sending "play" on connecting doesn't pause what is already playing.
  Car stereos and other displays show the title, artist and album, and
  whether it is playing.
- **USB DAC to Bluetooth.** Used as a USB DAC by a computer or phone, the
  player passes the audio on to the Bluetooth device (48 to 192 kHz in).
- **Right speed after reconnecting.** Fixes audio playing fast and high-pitched
  over SBC after a reconnect.
- **Reliable radio start-up and shutdown,** including turning Bluetooth off and
  on again.

Bluetooth needs hardware revision 4 of the player (the one with the BCM4343A1
radio). On other revisions the Bluetooth menu is hidden and the rest of the
firmware works as usual.

### Everything else

- **[Shuffle] at the top of every playlist.** Open a playlist from the
  Playlists menu and the first entry plays the whole playlist in random order,
  starting from a random track. The playlist file's order is left alone.
- **Now Playing on Backlight Off** (Settings > General Settings > Display >
  LCD Settings). When on, the browsers and menus go to the Now Playing screen once
  the backlight turns off, so the buttons always control playback while the
  screen is dark. Off by default.
- **Database updates no longer stop playback.** A database commit used to take
  all free memory, pausing playback with `playlist_resume(): OOM` until it
  finished.

## Known issues

- **Bluetooth occasionally doesn't turn on from Settings.** Turning it on from
  the debug menu works when this happens. Not yet understood.
- **Switching output mid-track.** If a Bluetooth device connects or disconnects
  during a track whose sample rate the new output can't play, that track plays
  at the wrong speed until the next one starts.
- **No album art over Bluetooth.** Cars that support it need an image server
  (AVRCP Cover Art) the player doesn't have yet.
- **Converting 192 kHz to 96 kHz has no anti-alias filter.** Recordings with a
  lot of ultrasonic noise (some DSD-sourced files) can have audible artefacts.

## Installing

1. Install the Rockbox bootloader for the Eros Q / H2 first, following the
   [Rockbox manual](https://www.rockbox.org/manual.shtml) for your player.
2. Download `rockbox.zip` from
   [Releases](https://github.com/fityp/rockbox-h2/releases).
3. Unzip it onto the root of the card, so that it replaces the `.rockbox`
   folder. Your settings, database and Bluetooth pairings are kept.
4. Eject the card or player and restart.

To go back to stock Rockbox, install a stock `rockbox.zip` the same way.

## Building

Build like any Rockbox target, from the `h2` branch:

```
tools/rockboxdev.sh            # once: choose "i" (MIPS)
mkdir build && cd build
../tools/configure             # choose 247 (Eros Q / K native), then Normal build
make -j && make zip
```

See [docs/README](docs/README) and the
[Rockbox wiki](https://www.rockbox.org/wiki/) for details.

## Credits

- The [Rockbox](https://www.rockbox.org/) project.
- Skye ([vgskye](https://github.com/vgskye/rockbox)) for porting Bluedroid and
  the A2DP codecs to Rockbox, which all of the Bluetooth work here builds on.
- Marcin Bukat and Ciro Iriarte, whose Rockbox Gerrit changes 8228, 8238 and
  8318 are included here.
- Bluedroid from Espressif's ESP-IDF; LDAC and aptX encoders from the Android
  Open Source Project.

## License

GPL version 2 or later, like Rockbox. See [docs/COPYING](docs/COPYING).
Bundled libraries keep their own licenses; see each library's folder under
`lib/`.
