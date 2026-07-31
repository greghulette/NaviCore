# Video assets

Production assets for NaviCore videos. Every file is a self-contained HTML page —
no build step, no dependencies, no network access. Open one in a Chromium-based
browser and it runs.

| File | What it is |
|---|---|
| [`intro-title-sequence.html`](intro-title-sequence.html) | 10-second animated intro |
| [`segment-title-cards.html`](segment-title-cards.html) | Ten 4-second chapter cards for the setup walkthrough |
| [`setup-video-breakout.html`](setup-video-breakout.html) | Part-by-part production plan for the setup walkthrough |

## Recording the animations

Both animated pages record the same way:

1. Start your screen recorder — **Win + Alt + R** for Windows Game Bar, or OBS with
   a Desktop Audio source if you want the sound.
2. Click **Replay** once. Browsers block audio until you interact with the page, so
   the first pass after loading is always silent.
3. Click **Record fullscreen**. The clip restarts from frame one with no page chrome
   in shot, so the capture is clean.

Keyboard: <kbd>R</kbd> replay, <kbd>F</kbd> fullscreen, <kbd>S</kbd> toggle sound.

### Intro title sequence

Runs exactly 10.0 seconds. The mark assembles from the real logo geometry in
[`assets-navicore/navicore-icon.svg`](../assets-navicore/navicore-icon.svg), the
photoreceptor ignites at 3.0 s, and the closing card holds from 9.0 s.

The score is synthesized live in Web Audio — hyperdrive spool-up, saber ignition
and hum, a swing on the output signal, and an astromech sign-off. There are no
audio files to keep in sync.

### Segment title cards

Ten cards, one per part of the setup walkthrough. Each runs 4.0 seconds: 1.6 s of
build, then a 2.4 s hold to trim against.

- Number keys <kbd>0</kbd>–<kbd>9</kbd> jump to a card; arrow keys step through.
- **Play all** runs the set back to back as a single take you cut up afterwards —
  usually faster than ten separate recordings.
- **Runtimes** toggles the minute estimates on the cards. Turn them off if the
  numbers no longer match what you filmed.

Card titles and runtimes live in the `CARDS` array near the top of the script.

## Editing

The colours come from the NaviCore brand: machined steel `#f4f7fb → #aab3bf → #646d79`,
signal blue `#2b86ff`, ground `#06080c`. Type is Bahnschrift for display and
Cascadia Mono for data, both shipped with Windows.

Animation timings are CSS `animation-delay` values, and the audio score schedules
against the same numbers. Change a visual beat and move its cue in `scheduleScore()`
(intro) or `accent()` (cards) to match.
