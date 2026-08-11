#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  rc_config.h — RC button / switch / knob mapping configuration
//
//  Adapted from Body_Controller_ESP32_GUI for the NaviCore project.
//  Key changes vs. BC version:
//    • Action types: RA_ESPNOW/RA_HCR/RA_ANIM → RA_WCB_UNICAST/RA_WCB_BROADCAST/
//                   RA_MAESTRO_LOCAL/RA_MAESTRO_REMOTE
//    • RA_SERIAL ports: S3/S4/S5 (S3 = hardware UART0, S4/S5 SoftwareSerial;
//      SBUS OUT now shares UART1, freeing the S5 header on both boards)
//    • Knob functions: KF_NONE / KF_MAESTRO_PASSTHROUGH / KF_HCR_VOLUME
//    • Default mappings: empty (all actions user-configured via GUI)
//    • Same NVS namespace "rcfg" and same JSON shape for GET_CONFIG/SET_CONFIG
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Preferences.h>
#include <LittleFS.h>     // config persists as /config.json; keep this header self-contained
#include <ArduinoJson.h>
#include "wcb_config.h"   // provides WCB_MAC_OCT2/3, WCB_PASSWORD, WCB_QUANTITY, WCB_DEVICE_ID used as factory defaults

// ─────────────────────────────────────────────────────────────────────────────
//  Action types
// ─────────────────────────────────────────────────────────────────────────────
enum RcActionType : uint8_t {
  RA_NONE              = 0,
  RA_WCB_UNICAST       = 1,   // wcb.send(boardId, cmd)   — boardId in target[]
  RA_WCB_BROADCAST     = 2,   // wcb.broadcast(cmd)
  RA_MAESTRO_LOCAL     = 3,   // local Maestro on Serial2 — cmd = "setTarget,ch,pos"
                               //   or "goHome" / "stopScript" / "restartScript,n"
  RA_MAESTRO_REMOTE    = 4,   // remote Maestro via WCBStream — target[] = "1"-"4",
                               //   cmd = same format as RA_MAESTRO_LOCAL
  RA_SERIAL            = 5,   // write cmd to S3/S4/S5 (target[] = port label).
  RA_HCR               = 6,   // Human-Cyborg Relations vocalizer command —
                               //   fn/chan/track describe the HCR call.
                               //   Destination (serial port OR WCB ID+port) is
                               //   a GLOBAL setting in RcConfig::hcrDest.
  RA_MP3               = 7,   // SparkFun MP3 Trigger (v2.x) on a WCB. fn = MP3
                               //   function (see RcMp3Fn); track = numeric arg
                               //   (track #, file index, or volume). Sent as a
                               //   ";A,..." WCB command (normal command path,
                               //   unicast) to RcConfig::mp3Dest — the WCB owns
                               //   the MP3 serial wiring (?MP3,S<port>).
  RA_RECORD            = 8,   // Record/replay control — toggle recording. cmd =
                               //   clip name (for the future clip library; the
                               //   current build has one in-RAM clip). NOT captured.
  RA_PLAY              = 9,   // Play a recorded clip. cmd = clip name; fn = loop
                               //   (0=once, 1=repeat). NOT captured. Both defer to
                               //   loop() so a remote (Core-0) trigger is safe.
  RA_STOP              = 10,  // Stop recording (save) OR playback — explicit halt,
                               //   in case you'd rather not use the Record/Play toggle.
  RA_SMOOTH_OVERRIDE   = 11,  // Global passthrough-smoothing override latch (a
                               //   switch can set speed/accel that supersede every
                               //   passthrough knob's own). cmd = "set,<spd>,<acc>"
                               //   | "clear". Runtime only — not a servo output.
  RA_WLED              = 12,  // WLED LED controller. cmd = ";L<id>,<verb>" string,
                              // authored via the command library. Per-id routing lives
                              // in RcConfig::wledSlots: id 1-9 → a local aux port OR a
                              // remote WCB; bare ";L," (id 0) = lowest-id LOCAL slot.
  RA_DFPLAYER          = 13,  // DFPlayer Mini (DFRobot / YX5300 clones) — the OTHER audio
                               //   device. fn = RcDfpFn; chan/track = its arguments.
                               //   Sent as a ";D,..." WCB command to RcConfig::dfpDest,
                               //   or spoken directly on a local aux port. Deliberately
                               //   NOT a mode of RA_MP3: the verb sets differ and the
                               //   volume scales are INVERSE (DFPlayer 0=silent..30=loud
                               //   vs MP3 Trigger 0=loud..64=silent). See
                               //   docs/DFPLAYER_DESIGN.md §2.
};

// MP3 Trigger function codes, stored in RcAction::fn for RA_MP3 actions.
// Map 1:1 to the WCB ";A,<CMD>" command set (WCB_MP3.cpp processMP3AudioCommand).
enum RcMp3Fn : uint8_t {
  MP3_PLAY   = 1,   // ;A,PLAY,<track>     track 1-255
  MP3_PLAYFS = 2,   // ;A,PLAYFS,<index>   index 0-255
  MP3_STOP   = 3,   // ;A,STOP             start/stop toggle
  MP3_NEXT   = 4,   // ;A,NEXT
  MP3_PREV   = 5,   // ;A,PREV
  MP3_VOL    = 6,   // ;A,VOL,<n>          0=loudest .. 64=inaudible
  MP3_VOLUP  = 7,   // ;A,VOLUP
  MP3_VOLDN  = 8,   // ;A,VOLDN
};

// DFPlayer Mini function codes, stored in RcAction::fn for RA_DFPLAYER actions.
// Map 1:1 to the ";D,<CMD>" set parsed by WcbCmd's DfPlayerCodec::handle() — the
// SAME codec a WCB runs on its receive side, so one command string drives the
// player identically over the mesh or on a local UART.
//   chan  = folder / EQ mode / device id / loop-all flag  (see each row)
//   track = track number, /MP3 index, or volume value
enum RcDfpFn : uint8_t {
  DFP_PLAY       = 1,   // ;D,PLAY,<track>          track 1-2999 (global index)
  DFP_FOLDER     = 2,   // ;D,FOLDER,<f>,<t>        chan = folder 1-99, track 1-255
  DFP_MP3FOLDER  = 3,   // ;D,MP3FOLDER,<n>         track 1-9999 (the /MP3 folder)
  DFP_STOP       = 4,   // ;D,STOP
  DFP_NEXT       = 5,   // ;D,NEXT
  DFP_PREV       = 6,   // ;D,PREV
  DFP_PAUSE      = 7,   // ;D,PAUSE
  DFP_RESUME     = 8,   // ;D,RESUME
  DFP_VOL        = 9,   // ;D,VOL,<n>               track 0-30 — 0=SILENT, 30=LOUDEST
  DFP_VOLUP      = 10,  // ;D,VOLUP
  DFP_VOLDN      = 11,  // ;D,VOLDN
  DFP_LOOP       = 12,  // ;D,LOOP,<track>          track 1-2999, repeats forever
  DFP_LOOPALL    = 13,  // ;D,LOOPALL,<0|1>         chan = 0 off / 1 on
  DFP_LOOPFOLDER = 14,  // ;D,LOOPFOLDER,<f>        chan = folder 1-99
  DFP_RANDOM     = 15,  // ;D,RANDOM
  DFP_EQ         = 16,  // ;D,EQ,<0-5>              chan: 0 Normal 1 Pop 2 Rock 3 Jazz 4 Classic 5 Bass
  DFP_DEVICE     = 17,  // ;D,DEVICE,<n>            chan: 1 USB 2 SD 3 AUX 4 sleep 5 flash
  DFP_RESET      = 18,  // ;D,RESET
};

// ─────────────────────────────────────────────────────────────────────────────
//  RcAction — a single atomic action fired by a button press or switch change
// ─────────────────────────────────────────────────────────────────────────────
struct RcAction {
  RcActionType type;
  // target[]:
  //   RA_WCB_UNICAST     = WCB board ID string ("1"–"20")
  //   RA_MAESTRO_REMOTE  = remote Maestro slot ("1"–"8")
  //   RA_SERIAL          = port label ("S3"/"S4")
  //   others             = unused
  char    target[6];
  // cmd[]:
  //   RA_WCB_UNICAST/BROADCAST = command string (e.g. ":PP100")
  //   RA_MAESTRO_LOCAL/REMOTE  = "setTarget,ch,pos" | "goHome" | "stopScript" | "restartScript,n"
  //   RA_SERIAL                = command string (terminated with \r by dispatcher)
  //   RA_HCR                   = unused (use fn/chan/track instead)
  // 96 bytes (95 chars + null) accommodates chained WCB commands like
  // ";h,play,a,1,fadein,4^;t6000^;h,fadeout,a,4" (44+ chars) plus headroom
  // for 3-4 ;-separated segments. Increase carefully — every byte here is
  // multiplied by RC_NUM_MAPPINGS × 3 tiers × RC_ACTIONS_PER_TIER = 945.
  char    cmd[96];
  uint16_t delayMs;       // optional pre-fire delay (ms)
  char    note[20];       // human-readable label shown in GUI (19 chars + null)
  bool    skipRunning;    // RA_MAESTRO_REMOTE: skip firing if a Maestro sequence is already running (getMovingState gate)

  // RA_HCR-specific fields (zero for other action types). The HCR destination
  // (transport, serial port or WCB ID/port) is a GLOBAL setting stored in
  // RcConfig::hcrDest — every HCR action shares it.  See RcHcrDest.
  // RA_HCR: HCR function/params.  RA_MP3 reuses fn + track (see RcMp3Fn):
  //   fn = MP3 function code, track = numeric arg (track #, index, or volume),
  //   chan unused.  RA_DFPLAYER reuses all three (see RcDfpFn): fn = DFPlayer
  //   function code, chan = folder / EQ / device / flag, track = track number or
  //   volume.  Zero for all non-HCR/MP3/DFPlayer action types.
  uint8_t fn;             // HCR function number (2=SetEmotion, 3=Trigger, 4=Stimulate,
                          //   5=Overload, 6=Muse, 7=Muse-gap, 8=Stop, 9=StopEmote,
                          //   10=OverrideEmotions, 11=ResetEmotions, 13=SetMuse,
                          //   14=PlayWAV, 16=StopWAV, 17=SetVolume, 18=VolumeUp, 19=VolumeDown,
                          //   12=FadeIn, 15=FadeOut) — matches the BC HCRFunction() convention.
                          //   fn 12/15 (Fade) are handled by HcrFade in executeHcrAction, NOT by
                          //   HcrCodec::format (which rejects them). OR, for RA_MP3, an RcMp3Fn (1-8).
  int8_t  chan;           // emotion (0=H,1=S,2=M,3=C — chan 4 rejected by WcbCmd 0.5.0, use fn 5 for Overload) or audio chan (0=V,1=A,2=B);
                          //   fn 7 = Muse min-gap (s); fn 10 = Override 0/1 — unused for RA_MP3.
                          //   fn 12/15 (Fade): chan = 1=A, 2=B (V is not faded, matches the WCB); track = seconds.
                          //   fn 18/19 (Vol +/-): chan = 0 ALL (legacy default), 1=V, 2=A, 3=B; track = step (0 = default 5).
                          //   NOTE the fn 18/19 chan encoding (0=ALL) DELIBERATELY differs from the fn 14/16/17
                          //   audio enum (0=V, 3=ALL): legacy Volume actions carry chan=0 meaning ALL, so 0 stays
                          //   ALL and V/A/B shift to 1/2/3. Do NOT "fix" this to match the audio enum.
  int16_t track;          // PlayWAV track number, SetVolume value, Trigger level, etc.;
                          //   fn 7 = Muse max-gap (s); fn 13 = SetMuse 0/1.
                          //   — for RA_MP3: track #, file index, or volume value.
};

#define RC_ACTIONS_PER_TIER  5   // max simultaneous actions per tap tier

struct RcTier {
  uint8_t  count;
  char     note[20];   // per-tier caption (single/double/triple tap · switch position); 19 chars + null
  RcAction a[RC_ACTIONS_PER_TIER];
};

struct RcMapping {
  bool    exclusive;  // true = tap2 replaces tap1 within the tap window
  RcTier  t[3];       // t[0]=1-tap  t[1]=2-tap  t[2]=3-tap
};

// ─────────────────────────────────────────────────────────────────────────────
//  PWM threshold bands on the matrix channel (RC_NUM_THRESHOLDS slots).
//  Slots 1-21 are physical (drawn on the TX graphic; slot 21 = inert
//  "Unassigned" sentinel). Slots 22-36 are user-defined LOGICAL buttons —
//  extra bands on the same matrix channel, not tied to a physical control.
//  All decode identically through pwmToButton().
// ─────────────────────────────────────────────────────────────────────────────
struct RcThreshold {
  int  id;          // 1–36 (1-21 physical, 22-36 logical)
  char label[24];
  int  minPwm;
  int  maxPwm;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Configurable toggle switches SA–SJ (10 total)
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_SWITCHES 10

struct RcSwitch {
  int     channel;     // SBUS channel 1–24; 0 = disabled
  uint8_t positions;   // 2 or 3
  RcTier  t[3];        // [down, mid, up] — for 2-pos switches t[1] unused
};

static const char*   RC_SWITCH_LABELS[RC_NUM_SWITCHES]     = {"SA","SB","SC","SD","SE","SF","SG","SH","SI","SJ"};
static const int     RC_SWITCH_DEFAULT_CH[RC_NUM_SWITCHES]  = {  8,   9,  10,  11,  12,  13,  14,  15,   0,   0 };
static const uint8_t RC_SWITCH_DEFAULT_POS[RC_NUM_SWITCHES] = {  3,   3,   3,   3,   3,   2,   3,   2,   2,   2 };

#define SW_SA 0
#define SW_SB 1
#define SW_SC 2
#define SW_SD 3
#define SW_SE 4
#define SW_SF 5
#define SW_SG 6
#define SW_SH 7
#define SW_SI 8
#define SW_SJ 9

// ─────────────────────────────────────────────────────────────────────────────
//  Configurable analog sources (8 total — 2 rotary knobs + 2 sliders + 4 gimbal axes)
//    Panel:    S1, S2 (rotary knobs) + LS, RS (side sliders)
//    Sticks:   J1, J2 (right gimbal X/Y axes — AIL/ELE)
//              J3, J4 (left  gimbal Y/X axes — THR/RUD)
//
//  The C struct is still called RcKnob (umbrella term) but the GUI labels
//  J1-J4 as joystick axes, LS/RS as sliders, and S1/S2 as knobs.
//
//  Each source samples one SBUS channel and dispatches its value through one
//  of two functions:
//
//    KF_MAESTRO_PASSTHROUGH  → outputs[] entries each drive a Maestro servo
//                              channel (Maestro ID 1-8 + maestroCh + qus min/max).
//    KF_HCR_VOLUME           → outputs[] entries each drive an HCR audio
//                              channel volume (audio chan + vol min/max 0-99).
//
//  Maestro Pass-Through and HCR Volume are PARALLEL choices — a given knob
//  does one or the other (not both at once). All outputs[] share the parent
//  knob's function.
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_KNOBS         11  // 8 X18 knobs (S1/S2 + LS/RS + J1-J4) + 3 X20 additions (MS slider + J5/J6 gimbal 3rd axes)
#define RC_KNOB_MAX_OUTPUTS  10   // max Maestro-passthrough / HCR-volume outputs a single knob/joystick drives

enum RcKnobFunction : uint8_t {
  KF_NONE               = 0,
  KF_MAESTRO_PASSTHROUGH = 1,   // outputs[] = Maestro servo passthroughs
  KF_HCR_VOLUME         = 2,    // outputs[] = HCR audio-channel volumes
};

struct RcKnobOutput {
  // Interpreted by the parent knob's function:
  //   KF_MAESTRO_PASSTHROUGH: target = Maestro slot ID (1-8 → rcConfig.maestros[id-1])
  //   KF_HCR_VOLUME:          target = HCR audio chan (0=V vocalizer, 1=A, 2=B)
  uint8_t  target;
  // KF_MAESTRO_PASSTHROUGH only: Maestro servo channel (0-31). Unused for HCR.
  uint8_t  maestroCh;
  // KF_MAESTRO_PASSTHROUGH: Maestro qus position at SBUS min/max (e.g. 4000-8000)
  // KF_HCR_VOLUME:          HCR volume value at SBUS min/max (0-99 clamped)
  uint16_t posMin;
  uint16_t posMax;
  // KF_MAESTRO_PASSTHROUGH smoothing (unused for HCR). 0 = don't manage — leave
  // the Maestro channel's own speed/accel. When >0, the passthrough applies
  // Set Speed / Set Acceleration to this channel so targets ramp in hardware
  // (S-curve), and re-applies them automatically after a script/reset clobbers
  // the channel (see g_maeSpeed/g_maeAccel cache in NaviCore.ino).
  uint16_t smoothSpeed;   // Maestro Set Speed units (0.25µs / 10ms); 0-16383; 0 = unmanaged
  uint8_t  smoothAccel;   // Maestro Set Accel 0-255; 0 = unmanaged
  // KF_MAESTRO_PASSTHROUGH only: when true, the joystick CENTER maps to posMin
  // (closed) and only the UPPER HALF of travel (center→full) sweeps posMin→posMax;
  // the lower half rests at posMin. Keeps a servo CLOSED at stick rest (no half-open
  // panels), at the cost of the lower half of stick range. Default false. See
  // sbusToRangeMidClosed() in NaviCore.ino.
  bool     midClosed;
  // KF_MAESTRO_PASSTHROUGH only: auto-release (de-energize) the servo after this
  // many milliseconds with NO stick movement. The passthrough only writes on
  // movement, so a resting servo holds its last target and can buzz/hunt; when
  // >0, the loop() idle tick sends Set Target 0 (Maestro stops pulses → servo
  // goes limp + silent) once the channel has been idle this long, and the next
  // stick move re-energizes it automatically. 0 = never release (default).
  // NOTE: a released servo has NO holding torque — for panels/doors held by
  // gravity/friction/magnets, NOT a servo bearing load. See maestroIdleReleaseTick().
  uint16_t releaseIdleMs;
};

struct RcKnob {
  int     channel;     // SBUS channel 1–24; 0 = disabled
  uint8_t function;    // RcKnobFunction
  bool    reverse;     // if true, invert the SBUS reading around the centre
                       // before mapping to outputs (so a stick "up" produces
                       // what would normally be "down").  Set per-knob in the
                       // GUI; per-output posMin/posMax can still trim further.
  bool    modeAware;   // OPT-IN: when true, the OUTPUTS follow the 3-way mode
                       // switch (FunctionSwState 1/2/3) — one stick drives
                       // different servos per mode. Default false = one output
                       // set for all modes (unchanged legacy behavior). The
                       // source (channel/function/reverse) stays global.
  int8_t  modeSwitchOverride;  // which switch selects THIS knob's mode when
                       // modeAware: -1 = the global mode switch (default), else
                       // a switch index 0-7 (SA-SH). Lets one knob follow e.g.
                       // SA independently of the droid's global mode switch.
  int8_t  smoothProfile;  // -1 = no smoothing; 0-5 = smoothing-profile index.
                       // Each passthrough output's Maestro speed/accel come from
                       // rcConfig.smoothProfiles[smoothProfile].entries[mid-1][ch]
                       // (replaces the retired per-output smoothSpeed/smoothAccel).
  bool    easeSwitchOverride;  // false (default): this knob's own profile wins over the
                       // switch's active easing. true: the switch's "Set active easing"
                       // action overrides this knob's profile. See resolveKnobEasing().
  uint8_t outputCount; // mode-1 outputs (and ALL modes when !modeAware)
  RcKnobOutput outputs[RC_KNOB_MAX_OUTPUTS];
  uint8_t outputCount2[2];                       // modes 2 & 3 (only when modeAware)
  RcKnobOutput outputs2[2][RC_KNOB_MAX_OUTPUTS];
};

// Output set for a given mode (1-3). Keeps every existing kn.outputs/
// kn.outputCount read valid — they ARE the mode-1 set. Non-mode-aware knobs
// always use mode 1; mode-aware knobs use outputs2[mode-2] for modes 2/3.
inline uint8_t rcKnobOutCount(const RcKnob& k, int mode) {
  return (k.modeAware && mode >= 2 && mode <= 3) ? k.outputCount2[mode - 2] : k.outputCount;
}
inline const RcKnobOutput* rcKnobOuts(const RcKnob& k, int mode) {
  return (k.modeAware && mode >= 2 && mode <= 3) ? k.outputs2[mode - 2] : k.outputs;
}

// Default SBUS channels assume typical X18 Mode-2 layout:
//   J1 = CH1 (AIL = R-stick X)   J3 = CH3 (THR = L-stick Y)
//   J2 = CH2 (ELE = R-stick Y)   J4 = CH4 (RUD = L-stick X)
//   S1 = CH5, S2 = CH6  (Kyber-aligned knobs)
//   LS/RS sliders disabled by default (channel 0); user assigns in tool.
// X20 additions (off by default — channel 0 — for X18/X-Lite):
//   MS = middle slider (sits between B1-B6 face buttons on the X20)
//   J5/J6 = 3rd-axis twist channels on the L/R sticks of an X20
//           build with the optional 3-axis gimbal upgrade.
// NOTE: index 4 is the X20 middle slider and MUST stay "S3" to match the config
// tool's knob key (knobDefaults in config_tool/index.html). It was "MS" here
// while the tool sent "S3"; rcConfigFromJSON/load skip any RC_KNOB_LABELS entry
// the incoming JSON lacks, so the tool's S3 knob (incl. its Maestro pass-through)
// silently failed to apply + persist and reverted on every reload.
static const char*   RC_KNOB_LABELS[RC_NUM_KNOBS]    = {"S1","S2","LS","RS","S3","J1","J2","J3","J4","J5","J6"};
static const int     RC_KNOB_DEFAULT_CH[RC_NUM_KNOBS] = {  5,   6,   0,   0,   0,   1,   2,   3,   4,   0,   0 };
static const uint8_t RC_KNOB_DEFAULT_FN[RC_NUM_KNOBS] = {
  KF_NONE, KF_NONE, KF_NONE, KF_NONE,
  KF_NONE,                                     // MS
  KF_NONE, KF_NONE, KF_NONE, KF_NONE,          // J1-J4
  KF_NONE, KF_NONE                             // J5/J6 (X20 3-axis)
};

// ─────────────────────────────────────────────────────────────────────────────
//  Mode selector binding — which switch drives the 1/2/3 mode that gates the
//  button matrix action set.  Set to -1 to disable the mode system.
// ─────────────────────────────────────────────────────────────────────────────
struct RcFuncBindings {
  int8_t modeSwitch;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Global HCR destination — one vocalizer wiring shared by all RA_HCR actions.
//  The user configures this once in the GUI; individual HCR actions only carry
//  fn/chan/track.
//
//    transport = 0  : local SoftwareSerial port (target = "S3"/"S4")
//    transport = 1  : WCB ESP-NOW raw forward (target = WCB ID "0"-"20",
//                      0 = broadcast; wcbPort = serial port on receiver 1-5)
// ─────────────────────────────────────────────────────────────────────────────
struct RcHcrDest {
  uint8_t transport;
  char    target[6];
  uint8_t wcbPort;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Global MP3 Trigger destination — every RA_MP3 action shares this.
//  Mirrors RcHcrDest's dual-transport model:
//    transport = 0 : LOCAL serial — MP3 Trigger wired to this board's S3/S4.
//                    target = "S3"/"S4"; the RC firmware speaks the MP3
//                    Trigger v2 serial protocol directly at `baud`.
//    transport = 1 : WCB unicast — target = WCB ID "1".."20". Sends a
//                    ";A,..." WCB command; that WCB's own MP3 driver
//                    (configured there via ?MP3,S<port>) does the serial work.
//  Broadcast is intentionally not offered (one MP3 Trigger, known location).
// ─────────────────────────────────────────────────────────────────────────────
struct RcMp3Dest {
  uint8_t  transport;     // 0 = local serial (S3/S4), 1 = WCB unicast
  char     target[6];     // serial: "S3"/"S4"  ·  wcb: WCB ID "1"-"20"
  // (baud is NOT here — it belongs to the port, see RcConfig::auxBaud. A local
  //  MP3 Trigger runs at whatever its S3/S4 port is configured to.)
};

// ─────────────────────────────────────────────────────────────────────────────
//  Global DFPlayer Mini destination — every RA_DFPLAYER action shares this.
//  Same dual-transport model as RcMp3Dest:
//    transport = 0 : LOCAL serial — DFPlayer wired to this board's S3/S4/S5.
//                    target = "S3"/"S4"/"S5"; the RC firmware speaks the
//                    DFPlayer's 10-byte frame protocol directly (WcbCmd
//                    DfPlayerCodec).
//    transport = 1 : WCB unicast — target = WCB ID "1".."20". Sends a
//                    ";D,..." WCB command; that WCB's own DFPlayer driver
//                    (configured there via ?DFP,S<port>) does the serial work.
//  Broadcast is intentionally not offered (one DFPlayer, known location) —
//  same reasoning as the MP3 Trigger above.
//  A DFPlayer's baud is FIXED at 9600 by the module, so the port carrying one
//  must have RcConfig::auxBaud set to 9600. Nothing enforces that in firmware;
//  the config tool warns.
// ─────────────────────────────────────────────────────────────────────────────
struct RcDfpDest {
  uint8_t  transport;     // 0 = local serial (S3/S4/S5), 1 = WCB unicast
  char     target[6];     // serial: "S3"/"S4"/"S5"  ·  wcb: WCB ID "1"-"20"
};

// ─────────────────────────────────────────────────────────────────────────────
//  WLED destinations (RC_NUM_WLED slots) — NaviCore's per-id WLED routing table,
//  mirroring the WCB's wledConfigs[]. Buttons/switches fire an RA_WLED action
//  carrying a ";L<id>,<verb>" string; executeWledAction() looks the id up here and
//  routes it to a LOCAL aux serial port (WcbWled::emit) or a REMOTE WCB (wcb->send).
//  One physical WLED per id.
//    configured == false                       : empty slot (ignored)
//    remoteWCB == 0 && serialPort in {3,4,5}    : LOCAL — wired to s3/s4/s5 here
//    remoteWCB in 1-20                          : REMOTE — the ;L<id> is forwarded
//  (No baud here — a local WLED runs at its port's RcConfig::auxBaud. WLED wants
//   115200, which only S3 = hardware UART0 does reliably; S4/S5 are SoftwareSerial.)
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_WLED 4   // WLED destination slots (wledID 1-9 addressable; a controller needs only a few)

struct RcWledSlot {
  uint8_t wledID;      // 1-9 (0 = empty)
  uint8_t serialPort;  // 3/4/5 = local aux port (s3/s4/s5); 0 = not local
  uint8_t remoteWCB;   // 1-20 = remote host WCB; 0 = local
  bool    configured;  // false = slot unused
};

// ─────────────────────────────────────────────────────────────────────────────
//  WCB network credentials — formerly compile-time #defines in wcb_config.h.
//  Now NVS-backed and runtime-editable via the GUI. The values in wcb_config.h
//  are used only as factory defaults on a fresh device with no stored NVS data.
//  Changes take effect on the next boot (WCBClient constructs once in setup()).
// ─────────────────────────────────────────────────────────────────────────────
struct RcWcbNetwork {
  uint8_t macOct2;        // 2nd octet of shared WCB MAC scheme (0x00-0xFF)
  uint8_t macOct3;        // 3rd octet of shared WCB MAC scheme
  char    password[40];   // ESP-NOW network password (≤39 chars)
  uint8_t quantity;       // total WCBs in the system
  uint8_t deviceId;       // this RC controller's ID on the WCB network.  By
                          // convention RCs always claim WCB special-peer
                          // slot 20 so the config tool's "Via WCB" bridge
                          // has a single known target.  Field is left
                          // writable for unusual multi-RC deployments.
  uint8_t channel;        // ESP-NOW mesh WiFi channel (1-13). The ESP32 has ONE radio,
                          // so this MUST match the channel every WCB is on or this RC is
                          // silently unreachable. Passed to wcb->setMeshChannel() before
                          // begin(); a reboot is required to change it. Default 1.
};

// Named snapshots of WCB-network credentials (a whole mesh identity) the config tool
// switches between — e.g. a bench "Dev" mesh vs the "In-droid" mesh — instead of
// re-typing the fields. INERT storage: the firmware never acts on the list; only the
// config tool reads it and applies one into wcbNetwork on request (over Direct USB).
// Lives in the config so it travels with the droid + rides along in backups/exports.
#define RC_MAX_WCB_PROFILES 6   // capacity — MUST match WCB_MAX_PROFILES in the config tool
struct RcWcbProfile {
  char    name[24];       // display name ("Dev", "In-droid"); "" = empty (23 chars + null)
  uint8_t macOct2;
  uint8_t macOct3;
  char    password[40];   // ESP-NOW network password (≤39 chars)
  uint8_t quantity;
  uint8_t deviceId;
  uint8_t channel;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Maestro locations (8 slots, ID 1-8)
//
//  Buttons / knobs / actions reference Maestros by their slot ID (1-8) only —
//  they don't care whether a given Maestro is wired locally to Serial2 or
//  routed via ESP-NOW broadcast. This struct holds the runtime wiring for
//  each slot, so the dispatcher knows where to send the Pololu bytes and
//  which device number to embed in the Pololu protocol header.
//
//    type = 0  : disabled (slot ignored by the dispatcher)
//    type = 1  : local — wired to Serial2 on this board
//    type = 2  : remote — broadcast via ESP-NOW to all WCBs; any WCB with
//                Kyber_Remote enabled forwards the bytes to its Maestro port
//
//  device: Pololu protocol device number (0-127).  Every Maestro on the
//  network MUST have a unique device # set in Maestro Control Center —
//  this firmware always uses Pololu protocol (not compact), because the
//  broadcast model puts multiple Maestros on a shared logical bus and the
//  device-number filter is what keeps them from all responding to every
//  command.
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_MAESTROS 8
#define RC_MAESTRO_CHANNELS 32   // Pololu Maestro max channels (Mini Maestro ≤24, Micro 6)

// Per-channel metadata IMPORTED from a Maestro Control Center settings file
// (the servo name + its min/max travel endpoints). The firmware does NOT act on
// this — it's carried in the config purely so the tool's channel names and
// passthrough limits persist ON the device and ride along with config import/
// export, instead of living only in browser localStorage. All-zero (name "",
// endpoints 0/0) = unset, and is skipped when serializing (sparse).
struct RcMaestroChannel {
  char     name[24];   // servo name ("" = unset)
  uint16_t minPos;     // ¼µs endpoint (0 = unset)
  uint16_t maxPos;     // ¼µs endpoint (0 = unset)
};

struct RcMaestroSlot {
  uint8_t type;       // 0 disabled · 1 local (Serial2) · 2 remote (broadcast)
  uint8_t device;     // 0-127, Pololu protocol device # (no compact support)
  RcMaestroChannel channels[RC_MAESTRO_CHANNELS];   // imported per-channel names/endpoints (tool metadata)
};

// ─────────────────────────────────────────────────────────────────────────────
//  Transmitter model — drives the GUI's SVG, default channel assignments,
//  switch/trim/knob/button labels, and which controls appear in the
//  calibration wizard.  The firmware-side struct slots are FIXED at their
//  maximums (RC_NUM_SWITCHES = 10, RC_NUM_KNOBS = 11, etc.) regardless of
//  model — unused slots just sit at channel=0 ("inactive") for models with
//  fewer physical controls.  This lets the firmware stay model-agnostic at
//  the dispatch layer while the GUI presents the right physical layout.
// ─────────────────────────────────────────────────────────────────────────────
enum RcTxModel : uint8_t {
  TX_MODEL_X18          = 0,  // FrSky Tandem X18 — full-size, 10 switches, 2 knobs (S1/S2), 2 sliders (LS/RS)
  TX_MODEL_TWIN_X_LITE  = 1,  // FrSky Twin X-Lite — handheld, 6 switches (SE/SF momentary), 2 knobs, no sliders, 4 face buttons
  TX_MODEL_X20          = 2,  // FrSky Twin X20 — X18 layout + MS middle slider + optional 3-axis gimbals (J5/J6 + 2 stick-click momentaries on matrix slots 19/20)
};

// ─────────────────────────────────────────────────────────────────────────────
//  Full runtime config block
//  buttonId  = (mode * 100) + btn  →  e.g. mode=1, btn=5 → 105
//  flat index = (mode-1)*RC_NUM_THRESHOLDS + (btn-1)  → 0..(RC_NUM_MAPPINGS-1)
//
//  RC_NUM_THRESHOLDS bumped from 19 → 21 to fit the X20's two extra matrix
//  slots (slots 19/20 = L-stick and R-stick click momentaries that come with
//  the optional 3-axis gimbal upgrade).  Slot 21 stays as the inert
//  "Unassigned" sentinel.  X18/X-Lite configs load fine because the two new
//  slots are persisted by name (JSON keys) and default to channel=0 when
//  the model doesn't use them.
//
//  LOGICAL BUTTONS: slots 22-36 are 15 user-defined "logical buttons" — extra
//  PWM bands on the SAME matrix channel, NOT tied to any physical control on the
//  TX graphic. They decode/debounce/tap/dispatch through the exact same path as
//  the physical matrix slots (pwmToButton + rcMapIndex), so the only firmware
//  change is the slot count. They default INERT (band 0/0 never matches) until
//  the user assigns a band + per-mode actions in the config tool's Logical
//  Buttons panel. rcConfig lives in PSRAM (EXT_RAM_BSS_ATTR on its definition in
//  NaviCore.ino), so the extra ~85 KB of mapping storage costs internal RAM nil.
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_PHYSICAL   21   // matrix slots on the TX graphic (1-20 buttons + slot-21 sentinel)
#define RC_NUM_LOGICAL    15   // user-defined logical buttons (matrix-channel bands, slots 22-36)
#define RC_NUM_THRESHOLDS 36   // RC_NUM_PHYSICAL + RC_NUM_LOGICAL — total matrix-channel bands
#define RC_NUM_MAPPINGS   108  // 3 modes × 36 slots

inline int rcMapIndex(int mode, int btn) { return (mode - 1) * RC_NUM_THRESHOLDS + (btn - 1); }
// True for logical-button slots (22-36) — i.e. not drawn on the TX graphic.
inline bool rcIsLogicalSlot(int btn) { return btn > RC_NUM_PHYSICAL && btn <= RC_NUM_THRESHOLDS; }

// ── Smoothing profiles ───────────────────────────────────────────────────────
// 6 named profiles, each a per-(Maestro id 1-8, channel 0-31) speed/accel map.
// A passthrough knob references one (RcKnob.smoothProfile); a Maestro script
// action can load one. DENSE in RAM (O(1) lookup in the SBUS hot path), SPARSE
// in JSON (only non-zero channels emitted). speed/accel BOTH 0 = channel unset.
#define RC_NUM_SMOOTH_PROFILES 6
struct RcSmoothEntry {
  uint16_t speed;   // Maestro Set Speed (0.25µs / 10ms), 0-16383; 0 = unset
  uint8_t  accel;   // Maestro Set Accel 0-255; 0 = unset
};
struct RcSmoothProfile {
  char          name[24];
  RcSmoothEntry entries[RC_NUM_MAESTROS][32];   // [maestroId-1][channel]
};

// ─────────────────────────────────────────────────────────────────────────────
//  Port-label slots (rcConfig.serialLabels)
// ─────────────────────────────────────────────────────────────────────────────
// One slot per labelable NaviCore port, indexed by FIRMWARE port so the config is
// independent of the mesh numbering. The three aux headers carry firmware names
// S3/S4/S5 (silkscreened "Serial 1/2/3" on a NaviCore v2) and are the ports the
// mesh addresses as S1/S2/S3; the local Maestro is a binary bus, so it gets a
// label but no text-addressable port. JSON keys match auxBaud's: "S3","S4","S5",
// "maestro".
#define RC_SLBL_S3       0
#define RC_SLBL_S4       1
#define RC_SLBL_S5       2
#define RC_SLBL_MAESTRO  3
#define RC_NUM_SLBL      4
// JSON key per slot — index with RC_SLBL_*. Order must match the defines above.
static const char* const RC_SLBL_KEYS[RC_NUM_SLBL] = { "S3", "S4", "S5", "maestro" };

// ── Mode-select report ───────────────────────────────────────────────────────
// Optional, OFF by default. When enabled, NaviCore sends a command to one WCB
// carrying the active mode-select position (FunctionSwState 1/2/3) on every mode
// change AND every 60 s. `tmpl` is the fallback command with "{mode}" substituted
// to the position digit; a non-empty per-mode `cmds[mode-1]` overrides `tmpl` for
// that position. Nothing is sent when the effective command is empty. Droid-
// specific plumbing (rcModeReportCmd builds the effective string; reportMode in
// rc_telemetry.h sends it).
struct RcModeReport {
  bool    enabled;      // master on/off (default false)
  uint8_t wcb;          // target WCB id 1-20 (0 = unset → nothing sent)
  char    tmpl[48];     // fallback command; "{mode}" → position digit
  char    cmds[3][48];  // per-mode overrides (index 0 = mode 1); win over tmpl when non-empty
};

struct RcConfig {
  uint8_t        txModel;        // RcTxModel — drives GUI layout + defaults
  // Per-build hardware option flag — meaningful only when txModel == TX_MODEL_X20.
  // When false, the X20 GUI hides the J5/J6 twist axes and the L-Stick/R-Stick
  // click matrix buttons (slots 19/20); when true, it shows and walks them
  // through calibration.  Firmware-side this is mostly cosmetic — the dispatch
  // layer already gates each knob/button on its `channel` field, so a build
  // without 3-axis hardware just leaves J5/J6/slot-19/slot-20 at channel=0.
  bool           threeAxisGimbals;
  bool           sbusOutEnabled;        // re-emit SBUS frames on SBUS_OUT_PIN; off saves the per-byte passthrough tee
  uint8_t        boardType;             // hardware pin profile: 0 = NaviCore v2 PCB (default), 1 = WCB HW 3.2
  int            tapWindowMs;
  uint8_t        chRateHz;        // rc_ch live-monitor broadcast rate in Hz (1–20; default 5). Runtime-tunable from the config tool — higher = smoother monitor, more mesh airtime (20Hz can flood the shared mesh and flap other boards' heartbeats).
  int            matrixChannel;   // SBUS channel carrying the multiplexed button matrix
  // Consecutive in-band SBUS frames a matrix button must hold before a press
  // is accepted. 1 = fastest (safe for a DIGITAL SBUS source — no analog
  // resistor-ladder sweep to filter). Raise to 2-4 if driven from a physical
  // transmitter matrix that sweeps through neighboring bands. Applied live on
  // SET_CONFIG (no reboot). Clamped 1-4 by the firmware.
  int            matrixDebounceFrames;
  RcThreshold    thresholds[RC_NUM_THRESHOLDS];
  RcMapping      mappings[RC_NUM_MAPPINGS];
  RcSwitch       switches[RC_NUM_SWITCHES];
  RcKnob         knobs[RC_NUM_KNOBS];
  RcFuncBindings funcBindings;
  RcHcrDest      hcrDest;
  RcMaestroSlot  maestros[RC_NUM_MAESTROS];  // ID 1-8 → maestros[0..7]
  RcWcbNetwork   wcbNetwork;
  RcWcbProfile   wcbProfiles[RC_MAX_WCB_PROFILES];  // saved mesh identities (Dev/In-droid/…)
  uint8_t        wcbProfileCount;                   // 0..RC_MAX_WCB_PROFILES entries in use
  RcMp3Dest      mp3Dest;
  RcDfpDest      dfpDest;                 // global DFPlayer Mini destination (the other audio device)
  RcWledSlot     wledSlots[RC_NUM_WLED];  // per-id WLED routing table (id 1-9 → local port | remote WCB)
  // Aux serial port baud — [0]=S3 (hardware UART0), [1]=S4, [2]=S5 (S4/S5
  // SoftwareSerial). One source of truth for the port line rate; HCR / MP3-local /
  // Serial actions all just use the port at this baud (the firmware opens S3/S4/S5
  // with these in setup()).
  uint32_t       auxBaud[3];   // [0]=S3, [1]=S4, [2]=S5 — all used on both boards (S5: v2 GPIO38/47, WCB 3.2 GPIO9/10)
  // Local Maestro bus baud (Serial2 hardware UART). Must match each wired
  // Maestro's Serial Settings (or its "Detect baud" mode). Remote/Kyber
  // Maestros are unaffected — that path is binary over ESP-NOW.
  uint32_t       maestroBaud;
  // Per-port WDP labels — what's attached to each of NaviCore's ports, so WCBs +
  // the Wizard show it (advertised as WDP PORTLABEL TLVs, same as the WCB boards).
  // Indexed by RC_SLBL_* — the FIRMWARE port, not the mesh port number, so the two
  // numbering schemes stay independent (see rcSerialLabel / rcWdpPortForLabel).
  // Stores the USER OVERRIDE only; "" = fall back to the auto-derived default.
  char           serialLabels[RC_NUM_SLBL][25];
  // Per-aux-port mesh bridging — makes S3/S4/S5 full mesh citizens the way a WCB's
  // serial ports are. Indexed like auxBaud: [0]=S3, [1]=S4, [2]=S5.
  //   bcastOut : a broadcast command arriving over the mesh is written OUT this port.
  //   bcastIn  : a line arriving ON this port is broadcast to the mesh (and out the
  //              other bcastOut ports), exactly as a WCB rebroadcasts serial input.
  // Both default OFF — a port only joins the broadcast domain when asked to. Targeted
  // writes (`;w20,;s<n><cmd>`) are always accepted and need neither flag.
  bool           serialBcastOut[3];
  bool           serialBcastIn[3];
  // Remote skip-if-running gate: how long a mesh-relayed Maestro "busy" (:MQR moving-state)
  // reply stays valid. If the last reply is older than this — or none has arrived (e.g. the
  // WCB relay isn't up yet) — the gate FAILS OPEN and the action fires anyway. Default 250.
  uint16_t       maeGateMs;
  // 6 smoothing profiles (see RcSmoothProfile). ~4.6 KB total; lives in the
  // PSRAM-heap RcConfig, so it costs no DRAM.
  RcSmoothProfile smoothProfiles[RC_NUM_SMOOTH_PROFILES];
  // Fired when a NEW WCB peer is first detected on the mesh (WDP onNeighbor),
  // after a boot-time grace window so the initial fleet discovery doesn't spam.
  // Global — one action set for any new peer; dedup + boot suppression are
  // handled in firmware (see the peer-event drain in NaviCore.ino loop()).
  RcTier   peerNewActions;   // up to 5 actions; count 0 = alert only, no action
  bool     peerAlert;        // also do a passive NeoPixel flash + terminal line
  RcModeReport modeReport;   // optional: report the mode-select position to one WCB (off by default)
};

// rcConfig is heap-allocated in PSRAM at boot (see NaviCore.ino setup()); the
// global is a pointer, and this macro keeps every `rcConfig.xxx` access working
// unchanged.  g_rcConfig MUST be allocated before any rcConfig access.
extern RcConfig* g_rcConfig;
#define rcConfig (*g_rcConfig)

// ─────────────────────────────────────────────────────────────────────────────
//  Construction helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline RcAction makeWCBUnicast(const char* wcbId, const char* cmd,
                                      uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_WCB_UNICAST;
  strlcpy(a.target, wcbId, sizeof(a.target));
  strlcpy(a.cmd,    cmd,   sizeof(a.cmd));
  strlcpy(a.note,   note,  sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline RcAction makeWCBBroadcast(const char* cmd,
                                         uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_WCB_BROADCAST;
  strlcpy(a.cmd,  cmd,  sizeof(a.cmd));
  strlcpy(a.note, note, sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline RcAction makeMaestroLocal(const char* cmd,
                                         uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_MAESTRO_LOCAL;
  strlcpy(a.cmd,  cmd,  sizeof(a.cmd));
  strlcpy(a.note, note, sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline RcAction makeMaestroRemote(const char* slot, const char* cmd,
                                          uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_MAESTRO_REMOTE;
  strlcpy(a.target, slot, sizeof(a.target));
  strlcpy(a.cmd,    cmd,  sizeof(a.cmd));
  strlcpy(a.note,   note, sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline RcAction makeSerial(const char* port, const char* cmd,
                                   uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_SERIAL;
  strlcpy(a.target, port, sizeof(a.target));
  strlcpy(a.cmd,    cmd,  sizeof(a.cmd));
  strlcpy(a.note,   note, sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline void setTier1(RcTier& tier, RcAction a0) {
  tier.count = 1; tier.a[0] = a0;
}
static inline void setTier2(RcTier& tier, RcAction a0, RcAction a1) {
  tier.count = 2; tier.a[0] = a0; tier.a[1] = a1;
}
static inline void setTier3(RcTier& tier, RcAction a0, RcAction a1, RcAction a2) {
  tier.count = 3; tier.a[0] = a0; tier.a[1] = a1; tier.a[2] = a2;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Factory defaults
//  All button/matrix mappings start empty — configure via GUI.
//  Switch channel assignments mirror the Kyber ELRS layout (SA-SH on CH8-CH15).
// ─────────────────────────────────────────────────────────────────────────────
void rcConfigLoadDefaults() {
  rcConfig.txModel          = TX_MODEL_X18;  // GUI default; user picks via Config → Transmitter
  rcConfig.threeAxisGimbals = false;         // X20 hardware option, off by default
  rcConfig.sbusOutEnabled   = false;         // SBUS-OUT passthrough off by default (saves CPU when unused)
  rcConfig.maeGateMs        = 250;           // remote skip-if-running fail-open window (ms)
  rcConfig.boardType        = 0;             // 0 = NaviCore v2 PCB (default pinout); 1 = WCB HW 3.2
  rcConfig.tapWindowMs      = 500;
  rcConfig.chRateHz         = 5;            // rc_ch live-monitor rate (Hz); 5 = smooth + light mesh load (20Hz floods the shared ESP-NOW channel and flaps other boards)
  rcConfig.matrixChannel    = 7;            // button matrix on CH7
  rcConfig.matrixDebounceFrames = 1;     // digital SBUS source — fastest; bump for analog matrix
  rcConfig.funcBindings.modeSwitch = SW_SE;  // SE (CH12) drives mode 1/2/3
  rcConfig.peerNewActions.count = 0;         // no action on a newly-detected WCB peer by default
  rcConfig.peerAlert            = true;      // passive NeoPixel + terminal alert on a new peer (on)

  // Default PWM threshold bands — measured SBUS values from the live X18
  // (matrix channel), center ±12 (~17-count neutral deadband between buttons;
  // SBUS noise is only ±1-2). T3/T2 are vertical trim buttons → Up/Down.
  // Slots 19/20 (L-Stick Click / R-Stick Click) are X20-only momentary
  // matrix buttons that come with the 3-axis gimbal upgrade — values
  // extrapolated from the X18 band cadence (~41 per slot) and will be
  // refined when the first real X20 is calibrated.  Slot 21
  // ("Unassigned") is INERT (0/0 can never match) and hidden in the
  // config tool; it is kept only so the 21-slot mapping/NVS index math
  // (RC_NUM_THRESHOLDS / RC_NUM_MAPPINGS / rcMapIndex) is unchanged.
  struct { const char* label; int mn; int mx; } bands[RC_NUM_THRESHOLDS] = {
    { "B1",            1799, 1823 },
    { "B2",            1758, 1782 },
    { "B3",            1718, 1742 },
    { "B4",            1676, 1700 },
    { "B5",            1634, 1658 },
    { "B6",            1594, 1618 },
    { "T4 Left",       1553, 1577 },
    { "T4 Right",      1512, 1536 },
    { "T5 Left",       1471, 1495 },
    { "T5 Right",      1430, 1454 },
    { "T3 Up",         1389, 1413 },
    { "T3 Down",       1348, 1372 },
    { "T2 Up",         1308, 1332 },
    { "T2 Down",       1266, 1290 },
    { "T6 Left",       1225, 1249 },
    { "T6 Right",      1184, 1208 },
    { "T1 Left",       1143, 1167 },
    { "T1 Right",      1103, 1127 },
    { "L-Stick Click", 1062, 1086 },   // X20 3-axis gimbal momentary (left stick press)
    { "R-Stick Click", 1021, 1045 },   // X20 3-axis gimbal momentary (right stick press)
    { "Unassigned",       0,    0 },
    // Logical buttons (slots 22-36): extra user-defined bands on the matrix
    // channel, off the TX graphic. INERT (0/0 never matches) until the user
    // sets a band + actions in the config tool's Logical Buttons panel.
    { "Logical 1",  0, 0 }, { "Logical 2",  0, 0 }, { "Logical 3",  0, 0 },
    { "Logical 4",  0, 0 }, { "Logical 5",  0, 0 }, { "Logical 6",  0, 0 },
    { "Logical 7",  0, 0 }, { "Logical 8",  0, 0 }, { "Logical 9",  0, 0 },
    { "Logical 10", 0, 0 }, { "Logical 11", 0, 0 }, { "Logical 12", 0, 0 },
    { "Logical 13", 0, 0 }, { "Logical 14", 0, 0 }, { "Logical 15", 0, 0 },
  };
  for (int i = 0; i < RC_NUM_THRESHOLDS; i++) {
    rcConfig.thresholds[i].id = i + 1;
    strlcpy(rcConfig.thresholds[i].label, bands[i].label, 24);
    rcConfig.thresholds[i].minPwm = bands[i].mn;
    rcConfig.thresholds[i].maxPwm = bands[i].mx;
  }

  // All matrix button mappings start empty
  memset(rcConfig.mappings, 0, sizeof(rcConfig.mappings));

  // Default switch channel / position assignments (no actions — user configures)
  memset(rcConfig.switches, 0, sizeof(rcConfig.switches));
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    rcConfig.switches[i].channel   = RC_SWITCH_DEFAULT_CH[i];
    rcConfig.switches[i].positions = RC_SWITCH_DEFAULT_POS[i];
  }

  // Default knob assignments (no outputs assigned — user configures in GUI)
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    rcConfig.knobs[i].channel     = RC_KNOB_DEFAULT_CH[i];
    rcConfig.knobs[i].function    = RC_KNOB_DEFAULT_FN[i];
    rcConfig.knobs[i].reverse     = false;
    rcConfig.knobs[i].modeAware   = false;
    rcConfig.knobs[i].modeSwitchOverride = -1;   // follow the global mode switch
    rcConfig.knobs[i].smoothProfile = -1;        // no smoothing profile
    rcConfig.knobs[i].easeSwitchOverride = false; // knob's own profile wins over the switch
    rcConfig.knobs[i].outputCount = 0;
    memset(rcConfig.knobs[i].outputs, 0, sizeof(rcConfig.knobs[i].outputs));
    rcConfig.knobs[i].outputCount2[0] = rcConfig.knobs[i].outputCount2[1] = 0;
    memset(rcConfig.knobs[i].outputs2, 0, sizeof(rcConfig.knobs[i].outputs2));
  }

  // Default global HCR destination — local Serial S3, disabled until user
  // changes it (no transport effect when no HCR actions are configured).
  rcConfig.hcrDest.transport = 0;
  strlcpy(rcConfig.hcrDest.target, "S3", sizeof(rcConfig.hcrDest.target));
  rcConfig.hcrDest.wcbPort   = 1;

  // Default global MP3 Trigger destination — WCB unicast to WCB 2 (no effect
  // until the user adds RA_MP3 actions and points this at the right place).
  rcConfig.mp3Dest.transport = 1;
  strlcpy(rcConfig.mp3Dest.target, "2", sizeof(rcConfig.mp3Dest.target));

  // Default global DFPlayer destination — LOCAL S3 (no effect until the user adds
  // RA_DFPLAYER actions). Local is the sensible default here where the MP3 Trigger
  // defaults to a WCB: a DFPlayer is cheap enough to be soldered straight onto the
  // controller's own aux header, which is the common build.
  rcConfig.dfpDest.transport = 0;
  strlcpy(rcConfig.dfpDest.target, "S3", sizeof(rcConfig.dfpDest.target));

  // Default WLED slots — all empty (no WLED configured). The user maps id→dest in
  // the config tool's WLED panel. All-zero ⇒ configured=false, wledID=0 (mirrors the
  // WCB's cleared wledConfigs state).
  memset(rcConfig.wledSlots, 0, sizeof(rcConfig.wledSlots));

  // Aux serial port baud — S3, S4. 9600 default (HCR's rate). Raise per port
  // for faster peripherals (e.g. an MP3 Trigger v2 on that port wants 38400).
  rcConfig.auxBaud[0] = 9600;   // S3
  rcConfig.auxBaud[1] = 9600;   // S4
  rcConfig.auxBaud[2] = 9600;   // S5 (NaviCore v2 "Serial 3")
  rcConfig.maestroBaud = LOCAL_MAESTRO_BAUD_RATE;   // local Maestro bus (Serial2)
  memset(rcConfig.serialLabels, 0, sizeof(rcConfig.serialLabels));   // no overrides → all ports auto-derive
  memset(rcConfig.serialBcastOut, 0, sizeof(rcConfig.serialBcastOut));   // no port bridges the mesh broadcast
  memset(rcConfig.serialBcastIn,  0, sizeof(rcConfig.serialBcastIn));    // domain until explicitly enabled
  rcConfig.modeReport = RcModeReport{};   // mode-select report OFF by default (all fields zero/empty)

  // Default Maestro slots — all 8 disabled until user enables them in the
  // GUI Maestro Locations panel.  Device numbers default to match the slot
  // ID (slot 1 → device 1, ..., slot 8 → device 8) so they're easy to
  // remember.  The user must set the matching device # in Maestro Control
  // Center on each physical Maestro for the dispatcher's address filter to
  // route correctly.
  for (int i = 0; i < RC_NUM_MAESTROS; i++) {
    rcConfig.maestros[i].type   = 0;
    rcConfig.maestros[i].device = (uint8_t)(1 + i);   // 1, 2, ..., 8
  }

  // Smoothing profiles — all empty; profile 0 pre-named "Default".
  memset(rcConfig.smoothProfiles, 0, sizeof(rcConfig.smoothProfiles));
  strlcpy(rcConfig.smoothProfiles[0].name, "Default", sizeof(rcConfig.smoothProfiles[0].name));

  // WCB network credentials — compile-time defaults from wcb_config.h.
  // NVS overrides these at runtime (see rcConfigLoadNVS).
  rcConfig.wcbNetwork.macOct2  = WCB_MAC_OCT2;
  rcConfig.wcbNetwork.macOct3  = WCB_MAC_OCT3;
  strlcpy(rcConfig.wcbNetwork.password, WCB_PASSWORD, sizeof(rcConfig.wcbNetwork.password));
  rcConfig.wcbNetwork.quantity = WCB_QUANTITY;
  rcConfig.wcbNetwork.deviceId = WCB_DEVICE_ID;
  rcConfig.wcbNetwork.channel  = 1;    // default ESP-NOW mesh channel (matches WCB_MESH_CHANNEL)

  // Saved WCB-credential profiles — none until the user creates them in the GUI.
  memset(rcConfig.wcbProfiles, 0, sizeof(rcConfig.wcbProfiles));
  rcConfig.wcbProfileCount = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  JSON helpers — action ↔ JSON object
// ─────────────────────────────────────────────────────────────────────────────
static void actionToJson(const RcAction& a, JsonObject obj) {
  switch (a.type) {
    case RA_WCB_UNICAST:
      obj["type"]   = "wcb_unicast";
      obj["target"] = a.target;   // WCB board ID string
      obj["cmd"]    = a.cmd;
      if (a.delayMs)     obj["delay"]       = a.delayMs;
      if (a.skipRunning) obj["skipRunning"] = true;   // via-WCB Maestro verb: skip-if-running gate
      break;
    case RA_WCB_BROADCAST:
      obj["type"] = "wcb_broadcast";
      obj["cmd"]  = a.cmd;
      if (a.delayMs)     obj["delay"]       = a.delayMs;
      if (a.skipRunning) obj["skipRunning"] = true;   // via-WCB Maestro verb: skip-if-running gate
      break;
    case RA_MAESTRO_LOCAL:
      obj["type"] = "maestro_local";
      obj["cmd"]  = a.cmd;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_MAESTRO_REMOTE:
      // Unified Maestro action — target = Maestro slot ID ("1"-"8") from the
      // GUI Maestro Locations panel. The enum value is still
      // RA_MAESTRO_REMOTE for backward compat with the dispatcher switch, but
      // the JSON name is the user-facing "maestro".
      obj["type"]   = "maestro";
      obj["target"] = a.target;
      obj["cmd"]    = a.cmd;
      if (a.delayMs)     obj["delay"]      = a.delayMs;
      if (a.skipRunning) obj["skipRunning"] = true;
      break;
    case RA_SERIAL:
      obj["type"]  = "serial";
      obj["port"]  = a.target;    // "S3"/"S4"
      obj["cmd"]   = a.cmd;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_HCR:
      // Destination (transport / target / wcbPort) lives in rcConfig.hcrDest
      // as a global setting — see RcHcrDest.  Each action only carries the
      // HCR command itself.
      obj["type"]  = "hcr";
      obj["fn"]    = a.fn;
      obj["chan"]  = a.chan;
      obj["track"] = a.track;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_MP3:
      // Destination (target WCB) is GLOBAL — see RcConfig::mp3Dest. The action
      // only carries the MP3 function code (fn) and its numeric arg (track).
      obj["type"]  = "mp3";
      obj["fn"]    = a.fn;
      obj["track"] = a.track;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_DFPLAYER:
      // Destination is GLOBAL — see RcConfig::dfpDest. The action carries the
      // DFPlayer function code (fn) plus its args: chan = folder/EQ/device/flag,
      // track = track number, /MP3 index, or volume.
      obj["type"]  = "dfplayer";
      obj["fn"]    = a.fn;
      obj["chan"]  = a.chan;
      obj["track"] = a.track;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_WLED:
      // Routing (per-id → local port | remote WCB) is GLOBAL — RcConfig::wledSlots.
      // The action only carries the ";L<id>,<verb>" command string.
      obj["type"] = "wled";
      obj["cmd"]  = a.cmd;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_RECORD:
      obj["type"] = "record";
      obj["cmd"]  = a.cmd;   // clip name (library-ready; ignored by the single-clip build)
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_PLAY:
      obj["type"] = "play";
      obj["cmd"]  = a.cmd;   // clip name to play
      obj["fn"]   = a.fn;    // loop: 0=once, 1=repeat
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_STOP:
      obj["type"] = "stop";
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    // RA_SMOOTH_OVERRIDE retired (smoothing profiles superseded it) — not emitted.
    default: break;
  }
  if (a.note[0]) obj["note"] = a.note;
}

static bool actionFromJson(const JsonObject& obj, RcAction& a) {
  memset(&a, 0, sizeof(a));
  const char* type = obj["type"] | "";
  bool ok = false;
  if (strcmp(type, "wcb_unicast") == 0) {
    a.type = RA_WCB_UNICAST;
    strlcpy(a.target, obj["target"] | "", sizeof(a.target));
    strlcpy(a.cmd,    obj["cmd"]    | "", sizeof(a.cmd));
    a.delayMs     = obj["delay"]       | 0;
    a.skipRunning = obj["skipRunning"] | false;
    ok = true;
  } else if (strcmp(type, "wcb_broadcast") == 0) {
    a.type = RA_WCB_BROADCAST;
    strlcpy(a.cmd, obj["cmd"] | "", sizeof(a.cmd));
    a.delayMs     = obj["delay"]       | 0;
    a.skipRunning = obj["skipRunning"] | false;
    ok = true;
  } else if (strcmp(type, "maestro_local") == 0) {
    a.type = RA_MAESTRO_LOCAL;
    strlcpy(a.cmd, obj["cmd"] | "", sizeof(a.cmd));
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "maestro_remote") == 0 || strcmp(type, "maestro") == 0) {
    // "maestro" is the unified action emitted by the new config tool.
    // "maestro_remote" is accepted for backward compat with older saved configs.
    // Both store target = Maestro slot ID 1-8 (firmware uses rcConfig.maestros[]
    // to decide where the bytes actually go).
    a.type = RA_MAESTRO_REMOTE;
    strlcpy(a.target, obj["target"] | "", sizeof(a.target));
    strlcpy(a.cmd,    obj["cmd"]    | "", sizeof(a.cmd));
    a.delayMs     = obj["delay"]      | 0;
    a.skipRunning = obj["skipRunning"] | false;
    ok = true;
  } else if (strcmp(type, "serial") == 0) {
    a.type = RA_SERIAL;
    strlcpy(a.target, obj["port"] | "", sizeof(a.target));
    strlcpy(a.cmd,    obj["cmd"]  | "", sizeof(a.cmd));
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "hcr") == 0) {
    // HCR destination is now a global config (RcHcrDest in RcConfig).
    // Each action only carries the HCR command parameters.
    a.type    = RA_HCR;
    a.fn      = (uint8_t)(obj["fn"]    | 0);
    a.chan    = (int8_t) (obj["chan"]  | 0);
    a.track   = (int16_t)(obj["track"] | 0);
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "mp3") == 0) {
    // MP3 destination is a global config (RcMp3Dest). The action only carries
    // the MP3 function code (fn) and numeric arg (track).
    a.type    = RA_MP3;
    a.fn      = (uint8_t)(obj["fn"]    | 0);
    a.track   = (int16_t)(obj["track"] | 0);
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "dfplayer") == 0) {
    // DFPlayer destination is a global config (RcDfpDest). The action carries the
    // function code (fn) plus chan (folder/EQ/device/flag) and track (number/volume).
    a.type    = RA_DFPLAYER;
    a.fn      = (uint8_t)(obj["fn"]    | 0);
    a.chan    = (int8_t) (obj["chan"]  | 0);
    a.track   = (int16_t)(obj["track"] | 0);
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "wled") == 0) {
    // WLED routing is a global config (RcConfig::wledSlots). The action only
    // carries the ";L<id>,<verb>" command string.
    a.type = RA_WLED;
    strlcpy(a.cmd, obj["cmd"] | "", sizeof(a.cmd));
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "record") == 0) {
    a.type    = RA_RECORD;
    strlcpy(a.cmd, obj["cmd"] | "", sizeof(a.cmd));   // clip name
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "play") == 0) {
    a.type    = RA_PLAY;
    strlcpy(a.cmd, obj["cmd"] | "", sizeof(a.cmd));   // clip name
    a.fn      = (uint8_t)(obj["fn"] | 0);             // loop 0/1
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "stop") == 0) {
    a.type    = RA_STOP;
    a.delayMs = obj["delay"] | 0;
    ok = true;
  }
  // (type "smooth" / RA_SMOOTH_OVERRIDE retired — an old config's smooth action
  //  matches nothing here, so ok stays false and it's silently dropped.)
  if (ok) strlcpy(a.note, obj["note"] | "", sizeof(a.note));
  return ok;
}

// Write a knob output set into a JSON array (shared by the 3 per-mode arrays).
static void rcWriteKnobOuts(JsonArray arr, uint8_t count, const RcKnobOutput* outs) {
  for (uint8_t o = 0; o < count && o < RC_KNOB_MAX_OUTPUTS; o++) {
    JsonObject oObj = arr.createNestedObject();
    oObj["target"]    = outs[o].target;
    oObj["maestroCh"] = outs[o].maestroCh;
    oObj["posMin"]    = outs[o].posMin;
    oObj["posMax"]    = outs[o].posMax;
    if (outs[o].midClosed) oObj["midClosed"] = true;   // center-of-stick = posMin (upper-half mapping)
    if (outs[o].releaseIdleMs) oObj["releaseIdleMs"] = outs[o].releaseIdleMs;   // auto-release servo after N ms idle
    // NOTE: per-output smoothSpeed/smoothAccel are RETIRED — smoothing now lives
    // in smoothing profiles (RcKnob.smoothProfile). We no longer emit them; the
    // parser still reads them from legacy configs to migrate into profile 0.
  }
}

// Parse a knob output set from a JSON array. Returns the count written.
static uint8_t rcReadKnobOuts(JsonArray arr, RcKnobOutput* outs) {
  uint8_t n = 0;
  for (JsonObject oObj : arr) {
    if (n >= RC_KNOB_MAX_OUTPUTS) break;
    RcKnobOutput& out = outs[n];
    // New schema: "target" = Maestro ID 1-8 / audio chan. Legacy: "slot" (0=local→ID 1).
    if (oObj.containsKey("target")) out.target = (uint8_t)(oObj["target"] | 0);
    else { uint8_t legacy = (uint8_t)(oObj["slot"] | 0); out.target = (legacy == 0) ? 1 : legacy; }
    out.maestroCh = (uint8_t) (oObj["maestroCh"] | 0);
    out.posMin    = (uint16_t)(oObj["posMin"]    | 4000);
    out.posMax    = (uint16_t)(oObj["posMax"]    | 8000);
    out.smoothSpeed = (uint16_t)(oObj["smoothSpeed"] | 0);   // legacy — migrated into profile 0, then ignored
    out.smoothAccel = (uint8_t) (oObj["smoothAccel"] | 0);
    out.midClosed   = oObj["midClosed"] | false;             // center = posMin (upper-half passthrough mapping)
    out.releaseIdleMs = (uint16_t)(oObj["releaseIdleMs"] | 0);   // 0 = never auto-release the servo
    n++;
  }
  return n;
}

// Migration: copy any legacy per-output smoothing from an output set into
// profile 0's per-(Maestro,channel) map. Returns true if it wrote anything.
static bool rcSeedProfile0FromOuts(const RcKnobOutput* outs, uint8_t cnt) {
  bool any = false;
  for (uint8_t o = 0; o < cnt && o < RC_KNOB_MAX_OUTPUTS; o++) {
    if (!(outs[o].smoothSpeed || outs[o].smoothAccel)) continue;
    if (outs[o].target >= 1 && outs[o].target <= RC_NUM_MAESTROS && outs[o].maestroCh < 32) {
      RcSmoothEntry& e = rcConfig.smoothProfiles[0].entries[outs[o].target - 1][outs[o].maestroCh];
      e.speed = outs[o].smoothSpeed; e.accel = outs[o].smoothAccel; any = true;
    }
  }
  return any;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Serialise full config to JSON string (for GET_CONFIG WebSocket response)
// ─────────────────────────────────────────────────────────────────────────────
String rcConfigToJSON() {   // doc bumped to 64 KB to hold up to 6 smoothing profiles
  // ArduinoJson 7: the constructor capacity is a legacy no-op — the document
  // grows elastically from the heap as needed. The REAL guard is the
  // overflowed() check at the bottom: if any allocation failed mid-build
  // (heap exhausted by a very heavily-mapped 36-slot config), the JSON would
  // be silently MISSING mappings. Sending that truncated config would poison
  // the tool's baseline and a subsequent save would erase the dropped
  // mappings — so we return an ERROR envelope instead of truncated data.
  DynamicJsonDocument doc(65536);

  doc["txModel"]              = rcConfig.txModel;          // RcTxModel — GUI uses this to swap SVG / labels
  doc["threeAxisGimbals"]     = rcConfig.threeAxisGimbals; // X20 hardware option (shows/hides J5/J6 + stick-click matrix slots)
  doc["sbusOutEnabled"]       = rcConfig.sbusOutEnabled;   // SBUS-OUT passthrough enable
  doc["maeGateMs"]            = rcConfig.maeGateMs;         // remote skip-if-running fail-open window (ms)
  doc["boardType"]            = rcConfig.boardType;        // hardware pin profile (0=NaviCore v2, 1=WCB 3.2)
  doc["tapWindowMs"]          = rcConfig.tapWindowMs;
  doc["chRateHz"]             = rcConfig.chRateHz;          // rc_ch live-monitor broadcast rate (Hz, 1–20)
  doc["matrixChannel"]        = rcConfig.matrixChannel;
  doc["matrixDebounceFrames"] = rcConfig.matrixDebounceFrames;

  JsonObject fb = doc.createNestedObject("funcBindings");
  fb["mode"] = rcConfig.funcBindings.modeSwitch;

  // New-peer event: a passive-alert flag + one global tier of actions.
  JsonObject pe = doc.createNestedObject("peerEvent");
  pe["alert"] = rcConfig.peerAlert;
  JsonArray peActs = pe.createNestedArray("actions");
  for (int ai = 0; ai < rcConfig.peerNewActions.count; ai++)
    actionToJson(rcConfig.peerNewActions.a[ai], peActs.createNestedObject());

  JsonArray thArr = doc.createNestedArray("thresholds");
  for (int i = 0; i < RC_NUM_THRESHOLDS; i++) {
    JsonObject th = thArr.createNestedObject();
    th["id"]     = rcConfig.thresholds[i].id;
    th["label"]  = rcConfig.thresholds[i].label;
    th["minPwm"] = rcConfig.thresholds[i].minPwm;
    th["maxPwm"] = rcConfig.thresholds[i].maxPwm;
  }

  JsonObject mapObj = doc.createNestedObject("mappings");
  for (int mode = 1; mode <= 3; mode++) {
    for (int btn = 1; btn <= RC_NUM_THRESHOLDS; btn++) {
      int idx = rcMapIndex(mode, btn);
      const RcMapping& m = rcConfig.mappings[idx];
      bool hasAny = false;
      for (int ti = 0; ti < 3 && !hasAny; ti++) if (m.t[ti].count > 0 || m.t[ti].note[0]) hasAny = true;
      if (!hasAny && !m.exclusive) continue;

      String key = String(mode * 100 + btn);
      JsonObject mObj = mapObj.createNestedObject(key);
      mObj["exclusive"] = m.exclusive;
      for (int ti = 0; ti < 3; ti++) {
        if (m.t[ti].count > 0) {
          String tierKey = String("t") + (ti + 1);
          JsonArray acts = mObj.createNestedArray(tierKey);
          for (int ai = 0; ai < m.t[ti].count; ai++) {
            actionToJson(m.t[ti].a[ai], acts.createNestedObject());
          }
        }
        if (m.t[ti].note[0]) mObj[String("t") + (ti + 1) + "note"] = m.t[ti].note;
      }
    }
  }

  JsonObject swObj = doc.createNestedObject("switches");
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    const RcSwitch& s = rcConfig.switches[i];
    JsonObject sObj = swObj.createNestedObject(RC_SWITCH_LABELS[i]);
    sObj["channel"]   = s.channel;
    sObj["positions"] = s.positions;
    for (int pi = 0; pi < 3; pi++) {
      if (s.t[pi].count > 0) {
        String tierKey = String("p") + pi;
        JsonArray acts = sObj.createNestedArray(tierKey);
        for (int ai = 0; ai < s.t[pi].count; ai++) {
          actionToJson(s.t[pi].a[ai], acts.createNestedObject());
        }
      }
      if (s.t[pi].note[0]) sObj[String("p") + pi + "note"] = s.t[pi].note;
    }
  }

  JsonObject knObj = doc.createNestedObject("knobs");
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    const RcKnob& kn = rcConfig.knobs[i];
    JsonObject kObj = knObj.createNestedObject(RC_KNOB_LABELS[i]);
    kObj["channel"]   = kn.channel;
    kObj["function"]  = kn.function;
    kObj["reverse"]   = kn.reverse;
    kObj["modeAware"] = kn.modeAware;
    kObj["modeSwitchOverride"] = kn.modeSwitchOverride;   // -1 = global mode switch
    if (kn.smoothProfile != -1) kObj["smoothProfile"] = kn.smoothProfile;   // -1 = none (omitted)
    if (kn.easeSwitchOverride)  kObj["easeSwitchOverride"] = true;          // false = omitted (knob wins)
    rcWriteKnobOuts(kObj.createNestedArray("outputs"), kn.outputCount, kn.outputs);  // mode 1
    if (kn.modeAware) {   // per-mode sets only when opted in
      rcWriteKnobOuts(kObj.createNestedArray("outputs2"), kn.outputCount2[0], kn.outputs2[0]);
      rcWriteKnobOuts(kObj.createNestedArray("outputs3"), kn.outputCount2[1], kn.outputs2[1]);
    }
  }

  // Global HCR destination — every RA_HCR action reads from here.
  JsonObject hcrObj = doc.createNestedObject("hcrDest");
  hcrObj["transport"] = (rcConfig.hcrDest.transport == 1) ? "wcb" : "serial";
  if (rcConfig.hcrDest.transport == 1) {
    hcrObj["target"]  = rcConfig.hcrDest.target;       // WCB ID string
    hcrObj["wcbPort"] = rcConfig.hcrDest.wcbPort;
  } else {
    hcrObj["port"]    = rcConfig.hcrDest.target;       // "S3"/"S4"
  }

  // Maestro locations — describes where each of the 8 logical Maestros is wired.
  JsonArray maeArr = doc.createNestedArray("maestros");
  for (int i = 0; i < RC_NUM_MAESTROS; i++) {
    JsonObject mObj = maeArr.createNestedObject();
    mObj["type"]   = rcConfig.maestros[i].type;        // 0 disabled / 1 local / 2 remote
    mObj["device"] = rcConfig.maestros[i].device;      // 0-127 protocol, 255 compact
    // Imported per-channel names/endpoints — SPARSE: only channels the tool
    // actually set are emitted; an absent "channels" key = none for this slot.
    JsonArray chArr;
    for (int c = 0; c < RC_MAESTRO_CHANNELS; c++) {
      const RcMaestroChannel& mc = rcConfig.maestros[i].channels[c];
      if (mc.name[0] == '\0' && mc.minPos == 0 && mc.maxPos == 0) continue;
      if (chArr.isNull()) chArr = mObj.createNestedArray("channels");
      JsonObject cObj = chArr.createNestedObject();
      cObj["ch"]   = c;
      cObj["name"] = mc.name;      // ArduinoJson copies the C-string into the doc
      cObj["min"]  = mc.minPos;
      cObj["max"]  = mc.maxPos;
    }
  }

  // WCB network credentials — required for ESP-NOW peer setup.
  JsonObject wcbObj = doc.createNestedObject("wcbNetwork");
  wcbObj["macOct2"]  = rcConfig.wcbNetwork.macOct2;
  wcbObj["macOct3"]  = rcConfig.wcbNetwork.macOct3;
  wcbObj["password"] = rcConfig.wcbNetwork.password;
  wcbObj["quantity"] = rcConfig.wcbNetwork.quantity;
  wcbObj["deviceId"] = rcConfig.wcbNetwork.deviceId;
  wcbObj["channel"]  = rcConfig.wcbNetwork.channel;

  // Saved WCB-credential profiles (Dev / In-droid / …). Count-based dense array —
  // only the in-use entries are emitted. Inert storage; see RcWcbProfile.
  JsonArray wpArr = doc.createNestedArray("wcbProfiles");
  for (uint8_t i = 0; i < rcConfig.wcbProfileCount && i < RC_MAX_WCB_PROFILES; i++) {
    JsonObject o = wpArr.createNestedObject();
    o["name"]     = rcConfig.wcbProfiles[i].name;
    o["macOct2"]  = rcConfig.wcbProfiles[i].macOct2;
    o["macOct3"]  = rcConfig.wcbProfiles[i].macOct3;
    o["password"] = rcConfig.wcbProfiles[i].password;
    o["quantity"] = rcConfig.wcbProfiles[i].quantity;
    o["deviceId"] = rcConfig.wcbProfiles[i].deviceId;
    o["channel"]  = rcConfig.wcbProfiles[i].channel;
  }

  // Global MP3 Trigger destination — every RA_MP3 action reads from here.
  JsonObject mp3Obj = doc.createNestedObject("mp3Dest");
  mp3Obj["transport"] = (rcConfig.mp3Dest.transport == 1) ? "wcb" : "serial";
  if (rcConfig.mp3Dest.transport == 1) {
    mp3Obj["target"] = rcConfig.mp3Dest.target;        // WCB ID string
  } else {
    mp3Obj["port"]   = rcConfig.mp3Dest.target;        // "S3"/"S4"
  }

  // Global DFPlayer destination — every RA_DFPLAYER action reads from here.
  // Same {transport, target|port} shape as mp3Dest so the tool can share code.
  JsonObject dfpObj = doc.createNestedObject("dfpDest");
  dfpObj["transport"] = (rcConfig.dfpDest.transport == 1) ? "wcb" : "serial";
  if (rcConfig.dfpDest.transport == 1) {
    dfpObj["target"] = rcConfig.dfpDest.target;        // WCB ID string
  } else {
    dfpObj["port"]   = rcConfig.dfpDest.target;        // "S3"/"S4"/"S5"
  }

  // Per-id WLED routing table — dense array (index 0..RC_NUM_WLED-1) so slot
  // positions stay stable across round-trips. Every RA_WLED action routes through
  // this by the ;L<id> in its command.
  JsonArray wledArr = doc.createNestedArray("wledSlots");
  for (int i = 0; i < RC_NUM_WLED; i++) {
    JsonObject wObj = wledArr.createNestedObject();
    wObj["id"]         = rcConfig.wledSlots[i].wledID;      // 1-9 (0 = empty)
    wObj["port"]       = rcConfig.wledSlots[i].serialPort;  // 3/4/5 local, 0 = not local
    wObj["wcb"]        = rcConfig.wledSlots[i].remoteWCB;   // 1-20 remote, 0 = local
    wObj["configured"] = rcConfig.wledSlots[i].configured;
  }

  // Aux serial port baud rates — one source of truth for S3/S4 line rate.
  JsonObject auxObj = doc.createNestedObject("auxBaud");
  auxObj["S3"]      = rcConfig.auxBaud[0];
  auxObj["S4"]      = rcConfig.auxBaud[1];
  auxObj["S5"]      = rcConfig.auxBaud[2];
  auxObj["maestro"] = rcConfig.maestroBaud;   // local Maestro bus (Serial2)
  // Per-port OVERRIDE labels — only the non-empty ones, keyed by FIRMWARE port
  // exactly like auxBaud above ("S3"/"S4"/"S5"/"maestro"), so the stored config is
  // independent of the mesh S1-S3 numbering. Literal keys (flash) so ArduinoJson
  // stores them safely; values point into the persistent rcConfig (serialized
  // before it can change), same as auxObj/hcrDest.
  {
    JsonObject slObj;
    bool created = false;
    for (int s = 0; s < RC_NUM_SLBL; s++) {
      if (!rcConfig.serialLabels[s][0]) continue;
      if (!created) { slObj = doc.createNestedObject("serialLabels"); created = true; }
      slObj[RC_SLBL_KEYS[s]] = rcConfig.serialLabels[s];
    }
  }
  // Per-aux-port mesh bridging, keyed by firmware port like auxBaud. Always emitted
  // in full (3 ports x 2 flags is cheap) so the tool never has to guess a default.
  {
    JsonObject bcObj = doc.createNestedObject("serialBcast");
    for (int i = 0; i < 3; i++) {
      JsonObject p = bcObj.createNestedObject(RC_SLBL_KEYS[i]);   // "S3"/"S4"/"S5"
      p["out"] = rcConfig.serialBcastOut[i];
      p["in"]  = rcConfig.serialBcastIn[i];
    }
  }
  // Optional mode-select report (off by default).
  {
    JsonObject mr = doc.createNestedObject("modeReport");
    mr["enabled"]  = rcConfig.modeReport.enabled;
    mr["wcb"]      = rcConfig.modeReport.wcb;
    mr["template"] = rcConfig.modeReport.tmpl;
    JsonArray mc = mr.createNestedArray("cmds");
    for (int i = 0; i < 3; i++) mc.add(rcConfig.modeReport.cmds[i]);
  }

  // Smoothing profiles — all 6 slots emitted (array index = profile id, so knob
  // refs stay aligned); entries are SPARSE (only channels with speed|accel).
  JsonArray spArr = doc.createNestedArray("smoothProfiles");
  for (int p = 0; p < RC_NUM_SMOOTH_PROFILES; p++) {
    const RcSmoothProfile& prof = rcConfig.smoothProfiles[p];
    JsonObject pObj = spArr.createNestedObject();
    pObj["name"] = prof.name;
    JsonArray ents = pObj.createNestedArray("entries");
    for (int mid = 1; mid <= RC_NUM_MAESTROS; mid++)
      for (int ch = 0; ch < 32; ch++) {
        const RcSmoothEntry& e = prof.entries[mid - 1][ch];
        if (!e.speed && !e.accel) continue;   // sparse
        JsonObject eObj = ents.createNestedObject();
        eObj["mid"] = mid; eObj["ch"] = ch; eObj["spd"] = e.speed; eObj["acc"] = e.accel;
      }
  }

  // Truncation guard — see the note at the top of this function. Better to
  // fail loudly than hand the tool an incomplete config it would re-save.
  if (doc.overflowed()) {
    Serial.println("[CONFIG] ERROR: GET_CONFIG overflowed memory — config too large to serialize");
    return String("{\"type\":\"ERROR\",\"msg\":\"GET_CONFIG overflow — config too large, not sent. "
                  "Reduce mapped actions and retry.\"}");
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// Re-seed request for processSwitches(): true at boot and set true again after
// every config apply so a changed switch channel/positions re-seeds switchPrevPos
// without firing a phantom action. Read + cleared on Core 1 in processSwitches().
inline bool g_switchSeedPending = true;

// ─────────────────────────────────────────────────────────────────────────────
//  Load config from JSON object (from SET_CONFIG WebSocket message)
// ─────────────────────────────────────────────────────────────────────────────
bool rcConfigFromJSON(const JsonObject& doc) {
  if (doc.containsKey("txModel"))       rcConfig.txModel       = (uint8_t)(doc["txModel"] | (int)TX_MODEL_X18);
  if (doc.containsKey("threeAxisGimbals")) rcConfig.threeAxisGimbals = doc["threeAxisGimbals"] | false;
  if (doc.containsKey("sbusOutEnabled"))   rcConfig.sbusOutEnabled   = doc["sbusOutEnabled"]   | false;
  if (doc.containsKey("maeGateMs"))        rcConfig.maeGateMs        = doc["maeGateMs"]         | 250;
  if (doc.containsKey("boardType"))        rcConfig.boardType        = (uint8_t)(doc["boardType"] | 0);
  if (doc.containsKey("tapWindowMs"))   rcConfig.tapWindowMs   = doc["tapWindowMs"];
  // A sub-100 ms window makes the multi-tap test (now - lastTap < tapWindowMs)
  // effectively always-false, silently killing double/triple taps while single
  // taps keep working. Treat a nonsensically small/zero value as unset → default.
  if (rcConfig.tapWindowMs < 100) rcConfig.tapWindowMs = 500;
  if (doc.containsKey("chRateHz")) {
    int hz = doc["chRateHz"] | 5;            // rc_ch live-monitor rate; 0/absent → default 5 Hz
    if (hz < 1)  hz = 5;                      // a zero/negative rate would stall the channel stream entirely
    if (hz > 20) hz = 20;                     // clamp to the mesh-safe ceiling the UI slider enforces
    rcConfig.chRateHz = (uint8_t)hz;
  }
  if (doc.containsKey("matrixChannel")) rcConfig.matrixChannel = doc["matrixChannel"];
  if (doc.containsKey("matrixDebounceFrames")) {
    int d = doc["matrixDebounceFrames"] | 1;
    rcConfig.matrixDebounceFrames = (d < 1) ? 1 : (d > 4 ? 4 : d);   // clamp 1-4
  }

  if (doc.containsKey("funcBindings")) {
    JsonObject fb = doc["funcBindings"];
    rcConfig.funcBindings.modeSwitch = fb["mode"] | rcConfig.funcBindings.modeSwitch;
  }

  // New-peer event (diff-safe: only touched when the branch is present).
  if (doc.containsKey("peerEvent")) {
    JsonObject pe = doc["peerEvent"];
    rcConfig.peerAlert = pe["alert"] | rcConfig.peerAlert;
    if (pe.containsKey("actions")) {
      JsonArray acts = pe["actions"];
      int ai = 0;
      rcConfig.peerNewActions.count = 0;
      for (JsonObject aObj : acts) {
        if (ai >= RC_ACTIONS_PER_TIER) break;
        if (actionFromJson(aObj, rcConfig.peerNewActions.a[ai])) rcConfig.peerNewActions.count = ++ai;
      }
    }
  }

  if (doc.containsKey("thresholds")) {
    JsonArray thArr = doc["thresholds"];
    int i = 0;
    for (JsonObject th : thArr) {
      if (i >= RC_NUM_THRESHOLDS) break;
      rcConfig.thresholds[i].id = th["id"] | (i + 1);
      strlcpy(rcConfig.thresholds[i].label, th["label"] | "", 24);
      rcConfig.thresholds[i].minPwm = th["minPwm"] | 0;
      rcConfig.thresholds[i].maxPwm = th["maxPwm"] | 0;
      i++;
    }
  }

  if (doc.containsKey("mappings")) {
    JsonObject mapObj = doc["mappings"];
    for (JsonPair kv : mapObj) {
      int buttonId = String(kv.key().c_str()).toInt();
      int mode = buttonId / 100, btn = buttonId % 100;
      if (mode < 1 || mode > 3 || btn < 1 || btn > RC_NUM_THRESHOLDS) continue;
      int idx = rcMapIndex(mode, btn);
      RcMapping& m = rcConfig.mappings[idx];
      memset(&m, 0, sizeof(m));
      JsonObject mObj = kv.value().as<JsonObject>();
      m.exclusive = mObj["exclusive"] | false;
      for (int ti = 0; ti < 3; ti++) {
        strlcpy(m.t[ti].note, mObj[String("t") + (ti + 1) + "note"] | "", sizeof(m.t[ti].note));
        String tierKey = String("t") + (ti + 1);
        if (!mObj.containsKey(tierKey)) continue;
        JsonArray acts = mObj[tierKey];
        int ai = 0;
        for (JsonObject aObj : acts) {
          if (ai >= RC_ACTIONS_PER_TIER) break;
          if (actionFromJson(aObj, m.t[ti].a[ai])) m.t[ti].count = ++ai;
        }
      }
    }
  }

  if (doc.containsKey("switches")) {
    JsonObject swObj = doc["switches"];
    for (int i = 0; i < RC_NUM_SWITCHES; i++) {
      if (!swObj.containsKey(RC_SWITCH_LABELS[i])) continue;
      JsonObject sObj = swObj[RC_SWITCH_LABELS[i]];
      RcSwitch& s = rcConfig.switches[i];
      memset(s.t, 0, sizeof(s.t));
      s.channel   = sObj["channel"]   | RC_SWITCH_DEFAULT_CH[i];
      s.positions = sObj["positions"] | RC_SWITCH_DEFAULT_POS[i];
      for (int pi = 0; pi < 3; pi++) {
        strlcpy(s.t[pi].note, sObj[String("p") + pi + "note"] | "", sizeof(s.t[pi].note));
        String tierKey = String("p") + pi;
        if (!sObj.containsKey(tierKey)) continue;
        JsonArray acts = sObj[tierKey];
        int ai = 0;
        for (JsonObject aObj : acts) {
          if (ai >= RC_ACTIONS_PER_TIER) break;
          if (actionFromJson(aObj, s.t[pi].a[ai])) s.t[pi].count = ++ai;
        }
      }
    }
  }

  if (doc.containsKey("knobs")) {
    JsonObject knObj = doc["knobs"];
    for (int i = 0; i < RC_NUM_KNOBS; i++) {
      if (!knObj.containsKey(RC_KNOB_LABELS[i])) continue;
      JsonObject kObj = knObj[RC_KNOB_LABELS[i]];
      RcKnob& kn = rcConfig.knobs[i];
      kn.channel   = kObj["channel"]   | RC_KNOB_DEFAULT_CH[i];
      kn.function  = kObj["function"]  | RC_KNOB_DEFAULT_FN[i];
      kn.reverse   = kObj["reverse"]   | false;
      kn.modeAware = kObj["modeAware"] | false;
      kn.modeSwitchOverride = kObj.containsKey("modeSwitchOverride") ? (int8_t)kObj["modeSwitchOverride"].as<int>() : -1;
      kn.smoothProfile      = kObj.containsKey("smoothProfile")      ? (int8_t)kObj["smoothProfile"].as<int>()      : -1;
      kn.easeSwitchOverride = kObj["easeSwitchOverride"] | false;
      kn.outputCount = 0; memset(kn.outputs, 0, sizeof(kn.outputs));
      kn.outputCount2[0] = kn.outputCount2[1] = 0; memset(kn.outputs2, 0, sizeof(kn.outputs2));
      if (kObj.containsKey("outputs"))                       // mode 1 (also loads legacy configs)
        kn.outputCount = rcReadKnobOuts(kObj["outputs"], kn.outputs);
      if (kn.modeAware) {                                    // modes 2 & 3 — seed from mode 1 if absent
        if (kObj.containsKey("outputs2")) kn.outputCount2[0] = rcReadKnobOuts(kObj["outputs2"], kn.outputs2[0]);
        else { kn.outputCount2[0] = kn.outputCount; memcpy(kn.outputs2[0], kn.outputs, sizeof(kn.outputs)); }
        if (kObj.containsKey("outputs3")) kn.outputCount2[1] = rcReadKnobOuts(kObj["outputs3"], kn.outputs2[1]);
        else { kn.outputCount2[1] = kn.outputCount; memcpy(kn.outputs2[1], kn.outputs, sizeof(kn.outputs)); }
      }
    }
  }

  // Smoothing profiles (array index = profile id). Update ONLY when the message
  // carries the branch — a diff-save (SET_CONFIG) that omits smoothProfiles must
  // NOT wipe them, so the clear lives INSIDE the containsKey guard.
  if (doc.containsKey("smoothProfiles")) {
    for (int p = 0; p < RC_NUM_SMOOTH_PROFILES; p++) {
      memset(rcConfig.smoothProfiles[p].entries, 0, sizeof(rcConfig.smoothProfiles[p].entries));
      rcConfig.smoothProfiles[p].name[0] = '\0';
    }
    int p = 0;
    for (JsonObject pObj : doc["smoothProfiles"].as<JsonArray>()) {
      if (p >= RC_NUM_SMOOTH_PROFILES) break;
      RcSmoothProfile& prof = rcConfig.smoothProfiles[p];
      strlcpy(prof.name, pObj["name"] | "", sizeof(prof.name));
      if (pObj.containsKey("entries"))
        for (JsonObject e : pObj["entries"].as<JsonArray>()) {
          int mid = e["mid"] | 1, ch = e["ch"] | 0;
          if (mid >= 1 && mid <= RC_NUM_MAESTROS && ch >= 0 && ch < 32) {
            prof.entries[mid - 1][ch].speed = (uint16_t)(e["spd"] | 0);
            prof.entries[mid - 1][ch].accel = (uint8_t) (e["acc"] | 0);
          }
        }
      p++;
    }
  }
  // MIGRATION: legacy per-output smoothing → Default profile 0, but ONLY for a
  // passthrough knob with no profile assigned (an old config). New configs have
  // smoothProfile set and no per-output smoothing, so this is a no-op for them.
  for (int ki = 0; ki < RC_NUM_KNOBS; ki++) {
    RcKnob& kn = rcConfig.knobs[ki];
    if (kn.function != KF_MAESTRO_PASSTHROUGH || kn.smoothProfile != -1) continue;
    bool any = rcSeedProfile0FromOuts(kn.outputs, kn.outputCount);
    if (kn.modeAware) {
      if (rcSeedProfile0FromOuts(kn.outputs2[0], kn.outputCount2[0])) any = true;
      if (rcSeedProfile0FromOuts(kn.outputs2[1], kn.outputCount2[1])) any = true;
    }
    if (any) {
      kn.smoothProfile = 0;
      if (!rcConfig.smoothProfiles[0].name[0]) strlcpy(rcConfig.smoothProfiles[0].name, "Default", sizeof(rcConfig.smoothProfiles[0].name));
    }
  }

  if (doc.containsKey("maestros")) {
    JsonArray maeArr = doc["maestros"];
    int i = 0;
    for (JsonObject mObj : maeArr) {
      if (i >= RC_NUM_MAESTROS) break;
      rcConfig.maestros[i].type   = (uint8_t)(mObj["type"]   | 0);
      // Presence check, not '|': ArduinoJson's '|' substitutes the default on a
      // falsy value, and JSON 0 is falsy — so a legal Pololu device #0 would be
      // silently rewritten to 1+i. Preserve an explicit 0.
      rcConfig.maestros[i].device = mObj.containsKey("device") ? (uint8_t)mObj["device"].as<int>() : (uint8_t)(1 + i);
      // Per-channel metadata: FULL-REPLACE this slot's channels, but only when
      // the "channels" key is present — so an older tool that omits the field
      // can't wipe the names/endpoints already stored on the device.
      if (mObj.containsKey("channels")) {
        memset(rcConfig.maestros[i].channels, 0, sizeof(rcConfig.maestros[i].channels));
        for (JsonObject cObj : mObj["channels"].as<JsonArray>()) {
          // Presence check, not '|': JSON 0 is falsy to ArduinoJson's '|', which
          // would drop channel 0 (a legal, common channel — e.g. the first servo).
          if (!cObj.containsKey("ch")) continue;
          int c = cObj["ch"].as<int>();
          if (c < 0 || c >= RC_MAESTRO_CHANNELS) continue;
          RcMaestroChannel& mc = rcConfig.maestros[i].channels[c];
          strlcpy(mc.name, cObj["name"] | "", sizeof(mc.name));
          mc.minPos = (uint16_t)(cObj["min"] | 0);
          mc.maxPos = (uint16_t)(cObj["max"] | 0);
        }
      }
      i++;
    }
  }

  if (doc.containsKey("hcrDest")) {
    JsonObject hcrObj = doc["hcrDest"];
    const char* tp = hcrObj["transport"] | "serial";
    rcConfig.hcrDest.transport = (strcmp(tp, "wcb") == 0) ? 1 : 0;
    if (rcConfig.hcrDest.transport == 1) {
      // HCR over WCB is unicast-only; default to WCB 2 (not 0/broadcast) when
      // the key is missing so a partial config doesn't produce an invalid target.
      strlcpy(rcConfig.hcrDest.target, hcrObj["target"] | "2", sizeof(rcConfig.hcrDest.target));
      rcConfig.hcrDest.wcbPort = (uint8_t)(hcrObj["wcbPort"] | 1);
    } else {
      strlcpy(rcConfig.hcrDest.target, hcrObj["port"]   | "S3", sizeof(rcConfig.hcrDest.target));
      rcConfig.hcrDest.wcbPort = 0;
    }
  }

  if (doc.containsKey("wcbNetwork")) {
    JsonObject wcbObj = doc["wcbNetwork"];
    // Presence-check, not '|': ArduinoJson's | treats a JSON 0 as "missing", but
    // 0x00 is a legitimate (factory-default) MAC octet, so | would silently keep
    // the old value when the user sets an octet to 0.
    if (wcbObj.containsKey("macOct2")) rcConfig.wcbNetwork.macOct2 = (uint8_t)(wcbObj["macOct2"].as<int>() & 0xFF);
    if (wcbObj.containsKey("macOct3")) rcConfig.wcbNetwork.macOct3 = (uint8_t)(wcbObj["macOct3"].as<int>() & 0xFF);
    strlcpy(rcConfig.wcbNetwork.password,
            wcbObj["password"] | rcConfig.wcbNetwork.password,
            sizeof(rcConfig.wcbNetwork.password));
    rcConfig.wcbNetwork.quantity = (uint8_t)(wcbObj["quantity"] | rcConfig.wcbNetwork.quantity);
    rcConfig.wcbNetwork.deviceId = (uint8_t)(wcbObj["deviceId"] | rcConfig.wcbNetwork.deviceId);
    { int ch = wcbObj["channel"] | (int)rcConfig.wcbNetwork.channel;   // ESP-NOW mesh channel (1-13)
      rcConfig.wcbNetwork.channel = (uint8_t)(ch < 1 ? 1 : ch > 13 ? 13 : ch); }
  }

  // Saved WCB-credential profiles. Guarded by containsKey so a diff-save that omits
  // "wcbProfiles" can't wipe the stored list; when present, the tool always sends the
  // FULL set, so the list is rebuilt wholesale (an empty array legitimately clears it).
  if (doc.containsKey("wcbProfiles")) {
    JsonArray wpArr = doc["wcbProfiles"];
    uint8_t n = 0;
    for (JsonObject o : wpArr) {
      if (n >= RC_MAX_WCB_PROFILES) break;
      RcWcbProfile& wp = rcConfig.wcbProfiles[n];
      strlcpy(wp.name, o["name"] | "", sizeof(wp.name));
      wp.macOct2  = (uint8_t)(o["macOct2"].as<int>() & 0xFF);
      wp.macOct3  = (uint8_t)(o["macOct3"].as<int>() & 0xFF);
      strlcpy(wp.password, o["password"] | "", sizeof(wp.password));
      wp.quantity = (uint8_t)(o["quantity"] | 4);
      wp.deviceId = (uint8_t)(o["deviceId"] | 20);
      { int ch = o["channel"] | 1; wp.channel = (uint8_t)(ch < 1 ? 1 : ch > 13 ? 13 : ch); }
      n++;
    }
    rcConfig.wcbProfileCount = n;
  }

  if (doc.containsKey("mp3Dest")) {
    JsonObject mp3Obj = doc["mp3Dest"];
    const char* tp = mp3Obj["transport"] | "wcb";
    rcConfig.mp3Dest.transport = (strcmp(tp, "wcb") == 0) ? 1 : 0;
    if (rcConfig.mp3Dest.transport == 1) {
      strlcpy(rcConfig.mp3Dest.target, mp3Obj["target"] | "2",
              sizeof(rcConfig.mp3Dest.target));
    } else {
      strlcpy(rcConfig.mp3Dest.target, mp3Obj["port"] | "S3",
              sizeof(rcConfig.mp3Dest.target));
    }
  }

  if (doc.containsKey("dfpDest")) {
    JsonObject dfpObj = doc["dfpDest"];
    const char* tp = dfpObj["transport"] | "serial";   // local is the DFPlayer default
    rcConfig.dfpDest.transport = (strcmp(tp, "wcb") == 0) ? 1 : 0;
    if (rcConfig.dfpDest.transport == 1) {
      strlcpy(rcConfig.dfpDest.target, dfpObj["target"] | "2",
              sizeof(rcConfig.dfpDest.target));
    } else {
      strlcpy(rcConfig.dfpDest.target, dfpObj["port"] | "S3",
              sizeof(rcConfig.dfpDest.target));
    }
  }

  // Per-id WLED routing table. Guarded by containsKey so a diff-save that omits
  // "wledSlots" can't wipe it. Numeric fields use presence checks, not '|': id/
  // port/wcb 0 is a legal value that '|' (0 is falsy) would silently drop.
  if (doc.containsKey("wledSlots")) {
    JsonArray wledArr = doc["wledSlots"];
    int i = 0;
    for (JsonObject wObj : wledArr) {
      if (i >= RC_NUM_WLED) break;
      rcConfig.wledSlots[i].wledID     = (uint8_t)(wObj["id"].as<int>()   & 0xFF);
      rcConfig.wledSlots[i].serialPort = (uint8_t)(wObj["port"].as<int>() & 0xFF);
      rcConfig.wledSlots[i].remoteWCB  = (uint8_t)(wObj["wcb"].as<int>()  & 0xFF);
      rcConfig.wledSlots[i].configured = wObj["configured"] | false;
      i++;
    }
    // A dense array always carries RC_NUM_WLED entries; a shorter (legacy/hand-
    // authored) array leaves trailing slots as-is — clear them so a shrink can't
    // leave a stale slot configured.
    for (; i < RC_NUM_WLED; i++) memset(&rcConfig.wledSlots[i], 0, sizeof(rcConfig.wledSlots[i]));
  }

  if (doc.containsKey("auxBaud")) {
    JsonObject auxObj = doc["auxBaud"];
    rcConfig.auxBaud[0] = (uint32_t)(auxObj["S3"]      | rcConfig.auxBaud[0]);
    rcConfig.auxBaud[1] = (uint32_t)(auxObj["S4"]      | rcConfig.auxBaud[1]);
    rcConfig.auxBaud[2] = (uint32_t)(auxObj["S5"]      | rcConfig.auxBaud[2]);
    rcConfig.maestroBaud = (uint32_t)(auxObj["maestro"] | rcConfig.maestroBaud);
  }
  // Per-port override labels, keyed by firmware port ("S3"/"S4"/"S5"/"maestro").
  // Guarded by containsKey so a diff-save that omits it can't wipe them;
  // FULL-replace when present so clearing a label (dropping its key) is honored.
  // An unrecognized key is ignored — a config written before the keys moved from
  // WDP port numbers ("1".."5") leaves the labels empty, so every port falls back
  // to its auto-derived default rather than silently landing on the wrong port.
  if (doc.containsKey("serialLabels")) {
    memset(rcConfig.serialLabels, 0, sizeof(rcConfig.serialLabels));
    JsonObject slObj = doc["serialLabels"];
    for (JsonPair kv : slObj) {
      for (int s = 0; s < RC_NUM_SLBL; s++) {
        if (strcmp(kv.key().c_str(), RC_SLBL_KEYS[s]) != 0) continue;
        strlcpy(rcConfig.serialLabels[s], kv.value() | "", sizeof(rcConfig.serialLabels[0]));
        break;
      }
    }
  }
  // Per-aux-port mesh bridging. containsKey-guarded (a diff-save that omits it keeps
  // the current flags) and per-flag defaulted to the current value, so a partial
  // object only changes what it actually carries.
  if (doc.containsKey("serialBcast")) {
    JsonObject bcObj = doc["serialBcast"];
    for (int i = 0; i < 3; i++) {
      if (!bcObj.containsKey(RC_SLBL_KEYS[i])) continue;
      JsonObject p = bcObj[RC_SLBL_KEYS[i]];
      rcConfig.serialBcastOut[i] = p["out"] | rcConfig.serialBcastOut[i];
      rcConfig.serialBcastIn[i]  = p["in"]  | rcConfig.serialBcastIn[i];
    }
  }
  // Optional mode-select report. containsKey-guarded, and per-field defaulted to the
  // current value, so a partial object only changes what it carries.
  if (doc.containsKey("modeReport")) {
    JsonObject mr = doc["modeReport"];
    rcConfig.modeReport.enabled = mr["enabled"] | rcConfig.modeReport.enabled;
    rcConfig.modeReport.wcb     = (uint8_t)(mr["wcb"] | rcConfig.modeReport.wcb);
    if (mr.containsKey("template"))
      strlcpy(rcConfig.modeReport.tmpl, mr["template"] | "", sizeof(rcConfig.modeReport.tmpl));
    if (mr.containsKey("cmds")) {
      JsonArray mc = mr["cmds"];
      for (int i = 0; i < 3; i++)
        strlcpy(rcConfig.modeReport.cmds[i],
                (!mc.isNull() && i < (int)mc.size()) ? (mc[i] | "") : "",
                sizeof(rcConfig.modeReport.cmds[0]));
    }
  }
  // A switch's channel/positions may have changed — re-seed switchPrevPos on the
  // next SBUS frame so processSwitches doesn't fire a phantom action from a stale
  // prior position. (Boot already starts with this true.)
  g_switchSeedPending = true;
  return true;
}

bool rcConfigFromJSON(const String& json) {
  DynamicJsonDocument doc(98304);
  DeserializationError err = deserializeJson(doc, json);
  if (err != DeserializationError::Ok) {
    Serial.printf("rcConfigFromJSON parse failed: %s\n", err.c_str());
    return false;
  }
  return rcConfigFromJSON(doc.as<JsonObject>());
}

// ── Port labels + mesh port numbering ────────────────────────────────────────
// NaviCore uses TWO port numbering schemes and they are deliberately distinct:
//
//   FIRMWARE port  S3/S4/S5  — the aux UARTs as this code, the config JSON, and
//                              hcrDest/mp3Dest/wledSlots.serialPort name them.
//   MESH port      S1/S2/S3  — what the rest of the mesh addresses, matching the
//                              NaviCore v2 silkscreen ("Serial 1/2/3"). This is
//                              what a WCB means in `;w20,;s2<cmd>` and what WDP
//                              PORTLABEL advertises, so a WCB prints "S2".
//
// rcFwPortForMesh/rcMeshPortForFw convert; every mesh-facing path converts once at
// the boundary and works in firmware numbering internally.
inline int rcFwPortForMesh(int meshPort) {                  // S1/S2/S3 → S3/S4/S5
  return (meshPort >= 1 && meshPort <= 3) ? meshPort + 2 : 0;
}
inline int rcMeshPortForFw(int fwPort) {                    // S3/S4/S5 → S1/S2/S3
  return (fwPort >= 3 && fwPort <= 5) ? fwPort - 2 : 0;
}

// The effective label for a port-label slot (RC_SLBL_*): the USER OVERRIDE if set,
// else an auto-derived default from what the config routes to that port.
// rcAdvertiseSerialLabels() (NaviCore.ino) pushes these to WCB_Client::setPortLabel
// so WCBs + the Wizard see what's attached to each NaviCore port.
inline const char* rcSerialLabelAuto(int slot) {
  if (slot == RC_SLBL_MAESTRO) return "Maestro";            // local Maestro on Serial2
  if (slot >= RC_SLBL_S3 && slot <= RC_SLBL_S5) {
    int fwPort = slot + 3;                                  // slot 0/1/2 → firmware S3/S4/S5
    if (rcConfig.hcrDest.transport == 0 && rcConfig.hcrDest.target[0] == 'S' && atoi(rcConfig.hcrDest.target + 1) == fwPort) return "HCR";
    if (rcConfig.mp3Dest.transport == 0 && rcConfig.mp3Dest.target[0] == 'S' && atoi(rcConfig.mp3Dest.target + 1) == fwPort) return "MP3";
    if (rcConfig.dfpDest.transport == 0 && rcConfig.dfpDest.target[0] == 'S' && atoi(rcConfig.dfpDest.target + 1) == fwPort) return "DFPlayer";
    for (int i = 0; i < RC_NUM_WLED; i++)
      if (rcConfig.wledSlots[i].configured && rcConfig.wledSlots[i].remoteWCB == 0 && rcConfig.wledSlots[i].serialPort == fwPort) return "WLED";
  }
  return "";
}
inline const char* rcSerialLabel(int slot) {
  if (slot < 0 || slot >= RC_NUM_SLBL) return "";
  if (rcConfig.serialLabels[slot][0]) return rcConfig.serialLabels[slot];   // user override wins
  return rcSerialLabelAuto(slot);
}
// Build the mode-select report command for `mode` (1-3) into `out`. A non-empty
// per-mode override wins; else the template with every "{mode}" replaced by the
// position digit; else empty (→ nothing to send). Returns `out`.
inline const char* rcModeReportCmd(int mode, char* out, size_t outSz) {
  if (!out || outSz == 0) return out;
  out[0] = '\0';
  if (mode < 1 || mode > 3) return out;
  const char* ov = rcConfig.modeReport.cmds[mode - 1];
  if (ov[0]) { strlcpy(out, ov, outSz); return out; }
  const char* t = rcConfig.modeReport.tmpl;
  if (!t[0]) return out;
  const char md = (char)('0' + mode);
  size_t o = 0;
  for (size_t i = 0; t[i] && o + 1 < outSz; ) {
    if (!strncmp(t + i, "{mode}", 6)) { if (o + 1 < outSz) out[o++] = md; i += 6; }
    else                              { out[o++] = t[i++]; }
  }
  out[o] = '\0';
  return out;
}
// Which WDP PORTLABEL port number a label slot is advertised under. The three aux
// headers go out as the mesh ports that address them (S1/S2/S3); the Maestro takes
// port 4 — visible in ?WDP,LIST and the Wizard, but not a port `;s<n>` can reach.
inline int rcWdpPortForLabel(int slot) {
  if (slot >= RC_SLBL_S3 && slot <= RC_SLBL_S5) return slot + 1;   // → WDP 1/2/3
  if (slot == RC_SLBL_MAESTRO)                  return 4;
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  NVS persistence (namespace "rcfg")
//  Same key layout as Body Controller for familiar tooling.
//  Split across keys to stay within the NVS 4000-byte per-value limit.
// ─────────────────────────────────────────────────────────────────────────────
// Serialize `doc` into NVS key `key`. Returns false (and logs which key + why)
// if the JSON pool overflowed, the serialized string exceeds the 4000-byte NVS
// per-value limit, or putString reports a short write. Without these checks an
// oversize value fails SILENTLY and the config is lost on the next boot while
// the user is told "saved".
static bool _nvsPutJson(Preferences& prefs, const char* key, JsonDocument& doc) {
  String s;
  serializeJson(doc, s);
  if (doc.overflowed()) {
    Serial.printf("[CONFIG] NVS '%s' FAILED — JSON pool overflowed (config too large)\n", key);
    return false;
  }
  if (s.length() >= 4000) {
    Serial.printf("[CONFIG] NVS '%s' FAILED — %u bytes exceeds the 4000-byte NVS limit\n",
                  key, (unsigned)s.length());
    return false;
  }
  size_t n = prefs.putString(key, s);
  if (n != s.length()) {
    Serial.printf("[CONFIG] NVS '%s' FAILED — wrote %u of %u bytes\n",
                  key, (unsigned)n, (unsigned)s.length());
    return false;
  }
  return true;
}

// ── LittleFS-backed config persistence ──────────────────────────────────────
// Config persists as ONE JSON file (/config.json) on the LittleFS data
// partition (the "spiffs" partition already present in the min_spiffs scheme).
// This removes the NVS 4000-byte-per-value limit that silently dropped a
// densely-mapped mode. rcConfigToJSON()/rcConfigFromJSON() are reused verbatim,
// so the on-disk shape matches the SET_CONFIG/GET_CONFIG wire shape. The legacy
// NVS save/load below is retained ONLY as a one-time migration source.
static const char* RC_CFG_PATH     = "/config.json";
static const char* RC_CFG_TMP_PATH = "/config.json.tmp";
bool g_lfsReady = false;

// Mount LittleFS once (format on first mount). Call from setup() before load.
bool rcConfigBeginLFS() {
  g_lfsReady = LittleFS.begin(true);   // true = format the partition if mount fails (first boot)
  if (!g_lfsReady)
    Serial.println("[CONFIG] LittleFS mount FAILED — new saves will NOT persist this boot");
  return g_lfsReady;
}

// Write the full config to /config.json ATOMICALLY (write tmp, then rename) so a
// power loss mid-write can never corrupt the live file. Returns false on any
// failure (the caller surfaces it in the SET_CONFIG ACK).
bool rcConfigSaveLFS() {
  if (!g_lfsReady) { Serial.println("[CONFIG] LittleFS not mounted — save skipped"); return false; }
  String json = rcConfigToJSON();
  if (json.startsWith("{\"type\":\"ERROR\"")) {   // rcConfigToJSON hit a heap overflow
    Serial.println("[CONFIG] LFS save aborted — config failed to serialize");
    return false;
  }
  File f = LittleFS.open(RC_CFG_TMP_PATH, "w");   // "w" truncates any stale tmp
  if (!f) { Serial.println("[CONFIG] LFS: cannot open temp file for write"); return false; }
  size_t n = f.print(json);
  f.close();
  if (n != json.length()) {
    Serial.printf("[CONFIG] LFS: short write %u/%u — keeping previous config\n",
                  (unsigned)n, (unsigned)json.length());
    LittleFS.remove(RC_CFG_TMP_PATH);
    return false;
  }
  // Re-open and verify the ON-FLASH size. f.print() only counts bytes into the
  // core's 4 KB stdio buffer; the final flush happens inside close(), whose error
  // the core discards. So a near-full-partition truncation of the last <=4 KB
  // would otherwise slip past the 'n' check and be promoted over the good config.
  {
    File chk = LittleFS.open(RC_CFG_TMP_PATH, "r");
    size_t onDisk = chk ? chk.size() : 0;
    if (chk) chk.close();
    if (onDisk != json.length()) {
      Serial.printf("[CONFIG] LFS: temp file truncated on flash (%u/%u) — config not updated\n",
                    (unsigned)onDisk, (unsigned)json.length());
      LittleFS.remove(RC_CFG_TMP_PATH);
      return false;
    }
  }
  // Atomic replace — esp_littlefs lfs_rename overwrites the destination
  // atomically, so the live config is never left half-written. On failure the
  // previous config stays intact and we just drop the temp.
  if (!LittleFS.rename(RC_CFG_TMP_PATH, RC_CFG_PATH)) {
    Serial.println("[CONFIG] LFS: rename failed — previous config kept");
    LittleFS.remove(RC_CFG_TMP_PATH);
    return false;
  }
  Serial.printf("RC config saved to LittleFS (%u bytes).\n", (unsigned)json.length());
  return true;
}

// Load /config.json into rcConfig. Returns false if the file is missing, empty,
// or unparseable (the caller then falls back to NVS migration / defaults).
bool rcConfigLoadLFS() {
  if (!g_lfsReady || !LittleFS.exists(RC_CFG_PATH)) return false;
  File f = LittleFS.open(RC_CFG_PATH, "r");
  if (!f) return false;
  if (f.size() == 0) { f.close(); return false; }
  DynamicJsonDocument doc(98304);                 // ArduinoJson 7: capacity is elastic; overflowed() is the guard
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.printf("[CONFIG] LFS: /config.json parse failed (%s) — keeping file, using defaults this boot\n", err.c_str());
    return false;
  }
  bool applied = rcConfigFromJSON(doc.as<JsonObject>());
  if (applied) Serial.println("RC config loaded from LittleFS.");
  return applied;
}

// ── Custom command-library storage (config tool's private boards) ────────────
// The RC stores this JSON OPAQUELY — it never parses or uses it. It's the config
// tool's private command-library boards, kept on the droid so they travel to any
// browser/machine that connects. Saved/served verbatim; the tool owns the shape.
static const char* RC_CMDLIB_PATH      = "/cmdlib.json";
static const char* RC_CMDLIB_TMP_PATH  = "/cmdlib.json.tmp";
// Bulk-transfer staging file — DISTINCT from RC_CMDLIB_TMP_PATH so a concurrent
// old-style rcCmdlibSaveLFS() (which truncates+renames+removes the .tmp) can never
// unlink the inode a multi-second bulk session is still seek-writing.
static const char* RC_CMDLIB_BULK_TMP  = "/cmdlib.json.bulk.tmp";
// Download staging — the wrapped {"type":"CMDLIB",...,"data":<lib>} payload is
// built here and streamed out fragmented, so a large library never sits in a RAM
// String on the RC (O(1) send). Overwritten each pull; cleaned at boot.
static const char* RC_CMDLIB_SEND_TMP  = "/cmdlib.send.tmp";

// Atomic write (tmp + rename + on-flash size verify), same discipline as the
// config save so a power loss can't corrupt the live file. `json` is the raw
// library JSON (the tool's {schema,enums,boards} object). Returns false on any
// failure (surfaced in the SET_CMDLIB ACK).
bool rcCmdlibSaveLFS(const String& json) {
  if (!g_lfsReady) { Serial.println("[CMDLIB] LittleFS not mounted — save skipped"); return false; }
  File f = LittleFS.open(RC_CMDLIB_TMP_PATH, "w");
  if (!f) { Serial.println("[CMDLIB] LFS: cannot open temp file for write"); return false; }
  size_t n = f.print(json);
  f.close();
  if (n != json.length()) {
    Serial.printf("[CMDLIB] LFS: short write %u/%u — keeping previous\n", (unsigned)n, (unsigned)json.length());
    LittleFS.remove(RC_CMDLIB_TMP_PATH); return false;
  }
  { File chk = LittleFS.open(RC_CMDLIB_TMP_PATH, "r"); size_t onDisk = chk ? chk.size() : 0; if (chk) chk.close();
    if (onDisk != json.length()) {
      Serial.printf("[CMDLIB] LFS: temp truncated on flash (%u/%u) — not updated\n", (unsigned)onDisk, (unsigned)json.length());
      LittleFS.remove(RC_CMDLIB_TMP_PATH); return false;
    } }
  if (!LittleFS.rename(RC_CMDLIB_TMP_PATH, RC_CMDLIB_PATH)) {
    Serial.println("[CMDLIB] LFS: rename failed — previous kept");
    LittleFS.remove(RC_CMDLIB_TMP_PATH); return false;
  }
  Serial.printf("[CMDLIB] saved to LittleFS (%u bytes).\n", (unsigned)json.length());
  return true;
}

// Read /cmdlib.json into `out` (raw JSON). Returns false if missing/empty.
bool rcCmdlibLoadLFS(String& out) {
  if (!g_lfsReady || !LittleFS.exists(RC_CMDLIB_PATH)) return false;
  File f = LittleFS.open(RC_CMDLIB_PATH, "r");
  if (!f) return false;
  if (f.size() == 0) { f.close(); return false; }
  out = f.readString();
  f.close();
  return out.length() > 0;
}

// FNV-1a 32-bit hash of a string — the config tool's "is my cached copy current?"
// signature. Computed by the RC over the stored library bytes and reported back
// (opaque to the tool, which just compares it) so a connect can skip re-pulling
// an unchanged library. Same bytes in → same value on every call.
uint32_t rcCmdlibHash(const String& s) {
  uint32_t h = 2166136261u;                 // FNV offset basis
  const size_t n = s.length();
  for (size_t i = 0; i < n; i++) { h ^= (uint8_t)s[i]; h *= 16777619u; }
  return h;
}

// Streamed FNV-1a over a FILE (O(1) RAM) — byte-for-byte identical to
// rcCmdlibHash() without loading the file into a String. Used by the bulk sink to
// verify a streamed library against the sender's expected hash. `bytes` (if
// non-null) receives the total size read.
uint32_t rcCmdlibHashFile(const char* path, size_t* bytes = nullptr) {
  if (bytes) *bytes = 0;
  if (!g_lfsReady) return 0;
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  uint32_t h = 2166136261u;
  uint8_t  buf[256];
  size_t   total = 0;
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    if (n == 0) break;
    for (size_t i = 0; i < n; i++) { h ^= buf[i]; h *= 16777619u; }
    total += n;
  }
  f.close();
  if (bytes) *bytes = total;
  return h;
}

// Returns true only if EVERY NVS value was written successfully.
bool rcConfigSaveNVS() {
  bool ok = true;
  Preferences prefs;
  prefs.begin("rcfg", false);

  prefs.putUChar("tx",     rcConfig.txModel);   // transmitter model (RcTxModel)
  prefs.putBool("3xg",     rcConfig.threeAxisGimbals);   // X20 3-axis hw option
  prefs.putBool("sbusout", rcConfig.sbusOutEnabled);     // SBUS-OUT passthrough enable
  prefs.putUChar("board",  rcConfig.boardType);          // hardware pin profile (0=NaviCore v2, 1=WCB 3.2)
  prefs.putInt("cfg",      rcConfig.tapWindowMs);
  prefs.putInt("matrixCh", rcConfig.matrixChannel);
  prefs.putInt("mtxDeb",   rcConfig.matrixDebounceFrames);
  prefs.putChar("fbMode",  rcConfig.funcBindings.modeSwitch);

  // Thresholds
  {
    DynamicJsonDocument doc(4096);   // 36 thresholds (was 21) — node pool, not the NVS string
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < RC_NUM_THRESHOLDS; i++) {
      JsonObject o = arr.createNestedObject();
      o["id"]     = rcConfig.thresholds[i].id;
      o["label"]  = rcConfig.thresholds[i].label;
      o["minPwm"] = rcConfig.thresholds[i].minPwm;
      o["maxPwm"] = rcConfig.thresholds[i].maxPwm;
    }
    ok &= _nvsPutJson(prefs, "th", doc);
  }

  // Mode mappings.  Physical buttons (slots 1..RC_NUM_PHYSICAL) go in m1/m2/m3;
  // logical buttons (slots 22..36) go in their OWN keys m1L/m2L/m3L.  Splitting
  // keeps each NVS value well under the 4000-byte putString ceiling even when
  // many slots are configured (NVS caps a single string value at 4000 bytes; an
  // over-cap putString silently fails).  Mappings are usually sparse, so each
  // blob normally stays small; the split just removes the worst-case overflow.
  // Keeping the physical range in the original m1/m2/m3 keys is backward
  // compatible — existing configs load unchanged, and old firmware simply
  // ignores the new mL keys.
  struct { const char* key[3]; int lo, hi; } mapGroups[2] = {
    { { "m1",  "m2",  "m3"  }, 1,                   RC_NUM_PHYSICAL   },
    { { "m1L", "m2L", "m3L" }, RC_NUM_PHYSICAL + 1, RC_NUM_THRESHOLDS },
  };
  for (int g = 0; g < 2; g++) {
    for (int mode = 1; mode <= 3; mode++) {
      DynamicJsonDocument doc(8192);
      JsonObject root = doc.to<JsonObject>();
      for (int btn = mapGroups[g].lo; btn <= mapGroups[g].hi; btn++) {
        int idx = rcMapIndex(mode, btn);
        const RcMapping& m = rcConfig.mappings[idx];
        bool hasAny = false;
        for (int ti = 0; ti < 3 && !hasAny; ti++) if (m.t[ti].count > 0) hasAny = true;
        if (!hasAny && !m.exclusive) continue;
        String key = String(mode * 100 + btn);
        JsonObject mObj = root.createNestedObject(key);
        mObj["exclusive"] = m.exclusive;
        for (int ti = 0; ti < 3; ti++) {
          if (m.t[ti].count == 0) continue;
          String tierKey = String("t") + (ti + 1);
          JsonArray acts = mObj.createNestedArray(tierKey);
          for (int ai = 0; ai < m.t[ti].count; ai++) {
            actionToJson(m.t[ti].a[ai], acts.createNestedObject());
          }
        }
      }
      ok &= _nvsPutJson(prefs, mapGroups[g].key[mode - 1], doc);
    }
  }

  // Switches
  {
    DynamicJsonDocument doc(4096);
    JsonObject root = doc.to<JsonObject>();
    for (int i = 0; i < RC_NUM_SWITCHES; i++) {
      const RcSwitch& s = rcConfig.switches[i];
      JsonObject sObj = root.createNestedObject(RC_SWITCH_LABELS[i]);
      sObj["channel"]   = s.channel;
      sObj["positions"] = s.positions;
      for (int pi = 0; pi < 3; pi++) {
        if (s.t[pi].count == 0) continue;
        String tierKey = String("p") + pi;
        JsonArray acts = sObj.createNestedArray(tierKey);
        for (int ai = 0; ai < s.t[pi].count; ai++) {
          actionToJson(s.t[pi].a[ai], acts.createNestedObject());
        }
      }
    }
    ok &= _nvsPutJson(prefs, "sw", doc);
  }

  // Knobs — now 11 knobs (X20 added MS, J5, J6) × up to 8 outputs × ~50
  // bytes + overhead.  Worst case ~4.8 KB; use 8 KB buffer.  NVS itself
  // is still capped at 4000 bytes per value — putString will fail
  // silently if the serialized JSON exceeds that, but realistic configs
  // (most knobs disabled, few outputs each) stay well under.
  {
    DynamicJsonDocument doc(8192);
    JsonObject root = doc.to<JsonObject>();
    for (int i = 0; i < RC_NUM_KNOBS; i++) {
      const RcKnob& kn = rcConfig.knobs[i];
      JsonObject kObj = root.createNestedObject(RC_KNOB_LABELS[i]);
      kObj["channel"]  = kn.channel;
      kObj["function"] = kn.function;
      kObj["reverse"]  = kn.reverse;
      JsonArray outsArr = kObj.createNestedArray("outputs");
      for (uint8_t o = 0; o < kn.outputCount && o < RC_KNOB_MAX_OUTPUTS; o++) {
        JsonObject oObj = outsArr.createNestedObject();
        oObj["target"]    = kn.outputs[o].target;
        oObj["maestroCh"] = kn.outputs[o].maestroCh;
        oObj["posMin"]    = kn.outputs[o].posMin;
        oObj["posMax"]    = kn.outputs[o].posMax;
        // (midClosed is NOT written here: this NVS "kn" path is legacy/migration-only
        //  and its reader doesn't parse it — config.json (rcWriteKnobOuts) is the live
        //  path that round-trips midClosed. Keep the two NVS halves symmetric.)
      }
    }
    ok &= _nvsPutJson(prefs, "kn", doc);
  }

  // Global HCR destination
  {
    DynamicJsonDocument doc(256);
    JsonObject root = doc.to<JsonObject>();
    root["transport"] = (rcConfig.hcrDest.transport == 1) ? "wcb" : "serial";
    if (rcConfig.hcrDest.transport == 1) {
      root["target"]  = rcConfig.hcrDest.target;
      root["wcbPort"] = rcConfig.hcrDest.wcbPort;
    } else {
      root["port"]    = rcConfig.hcrDest.target;
    }
    ok &= _nvsPutJson(prefs, "hcr", doc);
  }

  // Maestro locations
  {
    DynamicJsonDocument doc(512);
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < RC_NUM_MAESTROS; i++) {
      JsonObject mObj = arr.createNestedObject();
      mObj["type"]   = rcConfig.maestros[i].type;
      mObj["device"] = rcConfig.maestros[i].device;
    }
    ok &= _nvsPutJson(prefs, "mae", doc);
  }

  // WCB network credentials
  {
    DynamicJsonDocument doc(256);
    JsonObject root = doc.to<JsonObject>();
    root["macOct2"]  = rcConfig.wcbNetwork.macOct2;
    root["macOct3"]  = rcConfig.wcbNetwork.macOct3;
    root["password"] = rcConfig.wcbNetwork.password;
    root["quantity"] = rcConfig.wcbNetwork.quantity;
    root["deviceId"] = rcConfig.wcbNetwork.deviceId;
    ok &= _nvsPutJson(prefs, "wcb", doc);
  }

  // MP3 Trigger destination (transport + target)
  {
    DynamicJsonDocument doc(128);
    JsonObject root = doc.to<JsonObject>();
    root["transport"] = rcConfig.mp3Dest.transport;
    root["target"]    = rcConfig.mp3Dest.target;
    ok &= _nvsPutJson(prefs, "mp3", doc);
  }

  // Serial port baud — aux S3/S4 + local Maestro bus (Serial2)
  {
    DynamicJsonDocument doc(128);
    JsonObject root = doc.to<JsonObject>();
    root["S3"]      = rcConfig.auxBaud[0];
    root["S4"]      = rcConfig.auxBaud[1];
    root["S5"]      = rcConfig.auxBaud[2];
    root["maestro"] = rcConfig.maestroBaud;
    ok &= _nvsPutJson(prefs, "aux", doc);
  }

  prefs.end();
  if (ok) {
    Serial.println("RC config saved to NVS.");
  } else {
    Serial.println("[CONFIG] WARNING: NVS save INCOMPLETE — one or more values failed (see "
                   "errors above). Config may NOT survive a reboot. Reduce mapped actions/labels.");
  }
  return ok;
}

void rcConfigLoadNVS() {
  Preferences prefs;
  prefs.begin("rcfg", true);

  if (prefs.isKey("tx"))       rcConfig.txModel          = prefs.getUChar("tx", rcConfig.txModel);
  if (prefs.isKey("3xg"))      rcConfig.threeAxisGimbals = prefs.getBool("3xg", rcConfig.threeAxisGimbals);
  if (prefs.isKey("sbusout"))  rcConfig.sbusOutEnabled   = prefs.getBool("sbusout", rcConfig.sbusOutEnabled);
  if (prefs.isKey("board"))    rcConfig.boardType        = prefs.getUChar("board", rcConfig.boardType);
  if (prefs.isKey("cfg"))      rcConfig.tapWindowMs   = prefs.getInt("cfg",      rcConfig.tapWindowMs);
  if (rcConfig.tapWindowMs < 100) rcConfig.tapWindowMs = 500;   // guard a stale/zero NVS value that would disable multi-tap
  if (prefs.isKey("matrixCh")) rcConfig.matrixChannel = prefs.getInt("matrixCh", rcConfig.matrixChannel);
  if (prefs.isKey("mtxDeb")) {
    int d = prefs.getInt("mtxDeb", rcConfig.matrixDebounceFrames);
    rcConfig.matrixDebounceFrames = (d < 1) ? 1 : (d > 4 ? 4 : d);
  }
  if (prefs.isKey("fbMode"))   rcConfig.funcBindings.modeSwitch = prefs.getChar("fbMode", rcConfig.funcBindings.modeSwitch);

  if (prefs.isKey("th")) {
    String s = prefs.getString("th", "");
    DynamicJsonDocument doc(4096);   // 36 thresholds (was 21) — node pool, not the NVS string
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonArray arr = doc.as<JsonArray>();
      int i = 0;
      for (JsonObject o : arr) {
        if (i >= RC_NUM_THRESHOLDS) break;
        rcConfig.thresholds[i].id = o["id"] | (i + 1);
        strlcpy(rcConfig.thresholds[i].label, o["label"] | "", 24);
        rcConfig.thresholds[i].minPwm = o["minPwm"] | rcConfig.thresholds[i].minPwm;
        rcConfig.thresholds[i].maxPwm = o["maxPwm"] | rcConfig.thresholds[i].maxPwm;
        i++;
      }
    }
  }

  // Mode mappings: physical (m1/m2/m3) + logical (m1L/m2L/m3L).  Both groups
  // parse identically — the buttonId in each key (mode*100+btn) selects the
  // slot, and the b_ <= RC_NUM_THRESHOLDS gate accepts logical slots 22-36.
  // Old configs without the mL keys just leave the logical slots empty.
  const char* mapKeys[6] = { "m1", "m2", "m3", "m1L", "m2L", "m3L" };
  for (int k = 0; k < 6; k++) {
    if (!prefs.isKey(mapKeys[k])) continue;
    String s = prefs.getString(mapKeys[k], "");
    DynamicJsonDocument doc(8192);
    if (deserializeJson(doc, s) != DeserializationError::Ok) continue;
    JsonObject root = doc.as<JsonObject>();
    for (JsonPair kv : root) {
      int buttonId = String(kv.key().c_str()).toInt();
      int m_ = buttonId / 100, b_ = buttonId % 100;
      if (m_ < 1 || m_ > 3 || b_ < 1 || b_ > RC_NUM_THRESHOLDS) continue;
      int idx = rcMapIndex(m_, b_);
      RcMapping& m = rcConfig.mappings[idx];
      memset(&m, 0, sizeof(m));
      JsonObject mObj = kv.value().as<JsonObject>();
      m.exclusive = mObj["exclusive"] | false;
      for (int ti = 0; ti < 3; ti++) {
        String tierKey = String("t") + (ti + 1);
        if (!mObj.containsKey(tierKey)) continue;
        JsonArray acts = mObj[tierKey];
        int ai = 0;
        for (JsonObject aObj : acts) {
          if (ai >= RC_ACTIONS_PER_TIER) break;
          if (actionFromJson(aObj, m.t[ti].a[ai])) m.t[ti].count = ++ai;
        }
      }
    }
  }

  if (prefs.isKey("sw")) {
    String s = prefs.getString("sw", "");
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      for (int i = 0; i < RC_NUM_SWITCHES; i++) {
        if (!root.containsKey(RC_SWITCH_LABELS[i])) continue;
        JsonObject sObj = root[RC_SWITCH_LABELS[i]];
        RcSwitch& sw = rcConfig.switches[i];
        memset(sw.t, 0, sizeof(sw.t));
        sw.channel   = sObj["channel"]   | RC_SWITCH_DEFAULT_CH[i];
        sw.positions = sObj["positions"] | RC_SWITCH_DEFAULT_POS[i];
        for (int pi = 0; pi < 3; pi++) {
          String tierKey = String("p") + pi;
          if (!sObj.containsKey(tierKey)) continue;
          JsonArray acts = sObj[tierKey];
          int ai = 0;
          for (JsonObject aObj : acts) {
            if (ai >= RC_ACTIONS_PER_TIER) break;
            if (actionFromJson(aObj, sw.t[pi].a[ai])) sw.t[pi].count = ++ai;
          }
        }
      }
    }
  }

  if (prefs.isKey("kn")) {
    String s = prefs.getString("kn", "");
    // Buffer sized to match the writer — 8 KB to hold worst-case 11-knob
    // payload (MS / J5 / J6 added for X20).
    DynamicJsonDocument doc(8192);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      for (int i = 0; i < RC_NUM_KNOBS; i++) {
        if (!root.containsKey(RC_KNOB_LABELS[i])) continue;
        JsonObject kObj = root[RC_KNOB_LABELS[i]];
        RcKnob& kn = rcConfig.knobs[i];
        kn.channel  = kObj["channel"]  | RC_KNOB_DEFAULT_CH[i];
        kn.function = kObj["function"] | RC_KNOB_DEFAULT_FN[i];
        kn.reverse  = kObj["reverse"]  | false;
        kn.outputCount = 0;
        memset(kn.outputs, 0, sizeof(kn.outputs));
        if (kObj.containsKey("outputs")) {
          JsonArray outsArr = kObj["outputs"];
          for (JsonObject oObj : outsArr) {
            if (kn.outputCount >= RC_KNOB_MAX_OUTPUTS) break;
            RcKnobOutput& out = kn.outputs[kn.outputCount];
            if (oObj.containsKey("target")) {
              out.target = (uint8_t)(oObj["target"] | 0);
            } else {
              uint8_t legacy = (uint8_t)(oObj["slot"] | 0);
              out.target = (legacy == 0) ? 1 : legacy;
            }
            out.maestroCh = (uint8_t) (oObj["maestroCh"] | 0);
            out.posMin    = (uint16_t)(oObj["posMin"]    | 4000);
            out.posMax    = (uint16_t)(oObj["posMax"]    | 8000);
            kn.outputCount++;
          }
        }
      }
    }
  }

  if (prefs.isKey("mae")) {
    String s = prefs.getString("mae", "");
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonArray arr = doc.as<JsonArray>();
      int i = 0;
      for (JsonObject mObj : arr) {
        if (i >= RC_NUM_MAESTROS) break;
        rcConfig.maestros[i].type   = (uint8_t)(mObj["type"]   | 0);
        // Same presence-check as the SET_CONFIG path — preserve a stored device #0
        // instead of letting '|' rewrite a falsy 0 to the slot default.
        rcConfig.maestros[i].device = mObj.containsKey("device") ? (uint8_t)mObj["device"].as<int>() : (uint8_t)(1 + i);
        i++;
      }
    }
  }

  if (prefs.isKey("hcr")) {
    String s = prefs.getString("hcr", "");
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      const char* tp = root["transport"] | "serial";
      rcConfig.hcrDest.transport = (strcmp(tp, "wcb") == 0) ? 1 : 0;
      if (rcConfig.hcrDest.transport == 1) {
        // HCR over WCB is unicast-only; default to WCB 2 (not 0/broadcast).
        strlcpy(rcConfig.hcrDest.target, root["target"] | "2", sizeof(rcConfig.hcrDest.target));
        rcConfig.hcrDest.wcbPort = (uint8_t)(root["wcbPort"] | 1);
      } else {
        strlcpy(rcConfig.hcrDest.target, root["port"]   | "S3", sizeof(rcConfig.hcrDest.target));
        rcConfig.hcrDest.wcbPort = 0;
      }
    }
  }

  if (prefs.isKey("wcb")) {
    String s = prefs.getString("wcb", "");
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      // Presence-check, not '|' — 0x00 is a legitimate MAC octet (see rcConfigFromJSON).
      if (root.containsKey("macOct2")) rcConfig.wcbNetwork.macOct2 = (uint8_t)(root["macOct2"].as<int>() & 0xFF);
      if (root.containsKey("macOct3")) rcConfig.wcbNetwork.macOct3 = (uint8_t)(root["macOct3"].as<int>() & 0xFF);
      strlcpy(rcConfig.wcbNetwork.password,
              root["password"] | rcConfig.wcbNetwork.password,
              sizeof(rcConfig.wcbNetwork.password));
      rcConfig.wcbNetwork.quantity = (uint8_t)(root["quantity"] | rcConfig.wcbNetwork.quantity);
      rcConfig.wcbNetwork.deviceId = (uint8_t)(root["deviceId"] | rcConfig.wcbNetwork.deviceId);
    }
  }

  if (prefs.isKey("mp3")) {
    String s = prefs.getString("mp3", "");
    DynamicJsonDocument doc(128);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      rcConfig.mp3Dest.transport = (uint8_t)(root["transport"] | rcConfig.mp3Dest.transport);
      strlcpy(rcConfig.mp3Dest.target,
              root["target"] | rcConfig.mp3Dest.target,
              sizeof(rcConfig.mp3Dest.target));
    }
  }

  if (prefs.isKey("aux")) {
    String s = prefs.getString("aux", "");
    DynamicJsonDocument doc(128);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      rcConfig.auxBaud[0]  = (uint32_t)(root["S3"]      | rcConfig.auxBaud[0]);
      rcConfig.auxBaud[1]  = (uint32_t)(root["S4"]      | rcConfig.auxBaud[1]);
      rcConfig.auxBaud[2]  = (uint32_t)(root["S5"]      | rcConfig.auxBaud[2]);
      rcConfig.maestroBaud = (uint32_t)(root["maestro"] | rcConfig.maestroBaud);
    }
  }

  prefs.end();
  Serial.println("RC config loaded from NVS.");
}
