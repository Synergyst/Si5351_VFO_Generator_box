
/*
===============================================
== LED mapping for use with counterpart prog ==
===============================================
       (*)      (*)      (*)      (*)
        ^        ^        ^        ^
        |        |        |        |
        |        |        |        |
        ^        |        |        |
    Overheat     |        ^        |
                 |  SSH logged-in  |
                 |                 |
                 |                 |
                 ^                 |
    All USB devs online/working    |
                                   |
                                   |
                                   ^
                            CPU usage meter
===============================================

Overheat:-
 - unlit: temps all normal
 - lit: 1 or more temps at warn-temp-limit
 - flashing: 1 or more temps above critial-temp-limit

All USB devices online/working:-
 - unlit: 1 or more USB devices have a problem
 - lit: All critical USB devices have firmware/EEPROM flashed and/or programmed correctly and are online

SSH logged-in:-
 - unlit: No user(s) logged into SSH server
 - lit: 1 or more user(s) are logged into SSH server

CPU usage meter:-
 - unlit: CPU is entirely idle
 - lit (brightness stages ranging from 1 to 8): CPU is utilized; the brighter, the more utilization
*/

use anyhow::{anyhow, Context, Result};
use gpio_cdev::{Chip, LineRequestFlags};
use std::{
    collections::HashMap,
    env,
    io::{BufRead, BufReader, Write},
    net::TcpListener,
    process::Command,
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};

const LED_NAMES_DEFAULT: &[&str] = &["PIN_11", "PIN_29", "PIN_31", "PIN_33"];

// Scanner timing: one LED slot every SLOT_US microseconds.
// With 4 LEDs and SLOT_US = 250, each LED updates at ~1 kHz (flicker-free).
const SLOT_US: u64 = 250;

// Brightness levels per LED (0..=LEVELS). Keep modest so per-LED PWM stays > ~70 Hz.
// Effective PWM per LED ~= (per-LED slot rate) / LEVELS.
const LEVELS: u8 = 8; // 0..8 inclusive => 9 steps; 1kHz/8 ~= ~125 Hz PWM per LED.

// Demo defaults
const DEFAULT_IDLE_DELAY_MS: u64 = 150;      // step delay for KITT
const DEFAULT_KITT_SECONDS: u64 = 8;         // when in "cycle" mode
const DEFAULT_BREATH_SECONDS: u64 = 8;       // when in "cycle" mode
const DEFAULT_BREATH_STEP_MS: u64 = 20;      // granularity of breathing updates

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum IdleMode {
    None,
    Kitt,
    Cycle,
}

#[derive(Clone)]
struct Opts {
    host: String,
    port: u16,
    led_names: Vec<String>,
    idle_mode: IdleMode,
    idle_delay_ms: u64,
    kitt_secs: u64,
    breath_secs: u64,
    breath_step_ms: u64,
}

impl Default for Opts {
    fn default() -> Self {
        Self {
            host: "127.0.0.1".into(),
            port: 5555,
            led_names: LED_NAMES_DEFAULT.iter().map(|s| s.to_string()).collect(),
            idle_mode: IdleMode::None,
            idle_delay_ms: DEFAULT_IDLE_DELAY_MS,
            kitt_secs: DEFAULT_KITT_SECONDS,
            breath_secs: DEFAULT_BREATH_SECONDS,
            breath_step_ms: DEFAULT_BREATH_STEP_MS,
        }
    }
}

fn parse_args() -> Result<Opts> {
    let mut opts = Opts::default();
    let mut args = env::args().skip(1).peekable();
    while let Some(a) = args.next() {
        match a.as_str() {
            "--host" => {
                opts.host = args.next().ok_or_else(|| anyhow!("--host requires value"))?;
            }
            "--port" => {
                opts.port = args.next().ok_or_else(|| anyhow!("--port requires value"))?.parse()?;
            }
            "--names" | "--leds" => {
                let v = args.next().ok_or_else(|| anyhow!("--names requires value"))?;
                opts.led_names = v.split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty()).collect();
            }
            "--idle-demo" => {
                let v = args.next().ok_or_else(|| anyhow!("--idle-demo requires value"))?;
                opts.idle_mode = match v.to_lowercase().as_str() {
                    "none" => IdleMode::None,
                    "kitt" | "chase" | "knightrider" => IdleMode::Kitt,
                    "cycle" => IdleMode::Cycle,
                    _ => return Err(anyhow!("--idle-demo must be one of: none|kitt|cycle")),
                };
            }
            "--idle-delay-ms" => {
                opts.idle_delay_ms = args.next().ok_or_else(|| anyhow!("--idle-delay-ms requires value"))?.parse()?;
            }
            "--kitt-seconds" => {
                opts.kitt_secs = args.next().ok_or_else(|| anyhow!("--kitt-seconds requires value"))?.parse()?;
            }
            "--breath-seconds" => {
                opts.breath_secs = args.next().ok_or_else(|| anyhow!("--breath-seconds requires value"))?.parse()?;
            }
            "--breath-step-ms" => {
                opts.breath_step_ms = args.next().ok_or_else(|| anyhow!("--breath-step-ms requires value"))?.parse()?;
            }
            "-h" | "--help" => {
                print_help();
                std::process::exit(0);
            }
            _ => return Err(anyhow!("unknown arg '{}'. Use --help for options.", a)),
        }
    }
    if opts.led_names.is_empty() {
        return Err(anyhow!("At least one LED name is required"));
    }
    Ok(opts)
}

fn print_help() {
    eprintln!(
        "Usage: led_mplex [options]
Options:
  --host <addr>              Bind address (default 127.0.0.1)
  --port <port>              TCP port (default 5555)
  --names <comma-list>       GPIO line names (default: PIN_11,PIN_29,PIN_31,PIN_33)

Idle demo control (runs only when no clients connected):
  --idle-demo <mode>         none | kitt | cycle   (default: none)
  --idle-delay-ms <ms>       Step delay for KITT (default: {idl})
  --kitt-seconds <sec>       KITT segment length (cycle mode) (default: {kitt})
  --breath-seconds <sec>     Breathing segment length (cycle mode) (default: {breath})
  --breath-step-ms <ms>      Breathing update granularity (default: {bms})

API over TCP (one command per line):
  on <NAME>
  off <NAME>
  bright <NAME> <0..{lev}>
  status

Examples:
  led_mplex --idle-demo kitt --idle-delay-ms 120
  led_mplex --idle-demo cycle --kitt-seconds 10 --breath-seconds 8 --idle-delay-ms 100 --breath-step-ms 20
",
        idl = DEFAULT_IDLE_DELAY_MS,
        kitt = DEFAULT_KITT_SECONDS,
        breath = DEFAULT_BREATH_SECONDS,
        bms = DEFAULT_BREATH_STEP_MS,
        lev = LEVELS
    );
}

struct Led {
    name: String,
    handle: gpio_cdev::LineHandle,
}

fn resolve_line(name: &str) -> Result<(String, u32)> {
    let out = Command::new("gpiofind")
        .arg(name)
        .output()
        .with_context(|| "failed to exec gpiofind (is libgpiod-utils installed?)")?;
    if !out.status.success() {
        return Err(anyhow!("gpiofind could not locate line '{}'", name));
    }
    let s = String::from_utf8(out.stdout)?.trim().to_string();
    let mut parts = s.split_whitespace();
    let chip_part = parts.next().ok_or_else(|| anyhow!("bad gpiofind output"))?;
    let offset_str = parts.next().ok_or_else(|| anyhow!("bad gpiofind output"))?;
    let chip_path = if chip_part.starts_with("/dev/") { chip_part.to_string() } else { format!("/dev/{}", chip_part) };
    let offset: u32 = offset_str.parse()?;
    Ok((chip_path, offset))
}

fn request_output(chip_path: &str, offset: u32) -> Result<gpio_cdev::LineHandle> {
    let mut chip = Chip::new(chip_path).with_context(|| format!("open {}", chip_path))?;
    let line = chip.get_line(offset)?;
    let handle = line.request(LineRequestFlags::OUTPUT, 0, "led-mplex")?;
    Ok(handle)
}

fn main() -> Result<()> {
    let opts = parse_args().unwrap_or_else(|e| {
        eprintln!("Error: {e}");
        print_help();
        std::process::exit(2);
    });

    // Resolve lines and request outputs
    let mut leds: Vec<Led> = Vec::new();
    for name in &opts.led_names {
        let (chip_path, offset) = resolve_line(name).with_context(|| format!("resolving {}", name))?;
        let handle = request_output(&chip_path, offset).with_context(|| format!("requesting {} {}:{}", name, chip_path, offset))?;
        handle.set_value(0)?;
        leds.push(Led { name: name.clone(), handle });
    }

    // Shared brightness map: name -> level (0..=LEVELS)
    let levels: Arc<Mutex<HashMap<String, u8>>> = Arc::new(Mutex::new(HashMap::new()));
    for led in &leds {
        levels.lock().unwrap().insert(led.name.clone(), 0); // start OFF
    }

    // Track active clients; demo runs only when this is zero.
    let active_clients = Arc::new(AtomicUsize::new(0));

    // running flag and clean exit
    let running = Arc::new(AtomicBool::new(true));
    {
        let running = running.clone();
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
        })?;
    }

    // Scanner thread: owns the GPIO handles; displays the current levels map by TDM.
    let levels_for_scanner = levels.clone();
    let running_scanner = running.clone();
    let scanner_leds = leds; // move ownership here
    let scanner = thread::spawn(move || {
        let slot_sleep = Duration::from_micros(SLOT_US);
        // PWM phase counter per LED name
        let mut pwm_phase: HashMap<String, u8> = scanner_leds.iter().map(|l| (l.name.clone(), 0u8)).collect();
        let mut prev_on: Option<usize> = None;

        while running_scanner.load(Ordering::SeqCst) {
            for (idx, led) in scanner_leds.iter().enumerate() {
                // Turn previous off first to keep at most one LED on
                if let Some(prev) = prev_on.take() {
                    let _ = scanner_leds[prev].handle.set_value(0);
                }

                // Read desired level
                let level = {
                    let map = levels_for_scanner.lock().unwrap();
                    *map.get(&led.name).unwrap_or(&0u8)
                };

                // Decide ON/OFF for this slot
                let on = if level == 0 {
                    false
                } else if level >= LEVELS {
                    true
                } else {
                    let ph = pwm_phase.get_mut(&led.name).unwrap();
                    let result = *ph < level;
                    *ph = (*ph + 1) % LEVELS.max(1);
                    result
                };

                if on {
                    let _ = led.handle.set_value(1);
                    prev_on = Some(idx);
                }

                spin_or_sleep(slot_sleep);

                if !running_scanner.load(Ordering::SeqCst) {
                    break;
                }
            }
        }

        // Ensure all off on exit
        for led in &scanner_leds {
            let _ = led.handle.set_value(0);
        }
    });

    // Demo thread (idle-only)
    let levels_for_demo = levels.clone();
    let active_for_demo = active_clients.clone();
    let running_demo = running.clone();
    let led_names_for_demo: Vec<String> = opts.led_names.clone();
    let demo_opts = opts.clone();
    let demo = thread::spawn(move || {
        loop {
            if !running_demo.load(Ordering::SeqCst) {
                break;
            }
            if demo_opts.idle_mode == IdleMode::None {
                thread::sleep(Duration::from_millis(250));
                continue;
            }
            if active_for_demo.load(Ordering::SeqCst) > 0 {
                thread::sleep(Duration::from_millis(100));
                continue;
            }

            match demo_opts.idle_mode {
                IdleMode::Kitt => {
                    run_kitt_indef(
                        &levels_for_demo,
                        &led_names_for_demo,
                        demo_opts.idle_delay_ms,
                        &running_demo,
                        &active_for_demo,
                    );
                }
                IdleMode::Cycle => {
                    run_kitt_for_secs(
                        &levels_for_demo,
                        &led_names_for_demo,
                        demo_opts.idle_delay_ms,
                        demo_opts.kitt_secs,
                        &running_demo,
                        &active_for_demo,
                    );
                    if !running_demo.load(Ordering::SeqCst) {
                        break;
                    }
                    run_breathing_demo(
                        &levels_for_demo,
                        &led_names_for_demo,
                        demo_opts.breath_secs,
                        demo_opts.breath_step_ms,
                        &running_demo,
                        &active_for_demo,
                    );
                }
                IdleMode::None => {}
            }
        }
        set_all(&levels_for_demo, &led_names_for_demo, 0);
    });

    // TCP server; increments/decrements active client count; first connect clears LEDs.
    let server_levels = levels.clone();
    let server_running = running.clone();
    let server_active = active_clients.clone();
    let host = opts.host.clone();
    let port = opts.port;
    let server_led_names = opts.led_names.clone();
    let server = thread::spawn(move || -> Result<()> {
        let listener = TcpListener::bind((host.as_str(), port))?;
        listener.set_nonblocking(true)?;
        println!("LED mplex server listening on {}:{}", host, port);

        loop {
            if !server_running.load(Ordering::SeqCst) {
                break;
            }
            match listener.accept() {
                Ok((stream, addr)) => {
                    let was = server_active.fetch_add(1, Ordering::SeqCst);
                    println!("client connected: {}", addr);
                    if was == 0 {
                        // First client: stop demo and clear LEDs
                        set_all(&server_levels, &server_led_names, 0);
                    }
                    let levels = server_levels.clone();
                    let active = server_active.clone();
                    let running = server_running.clone();
                    thread::spawn(move || {
                        let mut stream = stream;
                        let _ = writeln!(
                            stream,
                            "ok ready. cmds: on <NAME> | off <NAME> | bright <NAME> <0..{}> | status",
                            LEVELS
                        );
                        let mut rdr = BufReader::new(stream.try_clone().unwrap());
                        let mut line = String::new();
                        loop {
                            line.clear();
                            let n = match rdr.read_line(&mut line) {
                                Ok(n) => n,
                                Err(_) => break,
                            };
                            if n == 0 || !running.load(Ordering::SeqCst) {
                                break;
                            }
                            let parts: Vec<_> = line.trim().split_whitespace().collect();
                            if parts.is_empty() {
                                continue;
                            }
                            let resp = match parts[0].to_lowercase().as_str() {
                                "on" if parts.len() == 2 => set_level(&levels, parts[1], LEVELS),
                                "off" if parts.len() == 2 => set_level(&levels, parts[1], 0),
                                "bright" if parts.len() == 3 => match parts[2].parse::<u8>() {
                                    Ok(v) if v <= LEVELS => set_level(&levels, parts[1], v),
                                    _ => Err(anyhow!("bad level (0..{})", LEVELS)),
                                },
                                "status" => {
                                    let map = levels.lock().unwrap();
                                    let mut s = String::new();
                                    for (k, v) in map.iter() {
                                        s.push_str(&format!("{} {}\n", k, v));
                                    }
                                    Ok(s)
                                }
                                _ => Err(anyhow!("bad cmd")),
                            };
                            let _ = match resp {
                                Ok(s) => writeln!(stream, "ok {}", s.trim_end()),
                                Err(e) => writeln!(stream, "err {}", e),
                            };
                        }
                        active.fetch_sub(1, Ordering::SeqCst);
                        let _ = writeln!(stream, "ok bye");
                    });
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    thread::sleep(Duration::from_millis(50));
                    continue;
                }
                Err(e) => {
                    eprintln!("accept error: {}", e);
                    thread::sleep(Duration::from_millis(200));
                }
            }
        }
        Ok(())
    });

    // Wait until Ctrl-C
    let _ = server.join();
    let _ = demo.join();
    let _ = scanner.join();

    Ok(())
}

fn set_level(levels: &Arc<Mutex<HashMap<String, u8>>>, name: &str, v: u8) -> Result<String> {
    let mut map = levels.lock().unwrap();
    if !map.contains_key(name) {
        return Err(anyhow!("unknown LED '{}'", name));
    }
    map.insert(name.to_string(), v);
    Ok(format!("{} {}", name, v))
}

fn set_all(levels: &Arc<Mutex<HashMap<String, u8>>>, names: &[String], v: u8) {
    let mut map = levels.lock().unwrap();
    for n in names {
        map.insert(n.clone(), v);
    }
}

// Knight Rider routines -------------------------------------------------------

struct KittState {
    idx: usize,
    dir: i32, // +1 forward, -1 backward
    len: usize,
}
impl KittState {
    fn new(len: usize) -> Self {
        Self { idx: 0, dir: 1, len }
    }
    fn step(&mut self) {
        if self.len <= 1 {
            self.idx = 0;
            return;
        }
        let next = self.idx as i32 + self.dir;
        if next < 0 {
            self.dir = 1;
            self.idx = 1;
        } else if next >= self.len as i32 {
            self.dir = -1;
            self.idx = self.len.saturating_sub(2);
        } else {
            self.idx = next as usize;
        }
    }
}

fn run_kitt_step(
    levels: &Arc<Mutex<HashMap<String, u8>>>,
    names: &[String],
    delay_ms: u64,
    state: &KittState,
) {
    if names.is_empty() {
        thread::sleep(Duration::from_millis(delay_ms));
        return;
    }

    // Single bright head, faint one-LED tails
    let head = state.idx;
    let left_tail = head.checked_sub(1);
    let right_tail = if head + 1 < names.len() { Some(head + 1) } else { None };

    {
        let mut map = levels.lock().unwrap();
        for (i, n) in names.iter().enumerate() {
            let val = if i == head {
                LEVELS
            } else if Some(i) == left_tail || Some(i) == right_tail {
                (LEVELS as u16 / 3) as u8
            } else {
                0
            };
            map.insert(n.clone(), val);
        }
    }

    thread::sleep(Duration::from_millis(delay_ms));
}

fn run_kitt_indef(
    levels: &Arc<Mutex<HashMap<String, u8>>>,
    names: &[String],
    delay_ms: u64,
    running: &Arc<AtomicBool>,
    active_clients: &Arc<AtomicUsize>,
) {
    let mut state = KittState::new(names.len());
    while running.load(Ordering::SeqCst) && active_clients.load(Ordering::SeqCst) == 0 {
        run_kitt_step(levels, names, delay_ms, &state);
        // advance after the delay
        let mut s = KittState { ..state };
        s.step();
        state = s;
    }
    set_all(levels, names, 0);
}

fn run_kitt_for_secs(
    levels: &Arc<Mutex<HashMap<String, u8>>>,
    names: &[String],
    delay_ms: u64,
    secs: u64,
    running: &Arc<AtomicBool>,
    active_clients: &Arc<AtomicUsize>,
) {
    let end = Instant::now() + Duration::from_secs(secs);
    let mut state = KittState::new(names.len());
    while Instant::now() < end && running.load(Ordering::SeqCst) && active_clients.load(Ordering::SeqCst) == 0 {
        run_kitt_step(levels, names, delay_ms, &state);
        let mut s = KittState { ..state };
        s.step();
        state = s;
    }
    set_all(levels, names, 0);
}

// Breathing (Arduino-style “breath” all LEDs together) ------------------------

fn run_breathing_demo(
    levels: &Arc<Mutex<HashMap<String, u8>>>,
    names: &[String],
    breath_secs: u64,
    step_ms: u64,
    running: &Arc<AtomicBool>,
    active_clients: &Arc<AtomicUsize>,
) {
    if names.is_empty() {
        thread::sleep(Duration::from_millis(step_ms));
        return;
    }
    let start = Instant::now();
    let step = Duration::from_millis(step_ms.max(5));
    let cycle_s: f32 = 2.0; // 2 seconds per full inhale+exhale
    let two_pi = std::f32::consts::PI * 2.0;

    while running.load(Ordering::SeqCst)
        && active_clients.load(Ordering::SeqCst) == 0
        && start.elapsed() < Duration::from_secs(breath_secs)
    {
        let t = start.elapsed().as_secs_f32();
        let phase = (t / cycle_s).fract(); // 0..1
        let norm = 0.5 * (1.0 - (two_pi * phase).cos()); // cosine ease: 0..1..0
        let mut level = (norm * LEVELS as f32).round() as i32;
        if level < 0 { level = 0; }
        if level > LEVELS as i32 { level = LEVELS as i32; }

        {
            let mut map = levels.lock().unwrap();
            for n in names {
                map.insert(n.clone(), level as u8);
            }
        }
        thread::sleep(step);
    }
    set_all(levels, names, 0);
}

// Short busy-wait to reduce jitter at small SLOT_US; falls back to sleep for larger values.
fn spin_or_sleep(dur: Duration) {
    if dur <= Duration::from_micros(200) {
        let start = Instant::now();
        while start.elapsed() < dur {
            std::hint::spin_loop();
        }
    } else {
        thread::sleep(dur);
    }
}