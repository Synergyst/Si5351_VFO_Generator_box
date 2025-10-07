
use anyhow::{anyhow, Result};
use clap::Parser;
use regex::Regex;
use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::Path;
use std::time::{Duration, Instant};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpStream;
use tokio::process::Command;
use tokio::time::{interval_at, timeout, Interval};

#[derive(Parser, Debug, Clone)]
#[command(name = "led_os_agent", about = "OS status to LED server agent")]
struct Opts {
    #[arg(long, default_value = "127.0.0.1")]
    host: String,
    #[arg(long, default_value_t = 5555)]
    port: u16,
    #[arg(long, default_value_t = 500, help = "Sampling interval in ms; also blink cadence")]
    interval: u64,
    #[arg(long = "max-level", default_value_t = 8, help = "Must match server LEVELS")]
    max_level: u8,

    // LEDs (optional; skip any you don't use)
    #[arg(long = "cpu-led")]
    cpu_led: Option<String>,
    #[arg(long = "ssh-led")]
    ssh_led: Option<String>,
    #[arg(long = "temp-led")]
    temp_led: Option<String>,
    #[arg(long = "usb-led")]
    usb_led: Option<String>,

    // SSH settings
    #[arg(long = "ssh-scale", default_value_t = false)]
    ssh_scale: bool,

    // Temperature settings
    #[arg(long = "temp-warn", default_value_t = 80.0)]
    temp_warn: f64,
    #[arg(long = "temp-crit", default_value_t = 95.0)]
    temp_crit: f64,
    #[arg(long = "blink-ms", default_value_t = 500)]
    blink_ms: u64,

    // USB VID:PID list (comma separated)
    // Let clap split on commas and validate each piece individually.
    #[arg(
        long = "usb-ids",
        value_delimiter = ',',
        value_parser = valid_vidpid,
        default_value = "1df7:2500,1a86:7523,2e8a:0003"
    )]
    usb_ids: Vec<String>,
}

// Validate and normalize a single VID:PID token.
fn valid_vidpid(s: &str) -> Result<String, String> {
    match normalize_vidpid(s) {
        Some(v) => Ok(v),
        None => Err("expected VID:PID hex like 1df7:2500".into()),
    }
}

struct ApiClient {
    host: String,
    port: u16,
    stream: Option<TcpStream>,
}

impl ApiClient {
    fn new(host: String, port: u16) -> Self {
        Self { host, port, stream: None }
    }

    async fn connect(&mut self) -> Result<()> {
        if self.stream.is_some() {
            return Ok(());
        }
        let s = TcpStream::connect((self.host.as_str(), self.port)).await?;
        // Optional small delay to let banner arrive (ignored later)
        tokio::time::sleep(Duration::from_millis(200)).await;
        self.stream = Some(s);
        Ok(())
    }

    async fn close(&mut self) {
        if let Some(mut s) = self.stream.take() {
            let _ = s.shutdown().await;
        }
        self.stream = None;
    }

    // Sends a command and waits for an "ok" line.
    // Ignores any non-"ok"/"err" banner lines.
    async fn send(&mut self, cmd: &str) -> Result<()> {
        let stream = match self.stream.as_mut() {
            Some(s) => s,
            None => return Err(anyhow!("not connected")),
        };

        // Split reader/writer by cloning stream (TcpStream is bidirectional and can be split)
        let (reader, writer) = stream.split();
        let mut reader = BufReader::new(reader);
        let mut writer = tokio::io::BufWriter::new(writer);

        writer.write_all(cmd.as_bytes()).await?;
        writer.write_all(b"\n").await?;
        writer.flush().await?;

        // Wait for a response; timeout to avoid hanging forever
        let mut line = String::new();
        let read_with_timeout = async {
            loop {
                line.clear();
                let n = reader.read_line(&mut line).await?;
                if n == 0 {
                    // EOF
                    return Err(anyhow!("connection closed"));
                }
                let trimmed = line.trim_end();
                if trimmed.is_empty() {
                    continue;
                }
                let low = trimmed.to_ascii_lowercase();
                if low.starts_with("ok") {
                    return Ok(());
                } else if low.starts_with("err") || low.starts_with("error") {
                    return Err(anyhow!("server error: {trimmed}"));
                } else {
                    // Banner or informational; ignore and keep reading
                    continue;
                }
            }
        };

        match timeout(Duration::from_secs(3), read_with_timeout).await {
            Ok(res) => res,
            Err(_) => Err(anyhow!("timeout waiting for server response")),
        }
    }
}

#[derive(Clone, Copy)]
struct CpuStat {
    idle: u64,
    total: u64,
}

fn read_cpu_stat() -> Option<CpuStat> {
    let s = fs::read_to_string("/proc/stat").ok()?;
    let line = s.lines().find(|l| l.starts_with("cpu "))?;
    // cpu  user nice system idle iowait irq softirq steal guest guest_nice
    let mut parts = line.split_whitespace().skip(1);
    let user = parts.next()?.parse::<u64>().ok()?;
    let nice = parts.next()?.parse::<u64>().ok()?;
    let system = parts.next()?.parse::<u64>().ok()?;
    let idle = parts.next()?.parse::<u64>().ok()?;
    let iowait = parts.next().and_then(|x| x.parse::<u64>().ok()).unwrap_or(0);
    let irq = parts.next().and_then(|x| x.parse::<u64>().ok()).unwrap_or(0);
    let softirq = parts.next().and_then(|x| x.parse::<u64>().ok()).unwrap_or(0);
    let steal = parts.next().and_then(|x| x.parse::<u64>().ok()).unwrap_or(0);
    let guest = parts.next().and_then(|x| x.parse::<u64>().ok()).unwrap_or(0);
    let guest_nice = parts.next().and_then(|x| x.parse::<u64>().ok()).unwrap_or(0);

    let idle_sum = idle + iowait;
    let total = user + nice + system + idle + iowait + irq + softirq + steal + guest + guest_nice;
    Some(CpuStat { idle: idle_sum, total })
}

struct CpuUsageSampler {
    prev: Option<CpuStat>,
}
impl CpuUsageSampler {
    fn new() -> Self {
        Self { prev: read_cpu_stat() }
    }

    fn sample(&mut self) -> f64 {
        let now = match read_cpu_stat() {
            Some(s) => s,
            None => return 0.0,
        };
        if let Some(prev) = self.prev {
            let idle_delta = now.idle.saturating_sub(prev.idle);
            let total_delta = now.total.saturating_sub(prev.total);
            self.prev = Some(now);
            if total_delta == 0 {
                return 0.0;
            }
            let usage = 1.0 - (idle_delta as f64) / (total_delta as f64);
            usage.clamp(0.0, 1.0)
        } else {
            self.prev = Some(now);
            0.0
        }
    }
}

// Shell helper with timeout
async fn sh(args: &[&str], timeout_ms: u64) -> Result<(bool, String, String)> {
    if args.is_empty() {
        return Ok((false, String::new(), String::new()));
    }
    let mut cmd = Command::new(args[0]);
    if args.len() > 1 {
        cmd.args(&args[1..]);
    }
    let fut = async {
        let out = cmd.output().await?;
        let ok = out.status.success();
        let stdout = String::from_utf8_lossy(&out.stdout).to_string();
        let stderr = String::from_utf8_lossy(&out.stderr).to_string();
        Ok::<_, anyhow::Error>((ok, stdout, stderr))
    };
    match timeout(Duration::from_millis(timeout_ms), fut).await {
        Ok(res) => res,
        Err(_) => Ok((false, String::new(), "timeout".into())),
    }
}

// SSH session counts
async fn count_ssh_sessions_loginctl() -> Option<usize> {
    let (ok, stdout, _) = sh(&["loginctl", "--no-legend", "list-sessions"], 2500).await.ok()?;
    if !ok {
        return None;
    }
    let ids: Vec<String> = stdout
        .lines()
        .filter_map(|l| l.split_whitespace().next().map(|s| s.to_string()))
        .collect();

    let mut count = 0usize;
    for id in ids {
        let (ok, stdout, _) = sh(
            &["loginctl", "show-session", &id, "-p", "Service", "-p", "Remote", "-p", "State"],
            2000,
        )
        .await
        .ok()?;
        if !ok {
            continue;
        }
        let mut svc = String::new();
        let mut remote = String::new();
        let mut state = String::new();
        for line in stdout.lines() {
            if let Some((k, v)) = line.split_once('=') {
                let key = k.trim();
                let val = v.trim().to_lowercase();
                match key {
                    "Service" => svc = val,
                    "Remote" => remote = val,
                    "State" => state = val,
                    _ => {}
                }
            }
        }
        if svc == "sshd" && state == "active" && (remote == "yes" || remote.is_empty()) {
            count += 1;
        }
    }
    Some(count)
}

async fn count_ssh_sessions_who() -> Option<usize> {
    let (ok, stdout, _) = sh(&["who"], 2000).await.ok()?;
    if !ok {
        return None;
    }
    let count = stdout
        .lines()
        .map(|l| l.trim())
        .filter(|l| !l.is_empty())
        .filter(|l| l.ends_with(')')) // crude heuristic: has "(remote)"
        .count();
    Some(count)
}

async fn count_ssh_sessions_ss() -> Option<usize> {
    let (ok, stdout, _) =
        sh(&["ss", "-Htan", "state", "established", "( sport = :22 or dport = :22 )"], 2500)
            .await
            .ok()?;
    if !ok {
        return None;
    }
    let count = stdout.lines().map(|l| l.trim()).filter(|l| !l.is_empty()).count();
    Some(count)
}

async fn count_ssh_sessions() -> usize {
    if let Some(n) = count_ssh_sessions_loginctl().await {
        return n;
    }
    if let Some(n) = count_ssh_sessions_who().await {
        return n;
    }
    if let Some(n) = count_ssh_sessions_ss().await {
        return n;
    }
    0
}

// Temperature reading from `sensors -u`
async fn read_max_temp_c() -> Option<f64> {
    let (ok, stdout, _) = sh(&["sensors", "-u"], 3000).await.ok()?;
    if !ok {
        return None;
    }
    let re = Regex::new(r"(?i)temp\d+_input:\s*([+\-]?\d+(?:\.\d+)?)").ok()?;
    let mut max_c: Option<f64> = None;
    for line in stdout.lines() {
        if let Some(caps) = re.captures(line) {
            if let Some(m) = caps.get(1) {
                if let Ok(v) = m.as_str().parse::<f64>() {
                    max_c = Some(match max_c {
                        None => v,
                        Some(prev) => prev.max(v),
                    });
                }
            }
        }
    }
    max_c
}

// USB presence listing via sysfs, fallback to lsusb
fn normalize_vidpid(s: &str) -> Option<String> {
    let s = s.trim().to_lowercase();
    let mut it = s.split(':');
    let v = it.next()?.trim();
    let p = it.next()?.trim();
    if v.len() == 4 && p.len() == 4 {
        Some(format!("{v}:{p}"))
    } else {
        None
    }
}

async fn list_usb_vidpid() -> HashSet<String> {
    let mut found = HashSet::new();
    let base = Path::new("/sys/bus/usb/devices");
    if let Ok(entries) = fs::read_dir(base) {
        for ent in entries.flatten() {
            let p = ent.path();
            if !p.is_dir() && !p.is_symlink() {
                continue;
            }
            let vid = fs::read_to_string(p.join("idVendor")).unwrap_or_default();
            let pid = fs::read_to_string(p.join("idProduct")).unwrap_or_default();
            let v = vid.trim().to_lowercase();
            let q = pid.trim().to_lowercase();
            if !v.is_empty() && !q.is_empty() {
                if let Some(s) = normalize_vidpid(&format!("{v}:{q}")) {
                    found.insert(s);
                }
            }
        }
    }

    if found.is_empty() {
        if let Ok((ok, stdout, _)) = sh(&["lsusb"], 2500).await {
            if ok {
                let re = Regex::new(r"ID\s+([0-9a-fA-F]{4}):([0-9a-fA-F]{4})").unwrap();
                for line in stdout.lines() {
                    if let Some(caps) = re.captures(line) {
                        let v = caps.get(1).unwrap().as_str().to_lowercase();
                        let p = caps.get(2).unwrap().as_str().to_lowercase();
                        found.insert(format!("{v}:{p}"));
                    }
                }
            }
        }
    }
    found
}

#[tokio::main]
async fn main() -> Result<()> {
    let opts = Opts::parse();

    let mut client = ApiClient::new(opts.host.clone(), opts.port);

    // Track last sent levels to de-dup
    let leds = Leds {
        cpu: opts.cpu_led.clone(),
        ssh: opts.ssh_led.clone(),
        temp: opts.temp_led.clone(),
        usb: opts.usb_led.clone(),
    };
    let mut last_sent: HashMap<String, Option<u8>> = HashMap::new();
    let all_led_names: Vec<String> = [leds.cpu.clone(), leds.ssh.clone(), leds.temp.clone(), leds.usb.clone()]
        .into_iter()
        .flatten()
        .collect();
    for n in &all_led_names {
        last_sent.insert(n.clone(), None); // unknown initially
    }

    let mut connected = false;
    let mut cpu_sampler = CpuUsageSampler::new();
    let mut blink_on = false;
    let mut last_blink = Instant::now();

    let interval_ms = opts.interval.max(250);
    let mut ticker = make_interval(Duration::from_millis(interval_ms));

    loop {
        ticker.tick().await;
        let now = Instant::now();

        // Desired levels per LED for this tick: 0..=max_level
        let mut desired: HashMap<String, u8> = HashMap::new();

        // CPU usage -> brightness
        if let Some(ref name) = leds.cpu {
            let usage = cpu_sampler.sample(); // 0..1
            let mut level = ((usage * (opts.max_level as f64)).round() as i64) as i64;
            if level < 0 {
                level = 0;
            }
            if level > opts.max_level as i64 {
                level = opts.max_level as i64;
            }
            desired.insert(name.clone(), level as u8);
        }

        // SSH presence
        if let Some(ref name) = leds.ssh {
            let count = count_ssh_sessions().await;
            let level = if count > 0 {
                if opts.ssh_scale {
                    count.min(opts.max_level as usize) as u8
                } else {
                    opts.max_level
                }
            } else {
                0
            };
            desired.insert(name.clone(), level);
        }

        // Temperature (lm-sensors)
        if let Some(ref name) = leds.temp {
            if let Some(t) = read_max_temp_c().await {
                let level = if t >= opts.temp_crit {
                    // Blink at full brightness
                    if now.duration_since(last_blink).as_millis() as u64 >= opts.blink_ms {
                        blink_on = !blink_on;
                        last_blink = now;
                    }
                    if blink_on { opts.max_level } else { 0 }
                } else if t >= opts.temp_warn {
                    (opts.max_level / 2).max(1)
                } else {
                    0
                };
                desired.insert(name.clone(), level);
            }
        }

        // USB presence: all required must be present
        if let Some(ref name) = leds.usb {
            let required: HashSet<String> = opts.usb_ids.iter().flat_map(|s| normalize_vidpid(s)).collect();
            let present = list_usb_vidpid().await;
            let all_present = required.iter().all(|id| present.contains(id));
            desired.insert(name.clone(), if all_present { opts.max_level } else { 0 });
        }

        let any_active = desired.values().any(|&v| v > 0);
        if any_active {
            // open connection if needed
            if !connected {
                if client.connect().await.is_ok() {
                    connected = true;
                } else {
                    connected = false;
                }
            }
            if connected {
                for (name, lvl) in desired.iter() {
                    if last_sent.get(name).copied().flatten() == Some(*lvl) {
                        continue; // no change
                    }
                    let res = if *lvl == 0 {
                        client.send(&format!("off {name}")).await
                    } else if *lvl >= opts.max_level {
                        client.send(&format!("on {name}")).await
                    } else {
                        client.send(&format!("bright {name} {lvl}")).await
                    };
                    match res {
                        Ok(_) => {
                            last_sent.insert(name.clone(), Some(*lvl));
                        }
                        Err(_) => {
                            connected = false;
                            let _ = client.close().await;
                            break;
                        }
                    }
                }
            }
        } else {
            // No active indicators. If any was non-zero previously, turn off managed once then disconnect.
            let had_non_zero = last_sent.values().any(|v| matches!(v, Some(x) if *x > 0));
            if had_non_zero {
                if !connected {
                    if client.connect().await.is_ok() {
                        connected = true;
                    }
                }
                if connected {
                    for n in &all_led_names {
                        let _ = client.send(&format!("off {n}")).await;
                        last_sent.insert(n.clone(), Some(0));
                    }
                }
            }
            if connected {
                client.close().await;
                connected = false;
            }
        }
    }
}

fn make_interval(period: Duration) -> Interval {
    let start = Instant::now() + period;
    interval_at(start.into(), period)
}

#[derive(Clone)]
struct Leds {
    cpu: Option<String>,
    ssh: Option<String>,
    temp: Option<String>,
    usb: Option<String>,
}