// from ns to hours
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub enum TimeUnit {
    #[default]
    Nanosecond,
    Microsecond,
    Millisecond,
    Second,
    Minute,
    Hour,
}

impl TimeUnit {
    pub fn divider(&self) -> u64 {
        match self {
            TimeUnit::Nanosecond => 1,
            TimeUnit::Microsecond => 1_000,
            TimeUnit::Millisecond => 1_000_000,
            TimeUnit::Second => 1_000_000_000,
            TimeUnit::Minute => 60 * 1_000_000_000,
            TimeUnit::Hour => 60 * 60 * 1_000_000_000,
        }
    }
    pub fn fixed(ns: u64) -> Self {
        match ns {
            1..=999 => TimeUnit::Nanosecond,
            1000..=999_999 => TimeUnit::Microsecond,
            1_000_000..=999_999_999 => TimeUnit::Millisecond,
            1_000_000_000..=59_999_999_999 => TimeUnit::Second,
            60_000_000_000..=3_599_999_999_999 => TimeUnit::Minute,
            _ => TimeUnit::Hour,
        }
    }
    pub fn fixed1k(ns: u64) -> Self {
        match ns {
            1..=999 => TimeUnit::Nanosecond,
            1000..=999_999 => TimeUnit::Nanosecond,
            1_000_000..=999_999_999 => TimeUnit::Microsecond,
            1_000_000_000..=59_999_999_999 => TimeUnit::Millisecond,
            60_000_000_000..=3_599_999_999_999 => TimeUnit::Second,
            _ => TimeUnit::Minute,
        }
    }
}

impl Into<&'static str> for TimeUnit {
    fn into(self) -> &'static str {
        match self {
            TimeUnit::Nanosecond => "ns",
            TimeUnit::Microsecond => "us",
            TimeUnit::Millisecond => "ms",
            TimeUnit::Second => "s",
            TimeUnit::Minute => "m",
            TimeUnit::Hour => "h",
        }
    }
}

pub struct TimeScale {
    pub time: u64,
    pub unit: TimeUnit,
}

impl TimeScale {
    pub fn from_ns(ns: u64) -> Self {
        let unit = TimeUnit::fixed(ns);
        let time = ns / unit.divider();
        Self { time, unit }
    }
}

// from mV to V
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub enum VoltageUnit {
    #[default]
    MilliVolt,
    Volt,
}

impl VoltageUnit {
    pub fn divider(&self) -> u64 {
        match self {
            VoltageUnit::MilliVolt => 1,
            VoltageUnit::Volt => 1_000,
        }
    }
    pub fn fixed(mv: u64) -> Self {
        match mv {
            1..=999 => VoltageUnit::MilliVolt,
            _ => VoltageUnit::Volt,
        }
    }
    pub fn fixed1k(_mv: u64) -> Self {
        VoltageUnit::MilliVolt
    }
    pub fn str(&self) -> &'static str {
        let s: &'static str = self.into();
        s
    }
    pub fn next(&self) -> Self {
        match self {
            _ => VoltageUnit::Volt,
        }
    }
}

impl Into<&'static str> for VoltageUnit {
    fn into(self) -> &'static str {
        match self {
            VoltageUnit::MilliVolt => "mV",
            VoltageUnit::Volt => "V",
        }
    }
}

impl Into<&'static str> for &VoltageUnit {
    fn into(self) -> &'static str {
        (*self).into()
    }
}

pub struct VoltageScale {
    pub voltage: u64,
    pub unit: VoltageUnit,
    buf: [u8; 8],
}

impl VoltageScale {
    pub fn from_mv(mv: u64) -> Self {
        let unit = VoltageUnit::fixed1k(mv);
        let voltage = mv / unit.divider();
        Self {
            voltage,
            unit,
            buf: Default::default(),
        }
    }
    pub fn str<'d>(&'d mut self) -> &'d str {
        if self.voltage >= 1000 {
            format_no_std::show(
                &mut self.buf,
                format_args!(
                    "{}{}",
                    self.voltage as f32 / 1000f32,
                    self.unit.next().str()
                ),
            )
            .unwrap()
        } else {
            format_no_std::show(
                &mut self.buf,
                format_args!("{}{}", self.voltage, self.unit.str()),
            )
            .unwrap()
        }
    }
}
