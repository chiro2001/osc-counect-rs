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
