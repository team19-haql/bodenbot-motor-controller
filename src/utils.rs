use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex as EmbassyMutex;

pub type Mutex<T> = EmbassyMutex<ThreadModeRawMutex, T>;

#[macro_export]
macro_rules! join {
    ($f0:expr, $f1: expr, $($rest:expr),+ $(,)?) => {{
        join!($f0, join!($f1, $($rest),+))
    }};
    ($f0:expr, $f1: expr $(,)?) => {{
        use embassy_futures::join::join;
        join($f0, $f1)
    }};
    ($f0:expr $(,)?) => {{
        $f0
    }};
}
