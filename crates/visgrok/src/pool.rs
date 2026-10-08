//! A small pool of worker threads that transforms items in parallel and
//! hands the results back in submission order.

use std::collections::BTreeMap;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// Parallel map with ordered output.
pub struct OrderedPool<T: Send + 'static, R: Send + 'static> {
    jobs: Option<Sender<(u64, T)>>,
    done: Receiver<(u64, R)>,
    workers: Vec<JoinHandle<()>>,
    next_seq: u64,
    next_out: u64,
    ready: BTreeMap<u64, R>,
}

impl<T: Send + 'static, R: Send + 'static> OrderedPool<T, R> {
    /// Starts `threads` workers (default: a share of the CPUs) running `f`.
    pub fn new(threads: Option<usize>, f: impl Fn(T) -> R + Send + Sync + 'static) -> Self {
        let threads = threads.unwrap_or_else(|| {
            std::thread::available_parallelism().map_or(2, |n| n.get().saturating_sub(4).clamp(1, 6))
        });
        let (jtx, jrx) = channel::<(u64, T)>();
        let jrx = Arc::new(Mutex::new(jrx));
        let (dtx, drx) = channel();
        let f = Arc::new(f);
        let workers = (0..threads.max(1))
            .map(|_| {
                let (jrx, dtx, f) = (jrx.clone(), dtx.clone(), f.clone());
                std::thread::spawn(move || {
                    loop {
                        let job = jrx.lock().unwrap().recv();
                        let Ok((seq, item)) = job else { return };
                        if dtx.send((seq, f(item))).is_err() {
                            return;
                        }
                    }
                })
            })
            .collect();
        OrderedPool { jobs: Some(jtx), done: drx, workers, next_seq: 0, next_out: 0, ready: BTreeMap::new() }
    }

    /// Items submitted but not yet returned.
    pub fn in_flight(&self) -> usize {
        (self.next_seq - self.next_out) as usize
    }

    /// Queues an item.
    pub fn submit(&mut self, item: T) {
        let seq = self.next_seq;
        self.next_seq += 1;
        if let Some(j) = &self.jobs {
            let _ = j.send((seq, item));
        }
    }

    /// The next result in order: waits for it when `block`, otherwise
    /// returns `None` if it isn't ready yet. `None` too when nothing is
    /// in flight.
    pub fn next(&mut self, block: bool) -> Option<R> {
        loop {
            if let Some(r) = self.ready.remove(&self.next_out) {
                self.next_out += 1;
                return Some(r);
            }
            if self.next_out == self.next_seq {
                return None;
            }
            let got = if block { self.done.recv().ok()? } else { self.done.try_recv().ok()? };
            self.ready.insert(got.0, got.1);
        }
    }
}

impl<T: Send + 'static, R: Send + 'static> Drop for OrderedPool<T, R> {
    fn drop(&mut self) {
        self.jobs = None;
        for w in self.workers.drain(..) {
            let _ = w.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keeps_order() {
        let mut p = OrderedPool::new(Some(4), |x: u64| {
            std::thread::sleep(std::time::Duration::from_micros((x * 7919) % 300));
            x * 2
        });
        for i in 0..200 {
            p.submit(i);
        }
        let out: Vec<u64> = std::iter::from_fn(|| p.next(true)).collect();
        assert_eq!(out, (0..200).map(|i| i * 2).collect::<Vec<_>>());
    }
}
