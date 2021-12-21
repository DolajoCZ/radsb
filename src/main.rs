use rtlsdr_mt;

fn main() {
    let (mut ctl, mut reader) = rtlsdr_mt::open(0).unwrap();

    // ctl.enable_agc().unwrap();
    // ctl.set_ppm(-2).unwrap();
    ctl.set_center_freq(1_090_000_000).unwrap();
    ctl.set_sample_rate(2_000_000).unwrap();
    ctl.disable_agc().unwrap();
    // ctl.set_bandwidth(100);

    // std::thread::spawn(move || loop {
    //     let next = ctl.center_freq() + 1000;
    //     ctl.set_center_freq(next).unwrap();

    //     std::thread::sleep(std::time::Duration::from_secs(1));
    // });

    reader
        .read_async(4, 32768, |bytes| {
            println!("----------------------------");
            println!("Len = {}", bytes.len());
            println!("Min = {:?}", bytes.iter().min());
            println!("Max = {:?}", bytes.iter().max());

            // println!("I = {}", bytes[0]);
            // println!("Q = {}", bytes[1]);

            // println!("I = {}", bytes[2]);
            // println!("Q = {}", bytes[3]);
            // println!("q[0] = {}", bytes[32767]);
        })
        .unwrap();
}
