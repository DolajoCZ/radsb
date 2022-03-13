#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

//! Application for parsing ADS-B messages from stream of IQ samples
//! Supported sources of IQ data are
//! - RTL-SDR
//! - binary file were IQ samples are stored u8 values were I is on n position and Q is on n+1 position

use itertools::Itertools;
use std::{self, io::Read, str::FromStr};

/// Size of ADS-B payload in bites
const MSG_BIT_LENGTH: usize = 112;

/// Polynomial indexes used in CRC calculation
/// https://mode-s.org/decode/content/ads-b/8-error-control.html
const CRC_GENERATOR: u128 = 0b1111111111111010000001001;

/// Errors for BitsCorrectionsError
#[derive(Debug)]
enum BitsCorrectionsError {
    /// Number of bits corrections is bigger than message payload
    ToBig,
    /// Unable to
    StrParse,
}

/// Maximal number of bits to try correct
struct BitsCorrections(usize);

impl BitsCorrections {
    /// Create new instance of BitsCorrections
    fn new(value: usize) -> Result<Self, BitsCorrectionsError> {
        match Self::valid_value(value) {
            true => Ok(BitsCorrections(value)),
            false => Err(BitsCorrectionsError::ToBig),
        }
    }

    /// Check if value is <= 112
    fn valid_value(value: usize) -> bool {
        value <= MSG_BIT_LENGTH
    }
}

impl std::str::FromStr for BitsCorrections {
    type Err = BitsCorrectionsError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.parse::<usize>() {
            Ok(value) => Ok(BitsCorrections::new(value)?),
            Err(_) => Err(BitsCorrectionsError::StrParse),
        }
    }
}

/// State of loading samples
enum State {
    /// Searching of preamble
    /// Value defines index in buffer where current sample should be stored
    PreambleSearch(usize),

    /// Loading of message
    /// Value defines number of loaded samples
    MessageLoading(usize),
}

/// Source of IQ samples
enum SamplesSource {
    /// From binary file
    File(String),

    /// From RTL-SDR
    /// Value defines device ID
    RtlSdr(u32),
}

/// Parser for ADS-B messages
struct AdsbParser {
    /// Maximal number of bits to try correct
    bits_corrections: BitsCorrections,

    /// Print only valid messages
    print_only_valid_msgs: bool,

    /// State of search
    state: State,

    ///Buffer for preamble values
    preamble: [f32; 16],

    /// Buffer for IQ samples
    msg_iq_samples: [f32; MSG_BIT_LENGTH * 2],

    /// Precalculated magnitude values
    magnitude_map: [f32; 65536],
}

impl AdsbParser {
    /// Create new instance of AdsbParser
    fn new(bits_corrections: BitsCorrections, print_only_valid_msgs: bool) -> AdsbParser {
        AdsbParser {
            bits_corrections,
            print_only_valid_msgs,
            state: State::PreambleSearch(0),
            preamble: [0.0; 16],
            msg_iq_samples: [0.0; MSG_BIT_LENGTH * 2],
            magnitude_map: get_magnitude_map(),
        }
    }

    /// Analyze new IQ samples
    fn analyze_iq_samples(&mut self, samples: &[u8]) {
        for sample in samples.windows(2).step_by(2) {
            // Get signal magnitude
            let signal_magnitude =
                self.magnitude_map[(sample[0] as usize) * 256 + (sample[1] as usize)];

            match self.state {
                // Searching of preamble phase
                State::PreambleSearch(index) => {
                    // Insert signal magnitude to preamble
                    self.preamble[index] = signal_magnitude;

                    let corr = get_preamble_correlation(&self.preamble, &index);

                    // Preamble found
                    if corr > 0.80 {
                        // Clear preamble
                        self.preamble = [0.0; 16];

                        self.state = State::MessageLoading(0)
                    // Preamble not found
                    } else {
                        self.state = State::PreambleSearch((index + 1) % 16)
                    }
                }
                // Loading message phase
                State::MessageLoading(counter) => {
                    self.msg_iq_samples[counter] = signal_magnitude;

                    // Loading of message is complete
                    if counter == (MSG_BIT_LENGTH * 2 - 1) {
                        let msg = get_msg(&self.msg_iq_samples, self.bits_corrections.0);

                        match msg {
                            Ok(msg) => print_msg(msg),
                            Err(msg) => {
                                if !self.print_only_valid_msgs {
                                    print_msg(msg)
                                }
                            }
                        }

                        self.state = State::PreambleSearch(0);
                    } else {
                        self.state = State::MessageLoading(counter + 1)
                    }
                }
            }
        }
    }
}

// ---------------------------------- Setup --------------------------------- //

/// Check if value is integer in range
fn check_value_in_range_int(value: &str, min: usize, max: usize) -> Result<(), String> {
    let error_msg = format!("Value must be integer in range [{}, {}]", min, max);

    match value.parse::<usize>() {
        Ok(value) => match min <= value && value <= max {
            true => Ok(()),
            false => Err(error_msg),
        },
        Err(_) => Err(error_msg),
    }
}

/// Load command line arguments
fn get_arg_parse() -> (SamplesSource, bool, BitsCorrections) {
    // Arguments common for both input sources
    let args_common = vec![
        clap::Arg::new("valid_crc")
            .short('c')
            .long("valid_crc")
            .help("Return only messages with valid CRC"),
        clap::Arg::new("bits_corrections")
            .short('b')
            .long("bits_corrections")
            .help("Set maximum number of bits correction")
            .default_value("0")
            .validator(|value| check_value_in_range_int(value, 0, MSG_BIT_LENGTH)),
    ];

    // Create argument parser for both sources
    let args = clap::App::new("Get ADS-B messages from IQ data")
        .setting(clap::AppSettings::SubcommandRequiredElseHelp)
        .subcommand(
            clap::App::new("rtl_sdr")
                .about("Get ADS-B messages from IQ data where source of IQ data is RTL-SDR")
                .arg(
                    clap::Arg::new("id")
                        .long("id")
                        .help("ID of RTL-SDR device")
                        .takes_value(true)
                        .default_value("0")
                        .validator(|value| {
                            check_value_in_range_int(value, 0, (u32::MAX - 1) as usize)
                        }),
                )
                .args(&args_common),
        )
        .subcommand(
            clap::App::new("file")
                .about("Get ADS-B messages from IQ data where source of IQ data is RTL-SDR")
                .arg(
                    clap::Arg::new("file_path")
                        .help("Binary file containing u8 IQ samples")
                        .required(true),
                )
                .args(&args_common),
        )
        .get_matches();

    let sub_argument_name = args.subcommand_name().unwrap();
    let sub_arguments = args.subcommand_matches(sub_argument_name).unwrap();

    // Get source type - RTL-SDR or file
    let source = match sub_argument_name == "file" {
        true => {
            let trim_chars: &[char] = &['"', '\''];
            SamplesSource::File(
                sub_arguments
                    .value_of("file_path")
                    .unwrap()
                    .trim_matches(trim_chars)
                    .to_string(),
            )
        }
        false => SamplesSource::RtlSdr(
            sub_arguments
                .value_of("id")
                .unwrap()
                .parse::<u32>()
                .unwrap(),
        ),
    };

    (
        source,
        sub_arguments.is_present("valid_crc"),
        BitsCorrections::from_str(sub_arguments.value_of("bits_corrections").unwrap()).unwrap(),
    )
}

// TODO: add errors for unwrap
/// Set RTL-SDR
fn set_rtl_sdr(controller: &mut rtlsdr_mt::Controller) {
    // Set frequency to 1090MHz
    controller.set_center_freq(1_090_000_000).unwrap();

    // Set sample frequency to 2Ms
    controller.set_sample_rate(2_000_000).unwrap();

    // Enable AGC
    controller.enable_agc().unwrap();
}

// ---------------------------------- Other --------------------------------- //

/// Pre calculate magnitudes for each combination of I and Q component
fn get_magnitude_map() -> [f32; 65536] {
    let mut result = [0_f32; 65536];
    (0_usize..=255).for_each(|i| {
        (0_usize..=255).for_each(|q| {
            result[i * 256 + q] = (i as f32 - 128_f32).hypot(q as f32 - 128_f32);
        })
    });
    result
}

/// Print message
fn print_msg(msg: u128) {
    println!("*{:x};", msg)
}

// ------------------------------ Calculate CRC ----------------------------- //

/// Get bits slice
fn get_bits_slice(value: &u128, start_index: u8, count: u8) -> u128 {
    (value >> start_index) & ((1 << count) - 1)
}

/// Get first n bits
fn get_first_n_bits(value: &u128, count: u8) -> u128 {
    value & ((1 << count) - 1)
}

/// Get bit on index
fn get_specific_bit(word: &u128, index: u8) -> u128 {
    (word >> index) & 1
}

/// Check if message has valid CRC or not
fn check_crc(msg: &u128) -> Result<(), u128> {
    match (0_u8..=87_u8).rev().fold(*msg, |msg, index| {
        match get_specific_bit(&msg, index + 24) {
            1 => {
                let slice = get_bits_slice(&msg, index, 25);
                let rest = get_first_n_bits(&msg, index);
                let xor_result = slice ^ CRC_GENERATOR;
                let xor_shift = xor_result << index;
                rest | xor_shift
            }
            _ => msg,
        }
    }) {
        0 => Ok(()),
        x => Err(x),
    }
}

// ------------------------------- Analyze msg ------------------------------ //

/// Calculate correlation for preamble
fn get_preamble_correlation(samples: &[f32; 16], index_of_last_sample: &usize) -> f32 {
    let samples_sum: f32 = samples.iter().sum();

    let numerator = 16.0
        * (samples[(index_of_last_sample + 1) % 16]
            + samples[(index_of_last_sample + 3) % 16]
            + samples[(index_of_last_sample + 8) % 16]
            + samples[(index_of_last_sample + 10) % 16])
        - 4.0 * samples_sum;
    let denominator = 48_f32.sqrt()
        * (16.0 * samples.iter().map(|value| value * value).sum::<f32>()
            - (samples_sum * samples_sum))
            .sqrt();

    numerator / denominator
}

/// Convert samples to ADS-B message - manchester coding
fn get_msg_from_samples<T>(samples: &[T; MSG_BIT_LENGTH * 2]) -> u128
where
    T: std::cmp::PartialOrd,
{
    samples
        .windows(2)
        .step_by(2)
        .enumerate()
        .filter(|(_, value)| value[0] > value[1])
        .fold(0_u128, |msg, (index, _)| {
            msg | (1_u128 << (MSG_BIT_LENGTH - index - 1))
        })
}

/// Get difference between 2*n (even) and 2*n + 1 (odd) sample
fn get_diff_between_odd_and_even_sample(
    samples: &[f32; MSG_BIT_LENGTH * 2],
) -> [f32; MSG_BIT_LENGTH] {
    let mut iq_diffs = [0.0; MSG_BIT_LENGTH];
    samples
        .windows(2)
        .step_by(2)
        .enumerate()
        .for_each(|(index, values)| iq_diffs[index] = (values[0] - values[1]).abs());
    iq_diffs
}

/// Get indexes of first n smallest values
fn get_indexes_of_first_n_smallest_values(
    differences: [f32; MSG_BIT_LENGTH],
    n: usize,
) -> std::collections::HashSet<usize> {
    let mut indexes = std::collections::HashSet::with_capacity(n);
    for _ in 0..n {
        indexes.insert(
            differences
                .iter()
                .enumerate()
                .filter(|(index, _)| !indexes.contains(index))
                .fold(
                    (0_usize, f32::MAX),
                    |(min_index, min_value), (cur_index, cur_value)| match cur_value < &min_value {
                        true => (cur_index, *cur_value),
                        false => (min_index, min_value),
                    },
                )
                .0,
        );
    }
    indexes
}

/// Get candidates for invalid bites
fn get_candidates_for_corrections(samples: &[f32; 2 * MSG_BIT_LENGTH], n: usize) -> Vec<u128> {
    let difference = get_diff_between_odd_and_even_sample(samples);

    let indexes = get_indexes_of_first_n_smallest_values(difference, n);

    indexes
        .iter()
        .map(|index| (1_u128 << (MSG_BIT_LENGTH - index - 1)))
        .collect_vec()
}

/// Get ADS-B messages
fn get_msg(
    msg_iq_samples: &[f32; MSG_BIT_LENGTH * 2],
    number_of_bits_corr: usize,
) -> Result<u128, u128> {
    let msg = get_msg_from_samples(msg_iq_samples);

    if check_crc(&msg).is_ok() {
        return Ok(msg);
    }

    if number_of_bits_corr == 0 {
        return Err(msg);
    }

    let correction_values = get_candidates_for_corrections(&msg_iq_samples, number_of_bits_corr);

    for correction_value in correction_values.iter().powerset().skip(1) {
        let msg_corrected = correction_value.iter().fold(msg, |msg, item| msg ^ **item);

        if check_crc(&msg).is_ok() {
            return Ok(msg_corrected);
        }
    }

    Err(msg)
}

// ---------------------------------- Main ---------------------------------- //

fn main() {
    // Load configuration
    let (source_parameters, print_only_valid_msgs, bits_corrections) = get_arg_parse();

    let mut adsb_parser = AdsbParser::new(bits_corrections, print_only_valid_msgs);

    match source_parameters {
        // File source
        SamplesSource::File(file_path) => {
            let mut file = match std::fs::File::open(&file_path) {
                Ok(file) => file,
                Err(err) => {
                    println!("File \"{}\" error. {}", file_path, err);
                    std::process::exit(1);
                }
            };

            let mut file_content = vec![];
            if let Err(err) = file.read_to_end(&mut file_content) {
                println!("File \"{}\" error. {}", file_path, err)
            };

            adsb_parser.analyze_iq_samples(file_content.as_slice());
        }

        // RTL-SDR source
        SamplesSource::RtlSdr(id) => {
            let (mut controller, mut reader) = match rtlsdr_mt::open(id) {
                Ok((controller, reader)) => (controller, reader),
                Err(_) => {
                    println!("Unable to open communication with RTL-SDR with id {}", id);
                    std::process::exit(2);
                }
            };

            // Set controller
            set_rtl_sdr(&mut controller);

            if reader.read_async(4, 32768, |samples: &[u8]| {
                adsb_parser.analyze_iq_samples(samples);
            }) == Err(())
            {
                println!("Communication with RTL-SDR interrupted");
                std::process::exit(3);
            };
        }
    };
}

/// Module for unit tests
#[cfg(test)]
mod tests {

    // ---------------------------------- Setup --------------------------------- //

    /// Test module for function check_value_in_range_int
    mod test_check_value_in_range_int {
        use crate::check_value_in_range_int;

        /// Test valid case
        #[test]
        fn test_valid_case() {
            assert_eq!(check_value_in_range_int("0", 0, 5), Ok(()));
            assert_eq!(check_value_in_range_int("1", 0, 5), Ok(()));
            assert_eq!(check_value_in_range_int("5", 0, 5), Ok(()));
        }

        /// Test not a number case
        #[test]
        fn test_not_number_case() {
            assert_eq!(check_value_in_range_int("r", 0, 5).is_err(), true);
        }

        /// Test value out of range case
        #[test]
        fn test_value_out_of_range_case() {
            assert_eq!(check_value_in_range_int("0", 1, 5).is_err(), true);
            assert_eq!(check_value_in_range_int("6", 1, 5).is_err(), true);
        }
    }

    // ------------------------------ Calculate CRC ----------------------------- //

    /// Test module for function get_bits_slice
    mod test_get_bits_slice {
        use crate::get_bits_slice;

        /// Test valid case
        #[test]
        fn test_valid_case() {
            assert_eq!(get_bits_slice(&140, 2, 8), 35);
            assert_eq!(get_bits_slice(&140, 2, 1), 1);
            assert_eq!(get_bits_slice(&140, 2, 0), 0);
            assert_eq!(get_bits_slice(&140, 1, 1), 0);
            assert_eq!(get_bits_slice(&0, 2, 8), 0);
        }
    }

    /// Test module for function get_first_n_bits
    mod test_get_first_n_bits {
        use crate::get_first_n_bits;

        /// Test valid case
        #[test]
        fn test_valid_case() {
            assert_eq!(get_first_n_bits(&0, 0), 0);
            assert_eq!(get_first_n_bits(&1, 0), 0);

            assert_eq!(get_first_n_bits(&11, 1), 1);
            assert_eq!(get_first_n_bits(&11, 2), 3);
            assert_eq!(get_first_n_bits(&11, 3), 3);
            assert_eq!(get_first_n_bits(&11, 4), 11);
            assert_eq!(get_first_n_bits(&11, 5), 11);
            assert_eq!(get_first_n_bits(&11, 6), 11);
        }
    }

    /// Test module for function get_specific_bit
    mod test_get_specific_bit {
        use crate::get_specific_bit;

        /// Test valid case
        #[test]
        fn test_valid_case() {
            assert_eq!(get_specific_bit(&10, 20), 0);
            assert_eq!(get_specific_bit(&10, 0), 0);
            assert_eq!(get_specific_bit(&10, 1), 1);
            assert_eq!(get_specific_bit(&10, 2), 0);
            assert_eq!(get_specific_bit(&10, 3), 1);
        }
    }

    /// Test module for function check_crc
    mod test_get_crc {
        use crate::check_crc;

        /// Test valid case
        #[test]
        fn test_valid_case() {
            assert_eq!(check_crc(&0x8d43beb2ea4478656d3c08fcd718), Ok(()));
            assert_eq!(check_crc(&0x8d4242a858b99071438a1219e436), Ok(()));
            assert_eq!(check_crc(&0x8d4242a8990d7e9ef80892877649), Ok(()));
            assert_eq!(check_crc(&0x8da9ddb058b5071d89b6cddeeb19), Ok(()));
            assert_eq!(check_crc(&0x8d4409cd5875e799c3a29e37d89c), Ok(()));
            assert_eq!(check_crc(&0x8d498b28590d844519866f919e9d), Ok(()));
            assert_eq!(check_crc(&0x8d461e18991530a038d88d36806c), Ok(()));
            assert_eq!(check_crc(&0x8d8964faea4e385f493f8cd33477), Ok(()));
            assert_eq!(check_crc(&0x8d4bc845990d28275010975b931b), Ok(()));
            assert_eq!(check_crc(&0x8d49d38c58af801cbdab5b7876da), Ok(()));
        }

        /// Test invalid case
        #[test]
        fn test_invalid_case() {
            assert_eq!(check_crc(&0xa8100f0dede9fd303ffc013a9677), Err(11650396));
            assert_eq!(check_crc(&0xc6a38f987c11800184243c68c8e7), Err(15834858));
            assert_eq!(check_crc(&0x8d49d38c58af8792a793aed408c8), Err(2048));
            assert_eq!(check_crc(&0x80e1999658cd67ec27b7c5f1f76b), Err(9004282));
            assert_eq!(check_crc(&0x9de7bb9e80805d6b184001c2a000), Err(8453817));
            assert_eq!(check_crc(&0x88000e1dea4a0134dbe7ff015ccd), Err(5040887));
            assert_eq!(check_crc(&0xa1118e9d029997344a6c207a0fc5), Err(11614455));
            assert_eq!(check_crc(&0xa000169082960144a004ef38ca8b), Err(9795626));
            assert_eq!(check_crc(&0x8d4bb84c990d391ed84c9073ad0c), Err(13915516));
            assert_eq!(check_crc(&0xc2b0a350efaca642612c3401256e), Err(1311362));
        }
    }

    /// Test module for function get_preamble_correlation
    mod test_get_preamble_correlation {
        use crate::get_preamble_correlation;

        const EXPECTED_OUTPUT: f32 = 0.503142201032944;

        /// Test last sample on index 0 case
        #[test]
        fn test_last_index_0_case() {
            let signal = [
                4.8, 175.4, 48.1, 85.5, 31.1, 74.8, 83.0, 1.0, 78.4, 8.0, 24.7, 47.5, 6.4, 95.47,
                55.5, 11.4,
            ];

            assert!((get_preamble_correlation(&signal, &0) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 1 case
        #[test]
        fn test_last_index_1_case() {
            let signal = [
                11.4, 4.8, 175.4, 48.1, 85.5, 31.1, 74.8, 83.0, 1.0, 78.4, 8.0, 24.7, 47.5, 6.4,
                95.47, 55.5,
            ];

            assert!((get_preamble_correlation(&signal, &1) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 2 case
        #[test]
        fn test_last_index_2_case() {
            let signal = [
                55.5, 11.4, 4.8, 175.4, 48.1, 85.5, 31.1, 74.8, 83.0, 1.0, 78.4, 8.0, 24.7, 47.5,
                6.4, 95.47,
            ];

            assert!((get_preamble_correlation(&signal, &2) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 3 case
        #[test]
        fn test_last_index_3_case() {
            let signal = [
                95.47, 55.5, 11.4, 4.8, 175.4, 48.1, 85.5, 31.1, 74.8, 83.0, 1.0, 78.4, 8.0, 24.7,
                47.5, 6.4,
            ];

            assert!((get_preamble_correlation(&signal, &3) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 4 case
        #[test]
        fn test_last_index_4_case() {
            let signal = [
                6.4, 95.47, 55.5, 11.4, 4.8, 175.4, 48.1, 85.5, 31.1, 74.8, 83.0, 1.0, 78.4, 8.0,
                24.7, 47.5,
            ];

            assert!((get_preamble_correlation(&signal, &4) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 5 case
        #[test]
        fn test_last_index_5_case() {
            let signal = [
                47.5, 6.4, 95.47, 55.5, 11.4, 4.8, 175.4, 48.1, 85.5, 31.1, 74.8, 83.0, 1.0, 78.4,
                8.0, 24.7,
            ];

            assert!((get_preamble_correlation(&signal, &5) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 6 case
        #[test]
        fn test_last_index_6_case() {
            let signal = [
                24.7, 47.5, 6.4, 95.47, 55.5, 11.4, 4.8, 175.4, 48.1, 85.5, 31.1, 74.8, 83.0, 1.0,
                78.4, 8.0,
            ];

            assert!((get_preamble_correlation(&signal, &6) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 7 case
        #[test]
        fn test_last_index_7_case() {
            let signal = [
                8.0, 24.7, 47.5, 6.4, 95.47, 55.5, 11.4, 4.8, 175.4, 48.1, 85.5, 31.1, 74.8, 83.0,
                1.0, 78.4,
            ];

            assert!((get_preamble_correlation(&signal, &7) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 8 case
        #[test]
        fn test_last_index_8_case() {
            let signal = [
                78.4, 8.0, 24.7, 47.5, 6.4, 95.47, 55.5, 11.4, 4.8, 175.4, 48.1, 85.5, 31.1, 74.8,
                83.0, 1.0,
            ];

            assert!((get_preamble_correlation(&signal, &8) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 9 case
        #[test]
        fn test_last_index_9_case() {
            let signal = [
                1.0, 78.4, 8.0, 24.7, 47.5, 6.4, 95.47, 55.5, 11.4, 4.8, 175.4, 48.1, 85.5, 31.1,
                74.8, 83.0,
            ];

            assert!((get_preamble_correlation(&signal, &9) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 10 case
        #[test]
        fn test_last_index_10_case() {
            let signal = [
                83.0, 1.0, 78.4, 8.0, 24.7, 47.5, 6.4, 95.47, 55.5, 11.4, 4.8, 175.4, 48.1, 85.5,
                31.1, 74.8,
            ];
            assert!((get_preamble_correlation(&signal, &10) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 11 case
        #[test]
        fn test_last_index_11_case() {
            let signal = [
                74.8, 83.0, 1.0, 78.4, 8.0, 24.7, 47.5, 6.4, 95.47, 55.5, 11.4, 4.8, 175.4, 48.1,
                85.5, 31.1,
            ];

            assert!((get_preamble_correlation(&signal, &11) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 12 case
        #[test]
        fn test_last_index_12_case() {
            let signal = [
                31.1, 74.8, 83.0, 1.0, 78.4, 8.0, 24.7, 47.5, 6.4, 95.47, 55.5, 11.4, 4.8, 175.4,
                48.1, 85.5,
            ];

            assert!((get_preamble_correlation(&signal, &12) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 13 case
        #[test]
        fn test_last_index_13_case() {
            let signal = [
                85.5, 31.1, 74.8, 83.0, 1.0, 78.4, 8.0, 24.7, 47.5, 6.4, 95.47, 55.5, 11.4, 4.8,
                175.4, 48.1,
            ];

            assert!((get_preamble_correlation(&signal, &13) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 14 case
        #[test]
        fn test_last_index_14_case() {
            let signal = [
                48.1, 85.5, 31.1, 74.8, 83.0, 1.0, 78.4, 8.0, 24.7, 47.5, 6.4, 95.47, 55.5, 11.4,
                4.8, 175.4,
            ];

            assert!((get_preamble_correlation(&signal, &14) - EXPECTED_OUTPUT).abs() < 1e-7);
        }

        /// Test last sample on index 15 case
        #[test]
        fn test_last_index_15_case() {
            let signal = [
                175.4, 48.1, 85.5, 31.1, 74.8, 83.0, 1.0, 78.4, 8.0, 24.7, 47.5, 6.4, 95.47, 55.5,
                11.4, 4.8,
            ];

            assert!((get_preamble_correlation(&signal, &15) - EXPECTED_OUTPUT).abs() < 1e-7);
        }
    }

    /// Test module for function get_msg_from_samples
    mod test_get_msg_from_samples {
        use crate::get_msg_from_samples;

        /// Test valid case
        #[test]
        fn test_valid_case() {
            let samples = [
                -1.24, -9.27, 7.3, 2.0, -4.0, -5.59, -8.93, -4.76, -0.85, 9.02, -4.4, -3.71, 2.62,
                4.05, 3.02, -4.38, -2.97, 7.67, -0.45, -5.58, -0.79, 2.74, 2.47, 9.05, 4.17, 0.27,
                8.90, -9.69, -3.7, 7.18, -6.63, -7.18, 3.53, -3.64, -2.54, 0.39, -6.91, 3.99,
                -8.36, -8.63, -6.22, -4.12, -2.68, 7.95, 6.97, 2.49, 8.25, -0.89, -7.0, -9.1, 4.08,
                -3.35, 1.97, 8.55, 5.80, 3.07, -4.57, -3.95, -8.93, -4.0, 7.9, -3.72, 4.47, -7.47,
                -4.95, 3.86, -2.4, 3.69, -1.81, -4.35, -3.2, 5.88, -6.57, 8.41, 1.06, -9.37, 4.18,
                1.72, 1.56, -9.86, 4.36, -7.24, -4.05, -1.02, 3.90, -2.76, -0.34, 3.16, -8.99,
                -3.97, 8.06, -3.75, 1.33, 1.51, -2.31, 5.83, 6.61, -6.54, 5.37, -6.79, -8.04, 4.76,
                3.85, 3.75, -8.79, 9.75, -7.29, -0.97, -4.62, -5.81, -2.54, 0.84, -8.9, 1.02,
                -9.13, 1.89, 8.07, 6.64, -6.28, 0.04, -6.19, 1.16, 2.56, 7.03, -0.47, 6.61, 8.46,
                4.42, 3.66, -1.38, 2.41, 4.02, -5.01, 8.54, 8.94, 2.61, 8.2, -3.62, 0.23, 0.52,
                -9.15, -7.9, 9.73, 8.95, -7.0, -4.39, -3.59, -3.35, 2.13, -0.52, 6.27, -0.5, -9.48,
                1.85, -9.66, 3.97, -7.74, -8.64, 1.39, -0.76, -6.33, -7.99, 7.35, -0.95, 6.97,
                5.69, 6.88, -7.67, -8.08, -6.04, 7.20, -7.66, 2.55, 1.88, -5.9, 0.93, 9.18, -4.9,
                7.61, -8.37, 0.29, -2.88, -2.2, -0.82, 8.77, -5.51, -7.76, 2.56, -5.94, -0.61,
                6.03, -0.05, 1.22, -6.88, 2.02, 6.73, 9.99, 3.24, -9.39, 2.15, 7.23, -8.03, -9.38,
                -7.25, 8.11, 4.0, 5.6, -5.9, 4.06, -3.76, -0.20, -6.53, -2.08, -7.8, 7.68, -2.58,
                -1.24, -2.36, -7.74, -9.72, 0.32, -5.81, 6.17, 1.03,
            ];

            assert_eq!(
                get_msg_from_samples(&samples),
                4569688478902228576206568020356095
            );
        }
    }

    /// Test module for function get_diff_between_odd_and_even_sample
    mod test_get_diff_between_odd_and_even_sample {
        use crate::get_diff_between_odd_and_even_sample;

        /// Test valid case
        #[test]
        fn test_valid_case() {
            let samples = [
                -1.24, -9.27, 7.3, 2.0, -4.0, -5.59, -8.93, -4.76, -0.85, 9.02, -4.4, -3.71, 2.62,
                4.05, 3.02, -4.38, -2.97, 7.67, -0.45, -5.58, -0.79, 2.74, 2.47, 9.05, 4.17, 0.27,
                8.90, -9.69, -3.7, 7.18, -6.63, -7.18, 3.53, -3.64, -2.54, 0.39, -6.91, 3.99,
                -8.36, -8.63, -6.22, -4.12, -2.68, 7.95, 6.97, 2.49, 8.25, -0.89, -7.0, -9.1, 4.08,
                -3.35, 1.97, 8.55, 5.80, 3.07, -4.57, -3.95, -8.93, -4.0, 7.9, -3.72, 4.47, -7.47,
                -4.95, 3.86, -2.4, 3.69, -1.81, -4.35, -3.2, 5.88, -6.57, 8.41, 1.06, -9.37, 4.18,
                1.72, 1.56, -9.86, 4.36, -7.24, -4.05, -1.02, 3.90, -2.76, -0.34, 3.16, -8.99,
                -3.97, 8.06, -3.75, 1.33, 1.51, -2.31, 5.83, 6.61, -6.54, 5.37, -6.79, -8.04, 4.76,
                3.85, 3.75, -8.79, 9.75, -7.29, -0.97, -4.62, -5.81, -2.54, 0.84, -8.9, 1.02,
                -9.13, 1.89, 8.07, 6.64, -6.28, 0.04, -6.19, 1.16, 2.56, 7.03, -0.47, 6.61, 8.46,
                4.42, 3.66, -1.38, 2.41, 4.02, -5.01, 8.54, 8.94, 2.61, 8.2, -3.62, 0.23, 0.52,
                -9.15, -7.9, 9.73, 8.95, -7.0, -4.39, -3.59, -3.35, 2.13, -0.52, 6.27, -0.5, -9.48,
                1.85, -9.66, 3.97, -7.74, -8.64, 1.39, -0.76, -6.33, -7.99, 7.35, -0.95, 6.97,
                5.69, 6.88, -7.67, -8.08, -6.04, 7.20, -7.66, 2.55, 1.88, -5.9, 0.93, 9.18, -4.9,
                7.61, -8.37, 0.29, -2.88, -2.2, -0.82, 8.77, -5.51, -7.76, 2.56, -5.94, -0.61,
                6.03, -0.05, 1.22, -6.88, 2.02, 6.73, 9.99, 3.24, -9.39, 2.15, 7.23, -8.03, -9.38,
                -7.25, 8.11, 4.0, 5.6, -5.9, 4.06, -3.76, -0.20, -6.53, -2.08, -7.8, 7.68, -2.58,
                -1.24, -2.36, -7.74, -9.72, 0.32, -5.81, 6.17, 1.03,
            ];

            let expected_output = [
                8.03, 5.30, 1.59, 4.17, 9.87, 0.69, 1.43, 7.40, 10.64, 5.13, 3.53, 6.58, 3.90,
                18.59, 10.88, 0.55, 7.17, 2.93, 10.90, 0.27, 2.10, 10.63, 4.48, 9.14, 2.10, 7.43,
                6.58, 2.73, 0.62, 4.93, 11.62, 11.94, 8.81, 6.09, 2.54, 9.08, 14.98, 10.43, 2.46,
                11.42, 11.60, 3.03, 6.66, 3.50, 5.02, 11.81, 0.18, 8.14, 13.15, 12.16, 12.80, 0.10,
                18.54, 6.32, 1.19, 3.38, 9.92, 11.02, 1.43, 6.32, 7.35, 4.47, 7.08, 4.04, 5.04,
                1.61, 13.55, 6.33, 11.82, 0.29, 1.25, 0.78, 2.61, 0.24, 2.65, 6.77, 11.33, 13.63,
                0.90, 2.15, 1.66, 8.30, 1.28, 14.55, 2.04, 14.86, 0.67, 6.83, 14.08, 15.98, 3.17,
                1.38, 14.28, 10.32, 5.33, 6.08, 8.10, 4.71, 6.75, 11.54, 15.26, 2.13, 4.11, 11.50,
                7.82, 6.33, 5.72, 10.26, 1.12, 1.98, 6.13, 5.14,
            ];

            get_diff_between_odd_and_even_sample(&samples)
                .iter()
                .zip(expected_output.iter())
                .for_each(|(x, y)| assert_eq!((x - y).abs() < 0.01, true))
        }
    }

    /// Test module for function get_indexes_of_first_n_smallest_values
    mod test_get_indexes_of_first_n_smallest_values {
        use crate::get_indexes_of_first_n_smallest_values;

        const DIFFERENCES: [f32; 112] = [
            8.03, 5.30, 1.59, 4.17, 9.87, 0.69, 1.43, 7.40, 10.64, 5.13, 3.53, 6.58, 3.90, 18.59,
            10.88, 0.55, 7.17, 2.93, 10.90, 0.27, 2.10, 10.63, 4.48, 9.14, 2.10, 7.43, 6.58, 2.73,
            0.62, 4.93, 11.62, 11.94, 8.81, 6.09, 2.54, 9.08, 14.98, 10.43, 2.46, 11.42, 11.60,
            3.03, 6.66, 3.50, 5.02, 11.81, 0.18, 8.14, 13.15, 12.16, 12.80, 0.10, 18.54, 6.32,
            1.19, 3.38, 9.92, 11.02, 1.43, 6.32, 7.35, 4.47, 7.08, 4.04, 5.04, 1.61, 13.55, 6.33,
            11.82, 0.29, 1.25, 0.78, 2.61, 0.24, 2.65, 6.77, 11.33, 13.63, 0.90, 2.15, 1.66, 8.30,
            1.28, 14.55, 2.04, 14.86, 0.67, 6.83, 14.08, 15.98, 3.17, 1.38, 14.28, 10.32, 5.33,
            6.08, 8.10, 4.71, 6.75, 11.54, 15.26, 2.13, 4.11, 11.50, 7.82, 6.33, 5.72, 10.26, 1.12,
            1.98, 6.13, 5.14,
        ];

        /// Test n = 0 case
        #[test]
        fn test_0_case() {
            assert_eq!(
                get_indexes_of_first_n_smallest_values(DIFFERENCES, 0),
                std::collections::HashSet::new()
            )
        }

        /// Test n = 1 case
        #[test]
        fn test_1_case() {
            assert_eq!(
                get_indexes_of_first_n_smallest_values(DIFFERENCES, 1),
                std::collections::HashSet::from([51])
            )
        }

        /// Test n = 2 case
        #[test]
        fn test_2_case() {
            assert_eq!(
                get_indexes_of_first_n_smallest_values(DIFFERENCES, 2),
                std::collections::HashSet::from([51, 46])
            )
        }

        /// Test n = 3 case
        #[test]
        fn test_3_case() {
            assert_eq!(
                get_indexes_of_first_n_smallest_values(DIFFERENCES, 3),
                std::collections::HashSet::from([51, 46, 73])
            )
        }

        /// Test n = 4 case
        #[test]
        fn test_4_case() {
            assert_eq!(
                get_indexes_of_first_n_smallest_values(DIFFERENCES, 4),
                std::collections::HashSet::from([51, 46, 73, 19])
            )
        }

        /// Test n = 5 case
        #[test]
        fn test_5_case() {
            assert_eq!(
                get_indexes_of_first_n_smallest_values(DIFFERENCES, 5),
                std::collections::HashSet::from([51, 46, 73, 19, 69])
            )
        }
    }
}
