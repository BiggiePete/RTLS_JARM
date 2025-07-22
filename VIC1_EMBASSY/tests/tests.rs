// THIS IS THE TEST MODULE
// It will only be included when running `cargo test`
#![no_std]
#[cfg(test)]
#[defmt_test::tests]

mod tests {
    use defmt::{assert, assert_eq, info};

    // This setup function is run before each test
    #[init]
    fn main() {
        // Perform any test-specific setup here
        info!("Test setup running");
    }

    // A simple test case
    #[test]
    fn it_works() {
        info!("Running test: it_works");
        assert!(true);
    }

    // Another test case
    #[test]
    fn addition() {
        info!("Running test: addition");
        assert_eq!(2 + 2, 4);
    }

    // Use #[ignore] to skip a test
    #[test]
    #[ignore]
    fn expensive_test() {
        // ...
    }
}
